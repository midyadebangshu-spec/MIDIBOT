"""MIDIBOT standing balance controller (Locomotion & Movement Unit, Phase 2).

Pipeline (runs on every new joint state, up to `rate` Hz wall; timing from message stamps):
  1. State estimate: joint positions (/joint_states) + pelvis roll/pitch (IMU) -> Pinocchio
     floating-base model in a gravity-aligned frame at the pelvis origin.
  2. Measurements: centre of mass (CoM), support centre (mid-feet), centre of pressure (CoP)
     from the ankle force/torque sensors.
  3. Control, on top of the joint PD loops of the torque-mode body controller:
       * ankle strategy: CoM-over-support PD (+ slow integral) -> ankle pitch/roll offsets
       * hip strategy:   pelvis roll/pitch PD -> hip roll/pitch offsets
  4. Safety: tilt > fall_tilt_deg -> FALLEN (stop commanding and hold the last posture).
  5. Walking (Phase 3): mode "walk" starts the WALKING state: midibot_locomotion.WalkPattern (footstep
     planner -> LIPM ZMP preview -> leg IK) gives the joint setpoints, plus pelvis-pitch feedback on the
     stance legs. Mode "stand" stops (feet together), then returns to BALANCING via a transition.

Topics
  sub  /joint_states, /imu_sensor_broadcaster/imu, /loco/{left,right}_foot_ft, /robot_description
       /loco/mode  (std_msgs/String: "stand" | "zero" | "walk")
       /loco/cmd_vel  (geometry_msgs/Twist: vx, vy, wz; used in walk mode, zero after `cmd_timeout`)
  pub  /body_controller/reference (control_msgs/MultiDOFCommand), /loco/status (String),
       /loco/com, /loco/cop, /loco/support_center  (geometry_msgs/PointStamped, frame "pelvis_level")
"""
import math

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PointStamped, Twist, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from control_msgs.msg import MultiDOFCommand, MultiDOFStateStamped
from midibot_locomotion.pose import ANKLE_H, BODY_JOINTS, FOOT_X, READY  # noqa: F401
from midibot_locomotion.gait import GaitParams
from midibot_locomotion.walker import WalkPattern

def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class BalanceController(Node):
    def __init__(self):
        super().__init__("balance_controller")
        p = self.declare_parameter
        p("rate", 100.0)
        p("transition_time", 3.0)
        # Gains per axis [sagittal x, lateral y], tuned in sim (push test passes at 12 N*s per direction).
        # The joint PD loops provide the fast stiffness; this layer re-centres the CoM over the feet.
        # The wide double-support stance is passively stable laterally, so the lateral loop is soft and
        # hip-roll feedback is off. NOTE: much of the early tuning was confounded by a starved control loop
        # (see on_controller_state); stronger gains may now be usable and are worth re-tuning.
        p("ankle_kp", [0.6, 0.1])   # rad per m of CoM error
        p("ankle_kd", [0.15, 0.03])  # rad per m/s: damps the ~0.8 Hz stance rocking mode
        p("ankle_ki", [0.3, 0.05])  # rad per m*s
        p("hip_kp", [0.3, 0.0])     # rad per rad of pelvis tilt (pitch, roll)
        p("hip_kd", [0.0, 0.0])     # rad per rad/s
        p("filter_alpha", 0.3)      # low-pass on CoM error and tilt (~6 Hz at 100 Hz)
        p("ankle_limit", 0.20)
        p("hip_limit", 0.20)
        p("com_offset_x", 0.0)    # desired CoM ahead of the support centre (m)
        p("fall_tilt_deg", 35.0)
        p("walk_blend_time", 1.0)   # s: blend from the standing setpoints to the walking pattern
        p("walk_roll_kp", 1.5)      # rad of stance hip roll per rad of pelvis roll while walking
        p("walk_roll_kd", 0.10)     # rad of stance hip roll per rad/s of roll rate
        p("walk_roll_ki", 6.0)      # rad of stance hip roll per rad*s of pelvis roll (removes the static sag)
        p("walk_roll_leak", 0.4)    # s: time constant of the roll integrator leak (the stance leg changes each step)
        p("walk_roll_limit", 0.30)  # rad: limit of the total roll correction
        p("walk_ankle_kp", [0.0, 0.0])  # rad per m of CoM-vs-stance-foot error (sagittal, lateral)
        p("walk_ankle_kd", [0.0, 0.0])  # rad per m/s
        p("walk_ankle_limit", 0.15)
        p("gait_t_ss", 0.30)        # s: single support
        p("gait_t_ds", 0.20)        # s: double support between steps
        p("gait_t_ds_first", 0.60)
        p("gait_foot_width", 0.15)  # m: distance between the soles' centres
        p("gait_min_width", 0.12)
        p("gait_step_height", 0.03)
        p("walk_tilt_kp", 0.3)      # rad of stance hip pitch per rad of pelvis pitch while walking
        p("load_wait_max", 0.25)    # s: the load gate never holds the swing longer than this
        p("load_transfer", 0.80)    # fraction of the weight the stance foot must carry before the swing starts
        p("cmd_timeout", 0.5)       # s without /loco/cmd_vel -> command zero
        p("log_file", "")         # optional CSV trace of every control step (debugging/tuning)

        self.model = self.data = None
        self.q_meas = {}
        self.v_meas = {}
        self.q_cmd = {}
        self.sim_t = 0.0
        self.last_step_t = None
        self.rpy = None
        self.gyro = np.zeros(3)
        self.ft = {}
        self.fz_f = {}
        self.mode = "stand"
        self.state = "WAIT_STATE"
        self.t_start = None
        self.q_start = None
        self.com_int = np.zeros(2)
        self.com_vel = np.zeros(3)
        self.err_f = None
        self.tilt_f = None
        self.err_d = np.zeros(2)
        self.debug_t = None
        self.walker = None
        self.walk_q0 = None
        self.walk_t0 = 0.0
        self.stop_requested = False
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_t = -1e9
        self.ik_fail = 0
        self.roll_int = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        # depth 1: always act on the newest sample, never on a backlog (a backlog made the loop run on
        # stale data and destabilised balance)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 1)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self.on_imu, 1)
        for s in ("left", "right"):
            self.create_subscription(WrenchStamped, f"/loco/{s}_foot_ft",
                                     lambda m, s=s: self.ft.__setitem__(s, m.wrench), 1)
        self.create_subscription(String, "/loco/mode", self.on_mode, 10)
        self.create_subscription(Twist, "/loco/cmd_vel", self.on_cmd_vel, 10)
        # The body controller publishes its state only once it is active. That message (29 DOFs at the
        # 500 Hz controller rate) is very expensive to deserialise in Python: staying subscribed starved
        # this loop to ~6 Hz and made the robot fall. Unsubscribe after the first message.
        self.controller_active = False
        self._ctrl_sub = self.create_subscription(MultiDOFStateStamped, "/body_controller/controller_state",
                                                  self.on_controller_state, 1)

        self.pub_ref = self.create_publisher(MultiDOFCommand, "/body_controller/reference", 10)
        self.pub_status = self.create_publisher(String, "/loco/status", 10)
        self.pub_com = self.create_publisher(PointStamped, "/loco/com", 10)
        self.pub_cop = self.create_publisher(PointStamped, "/loco/cop", 10)
        self.pub_sup = self.create_publisher(PointStamped, "/loco/support_center", 10)
        self.timer = self.create_timer(1.0 / self.get_parameter("rate").value, self.step)

    # ------------------------------------------------------------ callbacks
    def on_controller_state(self, msg):
        # the controller's current setpoints: transitions must start from these, not from the measured
        # (sagging) joint angles, or the PD holding torque vanishes in the first message (robot tips over)
        self.q_cmd = {d.name: d.reference for d in msg.dof_states}
        self.controller_active = True
        if self._ctrl_sub is not None:
            self.destroy_subscription(self._ctrl_sub)
            self._ctrl_sub = None

    def on_description(self, msg):
        if self.model is not None:
            return
        self.model = pin.buildModelFromXML(msg.data, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.frames = {s: self.model.getFrameId(f"{s}_foot") for s in ("left", "right")}
        self.get_logger().info(f"model loaded: {self.model.nq} q, mass {pin.computeTotalMass(self.model):.2f} kg")

    def on_joints(self, msg):
        self.sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for n, q in zip(msg.name, msg.position):
            self.q_meas[n] = q
        for n, v in zip(msg.name, msg.velocity):
            self.v_meas[n] = v

    def on_imu(self, msg):
        o = msg.orientation
        r, pch, _ = euler_from_quat(o.x, o.y, o.z, o.w)
        self.rpy = (r, pch)
        g = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.gyro = 0.8 * self.gyro + 0.2 * g  # low-pass

    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.cmd_t = self.sim_t

    def on_mode(self, msg):
        m = msg.data
        if m == "walk":
            if self.state == "WALKING":
                self.stop_requested = False
            elif self.state == "BALANCING":
                self.begin_walk()
            else:
                self.get_logger().warn(f"walk requested in state {self.state}: ignored (wait for BALANCING)")
        elif m in ("stand", "zero"):
            if self.state == "WALKING":
                if m == "stand":
                    self.stop_requested = True
                    self.get_logger().info("stand requested: stopping the walk")
                else:
                    self.get_logger().warn("zero is not allowed while walking: send stand first")
            elif m != self.mode:
                self.mode = m
                self.begin_transition()
                self.get_logger().info(f"mode -> {self.mode}")

    # ------------------------------------------------------------ helpers
    def target_pose(self):
        q = {j: 0.0 for j in BODY_JOINTS}
        if self.mode == "stand":
            q.update(READY)
        return q

    def begin_transition(self):
        self.t_start = self.now()
        self.q_start = {j: self.q_cmd.get(j, self.q_meas.get(j, 0.0)) for j in BODY_JOINTS}
        self.state = "TRANSITION"

    def begin_walk(self):
        if self.walker is None:
            g = self.get_parameter
            self.walker = WalkPattern(self.model, GaitParams(
                t_ss=g("gait_t_ss").value, t_ds=g("gait_t_ds").value, t_ds_first=g("gait_t_ds_first").value,
                foot_width=g("gait_foot_width").value, min_width=g("gait_min_width").value,
                step_height=g("gait_step_height").value))
        self.walker.reset()
        # blend from the current setpoints (not the sagging measured angles) into the pattern
        self.walk_q0 = {j: self.q_cmd.get(j, self.q_meas.get(j, 0.0)) for j in BODY_JOINTS}
        self.walk_t0 = self.now()
        self.stop_requested = False
        self.ik_fail = 0
        self.roll_int = 0.0
        self.walk_err = None
        self.wait_t0 = None
        self.mode = "walk"
        self.state = "WALKING"
        self.get_logger().info("mode -> walk")

    def current_cmd(self):
        if self.stop_requested or self.now() - self.cmd_t > self.get_parameter("cmd_timeout").value:
            return (0.0, 0.0, 0.0)
        return self.cmd

    def stance_loaded(self):
        """Load-transfer gate: the foot that will carry the swing phase holds >= `load_transfer` of the weight."""
        gen = self.walker.gen
        st = gen.steps[gen.k]
        stance = gen.stance_foot
        if not stance or st.kind != "walk":
            return True
        for s in ("left", "right"):  # the raw force rings on every impact: low-pass it
            raw = self.ft[s].force.z if s in self.ft else 0.0
            self.fz_f[s] = 0.85 * self.fz_f.get(s, raw) + 0.15 * raw
        fz = self.fz_f
        total = max(fz["left"], 0.0) + max(fz["right"], 0.0)
        if total < 0.3 * 9.81 * pin.computeTotalMass(self.model):
            return False  # no meaningful ground contact reading
        return max(fz[stance], 0.0) / total >= self.get_parameter("load_transfer").value

    def walking_step(self, dt):
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        cmd = self.current_cmd()
        if self.ik_fail > 10:  # IK keeps failing: stop walking instead of freezing mid-step
            cmd = (0.0, 0.0, 0.0)
        load_ok = self.stance_loaded()
        if self.walker.gen.waiting_for_load:
            if self.wait_t0 is None:
                self.wait_t0 = self.now()
            elif self.now() - self.wait_t0 > gp("load_wait_max"):
                load_ok = True  # do not let the CoM slide onto the foot centre: go ahead
        else:
            self.wait_t0 = None
        joints, st = None, None
        for _ in range(min(max(int(round(dt / self.walker.params.dt)), 1), 5)):
            joints, st = self.walker.step(cmd, load_ok)
            self.ik_fail = 0 if self.walker.converged else self.ik_fail + 1
        a = smoothstep((self.now() - self.walk_t0) / gp("walk_blend_time"))
        q = {j: (1 - a) * self.walk_q0[j] + a * joints[j] for j in BODY_JOINTS}
        # pelvis pitch feedback on the legs that carry weight (same sign convention as standing)
        fa = gp("filter_alpha")
        tilt_raw = np.array(self.rpy)
        self.tilt_f = tilt_raw if self.tilt_f is None else (1 - fa) * self.tilt_f + fa * tilt_raw
        h = float(np.clip(gp("walk_tilt_kp") * self.tilt_f[1], -gp("hip_limit"), gp("hip_limit"))) * a
        # ankle strategy on the stance foot: measured CoM relative to the stance foot(s) vs the pattern's
        stance = [s_ for s_ in ("left", "right") if st[s_][2] < 1e-4]
        sole = getattr(self, "sole", {})
        u = np.zeros(2)
        if stance and all(s_ in sole for s_ in stance) and getattr(self, "com_est", None) is not None:
            c, sn = math.cos(st["yaw"]), math.sin(st["yaw"])
            to_body = lambda v: np.array([c * v[0] + sn * v[1], -sn * v[0] + c * v[1]])  # noqa: E731
            d_com = to_body(st["com"] - st["pelvis"][:2])
            err = np.mean([(self.com_est[:2] - sole[s_][:2]) - (d_com - to_body(np.array(st[s_][:2]) - st["pelvis"][:2]))
                           for s_ in stance], axis=0)
            fa2 = gp("filter_alpha")
            prev = err if self.walk_err is None else self.walk_err
            self.walk_err = (1 - fa2) * prev + fa2 * err
            derr = (self.walk_err - prev) / max(dt, 1e-3)
            ga = lambda n: np.array(gp(n), dtype=float)  # noqa: E731
            u = np.clip(ga("walk_ankle_kp") * self.walk_err + ga("walk_ankle_kd") * derr,
                        -gp("walk_ankle_limit"), gp("walk_ankle_limit")) * a
            for s_ in stance:
                q[f"{s_}_ankle_pitch_joint"] += u[0]
                q[f"{s_}_hip_pitch_joint"] += -u[0]
                q[f"{s_}_ankle_roll_joint"] -= u[1]
                q[f"{s_}_hip_roll_joint"] += u[1]
        else:
            self.walk_err = None
        # pelvis roll feedback: the stance ankle roll joints are soft, so the body sags towards the stance side
        # Hip-roll feedback only makes sense in single support. With both soles planted, the same hip-roll
        # offset on both legs cannot roll the pelvis (the legs have equal length) and just tilts the feet.
        lim = gp("walk_roll_limit")
        single = len(stance) == 1
        roll_in = self.tilt_f[0] if single else 0.0
        self.roll_int = float(np.clip(self.roll_int * (1.0 - dt / gp("walk_roll_leak"))
                                      + gp("walk_roll_ki") * roll_in * dt, -lim, lim))
        hr = (float(np.clip(gp("walk_roll_kp") * roll_in + gp("walk_roll_kd") * (self.gyro[0] if single else 0.0)
                            + self.roll_int, -lim, lim)) * a) if single else 0.0
        for s in ("left", "right"):
            if st[s][2] < 1e-4:
                q[f"{s}_hip_pitch_joint"] += h
                q[f"{s}_hip_roll_joint"] += hr
        if self.debug_t is None or self.now() - self.debug_t > 0.5:
            self.debug_t = self.now()
            self.get_logger().info(
                f"[WALKING/{st['phase']}] cmd {cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f}  "
                f"pattern com {st['com'][0]:+.3f},{st['com'][1]:+.3f}  yaw {math.degrees(st['yaw']):+.1f} deg  "
                f"tilt r/p {math.degrees(self.rpy[0]):+.1f}/{math.degrees(self.rpy[1]):+.1f}  ik_fail {self.ik_fail}  "
                f"{'WAIT-LOAD' if self.walker.gen.waiting_for_load else ''}")
        ex, ey = (self.walk_err if self.walk_err is not None else (float('nan'), float('nan')))
        if gp("log_file"):
            if not hasattr(self, "_wlog"):
                self._wlog = open(gp("log_file") + ".walk.csv", "w")
                self._wlog.write("t,phase,wait,roll,pitch,gx,hr,h,fz_l,fz_r,com_x,com_y,left_z,right_z,ux,uy,ex,ey\n")
            fz = {s_: (self.ft[s_].force.z if s_ in self.ft else 0.0) for s_ in ("left", "right")}
            self._wlog.write(f"{self.now():.3f},{st['phase']},{int(self.walker.gen.waiting_for_load)},"
                             f"{self.rpy[0]:.4f},{self.rpy[1]:.4f},{self.gyro[0]:.4f},{hr:.4f},{h:.4f},"
                             f"{fz['left']:.1f},{fz['right']:.1f},{st['com'][0]:.4f},{st['com'][1]:.4f},"
                             f"{st['left'][2]:.4f},{st['right'][2]:.4f},{u[0]:.4f},{u[1]:.4f},{ex:.4f},{ey:.4f}\n")
            self._wlog.flush()
        self.publish(q)
        self.pub_status.publish(String(data="WALKING"))
        if self.stop_requested and self.walker.gen.settled:
            self.get_logger().info("walk finished: back to the ready pose")
            self.stop_requested = False
            self.mode = "stand"
            self.begin_transition()

    def now(self):
        """Simulation time from the joint-state stamps.

        The node deliberately does not use ROS sim time: Gazebo publishes /clock every 1 ms physics step,
        which costs rclpy tens of % CPU per node.
        """
        return self.sim_t

    def estimate(self):
        """CoM, CoP and support centre in a gravity-aligned frame at the pelvis origin."""
        q = pin.neutral(self.model)
        R = pin.rpy.rpyToMatrix(self.rpy[0], self.rpy[1], 0.0)
        q[3:7] = pin.Quaternion(R).coeffs()
        for jid in range(2, self.model.njoints):
            name = self.model.names[jid]
            if name in self.q_meas:
                q[self.model.joints[jid].idx_q] = self.q_meas[name]
        v = np.zeros(self.model.nv)
        v[3:6] = self.gyro  # base angular velocity (pelvis frame); base linear velocity unknown -> 0
        for jid in range(2, self.model.njoints):
            name = self.model.names[jid]
            if name in self.v_meas:
                v[self.model.joints[jid].idx_v] = self.v_meas[name]
        com = pin.centerOfMass(self.model, self.data, q, v)
        pin.updateFramePlacements(self.model, self.data)
        # d/dt(CoM - support): invariant to the (unknown) pelvis linear velocity
        v_sup = sum(pin.getFrameVelocity(self.model, self.data, self.frames[s],
                                         pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear for s in ("left", "right")) / 2
        self.com_vel = self.data.vcom[0] - v_sup
        sup, cop_num, fz_sum = np.zeros(3), np.zeros(3), 0.0
        self.sole = {}
        self.com_est = com
        for s in ("left", "right"):
            M = self.data.oMf[self.frames[s]]
            sup += M.act(np.array([FOOT_X, 0.0, -ANKLE_H])) / 2.0
            self.sole[s] = M.act(np.array([FOOT_X, 0.0, -ANKLE_H]))
            w = self.ft.get(s)
            if w is not None and w.force.z > 5.0:
                fz = w.force.z
                px = (-w.torque.y - ANKLE_H * w.force.x) / fz
                py = (w.torque.x - ANKLE_H * w.force.y) / fz
                cop_num += fz * M.act(np.array([px, py, -ANKLE_H]))
                fz_sum += fz
        cop = cop_num / fz_sum if fz_sum > 0 else None
        return com, cop, sup

    def point(self, v, pub):
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "pelvis_level"
        m.point.x, m.point.y, m.point.z = float(v[0]), float(v[1]), float(v[2])
        pub.publish(m)

    def publish(self, q):
        self.q_cmd = dict(q)
        # position reference + zero velocity reference: the joint PD's d-term then damps joint velocity
        self.pub_ref.publish(MultiDOFCommand(dof_names=BODY_JOINTS, values=[float(q[j]) for j in BODY_JOINTS],
                                             values_dot=[0.0] * len(BODY_JOINTS)))

    # ------------------------------------------------------------ main loop
    def step(self):
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        if (self.model is None or self.rpy is None or not self.controller_active
                or not all(j in self.q_meas for j in BODY_JOINTS)):
            self.pub_status.publish(String(data="WAIT_STATE"))
            return
        if self.state == "WAIT_STATE":
            self.get_logger().info("state received: moving to the ready pose")
            self.begin_transition()

        tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(self.rpy[0]) * math.cos(self.rpy[1])))))
        if tilt > gp("fall_tilt_deg") or self.state == "FALLEN":
            if self.state != "FALLEN":
                self.get_logger().error(f"fall detected (tilt {tilt:.1f} deg): commands stopped")
            self.state = "FALLEN"
            self.pub_status.publish(String(data="FALLEN"))
            return

        com, cop, sup = self.estimate()
        self.point(com, self.pub_com)
        self.point(sup, self.pub_sup)
        if cop is not None:
            self.point(cop, self.pub_cop)

        target = self.target_pose()
        if self.last_step_t is not None and self.now() <= self.last_step_t:
            return  # no new state since the last step
        dt = 0.01 if self.last_step_t is None else min(max(self.now() - self.last_step_t, 1e-4), 0.05)
        self.last_step_t = self.now()
        if self.state == "WALKING":
            self.walking_step(dt)
            return
        ramp = 1.0
        if self.state == "TRANSITION":
            a = smoothstep((self.now() - self.t_start) / gp("transition_time"))
            base = {j: (1 - a) * self.q_start[j] + a * target[j] for j in BODY_JOINTS}
            ramp = a  # fade the balance feedback in with the posture change
            if a >= 1.0:
                self.get_logger().info("ready pose reached: balancing")
                self.state = "BALANCING"
        else:
            base = target

        # ankle strategy on CoM error, hip strategy on pelvis tilt (active during transitions too,
        # so crouching never moves the CoM out of the support without feedback)
        raw = np.array([com[0] - (sup[0] + gp("com_offset_x")), com[1] - sup[1]])
        fa = gp("filter_alpha")
        self.err_f = raw if self.err_f is None else (1 - fa) * self.err_f + fa * raw
        tilt_raw = np.array(self.rpy)
        self.tilt_f = tilt_raw if self.tilt_f is None else (1 - fa) * self.tilt_f + fa * tilt_raw
        err = self.err_f
        # CoM velocity relative to the support (Jacobian-based), low-passed at ~6 Hz
        self.err_d = (1 - fa) * self.err_d + fa * self.com_vel[:2]
        if self.state == "BALANCING":  # integrate only once the posture is settled
            self.com_int = np.clip(self.com_int + err * dt, -0.05, 0.05)
        ga = lambda n: np.array(gp(n), dtype=float)  # noqa: E731
        u = ga("ankle_kp") * err + ga("ankle_kd") * self.err_d + ga("ankle_ki") * self.com_int
        u = ramp * np.clip(u, -gp("ankle_limit"), gp("ankle_limit"))
        hkp, hkd = ga("hip_kp"), ga("hip_kd")
        # h = [roll correction, pitch correction]
        h = np.clip(np.array([hkp[1] * self.tilt_f[0] + hkd[1] * self.gyro[0],
                              hkp[0] * self.tilt_f[1] + hkd[0] * self.gyro[1]]),
                    -gp("hip_limit"), gp("hip_limit")) * ramp

        if self.debug_t is None or self.now() - self.debug_t > 0.5:
            self.debug_t = self.now()
            self.get_logger().info(
                f"[{self.state}] tilt r/p {math.degrees(self.rpy[0]):+.1f}/{math.degrees(self.rpy[1]):+.1f} deg  "
                f"com-sup {err[0] * 100:+.1f},{err[1] * 100:+.1f} cm  ankle {u[0]:+.3f},{u[1]:+.3f}  "
                f"hip {h[0]:+.3f},{h[1]:+.3f}")
        if gp("log_file"):
            if not hasattr(self, "_log"):
                self._log = open(gp("log_file"), "w")
                self._log.write("t,state,roll,pitch,gx,gy,ex,ey,vx,vy,ux,uy,hr,hp\n")
            self._log.write(f"{self.now():.3f},{self.state},{self.rpy[0]:.4f},{self.rpy[1]:.4f},"
                            f"{self.gyro[0]:.4f},{self.gyro[1]:.4f},{err[0]:.4f},{err[1]:.4f},"
                            f"{self.com_vel[0]:.4f},{self.com_vel[1]:.4f},{u[0]:.4f},{u[1]:.4f},"
                            f"{h[0]:.4f},{h[1]:.4f}\n")
            self._log.flush()
        q = dict(base)
        for s in ("left", "right"):
            # Ankle offsets translate the body (+ankle pitch -> backwards, +ankle roll -> +y). With the feet
            # planted, pelvis pitch = -(ankle + knee + hip pitch) and pelvis roll = -(ankle + hip roll), so
            # the hips get the opposite offset to keep the pelvis level; h adds tilt feedback on top.
            q[f"{s}_ankle_pitch_joint"] += u[0]
            q[f"{s}_ankle_roll_joint"] -= u[1]
            q[f"{s}_hip_pitch_joint"] += -u[0] + h[1]
            q[f"{s}_hip_roll_joint"] += u[1] + h[0]
        self.publish(q)
        self.pub_status.publish(String(data=self.state))


def euler_from_quat(x, y, z, w):
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return r, p, yw


def main():
    rclpy.init()
    rclpy.spin(BalanceController())


if __name__ == "__main__":
    main()
