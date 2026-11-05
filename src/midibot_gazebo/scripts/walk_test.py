#!/usr/bin/env python3
"""Phase 3 acceptance: walking driven by /loco/cmd_vel.

Sequence (all on flat ground, balance controller running): wait for BALANCING -> mode "walk" ->
  1. forward  vx=+0.10 m/s for `walk_time` s, then stop
  2. backward vx=-0.08 m/s, then stop
  3. turn     wz=+0.30 rad/s, then stop
After every segment the robot must be standing again (status BALANCING after the stop returns to the
ready pose), never FALLEN, with the pelvis moved/turned in the commanded direction by a sensible amount.
The robot's own state estimate (/loco/odom) is compared with the ground-truth pelvis pose from gz-transport:
over each segment the position error must stay below max(5 cm, 15 % of the distance) and the yaw error below 5 deg.
The pelvis pose comes from gz-transport (/world/flat/pose/info, model midibot).

Usage: walk_test.py [walk_time_s]   (default 8)
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

WALK_TIME = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
WORLD = "flat"
# (name, (vx, vy, wz), minimum |displacement| m or turn rad expected as a fraction of the commanded)
SEGMENTS = [("forward", (0.10, 0.0, 0.0)), ("backward", (-0.08, 0.0, 0.0)), ("turn", (0.0, 0.0, 0.30))]


class Monitor(Node):
    def __init__(self):
        super().__init__("walk_test", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.status = ""
        self.pose = None  # (x, y, yaw, z)
        self.pose_names = []
        self.odom = None  # (x, y, yaw, z) from the controller's state estimator
        self.create_subscription(Odometry, "/loco/odom", self.on_odom, 10)
        self.create_subscription(String, "/loco/status", lambda m: setattr(self, "status", m.data), 10)
        self.pub_mode = self.create_publisher(String, "/loco/mode", 10)
        self.pub_cmd = self.create_publisher(Twist, "/loco/cmd_vel", 10)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self.on_pose)

    def on_pose(self, msg):
        # prefer the model pose; skip all-zero poses (not the robot)
        for want in ("midibot", "midibot::base_link", "base_link"):
            for p in msg.pose:
                if p.name == want and (p.position.z != 0.0 or p.position.x != 0.0):
                    o = p.orientation
                    yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
                    self.pose = (p.position.x, p.position.y, yaw, p.position.z)
                    return
        self.pose_names = sorted({p.name for p in msg.pose})[:12]

    def on_odom(self, m):
        o = m.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw, m.pose.pose.position.z)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def run_for(self, seconds, cmd=None, until=None):
        t0 = self.now()
        last = 0.0
        while rclpy.ok() and self.now() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)
            if cmd is not None and self.now() - last > 0.1:  # keep the command fresh (timeout 0.5 s)
                last = self.now()
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = cmd
                self.pub_cmd.publish(t)
            if self.status == "FALLEN" or (until and until()):
                break

    def mode(self, m):
        self.pub_mode.publish(String(data=m))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    rclpy.init()
    n = Monitor()
    print("waiting for BALANCING and the pelvis pose ...", flush=True)
    t_wait = time.time()
    while rclpy.ok() and (n.status != "BALANCING" or n.pose is None):
        rclpy.spin_once(n, timeout_sec=0.5)
        if n.pose is None and time.time() - t_wait > 15:
            print(f"no robot pose yet; pose names seen on the topic: {n.pose_names}", flush=True)
            t_wait = time.time()
    n.run_for(3.0)
    ok = True
    z0 = n.pose[3]
    for name, cmd in SEGMENTS:
        p0 = n.pose
        t_req = n.now()
        while rclpy.ok() and n.status not in ("WALKING", "FALLEN") and n.now() - t_req < 15.0:
            n.mode("walk")  # resend: the first message can be sent before the controller has subscribed
            n.run_for(0.5, until=lambda: n.status == "WALKING")
        if n.status != "WALKING":
            print(f"{name}: controller never entered WALKING (status {n.status})")
            ok = False
            break
        n.run_for(2.5)  # blend-in and weight shift (no command yet)
        p_start, o_start = n.pose, n.odom
        n.run_for(WALK_TIME, cmd)
        n.mode("stand")
        n.run_for(30.0, until=lambda: n.status == "BALANCING")
        n.run_for(3.0)
        p1, o1 = n.pose, n.odom
        fell = n.status == "FALLEN"
        dx, dy = p1[0] - p_start[0], p1[1] - p_start[1]
        along = dx * math.cos(p_start[2]) + dy * math.sin(p_start[2])
        dyaw = wrap(p1[2] - p_start[2])
        if name == "turn":
            expect = cmd[2] * WALK_TIME
            moved = dyaw
            good = moved > 0.5 * expect
        else:
            expect = cmd[0] * WALK_TIME
            moved = along
            good = moved * expect > 0 and abs(moved) > 0.4 * abs(expect)
        upright = n.pose[3] > z0 - 0.05
        passed = (not fell) and n.status == "BALANCING" and good and upright
        ok &= passed
        print(f"{name:9s} cmd {cmd}: advanced {along:+.3f} m, turned {math.degrees(dyaw):+.1f} deg "
              f"(commanded {expect:+.2f}), pelvis z {n.pose[3]:.3f} (start {z0:.3f}), status {n.status} "
              f"-> {'ok' if passed else 'FAIL'}", flush=True)
        if o_start is not None and o1 is not None and not fell:
            e_pos = math.hypot((o1[0] - o_start[0]) - dx, (o1[1] - o_start[1]) - dy)
            e_yaw = wrap((o1[2] - o_start[2]) - dyaw)
            dist = math.hypot(dx, dy)
            est_ok = e_pos < max(0.05, 0.15 * dist) and abs(e_yaw) < math.radians(5)
            ok &= est_ok
            print(f"          estimator vs ground truth: position error {e_pos * 100:.1f} cm over {dist:.2f} m, "
                  f"yaw error {math.degrees(e_yaw):+.1f} deg -> {'ok' if est_ok else 'FAIL'}", flush=True)
        elif not fell:
            print("          estimator: no /loco/odom received -> FAIL", flush=True)
            ok = False
        if fell:
            break
    print("RESULT:", "PASS" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
