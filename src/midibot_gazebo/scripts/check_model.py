#!/usr/bin/env python3
"""Phase 1 acceptance check for MIDIBOT in Gazebo.

Listens for a few seconds of sim time and reports:
  * joints reported on /joint_states (expects all 41 actuated + mimic joints),
  * pelvis tilt from the IMU (robot must stay upright),
  * total vertical foot force vs. weight (validates the mass model and that the
    robot is standing on both feet),
  * max joint position deviation from the zero pose (joints holding).
Exit code 0 when every check passes.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

EXPECTED_MASS = float(sys.argv[1]) if len(sys.argv) > 1 else 39.39
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
G = 9.81


class Checker(Node):
    def __init__(self):
        super().__init__("midibot_phase1_check", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.js, self.tilt, self.fz = None, [], {"left": [], "right": []}
        self.max_dev = 0.0
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self.on_imu, 10)
        for s in ("left", "right"):
            self.create_subscription(WrenchStamped, f"/loco/{s}_foot_ft",
                                     lambda m, s=s: self.fz[s].append(m.wrench.force.z), 10)
        self.t0 = None

    def on_js(self, m):
        self.js = m
        if self.t0 is not None and self.now() - self.t0 > 1.0:  # ignore settling
            body = [abs(p) for n, p in zip(m.name, m.position) if "_hand" not in n and not n[:2] in ("L_", "R_")]
            self.max_dev = max(self.max_dev, max(body) if body else 0.0)

    def on_imu(self, m):
        q = m.orientation
        # angle between body Z and world Z
        zz = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.tilt.append(math.degrees(math.acos(max(-1.0, min(1.0, zz)))))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    rclpy.init()
    n = Checker()
    while rclpy.ok() and n.now() == 0.0:
        rclpy.spin_once(n, timeout_sec=0.5)
    n.t0 = n.now()
    while rclpy.ok() and n.now() - n.t0 < DURATION:
        rclpy.spin_once(n, timeout_sec=0.5)

    ok = True
    nj = len(n.js.name) if n.js else 0
    print(f"joints on /joint_states : {nj}")
    ok &= nj >= 41
    if n.tilt:
        print(f"pelvis tilt (max)       : {max(n.tilt):.2f} deg")
        ok &= max(n.tilt) < 5.0
    else:
        print("pelvis tilt             : NO IMU DATA"); ok = False
    if n.fz["left"] and n.fz["right"]:
        k = max(1, len(n.fz["left"]) // 2)
        fl = sum(n.fz["left"][-k:]) / k
        fr = sum(n.fz["right"][-k:]) / k
        # ankle-roll F/T measures the load of everything above the foot links
        print(f"foot force left/right   : {abs(fl):.1f} N / {abs(fr):.1f} N")
        total = abs(fl) + abs(fr)
        print(f"sum vs weight above feet: {total:.1f} N  (model: {(EXPECTED_MASS - 2 * 0.40) * G:.1f} N)")
        ok &= abs(total - (EXPECTED_MASS - 0.8) * G) / ((EXPECTED_MASS - 0.8) * G) < 0.05
        ok &= min(abs(fl), abs(fr)) > 0.3 * total / 2
    else:
        print("foot force              : NO F/T DATA"); ok = False
    print(f"max body joint deviation: {n.max_dev:.4f} rad")
    ok &= n.max_dev < 0.05
    print("RESULT:", "PASS" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
