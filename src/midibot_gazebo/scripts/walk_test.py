#!/usr/bin/env python3
"""Phase 3 acceptance: walking driven by /loco/cmd_vel.

Sequence (all on flat ground, balance controller running): wait for BALANCING -> mode "walk" ->
  1. forward  vx=+0.10 m/s for `walk_time` s, then stop
  2. backward vx=-0.08 m/s, then stop
  3. turn     wz=+0.30 rad/s, then stop
After every segment the robot must be standing again (status BALANCING after the stop returns to the
ready pose), never FALLEN, with the pelvis moved/turned in the commanded direction by a sensible amount.
The pelvis pose comes from gz-transport (/world/flat/dynamic_pose/info, link base_link).

Usage: walk_test.py [walk_time_s]   (default 8)
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
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
        self.create_subscription(String, "/loco/status", lambda m: setattr(self, "status", m.data), 10)
        self.pub_mode = self.create_publisher(String, "/loco/mode", 10)
        self.pub_cmd = self.create_publisher(Twist, "/loco/cmd_vel", 10)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f"/world/{WORLD}/dynamic_pose/info", self.on_pose)

    def on_pose(self, msg):
        for p in msg.pose:
            if p.name == "base_link":
                o = p.orientation
                yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
                self.pose = (p.position.x, p.position.y, yaw, p.position.z)

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
    while rclpy.ok() and (n.status != "BALANCING" or n.pose is None):
        rclpy.spin_once(n, timeout_sec=0.5)
    n.run_for(3.0)
    ok = True
    z0 = n.pose[3]
    for name, cmd in SEGMENTS:
        p0 = n.pose
        n.mode("walk")
        n.run_for(1.8, until=lambda: n.status == "WALKING")  # let the mode switch + initial hold pass
        n.run_for(2.5)  # blend-in and weight shift (no command yet)
        p_start = n.pose
        n.run_for(WALK_TIME, cmd)
        n.mode("stand")
        n.run_for(30.0, until=lambda: n.status == "BALANCING")
        n.run_for(3.0)
        p1 = n.pose
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
        if fell:
            break
    print("RESULT:", "PASS" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
