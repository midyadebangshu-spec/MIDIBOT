#!/usr/bin/env python3
"""Disturbance rejection while walking: push the pelvis during a straight walk.

Walks forward at vx = 0.1 m/s, waits until the gait is steady, applies a horizontal force to the pelvis for
`duration` s (gz ApplyLinkWrench, cleared after a fixed span of *sim* time), keeps walking, then stops. The
robot must never be FALLEN, must keep walking afterwards (it advances along the walking direction) and must
return to BALANCING after the stop.

Usage: walk_push_test.py <direction: fwd|back|left|right> [force_N] [duration_s] [push_delay_s]
Defaults: 50 N for 0.2 s (10 N*s) after 4.5 s of walking. Vary the delay to hit different gait phases.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

DIRS = {"fwd": (1, 0), "back": (-1, 0), "left": (0, 1), "right": (0, -1)}
DIRECTION = sys.argv[1] if len(sys.argv) > 1 else "fwd"
FORCE = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 0.2
DELAY = float(sys.argv[4]) if len(sys.argv) > 4 else 4.5
WORLD, LINK = "flat", "midibot::base_link"


class Monitor(Node):
    def __init__(self):
        super().__init__("walk_push_test", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.status, self.pose = "", None
        self.create_subscription(String, "/loco/status", lambda m: setattr(self, "status", m.data), 10)
        self.pub_mode = self.create_publisher(String, "/loco/mode", 10)
        self.pub_cmd = self.create_publisher(Twist, "/loco/cmd_vel", 10)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self.on_pose)
        self.on = self.gz.advertise(f"/world/{WORLD}/wrench/persistent", EntityWrench)
        self.off = self.gz.advertise(f"/world/{WORLD}/wrench/clear", Entity)
        time.sleep(1.0)  # gz discovery

    def on_pose(self, msg):
        for p in msg.pose:
            if p.name == "midibot" and (p.position.z != 0.0 or p.position.x != 0.0):
                o = p.orientation
                yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
                self.pose = (p.position.x, p.position.y, yaw, p.position.z)
                return

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def run_for(self, seconds, vx=0.0):
        t0, last = self.now(), 0.0
        while rclpy.ok() and self.now() - t0 < seconds and self.status != "FALLEN":
            rclpy.spin_once(self, timeout_sec=0.02)
            if vx and self.now() - last > 0.1:
                last = self.now()
                t = Twist()
                t.linear.x = vx
                self.pub_cmd.publish(t)

    def push(self, fx, fy):
        m = EntityWrench()
        m.entity.name, m.entity.type = LINK, Entity.LINK
        m.wrench.force.x, m.wrench.force.y = fx, fy
        self.on.publish(m)

    def release(self):
        m = Entity()
        m.name, m.type = LINK, Entity.LINK
        self.off.publish(m)


def main():
    rclpy.init()
    n = Monitor()
    print("waiting for BALANCING and the pelvis pose ...", flush=True)
    while rclpy.ok() and (n.status != "BALANCING" or n.pose is None):
        rclpy.spin_once(n, timeout_sec=0.5)
    n.run_for(3.0)
    t_req = n.now()
    while rclpy.ok() and n.status != "WALKING" and n.now() - t_req < 15.0:
        n.pub_mode.publish(String(data="walk"))
        n.run_for(0.5)
    n.run_for(3.0)  # initial hold / blend
    dx, dy = DIRS[DIRECTION]
    n.run_for(DELAY, vx=0.1)
    p_before = n.pose
    n.push(dx * FORCE, dy * FORCE)
    t_on = n.now()
    n.run_for(DUR, vx=0.1)
    n.release()
    applied = n.now() - t_on
    n.run_for(5.0, vx=0.1)
    fell = n.status == "FALLEN"
    p_after = n.pose
    adv = (p_after[0] - p_before[0]) * math.cos(p_before[2]) + (p_after[1] - p_before[1]) * math.sin(p_before[2])
    n.pub_mode.publish(String(data="stand"))
    n.run_for(30.0)
    stopped = n.status == "BALANCING"
    walked_on = adv > 0.2
    ok = (not fell) and walked_on and stopped
    print(f"push {DIRECTION:5s} {FORCE:.0f} N x {DUR:.2f} s (~{FORCE * applied:.1f} N*s) after {DELAY:.1f} s of walking: "
          f"{'FELL' if fell else 'no fall'}, advanced {adv:+.2f} m in the 5 s after, "
          f"final status {n.status} -> {'ok' if ok else 'FAIL'}", flush=True)
    print("RESULT:", "PASS" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
