#!/usr/bin/env python3
"""Phase 2 acceptance: standing balance + push recovery.

1. Waits for the balance controller to report BALANCING, then checks quiet standing
   for `stand_time` seconds (max tilt, CoM-over-support error).
2. Applies horizontal pushes to the pelvis (gz ApplyLinkWrench, persistent for a fixed
   span of *sim* time, then cleared) and checks the robot recovers: never FALLEN,
   tilt back under 3 deg within 4 s.

Usage: push_test.py [force_N] [duration_s]   (defaults 60 N for 0.2 s = 12 N*s per push)
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.transport13 import Node as GzNode

FORCE = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
STAND_TIME = 10.0
WORLD = "flat"
LINK = "midibot::base_link"
PUSHES = [("forward", (1, 0)), ("backward", (-1, 0)), ("left", (0, 1)), ("right", (0, -1))]


class Pusher:
    """Low-latency pushes through gz-transport (ApplyLinkWrench listens on gz topics)."""

    def __init__(self):
        self.node = GzNode()
        self.on = self.node.advertise(f"/world/{WORLD}/wrench/persistent", EntityWrench)
        self.off = self.node.advertise(f"/world/{WORLD}/wrench/clear", Entity)
        time.sleep(1.0)  # discovery

    def start(self, fx, fy):
        m = EntityWrench()
        m.entity.name, m.entity.type = LINK, Entity.LINK
        m.wrench.force.x, m.wrench.force.y = fx, fy
        self.on.publish(m)

    def stop(self):
        m = Entity()
        m.name, m.type = LINK, Entity.LINK
        self.off.publish(m)


class Monitor(Node):
    def __init__(self):
        super().__init__("push_test", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.status, self.tilt, self.com, self.sup = "", 0.0, None, None
        self.create_subscription(String, "/loco/status", lambda m: setattr(self, "status", m.data), 10)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self.on_imu, 10)
        self.create_subscription(PointStamped, "/loco/com", lambda m: setattr(self, "com", m.point), 10)
        self.create_subscription(PointStamped, "/loco/support_center", lambda m: setattr(self, "sup", m.point), 10)

    def on_imu(self, m):
        q = m.orientation
        self.tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def run_for(self, seconds, cb=None):
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cb:
                cb()

    def com_err(self):
        if self.com is None or self.sup is None:
            return float("nan")
        return math.hypot(self.com.x - self.sup.x, self.com.y - self.sup.y)


def main():
    rclpy.init()
    n = Monitor()
    pusher = Pusher()
    print("waiting for BALANCING ...", flush=True)
    while rclpy.ok() and n.status != "BALANCING":
        rclpy.spin_once(n, timeout_sec=0.5)
    n.run_for(2.0)

    ok = True
    stats = {"tilt": 0.0, "err": 0.0}

    def track():
        stats["tilt"] = max(stats["tilt"], n.tilt)
        e = n.com_err()
        if not math.isnan(e):
            stats["err"] = max(stats["err"], e)

    n.run_for(STAND_TIME, track)
    print(f"quiet stance {STAND_TIME:.0f} s : max tilt {stats['tilt']:.2f} deg, "
          f"max CoM-support error {stats['err'] * 100:.1f} cm, status {n.status}")
    ok &= stats["tilt"] < 3.0 and n.status == "BALANCING"

    impulse = FORCE * DUR
    for name, (dx, dy) in PUSHES:
        stats["tilt"] = 0.0
        pusher.start(dx * FORCE, dy * FORCE)
        t_on = n.now()
        n.run_for(DUR, track)
        pusher.stop()
        actual = n.now() - t_on  # includes publish latency of the clear command
        recovered_at = None
        t0 = n.now()

        def watch():
            nonlocal recovered_at
            track()
            if recovered_at is None and n.tilt < 3.0 and n.now() - t0 > 0.2:
                recovered_at = n.now() - t0
        n.run_for(4.0, watch)
        passed = n.status != "FALLEN" and n.tilt < 3.0
        ok &= passed
        rec = f"{recovered_at:.2f} s" if recovered_at is not None else "never"
        print(f"push {name:8s} {FORCE:.0f} N x {DUR:.2f} s (~{FORCE * actual:.1f} N*s applied): max tilt {stats['tilt']:5.2f} deg, "
              f"back under 3 deg after {rec}, final tilt {n.tilt:.2f} deg, status {n.status} "
              f"-> {'ok' if passed else 'FAIL'}", flush=True)
        if n.status == "FALLEN":
            break
        n.run_for(1.0)
    print("RESULT:", "PASS" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
