#!/usr/bin/env python3
"""Keyboard teleoperation of MIDIBOT's walking (run it in a terminal and keep that terminal focused).

    W / S   walk forward / backward          A / D   turn left / right
    Q / E   sidestep left / right            SPACE   stop (the robot finishes its step and stands)
    + / -   walking speed up / down          X       quit (the robot returns to the standing pose)

Keys *set* a command that stays active until you press SPACE (or another key for that axis), so you do not have
to hold anything down; you can combine axes (W then A = walk forward while turning). The command is re-sent
at 10 Hz because the locomotion unit zeroes it after 0.5 s without a message (a crashed teleop stops the robot).
Starting the teleop switches the robot to walk mode; quitting switches it back to stand.

With stdin that is not a terminal (tests, pipes) the characters are read from the pipe and EOF quits.
"""
import os
import select
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

# nominal speeds (m/s, m/s, rad/s); the locomotion unit clips to its own limits (0.15 / 0.08 / 0.35)
FORWARD, BACKWARD, SIDESTEP, TURN = 0.10, 0.08, 0.06, 0.30
SCALE_STEP, SCALE_MIN, SCALE_MAX = 0.25, 0.25, 1.25

HELP = __doc__.split("\n\n")[0]


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop_wasd")
        self.status = ""
        self.vx = self.vy = self.wz = 0.0  # unscaled command in units of the nominal speeds: -1, 0, +1
        self.scale = 1.0
        self.create_subscription(String, "/loco/status", lambda m: setattr(self, "status", m.data), 10)
        self.pub_cmd = self.create_publisher(Twist, "/loco/cmd_vel", 10)
        self.pub_mode = self.create_publisher(String, "/loco/mode", 10)
        self.create_timer(0.1, self.send)

    def send(self):
        m = Twist()
        m.linear.x = self.vx * (FORWARD if self.vx > 0 else BACKWARD) * self.scale
        m.linear.y = self.vy * SIDESTEP * self.scale
        m.angular.z = self.wz * TURN * self.scale
        self.pub_cmd.publish(m)

    def mode(self, name):
        self.pub_mode.publish(String(data=name))

    def line(self):
        return (f"\rvx {self.vx * (FORWARD if self.vx > 0 else BACKWARD) * self.scale:+.2f} m/s  "
                f"vy {self.vy * SIDESTEP * self.scale:+.2f} m/s  wz {self.wz * TURN * self.scale:+.2f} rad/s  "
                f"speed x{self.scale:.2f}  [{self.status}]      ")

    def key(self, k):
        """Apply one key press. Returns False to quit."""
        k = k.lower()
        if k == "w":
            self.vx = 1.0
        elif k == "s":
            self.vx = -1.0
        elif k == "a":
            self.wz = 1.0
        elif k == "d":
            self.wz = -1.0
        elif k == "q":
            self.vy = 1.0
        elif k == "e":
            self.vy = -1.0
        elif k == " ":
            self.vx = self.vy = self.wz = 0.0
        elif k in ("+", "="):
            self.scale = min(self.scale + SCALE_STEP, SCALE_MAX)
        elif k in ("-", "_"):
            self.scale = max(self.scale - SCALE_STEP, SCALE_MIN)
        elif k in ("x", "\x03", "\x1b"):
            return False
        return True


def read_key(fd, timeout):
    """One character from stdin, '' on timeout, None on EOF."""
    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return ""
    data = os.read(fd, 1)
    return data.decode(errors="ignore") if data else None


def spin_for(node, seconds):
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    rclpy.init()
    node = Teleop()
    fd = sys.stdin.fileno()
    is_tty = sys.stdin.isatty()
    old = None
    if is_tty:
        import termios
        import tty
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)  # single keys without Enter; Ctrl-C still works
    try:
        print(HELP)
        print("waiting for the robot to stand up ...", flush=True)
        while rclpy.ok() and node.status != "BALANCING":
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.status == "FALLEN":
                print("the robot has fallen: restart the simulation")
                return 1
        t0 = time.time()
        while rclpy.ok() and node.status != "WALKING" and time.time() - t0 < 15.0:
            node.mode("walk")
            spin_for(node, 0.5)
        if node.status != "WALKING":
            print(f"the controller did not enter walk mode (status {node.status})")
            return 1
        print("walk mode on. Ready.\n", flush=True)
        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.0)
            k = read_key(fd, 0.05)
            if k is None:  # EOF (non-interactive use)
                break
            if k:
                running = node.key(k)
            if node.status == "FALLEN":
                print("\nthe robot has fallen: restart the simulation")
                return 1
            print(node.line(), end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if old is not None:
            import termios
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if rclpy.ok():
            node.vx = node.vy = node.wz = 0.0
            node.send()
            node.mode("stand")
            print("\nstopping: returning to the standing pose ...", flush=True)
            t0 = time.time()
            while rclpy.ok() and node.status != "BALANCING" and time.time() - t0 < 30.0:
                node.send()
                rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
