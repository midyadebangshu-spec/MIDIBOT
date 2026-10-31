#!/usr/bin/env python3
"""Save one frame from a Gazebo camera topic (ROS sensor_msgs/Image) as PNG.
Usage: snapshot.py <ros_image_topic> <out.png>"""
import sys
import zlib
import struct

import rclpy
from sensor_msgs.msg import Image


def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    open(path, "wb").write(png)


def main():
    rclpy.init()
    n = rclpy.create_node("snapshot")
    got = []
    n.create_subscription(Image, sys.argv[1], lambda m: got.append(m), 1)
    while rclpy.ok() and len(got) < 2:  # skip the first (possibly stale) frame
        rclpy.spin_once(n, timeout_sec=1.0)
    m = got[-1]
    write_png(sys.argv[2], m.width, m.height, bytes(m.data))
    print(f"saved {sys.argv[2]} ({m.width}x{m.height})")


if __name__ == "__main__":
    main()
