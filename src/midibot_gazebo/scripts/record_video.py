#!/usr/bin/env python3
"""Record a Gazebo camera topic (ROS sensor_msgs/Image) to MP4 at sim-time speed.

Frames are placed by their sim-time stamps, so the video plays at real (simulated) speed
regardless of how slowly the simulator runs. Requires ffmpeg.
Usage: record_video.py <image_topic> <out.mp4> <sim_seconds> [fps]
"""
import subprocess
import sys

import rclpy
from sensor_msgs.msg import Image


def main():
    topic, out, dur = sys.argv[1], sys.argv[2], float(sys.argv[3])
    fps = int(sys.argv[4]) if len(sys.argv) > 4 else 25
    rclpy.init()
    n = rclpy.create_node("record_video")
    st = {"proc": None, "t0": None, "frames": 0, "last": None}

    def cb(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st["proc"] is None:
            st["proc"] = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                 "-s", f"{m.width}x{m.height}", "-r", str(fps), "-i", "-", "-c:v", "libx264",
                 "-pix_fmt", "yuv420p", "-crf", "23", out], stdin=subprocess.PIPE)
            st["t0"] = t
        # write as many frames as sim time has advanced (duplicates fill dropped frames)
        target = int((t - st["t0"]) * fps) + 1
        while st["frames"] < target:
            st["proc"].stdin.write(bytes(m.data))
            st["frames"] += 1
        st["last"] = t

    n.create_subscription(Image, topic, cb, 10)
    while rclpy.ok() and (st["last"] is None or st["last"] - st["t0"] < dur):
        rclpy.spin_once(n, timeout_sec=0.5)
    st["proc"].stdin.close()
    st["proc"].wait()
    print(f"saved {out}: {st['frames']} frames, {st['frames'] / fps:.1f} s")


if __name__ == "__main__":
    main()
