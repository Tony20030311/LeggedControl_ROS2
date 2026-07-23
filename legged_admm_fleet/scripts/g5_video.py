#!/usr/bin/env python3
# Headless video recorder: subscribes a bridged Gazebo camera Image topic and encodes frames to
# an mp4 with cv2.VideoWriter (no ffmpeg, no GUI). Used only for the one "recorded" arena run.
#
# Usage: g5_video.py <image_topic> <out.mp4> [fps]
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = sys.argv[1]
OUT = sys.argv[2]
FPS = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0


def to_bgr(m):
    buf = np.frombuffer(m.data, dtype=np.uint8)
    if m.encoding in ("rgb8", "bgr8"):
        img = buf.reshape(m.height, m.width, 3)
        return img[:, :, ::-1] if m.encoding == "rgb8" else img
    if m.encoding in ("rgba8", "bgra8"):
        img = buf.reshape(m.height, m.width, 4)[:, :, :3]
        return img[:, :, ::-1] if m.encoding == "rgba8" else img
    if m.encoding == "mono8":
        return cv2.cvtColor(buf.reshape(m.height, m.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported encoding {m.encoding}")


class Recorder(Node):
    def __init__(self):
        super().__init__("g5_video")
        self.w = None
        self.n = 0
        self.create_subscription(Image, TOPIC, self.on_img, 10)
        self.get_logger().info(f"recording {TOPIC} -> {OUT} @ {FPS} fps")

    def on_img(self, m):
        frame = to_bgr(m)
        if self.w is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.w = cv2.VideoWriter(OUT, fourcc, FPS, (m.width, m.height))
        self.w.write(np.ascontiguousarray(frame))
        self.n += 1
        if self.n % 60 == 0:
            self.get_logger().info(f"{self.n} frames")

    def close(self):
        if self.w is not None:
            self.w.release()
        self.get_logger().info(f"wrote {self.n} frames -> {OUT}")


def main():
    rclpy.init(args=sys.argv)
    rec = Recorder()
    try:
        rclpy.spin(rec)
    except KeyboardInterrupt:
        pass
    finally:
        rec.close()


if __name__ == "__main__":
    main()
