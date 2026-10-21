#!/usr/bin/env python3
"""Subscribe tracker debug images and write mp4 until SIGINT / idle timeout."""
import argparse
import signal
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, topic: str, out_path: str, fps: float):
        super().__init__("tracker_mp4_recorder")
        self.bridge = CvBridge()
        self.out_path = out_path
        self.fps = fps
        self.writer = None
        self.frames = 0
        self.last_t = time.time()
        self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data)

    def cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.writer is None:
            h, w = img.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(self.out_path, fourcc, self.fps, (w, h))
            if not self.writer.isOpened():
                self.get_logger().error(f"cannot open writer {self.out_path}")
                rclpy.shutdown()
                return
            self.get_logger().info(f"recording {w}x{h} -> {self.out_path}")
        self.writer.write(img)
        self.frames += 1
        self.last_t = time.time()

    def close(self):
        if self.writer is not None:
            self.writer.release()
        self.get_logger().info(f"wrote {self.frames} frames to {self.out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--idle", type=float, default=8.0, help="exit after N seconds without frames")
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.topic, args.out, args.fps)
    stop = False

    def _sig(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    while rclpy.ok() and not stop:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.frames > 0 and (time.time() - node.last_t) > args.idle:
            break

    node.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if node.frames > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
