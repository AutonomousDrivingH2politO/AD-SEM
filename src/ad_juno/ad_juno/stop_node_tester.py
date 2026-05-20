#!/usr/bin/env python3

import os
import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy,
    HistoryPolicy, LivelinessPolicy,
)

from shared_objects.stop_utils_new_v1 import analysis

ZED2_TOPIC = '/zed/zed_node/rgb/color/rect/image'

QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    liveliness=LivelinessPolicy.AUTOMATIC,
)


class StopSignTestNode(Node):
    def __init__(self):
        super().__init__('stop_sign_test_node')
        self.bridge = CvBridge()

        self.create_subscription(Image, ZED2_TOPIC, self.image_callback, QOS)

        self.get_logger().info(f"Listening: {ZED2_TOPIC}")
        self.get_logger().info("Started. Ctrl+C to stop.")

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detected, output_frame, details = analysis(img)

        confirmed = details['confirmed']

        if detected:
            for det in confirmed:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                self.get_logger().info(
                    f"✅ STOP SIGN! Box=({x1},{y1},{x2},{y2}) "
                    f"C={det['conf']:.2f}"
                )
        else:
            self.get_logger().info("❌ No stop sign.")

        cv2.imshow('Stop Sign Test', output_frame)
        cv2.waitKey(1)


"""
*** IMAGE SAVER MODE ***

import os
from datetime import datetime

DESKTOP       = os.path.expanduser("~/Desktop")
CONFIRMED_DIR = os.path.join(DESKTOP, "stop_sign_detections", "confirmed")
REJECTED_DIR  = os.path.join(DESKTOP, "stop_sign_detections", "rejected")

class StopSignTestNode(Node):
    def __init__(self):
        super().__init__('stop_sign_test_node')
        self.bridge   = CvBridge()
        self.counter  = 0
        os.makedirs(CONFIRMED_DIR, exist_ok=True)
        os.makedirs(REJECTED_DIR,  exist_ok=True)
        self.create_subscription(Image, ZED2_TOPIC, self.image_callback, QOS)
        self.get_logger().info(f"Listening     : {ZED2_TOPIC}")
        self.get_logger().info(f"Confirmed dir : {CONFIRMED_DIR}")
        self.get_logger().info(f"Rejected dir  : {REJECTED_DIR}")
        self.get_logger().info("Started. Ctrl+C to stop.")

    def _ts(self):
        return datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detected, output_frame, details = analysis(img)
        confirmed = details['confirmed']
        ts = self._ts()
        if detected:
            for det in confirmed:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                self.get_logger().info(
                    f"✅ STOP SIGN! Box=({x1},{y1},{x2},{y2}) C={det['conf']:.2f}"
                )
            cv2.imwrite(f"{CONFIRMED_DIR}/{ts}_{self.counter:06d}.jpg", output_frame)
            self.counter += 1
        else:
            self.get_logger().info("❌ No stop sign.")
            cv2.imwrite(f"{REJECTED_DIR}/{ts}_{self.counter:06d}.jpg", output_frame)
        cv2.imshow('Stop Sign Test', output_frame)
        cv2.waitKey(1)
"""


def main():
    rclpy.init(args=sys.argv)
    node = StopSignTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
