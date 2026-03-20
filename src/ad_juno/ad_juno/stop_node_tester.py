'''#!/usr/bin/env python3 
#Use segmentation model for verifier stage.
import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time 
from shared_objects.stop_utils_new_v1 import analysis


class StopSignTestNode(Node):
    def __init__(self):
        super().__init__('stop_sign_test_node')
        self.bridge = CvBridge()

        ZED2_TOPIC = '/zed/zed_node/rgb/color/rect/image'

        self.create_subscription(Image, ZED2_TOPIC, self.image_callback, 1)

        self.get_logger().info(f"Listening on: {ZED2_TOPIC}")
        self.get_logger().info("Stop sign test node started. Press Ctrl+C to stop.")

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        detected, output_frame, details = analysis(img)

        confirmed = details['confirmed']
        rejected  = details['rejected']

        if detected:
            for det in confirmed:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                self.get_logger().info(
                    f"✅ STOP SIGN DETECTED! "
                    f"Box=({x1},{y1},{x2},{y2}) "
                    f"D_conf={det['detector_conf']:.2f} "
                    f"V_conf={det['verifier_conf']:.2f} "
                    f"→ Successful at Stage1 and Stage2"
                )
        elif rejected:
            for det in rejected:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                self.get_logger().info(
                    f"⚠️  Stage1 passed but Stage2 failed. "
                    f"Box=({x1},{y1},{x2},{y2}) "
                    f"D_conf={det['detector_conf']:.2f}"
                )
        else:
            self.get_logger().info("❌ No stop sign.")

        cv2.imshow('Stop Sign Test', output_frame)
        cv2.waitKey(1)


def main():
    rclpy.init(args=sys.argv)
    node = StopSignTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

#!/usr/bin/env python3 
#Use classification model for verifier stage.

#!/usr/bin/env python3
import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from shared_objects.stop_utils_new_v1 import analysis


class StopSignTestNode(Node):
    def __init__(self):
        super().__init__('stop_sign_test_node')
        self.bridge = CvBridge()

        ZED2_TOPIC = '/zed/zed_node/rgb/color/rect/image'

        self.create_subscription(Image, ZED2_TOPIC, self.image_callback, 1)

        self.get_logger().info(f"Listening on: {ZED2_TOPIC}")
        self.get_logger().info("Stop sign test node started. Press Ctrl+C to stop.")

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        detected, output_frame, details = analysis(img)

        confirmed = details['confirmed']
        rejected  = details['rejected']

        if detected:
            for det in confirmed:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                self.get_logger().info(
                    f"✅ STOP SIGN DETECTED! "
                    f"Box=({x1},{y1},{x2},{y2}) "
                    f"D_conf={det['detector_conf']:.2f} "
                    f"C_score={det['classifier_score']:.2f} "
                    f"→ Successful at Stage1 and Stage2"
                )
        elif rejected:
            for det in rejected:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                self.get_logger().info(
                    f"⚠️  Stage1 passed but Stage2 failed. "
                    f"Box=({x1},{y1},{x2},{y2}) "
                    f"D_conf={det['detector_conf']:.2f}"
                )
        else:
            self.get_logger().info("❌ No stop sign.")

        cv2.imshow('Stop Sign Test', output_frame)
        cv2.waitKey(1)


def main():
    rclpy.init(args=sys.argv)
    node = StopSignTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


