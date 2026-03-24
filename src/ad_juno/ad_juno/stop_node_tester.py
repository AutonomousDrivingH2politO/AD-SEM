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

'''
#İmage saver version
#!/usr/bin/env python3
import sys
import os
from datetime import datetime

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

        # Folder to save detection images
        self.save_dir = "/home/bafe/Desktop/detections"
        os.makedirs(self.save_dir, exist_ok=True)

        # Optional: save rejected detections too
        self.save_rejected = False

        # Counter to avoid same-name collisions
        self.image_counter = 0

        self.get_logger().info(f"Listening on: {ZED2_TOPIC}")
        self.get_logger().info(f"Detection images will be saved in: {self.save_dir}")
        self.get_logger().info("Stop sign test node started. Press Ctrl+C to stop.")

    def save_detection_image(self, frame, prefix="detected"):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{prefix}_{timestamp}_{self.image_counter:06d}.jpg"
        filepath = os.path.join(self.save_dir, filename)

        success = cv2.imwrite(filepath, frame)
        if success:
            self.get_logger().info(f"📸 Saved image: {filepath}")
            self.image_counter += 1
        else:
            self.get_logger().warn(f"Failed to save image: {filepath}")

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        detected, output_frame, details = analysis(img)

        confirmed = details.get('confirmed', [])
        rejected = details.get('rejected', [])

        saved_this_frame = False

        if detected:
            for i, det in enumerate(confirmed):
                x1, y1, x2, y2 = [int(c) for c in det['box']]

                detector_conf = det.get('detector_conf', -1.0)
                verifier_conf = det.get('verifier_conf', None)
                classifier_score = det.get('classifier_score', None)

                if verifier_conf is not None:
                    self.get_logger().info(
                        f"✅ STOP SIGN DETECTED! "
                        f"Box=({x1},{y1},{x2},{y2}) "
                        f"D_conf={detector_conf:.2f} "
                        f"V_conf={verifier_conf:.2f} "
                        f"→ Successful at Stage1 and Stage2"
                    )
                elif classifier_score is not None:
                    self.get_logger().info(
                        f"✅ STOP SIGN DETECTED! "
                        f"Box=({x1},{y1},{x2},{y2}) "
                        f"D_conf={detector_conf:.2f} "
                        f"C_score={classifier_score:.2f} "
                        f"→ Successful at Stage1 and Stage2"
                    )
                else:
                    self.get_logger().info(
                        f"✅ STOP SIGN DETECTED! "
                        f"Box=({x1},{y1},{x2},{y2}) "
                        f"D_conf={detector_conf:.2f}"
                    )

            # Save one image per frame when there is at least one confirmed detection
            self.save_detection_image(output_frame, prefix="confirmed")
            saved_this_frame = True

        elif rejected:
            for det in rejected:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                detector_conf = det.get('detector_conf', -1.0)

                self.get_logger().info(
                    f"⚠️ Stage1 passed but Stage2 failed. "
                    f"Box=({x1},{y1},{x2},{y2}) "
                    f"D_conf={detector_conf:.2f}"
                )

            if self.save_rejected and not saved_this_frame:
                self.save_detection_image(output_frame, prefix="rejected")

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
