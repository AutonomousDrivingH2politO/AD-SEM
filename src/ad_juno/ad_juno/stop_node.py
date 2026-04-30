#!/usr/bin/env python3
import sys
import time
import rclpy
import cv2
from typing import TypeVar
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from shared_objects.ROS_utils import Topics
from shared_objects.stop_utils_new_v1 import analysis
# Import Stepper for brake functionality
from shared_objects.Motor import Stepper

import threading

T = TypeVar("T")


def unwrap_nullable(value: T | None, error_msg: str) -> T:
    """Unwraps a nullable value, raises an Exception if value is None"""
    if value is None:
        raise ValueError(error_msg)
    return value


class StopSignDetector(Node):
    def __init__(self):
        super().__init__("stop_node", parameter_overrides=[])
        self.logger = self.get_logger()
        self.debug = self.declare_parameter("debug", False).get_parameter_value().bool_value
        self.debug = self.get_parameter("debug").value


        self.declare_parameter("brake_wait_duration", 10.0)
        self.brake_wait_duration = unwrap_nullable(
            self.get_parameter("brake_wait_duration").value,
            "brake_wait_duration can't be empty!",
        )

        # If True, stop detection permanently after the first stop sign
        self.declare_parameter("one_shot", True)
        self.one_shot = self.get_parameter("one_shot").value

        self.enable = True
        self.count_stop = 0
        self.count_img = 0
        self.threshold_stop = 1
        self.threshold_img = 15
        self.bridge = CvBridge()
        self.is_waiting = False

        topics = Topics()
        self.topic_names = topics.topic_names

        # Initialize Stepper for brake
        self.brake = Stepper(
            interface="can",
            data_rate=1000000,
            module_id=1,  # modulo dello sterzo
            max_velocity=80_000,
            max_acc=50_000,
            MaxDeceleration=50_000,  # parametro di esempio: regola se necessario
            V1=60_000,
            A1=50_000,  # parametro di esempio: regola se necessario
            D1=50_000,
        )

        # Publishers and subscribers
        self.stop_pub = self.create_publisher(
            Bool, self.topic_names["stop"], 1)
        self.debug_img_pub = self.create_publisher(
            Image, "/stop_debug_image", 1)

        self.original_sub = self.create_subscription(
            Image, self.topic_names["RGB_image"], self.original_img_callback, 1
        )

        # Logging
        self.get_logger().info("StopSignDetector node has been started")

    def original_img_callback(self, msg):

        self.get_logger().info("Image received for stop sign detection")
        if self.enable and not self.is_waiting:
            self.count_img += 1
            if self.count_img % self.threshold_img != 0:
                return

            # Read incoming ROS image in BGR format
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # New analysis() returns: (result, output_frame, details)
            result, output_frame, details = analysis(img)

            try:
                # Convert output_frame to ROS Image message and publish for debugging
                debug_msg = self.bridge.cv2_to_imgmsg(
                    output_frame, encoding="bgr8")
                self.debug_img_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Image could not be published: {e}")

            if result:
                if self.count_stop < self.threshold_stop:
                    self.count_stop += 1
                else:
                    # Trigger brake logic
                    self.get_logger().info("Stop sign detected. Activating brake.")

                    # Disable further stop detection temporarily
                    self.is_waiting = True
                    self.enable = False
                    self.count_stop = 0

                    # Publish stop=True to keep throttle closed during braking
                    stop_msg = Bool(data=True)
                    self.stop_pub.publish(stop_msg)

                    # Added a thread since i need to use sleep for the brake of the stepper
                    threading.Thread(
                        target=self._brake_sequence, daemon=True).start()
            else:
                self.count_stop = 0
        if self.debug:
            self.logger.info(f"Stop detection result: {result}, Details: {details}")
            self.logger.info(f"Stop detection count: {self.count_stop}/{self.threshold_stop}")
    
    def end_waiting(self):
        # Resume operation: publish stop=False to allow throttle
        stop_msg = Bool(data=False)
        self.stop_pub.publish(stop_msg)

        if self.one_shot:
            # Permanently disable: unsubscribe so no more image callbacks fire
            self.destroy_subscription(self.original_sub)
            self.get_logger().info(
                f"{self.brake_wait_duration:.0f}-second wait complete. "
                "One-shot mode: image subscription removed, detection permanently disabled."
            )
        else:
            # Reset internal state for next detection cycle
            self.is_waiting = False
            self.enable = True
            self.count_stop = 0
            self.get_logger().info(
                f"{self.brake_wait_duration:.0f}-second wait complete. Resuming stop detection."
            )

    def _brake_sequence(self):
        """Runs entirely off the ROS executor thread so the sleep is fine"""
        try:
            self.brake.brake()  # sleeps happen here, not on ROS thread
            self.get_logger().info("Brake complete.")
        except Exception as e:
            self.get_logger().error(f"Brake error: {e}")
        finally:
            # Signal resume on ROS thread via timer
            time.sleep(self.brake_wait_duration)
            self.end_waiting()


def main():
    rclpy.init(args=sys.argv)
    node = StopSignDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception as e:
        node.get_logger().error("Error: %r" % (e))
    finally:
        node.brake.disconnect_motor()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
