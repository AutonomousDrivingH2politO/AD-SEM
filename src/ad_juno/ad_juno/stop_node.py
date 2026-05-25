#!/usr/bin/env python3
import os
import sys
import time
import cv2
from typing import TypeVar
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import shared_objects
from shared_objects.ROS_utils import Topics
from shared_objects.stop_utils_new_v1 import analysis
from shared_objects.Motor import Stepper
import threading
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

T = TypeVar("T")
SHARED_OBJECTS_DIR = os.path.dirname(shared_objects.__file__)
BRAKE_PARAMS_PATH  = os.path.join(SHARED_OBJECTS_DIR, "params.json")

def unwrap_nullable(value: T | None, error_msg: str) -> T:
    if value is None:
        raise ValueError(error_msg)
    return value

class StopSignDetector(Node):
    def __init__(self):
        super().__init__("stop_node", parameter_overrides=[])
        self.logger = self.get_logger()

        self.debug = (
            self.declare_parameter("debug", False).get_parameter_value().bool_value
        )
        self.debug = self.get_parameter("debug").value

        self.declare_parameter("brake_wait_duration", 10.0)
        self.brake_wait_duration = unwrap_nullable(
            self.get_parameter("brake_wait_duration").value,
            "brake_wait_duration can't be empty!",
        )

        self.declare_parameter("one_shot", True)
        self.one_shot = self.get_parameter("one_shot").value

        self.enable    = True
        self.count_stop = 0
        self.count_img  = 0

        self.threshold_stop = 1
    
        self.threshold_img  = 3 

        self.bridge     = CvBridge()
        self.is_waiting = False

        topics          = Topics()
        self.topic_names = topics.topic_names

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.logger.info(f"Loaded brake params from: {BRAKE_PARAMS_PATH}")

        self.brake = Stepper.from_json(
            json_path=BRAKE_PARAMS_PATH,
            interface="can",
            module_id=1,
            data_rate=1_000_000,
        )

        self.stop_pub = self.create_publisher(
            Bool, self.topic_names["stop"], qos_profile
        )
        self.debug_img_pub = self.create_publisher(Image, "/stop_debug_image", 1)
        self.original_sub  = self.create_subscription(
            Image, self.topic_names["RGB_image"], self.original_img_callback, 1
        )

        self.get_logger().info("StopSignDetector node has been started")

    def original_img_callback(self, msg):
        if self.enable and not self.is_waiting:
            self.count_img += 1
            if self.count_img % self.threshold_img != 0:
                return

            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            result, output_frame, details = analysis(img)

            try:
                debug_msg = self.bridge.cv2_to_imgmsg(output_frame, encoding="bgr8")
                self.debug_img_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Image could not be published: {e}")

            if result:
                if self.count_stop < self.threshold_stop:
                    self.count_stop += 1
                else:
                    self.get_logger().info("Stop sign detected. Activating brake.")
                    self.is_waiting = True
                    self.enable     = False
                    self.count_stop = 0

                    stop_msg = Bool(data=True)
                    self.stop_pub.publish(stop_msg)

                    threading.Thread(
                        target=self._brake_sequence, daemon=True
                    ).start()
            else:
                self.count_stop = 0

        if self.debug:
            confirmed = details.get('confirmed', [])
            for det in confirmed:
                self.logger.info(
                    f"Stop sign confirmed: Box={det['box']}  conf={det['conf']:.2f}"
                )
            self.logger.info(
                f"Stop detection count: {self.count_stop}/{self.threshold_stop}"
            )

    def end_waiting(self):
        stop_msg = Bool(data=False)
        self.stop_pub.publish(stop_msg)

        if self.one_shot:
            self.destroy_subscription(self.original_sub)
            self.get_logger().info(
                f"{self.brake_wait_duration:.0f}-second wait complete. One-shot mode active."
            )
        else:
            self.is_waiting  = False
            self.enable      = True
            self.count_stop  = 0
            self.get_logger().info("Resuming stop detection.")

    def _brake_sequence(self):
        try:
            self.brake.brake()
            self.get_logger().info("Brake complete.")
        except Exception as e:
            self.get_logger().error(f"Brake error: {e}")
        finally:
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