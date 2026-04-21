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
from shared_objects.Motor import Stepper  # Import Stepper for brake functionality

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

        # Parameter for brake wait duration
        self.declare_parameter("brake_wait_duration", 10.0)
        self.brake_wait_duration = unwrap_nullable(
            self.get_parameter("brake_wait_duration").value,
            "brake_wait_duration can't be empty!",
        )

        # If True, stop detection permanently after the first stop sign
        self.declare_parameter("one_shot", True)
        self.one_shot = self.get_parameter("one_shot").value

        # NEW: Parameter to enable interactive tuning on startup
        self.declare_parameter("tune_on_startup", False) # Set to True for setup
        self.tune_on_startup = self.get_parameter("tune_on_startup").value

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
        self.get_logger().info("Initializing Stepper Motor...")
        self.brake = Stepper(
            interface="can",
            data_rate=1000000,
            module_id=1,  # modulo dello sterzo
            max_velocity=80_000,
            max_acc=50_000,
            MaxDeceleration=50_000,
            V1=100_000,
            A1=50_000,
            D1=50_000,
        )

        # ---------------------------------------------------------
        # CALIBRATION ROUTINE INTERCEPT
        # ---------------------------------------------------------
        if self.tune_on_startup:
            self._run_calibration_routine()
        # ---------------------------------------------------------

        # Publishers and subscribers
        self.stop_pub = self.create_publisher(Bool, self.topic_names["stop"], 1)
        self.debug_img_pub = self.create_publisher(Image, "/stop_debug_image", 1)

        self.original_sub = self.create_subscription(
            Image, self.topic_names["RGB_image"], self.original_img_callback, 1
        )

        # Logging
        self.get_logger().info("StopSignDetector node has fully started and is running.")

    def _run_calibration_routine(self):
        """Interactive loop to tune StallGuard and Current before ROS spin."""
        self.get_logger().info("--- ENTERING STARTUP CALIBRATION MODE ---")
        step_angle = 1.8
        microsteps = 256

        while True:
            try:
                move = input("\n[TUNE] Commands: [+] Move Pos, [-] Move Neg, [p] Zero Pos, [s] Tune StallGuard, [c] Tune Max Current, [q] Quit & Start Node\n> ").strip().lower()

                if move == "+":
                    steps = input("Enter steps to move (default 10000): ")
                    steps = int(steps) if steps else 10000
                    
                    current_pos = self.brake.get_actual_position()
                    self.brake.move_to_position(current_pos + steps)

                    while not self.brake.motor.get_position_reached():
                        time.sleep(0.1)

                    pos = self.brake.get_actual_position()
                    print(f"Position: {pos} | Angle: {(pos / microsteps) * step_angle:.2f}°")

                elif move == "-":
                    steps = input("Enter steps to move (default 10000): ")
                    steps = int(steps) if steps else 10000
                    
                    current_pos = self.brake.get_actual_position()
                    self.brake.move_to_position(current_pos - steps)

                    while not self.brake.motor.get_position_reached():
                        time.sleep(0.1)

                    pos = self.brake.get_actual_position()
                    print(f"Position: {pos} | Angle: {(pos / microsteps) * step_angle:.2f}°")

                elif move == "p":
                    self.brake.motor.set_axis_parameter(self.brake.motor.AP.ActualPosition, 0)
                    self.brake.motor.set_axis_parameter(self.brake.motor.AP.EncoderPosition, 0)
                    self.brake.virtual_position = 0
                    print("Positions reset to 0.")

                elif move == "s":
                    current_sg = self.brake.motor.get_axis_parameter(self.brake.motor.AP.SG2Threshold)
                    print(f"Current StallGuard Threshold: {current_sg}")
                    sg_val = input("Enter new StallGuard Threshold (e.g., 11 to 63): ")
                    if sg_val.isdigit():
                        self.brake.motor.set_axis_parameter(self.brake.motor.AP.SG2Threshold, int(sg_val))
                        print(f"StallGuard threshold updated to {sg_val}")

                elif move == "c":
                    current_c = self.brake.motor.get_axis_parameter(self.brake.motor.AP.MaxCurrent)
                    print(f"Current Max Current: {current_c}")
                    current_val = input("Enter new Max Current (e.g., 160, 200, 255): ")
                    if current_val.isdigit():
                        self.brake.motor.set_axis_parameter(self.brake.motor.AP.MaxCurrent, int(current_val))
                        print(f"Max Current updated to {current_val}")

                elif move == "q":
                    # Fetch final values so the user can save them for later hardcoding
                    final_sg = self.brake.motor.get_axis_parameter(self.brake.motor.AP.SG2Threshold)
                    final_curr = self.brake.motor.get_axis_parameter(self.brake.motor.AP.MaxCurrent)
                    print("\n--- Exiting Calibration Mode ---")
                    print(f"Final Tuned Parameters -> StallGuard: {final_sg}, Max Current: {final_curr}")
                    print("Starting autonomous ROS2 node operation...\n")
                    break
                    
                else:
                    print("Unrecognized command.")

            except KeyboardInterrupt:
                print("\nKeyboard Interrupt detected. Stopping motor and exiting calibration.")
                self.brake.motor.stop()
                break
            except Exception as e:
                print(f"\nAn error occurred: {e}")

    # ... [Keep original_img_callback, end_waiting, and _brake_sequence exactly as they were] ...
    def original_img_callback(self, msg):
        self.get_logger().info("Image received for stop sign detection")
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
                    self.enable = False
                    self.count_stop = 0

                    stop_msg = Bool(data=True)
                    self.stop_pub.publish(stop_msg)

                    threading.Thread(target=self._brake_sequence, daemon=True).start()
            else:
                self.count_stop = 0

    def end_waiting(self):
        stop_msg = Bool(data=False)
        self.stop_pub.publish(stop_msg)

        if self.one_shot:
            self.destroy_subscription(self.original_sub)
            self.get_logger().info(f"{self.brake_wait_duration:.0f}-second wait complete. One-shot mode: disabled.")
        else:
            self.is_waiting = False
            self.enable = True
            self.count_stop = 0
            self.get_logger().info(f"{self.brake_wait_duration:.0f}-second wait complete. Resuming.")

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
