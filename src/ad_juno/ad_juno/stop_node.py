#!/usr/bin/env python3
import csv
import inspect
import json
import os
import sys
import time
from datetime import datetime
from typing import TypeVar
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import shared_objects
from shared_objects.ROS_utils import Topics
from shared_objects.stop_utils_new_v1 import analysis
from shared_objects.Motor import Stepper
from std_srvs.srv import Trigger
# The brake without active slip detection
#from shared_objects.TestBrakeMotor import TestBrakeMotor as Stepper 
import threading
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

CSV_FIELDS = [
    "timestamp",
    "event",
    "count_stop",
    "count_img",
    "conf",
    "box",
    "duration_s",
    "status",
    "error_msg",
    "brake_motor_params",
    "brake_call_args",
    "actual_pos",
    "target_pos",
    "encoder_pos",
    "slip",
    "corrections",
]

T = TypeVar("T")
SHARED_OBJECTS_DIR = os.path.dirname(shared_objects.__file__)
BRAKE_PARAMS_PATH  = os.path.join(SHARED_OBJECTS_DIR, "params.json")

def unwrap_nullable(value: T | None, error_msg: str) -> T:
    if value is None:
        raise ValueError(error_msg)
    return value
def _json_default(o):
    if hasattr(o, "item"):      # numpy scalars (int64, float64, bool_)
        return o.item()
    if hasattr(o, "tolist"):    # numpy arrays
        return o.tolist()
    return str(o)
class StopSignDetector(Node):
    def __init__(self):
        super().__init__("stop_node", parameter_overrides=[])
        self.logger = self.get_logger()

        self.debug = (
            self.declare_parameter("debug", True).get_parameter_value().bool_value
        )
        self.debug = self.get_parameter("debug").value

        self.declare_parameter("brake_wait_duration", 10.0)
        self.brake_wait_duration = unwrap_nullable(
            self.get_parameter("brake_wait_duration").value,
            "brake_wait_duration can't be empty!",
        )

        self.declare_parameter("one_shot", True)
        self.one_shot = self.get_parameter("one_shot").value
        self.brake_srv = self.create_service(
            Trigger, "trigger_brake", self.handle_trigger_brake
        )
        self.enable    = True
        self.count_stop = 0
        self.count_img  = 0

        self.threshold_stop = 1
    
        self.threshold_img  = 6

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

        self.declare_parameter("debug_csv_path", "/home/bylogix/AD-SEM/DEBUG")
        self._init_debug_csv(self.get_parameter("debug_csv_path").value)

        self.get_logger().info("StopSignDetector node has been started")

    def handle_trigger_brake(self, request, response):
        if self.is_waiting:
            response.success = False
            response.message = "Brake already in progress; ignoring."
            return response

        self.get_logger().info("Manual brake trigger received (service).")
        self.is_waiting = True
        self.enable     = False
        self.count_stop = 0

        self.stop_pub.publish(Bool(data=True))
        self._log_csv(
            "brake_trigger_manual",
            brake_motor_params=self._brake_motor_params_snapshot,
            brake_call_args=self._brake_call_args_snapshot,
        )

        threading.Thread(target=self._brake_sequence, daemon=True).start()

        response.success = True
        response.message = "Brake sequence started."
        return response

    def _init_debug_csv(self, configured_path: str) -> None:
        self._csv_file = None
        self._csv_writer = None
        self._brake_motor_params_snapshot = ""
        self._brake_call_args_snapshot = ""

        try:
            self.debug_csv_path = os.path.expanduser(configured_path) if configured_path else ""

            # If empty or a directory, build a timestamped filename inside it
            if not self.debug_csv_path or os.path.isdir(self.debug_csv_path):
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                base_dir = self.debug_csv_path or os.path.expanduser("~")
                os.makedirs(base_dir, exist_ok=True)
                self.debug_csv_path = os.path.join(base_dir, f"stop_node_debug_{ts}.csv")
            else:
                # ensure the parent dir exists for an explicit file path
                os.makedirs(os.path.dirname(self.debug_csv_path) or ".", exist_ok=True)            

            new_file = not os.path.exists(self.debug_csv_path)
            self._csv_file = open(self.debug_csv_path, "a", newline="", buffering=1)
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
            if new_file:
                self._csv_writer.writeheader()

            try:
                with open(BRAKE_PARAMS_PATH) as fh:
                    self._brake_motor_params_snapshot = json.dumps(
                        json.load(fh), separators=(",", ":")
                    )
            except Exception as exc:
                self.logger.warn(f"Could not snapshot brake params for CSV: {exc}")

            sig = inspect.signature(self.brake.brake)
            self._brake_call_args_snapshot = json.dumps(
                {
                    name: p.default
                    for name, p in sig.parameters.items()
                    if p.default is not inspect.Parameter.empty
                },
                separators=(",", ":"),
            )

            self.logger.info(f"Debug CSV: {self.debug_csv_path}")
            self._log_csv(
                "init",
                brake_motor_params=self._brake_motor_params_snapshot,
                brake_call_args=self._brake_call_args_snapshot,
            )
        except Exception as exc:
            self.logger.error(f"Failed to set up debug CSV: {exc}")
            self._csv_writer = None

    def _log_csv(self, event: str, **fields) -> None:
        if self._csv_writer is None:
            return
        try:
            row = {k: "" for k in CSV_FIELDS}
            row["timestamp"] = datetime.now().isoformat(timespec="milliseconds")
            row["event"] = event
            row["count_stop"] = self.count_stop
            row["count_img"] = self.count_img
            for k, v in fields.items():
                if v is None:
                    continue
                if isinstance(v, float):
                    row[k] = f"{v:.4f}"
                elif isinstance(v, (list, tuple, dict)):
                    row[k] = json.dumps(v, separators=(",", ":"), default=_json_default)
                else:
                    row[k] = v
            self._csv_writer.writerow(row)
        except Exception as exc:
            self.logger.warn(f"CSV log failed (event={event}): {exc}")

    def close_debug_csv(self) -> None:
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except Exception:
                pass
            self._csv_file = None
            self._csv_writer = None

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

            for det in details.get("confirmed", []) if isinstance(details, dict) else []:
                box = det.get("box")
                self._log_csv(
                    "detection",
                    conf=float(det.get("conf")) if det.get("conf") is not None else None,
                    box=[int(x) for x in box] if box is not None else None,
                )

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

                    self._log_csv(
                        "brake_trigger",
                        brake_motor_params=self._brake_motor_params_snapshot,
                        brake_call_args=self._brake_call_args_snapshot,
                    )

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
        self._log_csv(
            "brake_start",
            brake_call_args=self._brake_call_args_snapshot,
        )
        t_start = time.time()
        status = "success"
        err_msg = ""
        try:
            self.brake.brake(log=self._log_csv)
            self.get_logger().info("Brake complete.")
        except Exception as e:
            status = "error"
            err_msg = repr(e)
            self.get_logger().error(f"Brake error: {e}")
        finally:
            self._log_csv(
                "brake_end",
                duration_s=time.time() - t_start,
                status=status,
                error_msg=err_msg,
            )
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
        node.close_debug_csv()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
