#!/usr/bin/env python3
import csv
import json
import os
import sys
import threading
from datetime import datetime
import time
from pathlib import Path
from typing import TypeVar

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import shared_objects
from shared_objects.Motor import Stepper
from shared_objects.ROS_utils import Topics
from shared_objects.stop_utils_new_v1 import analysis

T = TypeVar("T")

SHARED_OBJECTS_DIR = os.path.dirname(shared_objects.__file__)
BRAKE_PARAMS_PATH = os.path.join(SHARED_OBJECTS_DIR, "params.json")

# ---------------------------------------------------------------------------
# CSV layout — two row kinds share the same file:
#
#   INIT rows  — written once at startup, document the stepper configuration.
#                row_type="init", all detection/motor columns are empty.
#
#   FRAME rows — written for every processed frame.
#                row_type="frame", init columns are empty.
# ---------------------------------------------------------------------------
_CSV_HEADERS = [
    "row_type",  # "init" | "frame"
    # ── init columns ────────────────────────────────────────────────────
    "param_key",  # e.g. "MaxVelocity", "params_file", …
    "param_value",  # value as a string
    # ── frame columns ───────────────────────────────────────────────────
    "frame_counter",  # absolute count of processed frames
    "result",  # True/False — did analysis() fire?
    "confirmed_count",  # number of confirmed detections this frame
    "best_conf",  # highest confidence score (empty if none)
    "count_stop",  # consecutive detection counter
    "actual_pos",  # motor actual position
    "encoder_pos",  # motor encoder position
    "slip",  # abs(actual - encoder)
    "is_waiting",  # True while brake hold is active
    # ── shared ──────────────────────────────────────────────────────────
    "timestamp",  # ROS time  sec.nanosec
]


def unwrap_nullable(value: T | None, error_msg: str) -> T:
    if value is None:
        raise ValueError(error_msg)
    return value


def _ros_ts(stamp) -> str:
    """Format a builtin_interfaces/Time stamp as 'sec.nanosec'."""
    return f"{stamp.sec}.{stamp.nanosec:09d}"


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

        self.enable = True
        self.count_stop = 0
        self.count_img = 0

        self.threshold_stop = 1
        self.threshold_img = 15

        self.bridge = CvBridge()
        self.is_waiting = False

        topics = Topics()
        self.topic_names = topics.topic_names

        # ── Debug CSV setup ────────────────────────────────────────────────
        home_dir = Path.home()
        debug_path = str(home_dir / "AD-SEM" / "DEBUG")

        self.declare_parameter("debug_root", debug_path)
        self.debug_root = Path(
            self.get_parameter("debug_root").get_parameter_value().string_value
        )

        self._csv_file = None
        self._csv_writer = None

        # ── Stepper init ───────────────────────────────────────────────────
        self.logger.info(f"Loading brake params from: {BRAKE_PARAMS_PATH}")

        self.brake = Stepper.from_json(
            json_path=BRAKE_PARAMS_PATH,
            interface="can",
            module_id=1,
            data_rate=1_000_000,
        )

        # ── Open CSV and write init block ──────────────────────────────────
        # Opened after Stepper so a failed CAN connection leaves no empty
        # debug folder on disk.
        if self.debug:
            self._init_csv_logger()
            self._log_stepper_init()

        # ── ROS interfaces ─────────────────────────────────────────────────
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.stop_pub = self.create_publisher(
            Bool, self.topic_names["stop"], qos_profile
        )
        self.debug_img_pub = self.create_publisher(Image, "/stop_debug_image", 1)
        self.original_sub = self.create_subscription(
            Image, self.topic_names["RGB_image"], self.original_img_callback, 1
        )

        self.logger.info(
            f"StopSignDetector started — debug={self.debug}, "
            f"one_shot={self.one_shot}, "
            f"brake_wait={self.brake_wait_duration}s"
        )

    # ------------------------------------------------------------------
    # CSV logger
    # ------------------------------------------------------------------

    def _init_csv_logger(self):
        try:
            self.debug_root.mkdir(parents=True, exist_ok=True)
            ts_folder = self.debug_root / (
                datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + "_stop"
            )
            ts_folder.mkdir()
            csv_path = ts_folder / "stop_debug.csv"

            self._csv_file = open(csv_path, "w", newline="", buffering=1)
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(_CSV_HEADERS)

            self.logger.info(f"DEBUG CSV → {csv_path}")
        except Exception as e:
            self.logger.error(f"Failed to initialise CSV logger: {e}")
            raise

    def _close_csv_logger(self):
        if self._csv_file and not self._csv_file.closed:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except OSError as e:
                self.logger.error(f"Failed to close debug CSV: {e}")

    def _write_row(self, row: list):
        try:
            self._csv_writer.writerow(row)
        except OSError as e:
            self.logger.error(f"Failed to write debug row: {e}")

    # ------------------------------------------------------------------
    # Stepper init logging
    # ------------------------------------------------------------------

    def _log_stepper_init(self):
        """
        Write one INIT row per parameter from the JSON file, plus metadata rows
        for the file path, module_id, interface, and data_rate. The CSV is fully
        self-documenting: reading the init rows tells you exactly which motor
        config was active during that session.
        """
        ts = _ros_ts(self.get_clock().now().to_msg())

        for key, value in [
            ("params_file", BRAKE_PARAMS_PATH),
            ("interface", "can"),
            ("module_id", 1),
            ("data_rate", 1_000_000),
        ]:
            self._write_row(
                ["init", key, value, "", "", "", "", "", "", "", "", "", ts]
            )

        try:
            with open(BRAKE_PARAMS_PATH) as fh:
                params: dict = json.load(fh)

            for key, value in params.items():
                self._write_row(
                    ["init", key, value, "", "", "", "", "", "", "", "", "", ts]
                )
        except Exception as e:
            self.logger.error(f"Could not read params for CSV init block: {e}")
            self._write_row(
                [
                    "init",
                    "params_read_error",
                    str(e),
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    ts,
                ]
            )

    # ------------------------------------------------------------------
    # Per-frame debug row
    # ------------------------------------------------------------------

    def _save_debug(
        self,
        result: bool,
        details: dict,
        actual_pos: "int | str",
        encoder_pos: "int | str",
        slip: "int | str",
        stamp,
    ):
        confirmed = details.get("confirmed", [])
        best_conf = f"{max(d['conf'] for d in confirmed):.3f}" if confirmed else ""
        self._write_row(
            [
                "frame",
                "",
                "",  # init columns — empty
                self.count_img,
                result,
                len(confirmed),
                best_conf,
                self.count_stop,
                actual_pos,
                encoder_pos,
                slip,
                self.is_waiting,
                _ros_ts(stamp),
            ]
        )

    # ------------------------------------------------------------------
    # Image callback
    # ------------------------------------------------------------------

    def original_img_callback(self, msg):
        # ── Frame skip ────────────────────────────────────────────────────
        self.count_img += 1
        if self.count_img % self.threshold_img != 0:
            return

        if not self.enable or self.is_waiting:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        result, output_frame, details = analysis(img)

        # ── Publish debug image ───────────────────────────────────────────
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(output_frame, encoding="bgr8")
            self.debug_img_pub.publish(debug_msg)
        except Exception as e:
            self.logger.error(f"Image could not be published: {e}")

        # ── Motor telemetry ───────────────────────────────────────────────
        actual_pos = encoder_pos = slip = ""
        if self.debug:
            try:
                actual_pos = self.brake.get_actual_position()
                encoder_pos = self.brake.get_encoder_position()
                slip = abs(actual_pos - encoder_pos)
            except Exception as e:
                self.logger.warning(f"[motor] telemetry read failed: {e}")

        # ── CSV row ───────────────────────────────────────────────────────
        if self.debug:
            self._save_debug(
                result, details, actual_pos, encoder_pos, slip, msg.header.stamp
            )

            confirmed = details.get("confirmed", [])
            if confirmed:
                for det in confirmed:
                    self.logger.info(
                        f"[detection] box={det['box']}  conf={det['conf']:.3f}"
                    )
            self.logger.info(
                f"[counter] stop={self.count_stop}/{self.threshold_stop}  "
                f"result={result}  slip={slip}"
            )

        # ── Detection state machine ───────────────────────────────────────
        if result:
            if self.count_stop < self.threshold_stop:
                self.count_stop += 1
            else:
                self.logger.info("Stop sign detected. Activating brake.")
                self.is_waiting = True
                self.enable = False
                self.count_stop = 0

                self.stop_pub.publish(Bool(data=True))
                threading.Thread(target=self._brake_sequence, daemon=True).start()
        else:
            self.count_stop = 0

    # ------------------------------------------------------------------
    # Brake sequence
    # ------------------------------------------------------------------

    def _brake_sequence(self):
        if self.debug:
            try:
                self.logger.info(
                    f"[brake] sequence started — "
                    f"actual={self.brake.get_actual_position()}  "
                    f"encoder={self.brake.get_encoder_position()}"
                )
            except Exception:
                pass
        try:
            self.brake.brake()
            self.logger.info("Brake complete.")
        except Exception as e:
            self.logger.error(f"Brake error: {e}")
        finally:
            if self.debug:
                try:
                    self.logger.info(
                        f"[brake] sequence ended — "
                        f"actual={self.brake.get_actual_position()}  "
                        f"encoder={self.brake.get_encoder_position()}"
                    )
                except Exception:
                    pass
            time.sleep(self.brake_wait_duration)
            self.end_waiting()

    def end_waiting(self):
        self.stop_pub.publish(Bool(data=False))

        if self.one_shot:
            self.destroy_subscription(self.original_sub)
            self.logger.info(
                f"{self.brake_wait_duration:.0f}-second wait complete. "
                "One-shot mode: detection permanently disabled."
            )
        else:
            self.is_waiting = False
            self.enable = True
            self.count_stop = 0
            self.logger.info(
                f"{self.brake_wait_duration:.0f}-second wait complete. "
                "Resuming stop detection."
            )


# ------------------------------------------------------------------
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
        node._close_csv_logger()
        node.brake.disconnect_motor()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
