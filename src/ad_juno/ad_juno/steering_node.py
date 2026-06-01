#!/usr/bin/env python3
import csv
import threading
import queue
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float64, Float32, Bool
from shared_objects.Motor import Stepper
from shared_objects.ROS_utils import Topics
from pathlib import Path
from datetime import datetime

TRANSMISSION_RATIO = 14
MAX_ANGLE = 10
STEP_ANGLE = 1.8

_CSV_HEADERS = [
    "counter",
    "angle",
    "target_position",
    "timestamp",
]  # CSV column headers
_FLUSH_EVERY = 50  # rows buffered before a forced flush


class SteeringNode(Node):
    def __init__(self):
        super().__init__("steering_node")
        self.logger = self.get_logger()

        self.steering = Stepper(
            interface="can",
            data_rate=1000000,
            module_id=3,
            max_velocity=80_000,
            max_acc=50_000,
            MaxDeceleration=50_000,
            V1=100_000,
            A1=50_000,
            D1=50_000,
        )

        self.latest_angle = 0.0
        self.counter = 0
        self._speed = 0.0
        self._is_frozen = False

        home_dir = Path.home()
        debug_path = str(home_dir / "AD-SEM" / "DEBUG")

        self.declare_parameter("debug_root", debug_path)
        self.debug = (
            self.declare_parameter("debug", True).get_parameter_value().bool_value
        )
        self.debug_root = Path(
            self.get_parameter("debug_root").get_parameter_value().string_value
        )

        self._csv_file = None  # single file handle kept open for the session
        self._csv_writer = None  # csv.writer bound to _csv_file
        self._log_queue = queue.Queue(
            maxsize=1000
        )  # bounded so put_nowait can raise queue.Full
        self._writer_thread = None  # background thread that drains the queue

        if self.debug:
            self._init_csv_logger()

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        topics = Topics()
        topic_names = topics.topic_names

        self.subscription = self.create_subscription(
            Float64, topic_names["steering"], self.callback_stepangle, qos_profile
        )

        self._stopped = False
        self.stop_sub = self.create_subscription(
            Bool, topic_names["stop"], self.callback_stop, qos_profile
        )
        self.speed_sub = self.create_subscription(Float32 ,topic_names["speed"],self.callback_speed, qos_profile)

        self.get_logger().info(
            "Steering node initialized (direct move_to, firmware ramp)"
        )
        

    def callback_speed(self, msg):
        self._speed = msg.data

    def _init_csv_logger(self):
        try:
            self.debug_root.mkdir(parents=True, exist_ok=True)
            ts_folder = self.debug_root / (
                datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + "_steering"
            )
            ts_folder.mkdir()
            csv_path = (
                ts_folder / "steering_debug.csv"
            )  # single CSV file instead of one file per message

            self._csv_file = open(
                csv_path, "w", newline="", buffering=1
            )  # line-buffered file handle
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(_CSV_HEADERS)

            self._writer_thread = threading.Thread(
                target=self._writer_loop, daemon=True
            )  # daemon so it won't block process exit
            self._writer_thread.start()

            self.get_logger().info(f"DEBUG CSV → {csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialise CSV logger: {e}")
            raise

    def _writer_loop(self):
        rows_since_flush = 0
        while True:
            try:
                row = self._log_queue.get(timeout=1.0)
            except queue.Empty:
                if self._csv_file and not self._csv_file.closed:
                    self._csv_file.flush()  # periodic flush when queue is idle
                continue

            if row is None:  # None is the sentinel value that signals shutdown
                break

            try:
                self._csv_writer.writerow(row)
            except OSError as e:
                self.get_logger().error(
                    f"Failed to write debug row: {e}"
                )  # disk full / unmounted — log and continue rather than crashing
                continue

            rows_since_flush += 1
            if rows_since_flush >= _FLUSH_EVERY:
                try:
                    self._csv_file.flush()
                except OSError as e:
                    self.get_logger().error(f"Failed to flush debug CSV: {e}")
                rows_since_flush = 0

        if self._csv_file and not self._csv_file.closed:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except OSError as e:
                self.get_logger().error(f"Failed to close debug CSV: {e}")

    def _save_debug(self, angle: float, target_position: int):
        try:
            self._log_queue.put_nowait(
                [
                    self.counter,
                    f"{angle:.3f}",
                    target_position,
                    datetime.now().isoformat(),
                ]
            )
        except queue.Full:
            self.get_logger().warn(
                "Logging queue full! Dropping debug data.", throttle_duration_sec=1.0
            )  # drop rather than blocking the callback or crashing

    def _close_csv_logger(self):
        if self._writer_thread and self._writer_thread.is_alive():
            self._log_queue.put(None)  # send sentinel to stop the writer loop
            self._writer_thread.join(timeout=5.0)

    def callback_stepangle(self, msg):
        should_freeze = self._stopped and self._speed < 2.0
        
        if should_freeze and not self._is_frozen:
            self.get_logger().info("Speed < 2.0 while stopped => steering frozen")
            self._is_frozen = True
        elif not should_freeze and self._is_frozen:
            self.get_logger().info("Steering resumed")
            self._is_frozen = False

        if self._is_frozen:
            return

        raw = msg.data
        if not isinstance(raw, (int, float)):
            raise TypeError(f"Expected numeric angle got {type(raw).__name__}")

        angle = float(raw)
        if math.isnan(angle):
            raise ValueError("Angle must not be NaN")

        angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle))

        self.latest_angle = angle
        target_position = self.convert_angle_to_position(angle)

        if self.debug:
            self.logger.info(f"Angle: {angle:.3f}°, Target Pos: {target_position}")
            self._save_debug(angle, target_position)
            self.counter += 1

        self.steering.move_stepper(target_position)

    def callback_stop(self, msg):
        self._stopped = msg.data
        if self._stopped:
            self.get_logger().info("Stop intent received")
        else:
            self.get_logger().info("Stop intent released")

        if self.debug:
            self.logger.info("Debug mode enabled")

    def convert_angle_to_position(self, angle: float) -> int:
        steps = (angle / STEP_ANGLE) * 256
        return -int(steps * TRANSMISSION_RATIO)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Steering node shutting down")
    finally:
        node._close_csv_logger()  # flush and join the writer thread before teardown
        node.steering.disconnect_motor()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
