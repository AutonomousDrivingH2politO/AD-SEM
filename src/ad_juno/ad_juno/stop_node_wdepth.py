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
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
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
    qos_profile_sensor_data,
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
    # depth + speed snapshot at detection / trigger
    "depth_m",
    "depth_age_s",
    "depth_n_valid",
    "speed_kmh",
    # computed delay logic at trigger
    "d_brake_m",
    "delay_s",
]

T = TypeVar("T")
SHARED_OBJECTS_DIR = os.path.dirname(shared_objects.__file__)
BRAKE_PARAMS_PATH  = os.path.join(SHARED_OBJECTS_DIR, "params.json")


def unwrap_nullable(value: T | None, error_msg: str) -> T:
    if value is None:
        raise ValueError(error_msg)
    return value


def _json_default(o):
    if hasattr(o, "item"):
        return o.item()
    if hasattr(o, "tolist"):
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

        # depth topic
        self.declare_parameter(
            "depth_topic", "/zed/zed_node/depth/depth_registered"
        )
        depth_topic = self.get_parameter("depth_topic").value

        # ramp phase: a(t) = (a_full / t_eng) * t   for t in [0, t_eng]
        # full phase: a(t) = a_full                 for t > t_eng
        # Calibrated from the 8 km/h test: t_eng=2.0s, a_full=0.8 m/s².
        self.declare_parameter("brake_t_eng_s",       2.0)
        self.declare_parameter("brake_a_full_ms2",    1.0)
        # offset from the sign at which we *want* to come to rest (m).
        # 0.0 = aim at the sign. Negative = stop short of sign. Positive = past sign.
        self.declare_parameter("brake_aim_offset_m",  -1.0)
        # safety cap on computed delay (s)
        self.declare_parameter("brake_delay_max_s",   5.0)
        # depth older than this is rejected → brake immediately
        self.declare_parameter("depth_max_age_s",     0.7)
        # skip the trigger if the sign is farther than this; ZED depth gets
        # noisy past ~18 m so we'd be computing a delay from junk. Keeps
        # detecting; the next frame (closer) will re-evaluate.
        # and ZED depth is max 20 meters
        self.declare_parameter("depth_max_trigger_m", 17.0)

        # time to engage is 2 from the csv logs and timing from real life 
        # this has to be taken into account since the car starts braking at max pedal position

        self._brake_t_eng       = float(self.get_parameter("brake_t_eng_s").value)
        # a_full is calculated from the track test of 05.06.2026 with the speed of 8 km/h
        self._brake_a_full      = float(self.get_parameter("brake_a_full_ms2").value)
        self._brake_aim_offset  = float(self.get_parameter("brake_aim_offset_m").value)
        self._brake_delay_max   = float(self.get_parameter("brake_delay_max_s").value)
        self._depth_max_age     = float(self.get_parameter("depth_max_age_s").value)
        self._depth_max_trigger = float(self.get_parameter("depth_max_trigger_m").value)

        self.brake_srv = self.create_service(
            Trigger, "trigger_brake", self.handle_trigger_brake
        )
        self.enable    = True
        self.count_stop = 0
        self.count_img  = 0

        self.threshold_stop = 2
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

        # latest sensor caches (single-threaded executor => no lock needed)
        self._latest_depth = None
        self._latest_depth_stamp = None
        self._current_speed_kmh = None

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

        self.depth_sub = self.create_subscription(
            Image, depth_topic, self._depth_callback, qos_profile_sensor_data
        )
        self.logger.info(f"Subscribed to depth topic: {depth_topic}")

        self.speed_sub = self.create_subscription(
            Float32, "/ECU/speed", self._speed_callback, qos_profile
        )
        self.logger.info(f"Subscribed to speed topic: {self.topic_names['speed']}")

        self.logger.info(
            f"Brake model: t_eng={self._brake_t_eng}s  a_full={self._brake_a_full}m/s²  "
            f"aim_offset={self._brake_aim_offset}m  delay_max={self._brake_delay_max}s  "
            f"depth_max_trigger={self._depth_max_trigger}m"
        )

        self.declare_parameter("debug_csv_path", "/home/bylogix/AD-SEM/DEBUG")
        self._init_debug_csv(self.get_parameter("debug_csv_path").value)

        self.get_logger().info("StopSignDetector node has been started")


    def _depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.logger.warn(f"Depth image conversion failed: {e}")
            return
        self._latest_depth = depth
        self._latest_depth_stamp = self.get_clock().now()

    def _speed_callback(self, msg: Float32):
        self._current_speed_kmh = float(msg.data)

    def _sample_depth_at_bbox(self, box, patch_half: int = 3):
        if self._latest_depth is None or box is None:
            return None, None, 0

        depth = self._latest_depth
        h, w = depth.shape[:2]
        x1, y1, x2, y2 = [int(c) for c in box]
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        if not (0 <= cx < w and 0 <= cy < h):
            return None, None, 0

        y_lo = max(0, cy - patch_half)
        y_hi = min(h, cy + patch_half + 1)
        x_lo = max(0, cx - patch_half)
        x_hi = min(w, cx + patch_half + 1)

        patch = depth[y_lo:y_hi, x_lo:x_hi]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]

        if valid.size == 0:
            return None, None, 0

        median_d = float(np.median(valid))

        age_s = None
        if self._latest_depth_stamp is not None:
            try:
                age_s = (
                    self.get_clock().now() - self._latest_depth_stamp
                ).nanoseconds * 1e-9
            except Exception:
                age_s = None

        return median_d, age_s, int(valid.size)


    def _compute_brake_distance(self, v_ms):
        if v_ms is None or v_ms <= 0.0:
            return None
        t_eng  = self._brake_t_eng       # 2.0 s
        a_full = self._brake_a_full      # 1.00 m/s² (was 0.69 for ramp model)
        if t_eng <= 0.0 or a_full <= 0.0:
            return None

        d_dead = v_ms * t_eng                  # car coasts during engagement
        d_full = (v_ms ** 2) / (2.0 * a_full)  # then constant-decel brake
        return d_dead + d_full
    def _compute_brake_delay(self, depth_m, depth_age_s, speed_kmh):
        """Decide how long to coast before triggering the brake.

        Returns (delay_s, d_brake_m, reason).
        reason is a short string for logging ("ok", "no_depth", "stale_depth",
        "no_speed", "v_too_low", "too_close", "delay_exceeds_max", "no_d_brake").
        On any non-"ok" reason the caller should brake immediately (delay 0).
        """
        if depth_m is None:
            return 0.0, None, "no_depth"
        if depth_age_s is not None and depth_age_s > self._depth_max_age:
            return 0.0, None, "stale_depth"
        if speed_kmh is None:
            return 0.0, None, "no_speed"

        v_ms = speed_kmh / 3.6
        # below ~1.8 km/h the delay math diverges and the car is essentially stopped
        if v_ms < 0.5:
            return 0.0, None, "v_too_low"

        d_brake = self._compute_brake_distance(v_ms)
        if d_brake is None:
            return 0.0, None, "no_d_brake"

        # target = distance from camera at which we want the car to come to rest
        # depth measures camera→sign, aim_offset shifts target relative to sign
        target_stop_distance = depth_m + self._brake_aim_offset

        coast_distance = target_stop_distance - d_brake
        if coast_distance <= 0.0:
            # Already inside/past braking distance → can't make the window, brake now
            return 0.0, d_brake, "too_close"

        delay = coast_distance / v_ms
        if delay > self._brake_delay_max:
            return 0.0, d_brake, "delay_exceeds_max"

        return delay, d_brake, "ok"


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
            speed_kmh=self._current_speed_kmh,
        )

        # Manual = immediate, no delay
        threading.Thread(target=self._brake_sequence, args=(0.0,), daemon=True).start()

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

            if not self.debug_csv_path or os.path.isdir(self.debug_csv_path):
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                base_dir = self.debug_csv_path or os.path.expanduser("~")
                os.makedirs(base_dir, exist_ok=True)
                self.debug_csv_path = os.path.join(base_dir, f"stop_node_debug_{ts}.csv")
            else:
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

            # Snapshot speed once for this frame
            speed_kmh = self._current_speed_kmh

            confirmed_dets = details.get("confirmed", []) if isinstance(details, dict) else []

            # Per-detection logging (every confirmed bbox in this frame)
            for det in confirmed_dets:
                box = det.get("box")
                depth_m, depth_age_s, depth_n_valid = self._sample_depth_at_bbox(box)
                self._log_csv(
                    "detection",
                    conf=float(det.get("conf")) if det.get("conf") is not None else None,
                    box=[int(x) for x in box] if box is not None else None,
                    depth_m=depth_m,
                    depth_age_s=depth_age_s,
                    depth_n_valid=depth_n_valid,
                    speed_kmh=speed_kmh,
                )

            if result:
                if self.count_stop < self.threshold_stop:
                    self.count_stop += 1
                else:
                    # Sample depth FIRST so we can decide whether to fire the
                    # trigger at all. If the sign is past depth_max_trigger we
                    # defer: keep is_waiting=False / count_stop at threshold so
                    # the next confirmed frame re-evaluates.
                    best_det = max(
                        confirmed_dets, key=lambda d: d.get("conf", 0.0)
                    ) if confirmed_dets else None
                    best_box  = best_det.get("box") if best_det else None
                    best_conf = float(best_det["conf"]) if best_det else None
                    d_m, d_age, d_n = self._sample_depth_at_bbox(best_box)

                    depth_fresh = (
                        d_age is None or d_age <= self._depth_max_age
                    )
                    # Skip when depth isn't usable for the delay calc:
                    #   d_m is None  → no valid depth pixels at the bbox
                    #                  (tiny/distant sign, sky behind it).
                    #                  Treated as "too far" rather than a
                    #                  sensor failure.
                    #   d_m > max    → fresh value past ZED's reliable range.
                    # A STALE non-None depth still falls through to the
                    # brake-immediately fallback in _compute_brake_delay.
                    if d_m is None or (depth_fresh and d_m > self._depth_max_trigger):
                        self._log_csv(
                            "too_far",
                            conf=best_conf,
                            box=[int(x) for x in best_box] if best_box else None,
                            depth_m=d_m,
                            depth_age_s=d_age,
                            depth_n_valid=d_n,
                            speed_kmh=speed_kmh,
                            status="too_far",
                        )
                        if self.debug:
                            if d_m is None:
                                self.logger.info(
                                    f"No valid depth pixels at bbox "
                                    f"(box={best_box}, n_valid={d_n}); deferring."
                                )
                            else:
                                self.logger.info(
                                    f"Stop sign at {d_m:.2f}m > "
                                    f"{self._depth_max_trigger:.1f}m; deferring trigger."
                                )
                        return

                    self.get_logger().info("Stop sign detected. Computing delay.")
                    self.is_waiting = True
                    self.enable     = False
                    self.count_stop = 0

                    stop_msg = Bool(data=True)
                    self.stop_pub.publish(stop_msg)

                    # Decide the coast delay
                    delay_s, d_brake_m, reason = self._compute_brake_delay(
                        d_m, d_age, speed_kmh
                    )

                    self._log_csv(
                        "brake_trigger",
                        conf=best_conf,
                        box=[int(x) for x in best_box] if best_box else None,
                        depth_m=d_m,
                        depth_age_s=d_age,
                        depth_n_valid=d_n,
                        speed_kmh=speed_kmh,
                        d_brake_m=d_brake_m,
                        delay_s=delay_s,
                        status=reason,
                        brake_motor_params=self._brake_motor_params_snapshot,
                        brake_call_args=self._brake_call_args_snapshot,
                    )

                    d_m_str     = f"{d_m:.2f}m"        if d_m       is not None else "N/A"
                    d_brake_str = f"{d_brake_m:.2f}m"  if d_brake_m is not None else "N/A"
                    speed_str   = f"{speed_kmh:.2f}km/h" if speed_kmh is not None else "N/A"
                    self.logger.info(
                        f"=== BRAKE TRIGGER === reason={reason}  "
                        f"D={d_m_str}  v={speed_str}  d_brake={d_brake_str}  delay={delay_s:.2f}s"
                    )

                    threading.Thread(
                        target=self._brake_sequence, args=(delay_s,), daemon=True
                    ).start()
            else:
                self.count_stop = 0

            if self.debug:
                for det in confirmed_dets:
                    self.logger.info(
                        f"Stop sign confirmed: Box={det['box']}  conf={det['conf']:.2f}"
                    )
                self.logger.info(
                    f"Stop detection count: {self.count_stop}/{self.threshold_stop}  "
                    f"speed={speed_kmh if speed_kmh is not None else 'N/A'} km/h"
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

    def _brake_sequence(self, delay_s: float = 0.0):
        # Coast for delay_s. is_waiting=True (already set by trigger) blocks the
        # detection callback during this period so nothing else can fire a brake.
        if delay_s and delay_s > 0.0:
            self.logger.info(f"Coasting for {delay_s:.2f}s before braking…")
            time.sleep(delay_s)

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
