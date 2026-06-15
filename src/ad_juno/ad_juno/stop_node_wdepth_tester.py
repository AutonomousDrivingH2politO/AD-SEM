#!/usr/bin/env python3
import os
import sys
import cv2
import numpy as np
import rclpy
from datetime import datetime
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy,
    HistoryPolicy, LivelinessPolicy,
    qos_profile_sensor_data,
)
from shared_objects.stop_utils_new_v1 import analysis

ZED2_TOPIC  = '/zed/zed_node/rgb/color/rect/image'
DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'
SPEED_TOPIC = '/ECU/speed'

QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    liveliness=LivelinessPolicy.AUTOMATIC,
)

DESKTOP       = os.path.expanduser("~/Desktop")
BASE_DIR = os.path.join(DESKTOP, "stop_sign_detections")

BG_DARK       = (24, 22, 20)
TEXT_PRIMARY  = (245, 245, 245)
TEXT_MUTED    = (170, 170, 170)
ACCENT_CYAN   = (235, 206, 135)
ACCENT_RED    = (80, 80, 235)
ACCENT_GREEN  = (120, 200, 120)
ACCENT_AMBER  = (90, 170, 230)
BORDER        = (90, 88, 84)
SEPARATOR     = (60, 58, 56)

FONT = cv2.FONT_HERSHEY_DUPLEX


class StopSignTestNode(Node):
    def __init__(self):
        super().__init__('stop_sign_test_node')
        self.bridge = CvBridge()
        self.counter = 0
        self.count_img = 0
        self.threshold_img = 6

        # latest sensor caches (single-threaded executor => no lock needed)
        self._latest_depth = None
        self._latest_depth_stamp = None
        self._current_speed_kmh = None

        # ---- brake model parameters (mirror stop_node defaults) ----
        self.declare_parameter("brake_t_eng_s",       2.0)
        self.declare_parameter("brake_a_full_ms2",    1.0)
        self.declare_parameter("brake_aim_offset_m", -1.0)
        self.declare_parameter("brake_delay_max_s",   5.0)
        self.declare_parameter("depth_max_age_s",     0.7)
        self.declare_parameter("depth_max_trigger_m", 17.0)

        self._brake_t_eng       = float(self.get_parameter("brake_t_eng_s").value)
        self._brake_a_full      = float(self.get_parameter("brake_a_full_ms2").value)
        self._brake_aim_offset  = float(self.get_parameter("brake_aim_offset_m").value)
        self._brake_delay_max   = float(self.get_parameter("brake_delay_max_s").value)
        self._depth_max_age     = float(self.get_parameter("depth_max_age_s").value)
        self._depth_max_trigger = float(self.get_parameter("depth_max_trigger_m").value)

        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.confirmed_dir = os.path.join(BASE_DIR, f"run_{run_id}", "full_frames")
        self.processed_dir = os.path.join(BASE_DIR, f"run_{run_id}", "processed_only")
        os.makedirs(self.confirmed_dir, exist_ok=True)
        os.makedirs(self.processed_dir, exist_ok=True)

        self.create_subscription(Image, ZED2_TOPIC, self.image_callback, QOS)
        self.create_subscription(
            Image, DEPTH_TOPIC, self._depth_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Float32, SPEED_TOPIC, self._speed_callback, 10
        )

        self.get_logger().info(f"Listening RGB:   {ZED2_TOPIC}")
        self.get_logger().info(f"Listening depth: {DEPTH_TOPIC}")
        self.get_logger().info(f"Listening speed: {SPEED_TOPIC}")
        self.get_logger().info(f"Processed dir:      {self.processed_dir}")
        self.get_logger().info(
            f"Brake model: t_eng={self._brake_t_eng}s  a_full={self._brake_a_full}m/s²  "
            f"aim_offset={self._brake_aim_offset}m  delay_max={self._brake_delay_max}s  "
            f"depth_max_trigger={self._depth_max_trigger}m"
        )
        self.get_logger().info("Started. Ctrl+C to stop.")

    def _ts(self):
        return datetime.now().strftime("%H%M%S_%f")[:10]


    def _depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().warn(f"Depth image conversion failed: {e}")
            return
        self._latest_depth = depth
        self._latest_depth_stamp = self.get_clock().now()

    def _speed_callback(self, msg: Float32):
        self._current_speed_kmh = float(msg.data)

    def _depth_age_s(self):
        if self._latest_depth_stamp is None:
            return None
        try:
            return (
                self.get_clock().now() - self._latest_depth_stamp
            ).nanoseconds * 1e-9
        except Exception:
            return None

    def _sample_depth_at_bbox(self, box, patch_half: int = 3):
        """Median depth (m) inside a (2*patch_half+1)^2 patch at bbox center.
        Returns (depth_m, depth_age_s, n_valid).  (None, None, 0) on failure.
        """
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
        return median_d, self._depth_age_s(), int(valid.size)

    # ------------------ brake math (mirrors stop_node) ------------------

    def _compute_brake_distance(self, v_ms):
        if v_ms is None or v_ms <= 0.0:
            return None
        t_eng  = self._brake_t_eng
        a_full = self._brake_a_full
        if t_eng <= 0.0 or a_full <= 0.0:
            return None
        d_dead = v_ms * t_eng
        d_full = (v_ms ** 2) / (2.0 * a_full)
        return d_dead + d_full

    def _compute_brake_delay(self, depth_m, depth_age_s, speed_kmh):
        """Returns (delay_s, d_brake_m, reason)."""
        if depth_m is None:
            return 0.0, None, "no_depth"
        if depth_age_s is not None and depth_age_s > self._depth_max_age:
            return 0.0, None, "stale_depth"
        if speed_kmh is None:
            return 0.0, None, "no_speed"

        v_ms = speed_kmh / 3.6
        if v_ms < 0.5:
            return 0.0, None, "v_too_low"

        d_brake = self._compute_brake_distance(v_ms)
        if d_brake is None:
            return 0.0, None, "no_d_brake"

        target_stop_distance = depth_m + self._brake_aim_offset
        coast_distance = target_stop_distance - d_brake
        if coast_distance <= 0.0:
            return 0.0, d_brake, "too_close"
        delay = coast_distance / v_ms
        if delay > self._brake_delay_max:
            return 0.0, d_brake, "delay_exceeds_max"
        return delay, d_brake, "ok"


    @staticmethod
    def _draw_panel(img, p1, p2, color=BG_DARK, alpha=0.62, border=BORDER):
        overlay = img.copy()
        cv2.rectangle(overlay, p1, p2, color, -1)
        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, dst=img)
        if border is not None:
            cv2.rectangle(img, p1, p2, border, 1)

    @staticmethod
    def _put_text(img, s, org, scale=0.55, color=TEXT_PRIMARY, thick=1):
        cv2.putText(img, s, org, FONT, scale, color, thick, cv2.LINE_AA)

    def _draw_telemetry(self, frame,
                        depth_m=None, depth_age_s=None, depth_n=0,
                        d_brake=None, delay_s=None, reason=None,
                        sign_detected=False):
        h, w = frame.shape[:2]

        try:
            ros_t = self.get_clock().now().nanoseconds * 1e-9
            ros_t_str = f"{ros_t:12.3f}"
        except Exception:
            ros_t_str = "         n/a"
        sys_t = datetime.now().strftime("%H:%M:%S.%f")[:-3]


        d_age = self._depth_age_s()
        if self._latest_depth is None:
            depth_status, depth_color = "NO DATA", ACCENT_RED
        elif d_age is None:
            depth_status, depth_color = "?", ACCENT_AMBER
        elif d_age > 1.0:
            depth_status, depth_color = f"STALE {d_age:.1f}s", ACCENT_AMBER
        else:
            depth_status, depth_color = f"{d_age*1000:.0f}ms ago", ACCENT_GREEN

        if self._current_speed_kmh is None:
            speed_str, speed_color = "N/A", TEXT_MUTED
        else:
            speed_str   = f"{self._current_speed_kmh:6.2f} km/h"
            speed_color = TEXT_PRIMARY

        header_lines = [
            ("ROS TIME",  ros_t_str,    TEXT_PRIMARY),
            ("SYS TIME",  sys_t,        TEXT_MUTED),
            ("DEPTH",     depth_status, depth_color),
            ("ECU SPEED", speed_str,    speed_color),
        ]
        pad_x, pad_y, line_h, label_w = 14, 12, 22, 110
        block_w = label_w + 200
        block_h = pad_y * 2 + line_h * len(header_lines)
        x0, y0 = 12, 12
        self._draw_panel(frame, (x0, y0), (x0 + block_w, y0 + block_h))
        for i, (label, value, color) in enumerate(header_lines):
            ly = y0 + pad_y + line_h * (i + 1) - 6
            self._put_text(frame, label, (x0 + pad_x, ly),
                           scale=0.42, color=TEXT_MUTED)
            self._put_text(frame, value, (x0 + pad_x + label_w, ly),
                           scale=0.50, color=color)

        v_kmh = self._current_speed_kmh
        v_ms  = (v_kmh / 3.6) if v_kmh is not None else None

        # Running d_brake estimate is meaningful even with no detection
        d_brake_est = self._compute_brake_distance(v_ms) if v_ms is not None else None
        d_b_display = d_brake if d_brake is not None else d_brake_est

        d_m_str    = f"{depth_m:.2f} m"       if depth_m       is not None else "—"
        n_str      = f"{depth_n}"             if depth_n                   else "—"
        v_ms_str   = f"{v_ms:.2f} m/s"        if v_ms          is not None else "—"
        d_b_str    = f"{d_b_display:.2f} m"   if d_b_display   is not None else "—"
        delay_str  = f"{delay_s:.2f} s"       if delay_s       is not None else "—"
        reason_str = reason if reason else "—"

        reason_color = {
            "ok":                ACCENT_GREEN,
            "too_close":         ACCENT_RED,
            "stale_depth":       ACCENT_AMBER,
            "no_depth":          ACCENT_AMBER,
            "no_speed":          ACCENT_AMBER,
            "v_too_low":         TEXT_MUTED,
            "no_d_brake":        ACCENT_AMBER,
            "delay_exceeds_max": ACCENT_AMBER,
        }.get(reason_str, TEXT_MUTED)

        calc_lines = [
            ("DEPTH",   d_m_str,    ACCENT_CYAN),
            ("SAMPLES", n_str,      TEXT_MUTED),
            ("V",       v_ms_str,   TEXT_PRIMARY),
            ("D_BRAKE", d_b_str,    ACCENT_CYAN),
            ("DELAY",   delay_str,  ACCENT_CYAN),
            ("REASON",  reason_str, reason_color),
        ]
        title_h     = 26
        c_block_w   = label_w + 200
        c_block_h   = pad_y * 2 + line_h * len(calc_lines) + title_h
        c_x0, c_y0  = 12, h - c_block_h - 12
        self._draw_panel(frame, (c_x0, c_y0), (c_x0 + c_block_w, c_y0 + c_block_h))
        self._put_text(frame, "BRAKE CALC", (c_x0 + pad_x, c_y0 + 20),
                       scale=0.45, color=ACCENT_CYAN)
        # subtle separator under title
        cv2.line(frame,
                 (c_x0 + pad_x,         c_y0 + title_h),
                 (c_x0 + c_block_w - pad_x, c_y0 + title_h),
                 SEPARATOR, 1)
        for i, (label, value, color) in enumerate(calc_lines):
            ly = c_y0 + title_h + pad_y + line_h * (i + 1) - 6
            self._put_text(frame, label, (c_x0 + pad_x, ly),
                           scale=0.42, color=TEXT_MUTED)
            self._put_text(frame, value, (c_x0 + pad_x + label_w, ly),
                           scale=0.50, color=color)

        # ---- TOP-RIGHT: STOP SIGN DETECTED red badge ----
        if sign_detected:
            badge = "STOP SIGN DETECTED"
            (bw, bh), _ = cv2.getTextSize(badge, FONT, 0.55, 1)
            bx2, by1 = w - 14, 14
            bx1, by2 = bx2 - bw - 44, by1 + bh + 16
            overlay = frame.copy()
            cv2.rectangle(overlay, (bx1, by1), (bx2, by2), ACCENT_RED, -1)
            cv2.addWeighted(overlay, 0.88, frame, 0.12, 0, dst=frame)
            cv2.circle(frame, (bx1 + 16, (by1 + by2) // 2), 5, (255, 255, 255), -1)
            self._put_text(frame, badge, (bx1 + 30, by2 - 10),
                           scale=0.55, color=(255, 255, 255))

    def _draw_depth_for_detection(self, frame, box, depth_m, n_valid):
        """Cyan `D:` label just below the existing red bbox + C: text."""
        x1, y1, x2, y2 = [int(c) for c in box]
        h, w = frame.shape[:2]

        text = f"D:{depth_m:.2f}m n={n_valid}" if depth_m is not None else "D: N/A"

        (tw, th), _ = cv2.getTextSize(text, FONT, 0.55, 1)
        tx = x1
        ty = min(y2 + th + 6, h - 4)
        cv2.rectangle(frame, (tx - 2, ty - th - 4), (tx + tw + 4, ty + 4), BG_DARK,   -1)
        cv2.rectangle(frame, (tx - 2, ty - th - 4), (tx + tw + 4, ty + 4), BORDER,     1)
        self._put_text(frame, text, (tx, ty), scale=0.55, color=ACCENT_CYAN, thick=1)

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        cv2.drawMarker(frame, (cx, cy), ACCENT_CYAN,
                       markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)


    def image_callback(self, msg: Image):
        self.count_img += 1
        if self.count_img % self.threshold_img != 0:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detected, output_frame, details = analysis(img)
        confirmed = details['confirmed']
        h, w = output_frame.shape[:2]

        # Per-detection depth overlay (drawn BEFORE crop so it lands in processed_part too)
        best_det, best_depth_m, best_age, best_n = None, None, None, 0
        for det in confirmed:
            box = det.get('box')
            depth_m, depth_age_s, n_valid = self._sample_depth_at_bbox(box)
            self._draw_depth_for_detection(output_frame, box, depth_m, n_valid)
            # track highest-confidence det for the telemetry panel
            if best_det is None or det.get('conf', 0.0) > best_det.get('conf', 0.0):
                best_det = det
                best_depth_m, best_age, best_n = depth_m, depth_age_s, n_valid

        # crop line (Ignores Sky)
        crop_y1 = int(h * 0.40)
        cv2.line(output_frame, (0, crop_y1), (w, crop_y1), ACCENT_CYAN, 2)
        self._put_text(output_frame, "NO PROCESS FOR UPPER PART",
                       (10, crop_y1 + 20), scale=0.55, color=ACCENT_CYAN, thick=1)
        processed_part = output_frame[crop_y1:h, 0:w]
        # Ignores Ground (kept as-is, commented out)
        '''
        crop_y2 = int(h * 0.60)
        cv2.line(output_frame, (0, crop_y2), (w, crop_y2), ACCENT_CYAN, 2)
        self._put_text(output_frame, "NO PROCESS FOR LOWER PART",
                       (10, crop_y2 - 10), scale=0.55, color=ACCENT_CYAN, thick=1)
        processed_part = output_frame[0:crop_y2, 0:w]
        '''

        # Compute brake delay for the best detection (matches stop_node logic)
        delay_s, d_brake_m, reason = (None, None, None)
        if best_det is not None:
            delay_s, d_brake_m, reason = self._compute_brake_delay(
                best_depth_m, best_age, self._current_speed_kmh
            )

        # Modern HUD on top of full frame (after crop slice so it's not in processed_part)
        self._draw_telemetry(
            output_frame,
            depth_m=best_depth_m,
            depth_age_s=best_age,
            depth_n=best_n,
            d_brake=d_brake_m,
            delay_s=delay_s,
            reason=reason,
            sign_detected=detected,
        )

        ts = self._ts()
        if detected:
            for det in confirmed:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                d_m, d_age, d_n = self._sample_depth_at_bbox(det.get('box'))
                v_kmh = self._current_speed_kmh
                if d_m is not None:
                    age_ms = f"{d_age*1000:.0f}ms" if d_age is not None else "?"
                    d_b_str   = f"{d_brake_m:.2f}m"   if d_brake_m is not None else "N/A"
                    delay_str = f"{delay_s:.2f}s"     if delay_s   is not None else "N/A"
                    v_str     = f"{v_kmh:.2f}km/h"    if v_kmh     is not None else "N/A"
                    self.get_logger().info(
                        f"✅ STOP SIGN! Box=({x1},{y1},{x2},{y2}) "
                        f"C={det['conf']:.2f} D={d_m:.2f}m n={d_n} age={age_ms} "
                        f"v={v_str} d_brake={d_b_str} delay={delay_str} reason={reason}"
                    )
                else:
                    self.get_logger().info(
                        f"✅ STOP SIGN! Box=({x1},{y1},{x2},{y2}) "
                        f"C={det['conf']:.2f} D=N/A reason={reason}"
                    )

            full_path = os.path.join(self.processed_dir, f"{ts}_{self.counter:04d}_full.jpg")
            cv2.imwrite(full_path, output_frame)
            proc_path = os.path.join(self.processed_dir, f"{ts}_{self.counter:04d}_processed.jpg")
            cv2.imwrite(proc_path, processed_part)
            self.counter += 1
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
        node.get_logger().info("Stopped.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
