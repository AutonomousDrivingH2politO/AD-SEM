#!/usr/bin/env python3
import os
import sys
import cv2
import numpy as np
import rclpy
from datetime import datetime
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy,
    HistoryPolicy, LivelinessPolicy,
    qos_profile_sensor_data,
)
from shared_objects.stop_utils_new_v1 import analysis

ZED2_TOPIC  = '/zed/zed_node/rgb/color/rect/image'
DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'

QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    liveliness=LivelinessPolicy.AUTOMATIC,
)

DESKTOP       = os.path.expanduser("~/Desktop")
CONFIRMED_DIR = os.path.join(DESKTOP, "stop_sign_detections", "full_frames")
PROCESSED_DIR = os.path.join(DESKTOP, "stop_sign_detections", "processed_only")

DEPTH_TEXT_COLOR = (0, 255, 255)   # yellow (BGR), distinct from alarm red
HUD_TEXT_COLOR   = (255, 255, 255) # white
HUD_BG_COLOR     = (0, 0, 0)


class StopSignTestNode(Node):
    def __init__(self):
        super().__init__('stop_sign_test_node')
        self.bridge = CvBridge()
        self.counter = 0
        self.count_img = 0
        self.threshold_img = 6

        # latest depth cache (single-threaded executor => no lock needed)
        self._latest_depth = None
        self._latest_depth_stamp = None

        os.makedirs(CONFIRMED_DIR, exist_ok=True)
        os.makedirs(PROCESSED_DIR, exist_ok=True)

        self.create_subscription(Image, ZED2_TOPIC, self.image_callback, QOS)
        self.create_subscription(
            Image, DEPTH_TOPIC, self._depth_callback, qos_profile_sensor_data
        )

        self.get_logger().info(f"Listening RGB:   {ZED2_TOPIC}")
        self.get_logger().info(f"Listening depth: {DEPTH_TOPIC}")
        self.get_logger().info(f"All frames:      {CONFIRMED_DIR}")
        self.get_logger().info(f"Processeds:      {PROCESSED_DIR}")
        self.get_logger().info("Started. Ctrl+C to stop.")

    def _ts(self):
        return datetime.now().strftime("%H%M%S_%f")[:10]

    # ----------------------- depth handling -----------------------

    def _depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().warn(f"Depth image conversion failed: {e}")
            return
        self._latest_depth = depth
        self._latest_depth_stamp = self.get_clock().now()

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

        age_s = None
        if self._latest_depth_stamp is not None:
            try:
                age_s = (
                    self.get_clock().now() - self._latest_depth_stamp
                ).nanoseconds * 1e-9
            except Exception:
                age_s = None

        return median_d, age_s, int(valid.size)

    # ----------------------- HUD / overlays -----------------------

    def _draw_hud(self, frame):
        """Top-left badge with depth stream health. Drawn every frame."""
        if self._latest_depth is None:
            text = "Depth: NO DATA"
            bg = (0, 0, 180)
        else:
            age_s = None
            if self._latest_depth_stamp is not None:
                age_s = (
                    self.get_clock().now() - self._latest_depth_stamp
                ).nanoseconds * 1e-9
            if age_s is None or age_s > 1.0:
                text = f"Depth: stale ({age_s:.1f}s)" if age_s else "Depth: ?"
                bg = (0, 100, 180)
            else:
                text = f"Depth: {age_s*1000:.0f}ms ago"
                bg = (0, 120, 0)

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thick = 1
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
        x, y = 10, 30
        cv2.rectangle(frame, (x - 6, y - th - 6), (x + tw + 6, y + 6), bg, -1)
        cv2.putText(frame, text, (x, y), font, scale, HUD_TEXT_COLOR, thick, cv2.LINE_AA)

    def _draw_depth_for_detection(self, frame, box, depth_m, n_valid):
        """Yellow `D:` label just below the existing red bbox + C: text."""
        x1, y1, x2, y2 = [int(c) for c in box]
        h, w = frame.shape[:2]

        if depth_m is None:
            text = "D: N/A"
        else:
            text = f"D:{depth_m:.2f}m n={n_valid}"

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thick = 2
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)

        # Default: just below the bbox.  If that falls off-screen, put it inside the bottom edge.
        tx = x1
        ty = min(y2 + th + 6, h - 4)
        # Small dark background for legibility against bright scenes
        cv2.rectangle(frame, (tx - 2, ty - th - 4), (tx + tw + 4, ty + 4), HUD_BG_COLOR, -1)
        cv2.putText(frame, text, (tx, ty), font, scale, DEPTH_TEXT_COLOR, thick, cv2.LINE_AA)

        # Tiny marker at the exact sampled center so misalignment is visible by eye
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        cv2.drawMarker(frame, (cx, cy), DEPTH_TEXT_COLOR,
                       markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)

    # ----------------------- main RGB callback -----------------------

    def image_callback(self, msg: Image):
        self.count_img += 1
        if self.count_img % self.threshold_img != 0:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        detected, output_frame, details = analysis(img)
        confirmed = details['confirmed']
        h, w = output_frame.shape[:2]

        # --- depth overlay per detection (drawn BEFORE the crop so it lands in processed_part too) ---
        for det in confirmed:
            box = det.get('box')
            depth_m, depth_age_s, n_valid = self._sample_depth_at_bbox(box)
            self._draw_depth_for_detection(output_frame, box, depth_m, n_valid)

        # --- existing crop visualization (Ignores Sky) ---
        crop_y1 = int(h * 0.40)
        cv2.line(output_frame, (0, crop_y1), (w, crop_y1), (255, 0, 0), 2)
        cv2.putText(output_frame, "NO PROCESS FOR UPPER PART",
                    (10, crop_y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        processed_part = output_frame[crop_y1:h, 0:w]
        # Ignores Ground (kept as-is, commented out)
        '''
        crop_y2 = int(h * 0.60)
        cv2.line(output_frame, (0, crop_y2), (w, crop_y2), (255, 0, 0), 2)
        cv2.putText(output_frame, "NO PROCESS FOR LOWER PART)",
                    (10, crop_y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        processed_part = output_frame[0:crop_y2, 0:w]
        '''

        # --- HUD on top of the full frame (after crop slice so it's not duplicated into processed_part) ---
        self._draw_hud(output_frame)

        ts = self._ts()
        if detected:
            for det in confirmed:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                d_m, d_age, d_n = self._sample_depth_at_bbox(det.get('box'))
                if d_m is not None:
                    age_ms = f"{d_age*1000:.0f}ms" if d_age is not None else "?"
                    self.get_logger().info(
                        f"✅ STOP SIGN! Box=({x1},{y1},{x2},{y2}) "
                        f"C={det['conf']:.2f} D={d_m:.2f}m n={d_n} age={age_ms}"
                    )
                else:
                    self.get_logger().info(
                        f"✅ STOP SIGN! Box=({x1},{y1},{x2},{y2}) "
                        f"C={det['conf']:.2f} D=N/A"
                    )

            full_path = os.path.join(CONFIRMED_DIR, f"{ts}_{self.counter:04d}_full.jpg")
            cv2.imwrite(full_path, output_frame)
            proc_path = os.path.join(PROCESSED_DIR, f"{ts}_{self.counter:04d}_processed.jpg")
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
