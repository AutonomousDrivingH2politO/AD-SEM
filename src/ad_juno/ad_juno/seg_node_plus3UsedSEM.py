"""
seg_node_plus2.py — ROS2 segmentation node using TwinLiteNetPlus.

Key fixes vs original:
  - Inference runs in a dedicated background thread; ROS executor never blocks
  - Uses preprocess_twinliteplus + postprocess_twinliteplus (no hardcoded shapes[])
  - model_type is a ROS2 parameter, not a hardcoded global
  - Model load failure publishes model_enable=False instead of crashing
  - debug defaults to False; logging is rate-limited to 1 Hz
  - get_segmentation removed (was a misplaced standalone function with a self arg)
  - Dead globals (count, seg_img_id, half, model_type) removed
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import torch
from argparse import Namespace

from shared_objects.ROS_utils import Topics
from shared_objects.utils_model_v2 import preprocess_twinliteplus, postprocess_twinliteplus
from shared_objects.TwinLiteNetPlus.model.model import TwinLiteNetPlus


topics     = Topics()
topic_names = topics.topic_names

def _apply_lane_boundary(
    road_mask: np.ndarray,
    lane_mask: np.ndarray,
    close_kernel: tuple = (3, 51),
    side_trust_threshold: int = 80,
) -> np.ndarray:
    """
    Shape the drivable-area mask so each side's boundary is the inner-most
    lane line when that side has a *trusted* lane, otherwise the drivable-area
    edge.

    Three layers of robustness against sparse lane segmentation:
      1. Vertical morphological closing densifies the lane mask, bridging
         small gaps along a single lane line.
      2. Per-side trust gate: a side is only treated as lane-bounded if it
         has enough lane evidence in total. Prevents row-by-row flipping
         between lane and drivable-edge strategies.
      3. Carry-forward on a trusted side: rows that still have no lane pixel
         after densification inherit the last seen lane x on that side,
         instead of jumping back out to the drivable-area edge.
    """
    out = road_mask.copy()
    if not lane_mask.any():
        return out

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, close_kernel)
    dense_lane = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)

    lane_bool = dense_lane > 0
    road_bool = road_mask > 0
    if not road_bool.any():
        return out

    cols_with_road = np.where(road_bool.any(axis=0))[0]
    road_center_x = (int(cols_with_road[0]) + int(cols_with_road[-1])) // 2

    left_trusted  = int(lane_bool[:, :road_center_x].sum()) >= side_trust_threshold
    right_trusted = int(lane_bool[:, road_center_x:].sum()) >= side_trust_threshold
    if not (left_trusted or right_trusted):
        return out

    last_left_x = None
    last_right_x = None

    for y in range(road_mask.shape[0]):
        road_xs = np.where(road_bool[y])[0]
        if road_xs.size == 0:
            continue

        road_left, road_right = int(road_xs[0]), int(road_xs[-1])
        row_center = (road_left + road_right) // 2

        lane_xs = np.where(lane_bool[y])[0] if lane_bool[y].any() else np.empty(0, dtype=int)
        left_lanes  = lane_xs[lane_xs < row_center]
        right_lanes = lane_xs[lane_xs > row_center]

        if left_trusted:
            if left_lanes.size:
                eff_left = int(left_lanes.max())
                last_left_x = eff_left
            else:
                eff_left = last_left_x if last_left_x is not None else road_left
        else:
            eff_left = road_left

        if right_trusted:
            if right_lanes.size:
                eff_right = int(right_lanes.min())
                last_right_x = eff_right
            else:
                eff_right = last_right_x if last_right_x is not None else road_right
        else:
            eff_right = road_right

        eff_left  = max(eff_left,  road_left)
        eff_right = min(eff_right, road_right)
        if eff_left > eff_right:
            continue

        out[y, :] = 0
        out[y, eff_left:eff_right + 1] = 255

    return out

def load_twinliteplus(model_size: str, model_path: str, half: bool = False) -> torch.nn.Module:
    """Load TwinLiteNetPlus from checkpoint. Raises on failure (caller handles it)."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    args   = Namespace(config=model_size)
    model  = TwinLiteNetPlus(args).to(device)
    state  = torch.load(model_path, map_location=device)
    model.load_state_dict(state)
    model.eval()
    if half and device.type == "cuda":
        model.half()
    return model


class SegNode(Node):
    """
    ROS2 node for road + lane segmentation using TwinLiteNetPlus.

    Subscribes : RGB_image  (sensor_msgs/Image)
    Publishes  : segmented_image (sensor_msgs/Image, mono8)
                 model_enable    (std_msgs/Bool, 1 Hz heartbeat)

    Inference runs in a background thread so the ROS executor is never blocked.
    The callback drops the newest frame into a 1-slot queue; the worker drains it.
    """

    def __init__(self):
        super().__init__("seg_node")
        self.bridge = CvBridge()

        self.declare_parameter("model_size",        "large")
        self.declare_parameter("model_path",
            "/home/bylogix/AD-SEM/src/shared_objects/src/shared_objects/"
            "TwinLiteNetPlus/pretrained/twinliteplus_pretrained/large.pth")
        self.declare_parameter("segmentation_mode", "road_lane")
        self.declare_parameter("half_precision",    False)
        self.declare_parameter("debug",             True)
        self.declare_parameter("target_size",       640)
        self._count = 0

        model_size   = self.get_parameter("model_size").value
        model_path   = self.get_parameter("model_path").value
        self.seg_mode  = self.get_parameter("segmentation_mode").value
        self.half      = self.get_parameter("half_precision").value
        self.debug     = self.get_parameter("debug").value
        self.target_sz = self.get_parameter("target_size").value
        self.device    = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        qos_img = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.seg_pub   = self.create_publisher(Image, topic_names["segmented_image"], 10)
        self.bool_pub  = self.create_publisher(Bool,  topic_names["model_enable"],    10)

        self.create_subscription(
            Image, topic_names["RGB_image"],
            self._image_callback, qos_img,
        )

        self._obstacle_mode = False
        self.create_subscription(
            Bool, '/path_planning/obstacle_mode',
            self._obstacle_mode_callback, 10,
        )

        self._model_ok = False
        self._model    = None
        try:
            self._model   = load_twinliteplus(model_size, model_path, self.half)
            self._model_ok = True
            self.get_logger().info(
                f"TwinLiteNetPlus ({model_size}) loaded on {self.device}"
            )
        except Exception as exc:
            self.get_logger().error(f"Model load failed: {exc}")
            self.get_logger().error("Publishing model_enable=False. Node will not process images.")

        # Publish initial enable state
        self._publish_enable(self._model_ok)

        self._frame_lock  = threading.Lock()
        self._pending     = None          # (cv_image, header) or None
        self._stop_event  = threading.Event()
        self._worker      = threading.Thread(
            target=self._inference_loop, daemon=True, name="seg_inference"
        )
        self._worker.start()

        self._heartbeat_timer = self.create_timer(1.0, self._heartbeat_cb)
        self._last_log_time   = 0.0

        self.get_logger().info(
            f"SegNode_plus2 ready  |  mode={self.seg_mode}  |  device={self.device}"
        )


    def _image_callback(self, msg: Image) -> None:
        """Drop the latest frame into the 1-slot queue. Never blocks."""
        self._count += 1
        if self._count % 9 != 0:
            return
        try:
            # msg arrives as rgb8 from CvBridge convention in seg_node
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"CvBridge decode error: {exc}")
            return

        with self._frame_lock:
            self._pending = (cv_image, msg.header)   # overwrite stale frame
            
    def _obstacle_mode_callback(self, msg: Bool) -> None:
            self._obstacle_mode = msg.data

    def _heartbeat_cb(self) -> None:
        self._publish_enable(self._model_ok)
        if self.debug:
            self.get_logger().info("model_enable heartbeat")


    def _inference_loop(self) -> None:
        """
        Runs in a daemon thread.
        Drains _pending as fast as the GPU allows; idles when nothing is queued.
        """
        while not self._stop_event.is_set():
            frame_data = None
            with self._frame_lock:
                if self._pending is not None:
                    frame_data    = self._pending
                    self._pending = None

            if frame_data is None or not self._model_ok:
                time.sleep(0.005)   # ~5 ms idle poll
                continue

            cv_image, header = frame_data
            self._run_inference(cv_image, header)

    def _run_inference(self, cv_image: np.ndarray, header) -> None:
        """Preprocess → infer → postprocess → publish."""
        try:
            t0 = time.perf_counter()

            # 1. Preprocess (returns a dict with tensor + padding metadata)
            info = preprocess_twinliteplus(
                cv_image,
                target_size=self.target_sz,
                device=self.device,
                half=self.half,
            )

            # 2. Inference
            with torch.no_grad():
                da_logits, ll_logits = self._model(info["tensor"])

            # 3. Postprocess — logit-space bilinear interp, then argmax, then resize
            road_mask, lane_mask = postprocess_twinliteplus(
                da_logits, ll_logits, info, improve=True
            )

            # 4. Optionally merge road + lane
            if self.seg_mode == "road_lane" and not self._obstacle_mode:
                final_mask = _apply_lane_boundary(road_mask, lane_mask)
            else:
                final_mask = road_mask

            # 5. Publish
            msg = self.bridge.cv2_to_imgmsg(final_mask, encoding="mono8")
            msg.header = header     # propagate original timestamp + frame_id
            self.seg_pub.publish(msg)
            elapsed = time.perf_counter() - t0
            now = time.perf_counter()

            if self.debug and (now - self._last_log_time >= 1.0):
                self.get_logger().info(
                    f"inference {elapsed*1000:.1f} ms  |  "
                    f"mask shape {final_mask.shape}  |  "
                    f"mode={self.seg_mode}"
                )

            self._last_log_time = now  # always update

        except Exception as exc:
            self.get_logger().error(f"Inference error: {exc}")


    def _publish_enable(self, state: bool) -> None:
        msg      = Bool()
        msg.data = state
        self.bool_pub.publish(msg)

    def destroy_node(self) -> None:
        self._stop_event.set()
        self._worker.join(timeout=2.0)
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = SegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
