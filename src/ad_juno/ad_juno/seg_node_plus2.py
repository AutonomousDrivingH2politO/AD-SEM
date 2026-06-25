"""
seg_node_plus2.py — seg_node_plus1 with dual-channel mask encoding.

Change from plus1:
  - Publishes a 3-value mono8 mask instead of a binary OR:
      0   = non-drivable background
      128 = drivable road area
      255 = lane line
  This lets path_planning distinguish road edges from lane lines so it
  can apply the lane-aware BEV edge strategy in new_utils_lane_aware_path.
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


topics      = Topics()
topic_names = topics.topic_names


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

    Subscribes : RGB_image        (sensor_msgs/Image)
    Publishes  : segmented_image  (sensor_msgs/Image, mono8, values 0/128/255)
                 model_enable     (std_msgs/Bool, 1 Hz heartbeat)

    Mask encoding:
        0   = non-drivable
        128 = drivable road
        255 = lane line  (only set when segmentation_mode == "road_lane")

    Inference runs in a background thread so the ROS executor is never blocked.
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

        model_size    = self.get_parameter("model_size").value
        model_path    = self.get_parameter("model_path").value
        self.seg_mode = self.get_parameter("segmentation_mode").value
        self.half     = self.get_parameter("half_precision").value
        self.debug    = self.get_parameter("debug").value
        self.target_sz = self.get_parameter("target_size").value
        self.device   = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        qos_img = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.seg_pub  = self.create_publisher(Image, topic_names["segmented_image"], 10)
        self.bool_pub = self.create_publisher(Bool,  topic_names["model_enable"],    10)

        self.create_subscription(
            Image, topic_names["RGB_image"],
            self._image_callback, qos_img,
        )

        self._model_ok = False
        self._model    = None
        try:
            self._model    = load_twinliteplus(model_size, model_path, self.half)
            self._model_ok = True
            self.get_logger().info(
                f"TwinLiteNetPlus ({model_size}) loaded on {self.device}"
            )
        except Exception as exc:
            self.get_logger().error(f"Model load failed: {exc}")
            self.get_logger().error("Publishing model_enable=False. Node will not process images.")

        self._publish_enable(self._model_ok)

        self._frame_lock = threading.Lock()
        self._pending    = None
        self._stop_event = threading.Event()
        self._worker     = threading.Thread(
            target=self._inference_loop, daemon=True, name="seg_inference"
        )
        self._worker.start()

        self._heartbeat_timer = self.create_timer(1.0, self._heartbeat_cb)
        self._last_log_time   = 0.0

        self.get_logger().info(
            f"SegNode ready  |  mode={self.seg_mode}  |  device={self.device}  "
            f"|  encoding: road=128, lane=255"
        )

    def _image_callback(self, msg: Image) -> None:
        self._count += 1
        if self._count % 9 != 0:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"CvBridge decode error: {exc}")
            return

        with self._frame_lock:
            self._pending = (cv_image, msg.header)

    def _heartbeat_cb(self) -> None:
        self._publish_enable(self._model_ok)
        if self.debug:
            self.get_logger().info("model_enable heartbeat")

    def _inference_loop(self) -> None:
        while not self._stop_event.is_set():
            frame_data = None
            with self._frame_lock:
                if self._pending is not None:
                    frame_data    = self._pending
                    self._pending = None

            if frame_data is None or not self._model_ok:
                time.sleep(0.005)
                continue

            cv_image, header = frame_data
            self._run_inference(cv_image, header)

    def _run_inference(self, cv_image: np.ndarray, header) -> None:
        try:
            t0 = time.perf_counter()

            info = preprocess_twinliteplus(
                cv_image,
                target_size=self.target_sz,
                device=self.device,
                half=self.half,
            )

            with torch.no_grad():
                da_logits, ll_logits = self._model(info["tensor"])

            road_mask, lane_mask = postprocess_twinliteplus(
                da_logits, ll_logits, info, improve=True
            )

            # Encode: road=128, lane=255 (lane pixels overwrite road pixels).
            # Using distinct values lets the BEV node count lane lines vs road edges
            # without needing a second topic.
            final_mask = np.where(road_mask > 0, np.uint8(128), np.uint8(0))
            if self.seg_mode == "road_lane":
                final_mask = np.where(lane_mask > 0, np.uint8(255), final_mask)

            msg = self.bridge.cv2_to_imgmsg(final_mask, encoding="mono8")
            msg.header = header
            self.seg_pub.publish(msg)

            elapsed = time.perf_counter() - t0
            now     = time.perf_counter()

            if self.debug and (now - self._last_log_time >= 1.0):
                self.get_logger().info(
                    f"inference {elapsed*1000:.1f} ms  |  "
                    f"mask shape {final_mask.shape}  |  "
                    f"mode={self.seg_mode}  |  "
                    f"lane px={int(np.sum(lane_mask > 0))}"
                )

            self._last_log_time = now

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
