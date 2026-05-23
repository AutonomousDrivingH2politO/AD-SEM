#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import threading
import numpy as np
import cv2
from shared_objects.ROS_utils import Topics
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

# ZED optical frame: X=right, Y=down, Z=forward
# To spoof a point as "tall", set Y negative (upward in optical frame)
SPOOF_AXIS = "y"
SPOOF_VALUE = -1.2  # 1.5m above camera in optical frame
BOUNDARY_KERNEL_PX = 3
BOUNDARY_ITERATIONS = 2  # erode inward — wall inside road edge, not outside


class PointCloudMaskerNode(Node):
    def __init__(self):
        super().__init__("pc_masker_node")
        self.bridge = CvBridge()
        topics = Topics()

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Cache latest mask instead of synchronizing
        self.latest_mask = None
        self.masking_lock = threading.Lock()

        # Independent subscriptions instead of ApproximateTimeSynchronizer
        self.create_subscription(
            Image,
            topics.topic_names["segmented_image"],
            self.mask_callback,
            qos_be,
        )
        self.create_subscription(
            PointCloud2,
            "/zed/zed_node/point_cloud/cloud_registered",
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.pub_pc = self.create_publisher(
            PointCloud2, "/zed/zed_node/point_cloud/road_boundaries", 1
        )

        self.get_logger().info(
            f"Boundary masker ready — spoofing axis '{SPOOF_AXIS}' = {SPOOF_VALUE}m"
        )

    def _get_axis_offset(self, pc_msg: PointCloud2):
        offset = next((f.offset for f in pc_msg.fields if f.name == SPOOF_AXIS), None)
        if offset is None:
            self.get_logger().error(
                f"Field '{SPOOF_AXIS}' not found in point cloud. "
                f"Available: {[f.name for f in pc_msg.fields]}"
            )

        return offset


    def mask_callback(self, mask_msg: Image):
        cv_mask = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")

        # Resize mask to match point cloud resolution BEFORE morphology
        # so the kernel size is in point-cloud pixel units, not image pixel units
        TARGET_H, TARGET_W = 256, 448  # match your ZED point cloud
        if cv_mask.shape != (TARGET_H, TARGET_W):
            cv_mask = cv2.resize(
                cv_mask, (TARGET_W, TARGET_H), interpolation=cv2.INTER_NEAREST
            )

        kernel = np.ones((BOUNDARY_KERNEL_PX, BOUNDARY_KERNEL_PX), np.uint8)
        road_eroded = cv2.erode(cv_mask, kernel, iterations=BOUNDARY_ITERATIONS)
        new_mask = {
            "boundary_pixels": (cv_mask == 255) & (road_eroded == 0),
            "shape": cv_mask.shape,  # now already matches cloud shape
        }
        with self.masking_lock:
            self.latest_mask = new_mask

    def cloud_callback(self, pc_msg: PointCloud2):
        with self.masking_lock:
            latest = self.latest_mask
        if latest is None:
            return

        if pc_msg.height == 1:
            self.get_logger().warn_once("Unordered point cloud — skipping")
            return

        axis_offset = self._get_axis_offset(pc_msg)
        if axis_offset is None:
            return

        boundary_pixels = latest["boundary_pixels"]
        mask_shape = latest["shape"]

        if mask_shape != (pc_msg.height, pc_msg.width):
            boundary_pixels = cv2.resize(
                boundary_pixels.astype(np.uint8),
                (pc_msg.width, pc_msg.height),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)

        cloud_data = np.frombuffer(pc_msg.data, dtype=np.uint8).copy()

        axis_view = np.ndarray(
            shape=(pc_msg.height, pc_msg.width),
            dtype=np.float32,
            buffer=cloud_data,
            offset=axis_offset,
            strides=(pc_msg.row_step, pc_msg.point_step),
        )

        valid = np.isfinite(axis_view)
        spoof = boundary_pixels & valid
        axis_view[spoof] = SPOOF_VALUE

        out_msg = PointCloud2()
        out_msg.header = pc_msg.header
        out_msg.height = pc_msg.height
        out_msg.width = pc_msg.width
        out_msg.fields = pc_msg.fields
        out_msg.is_bigendian = pc_msg.is_bigendian
        out_msg.point_step = pc_msg.point_step
        out_msg.row_step = pc_msg.row_step
        out_msg.is_dense = pc_msg.is_dense
        out_msg.data = cloud_data.tobytes()

        self.pub_pc.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMaskerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
