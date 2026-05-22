#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2
from shared_objects.ROS_utils import Topics
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

# ZED optical frame: X=right, Y=down, Z=forward
# To spoof a point as "tall", set Y negative (upward in optical frame)
SPOOF_AXIS   = 'y'
SPOOF_VALUE  = -1.5   # 1.5m above camera in optical frame
BOUNDARY_KERNEL_PX = 3
BOUNDARY_ITERATIONS = 2  # erode inward — wall inside road edge, not outside

class PointCloudMaskerNode(Node):
    def __init__(self):
        super().__init__('pc_masker_node')
        self.bridge = CvBridge()
        topics = Topics()

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.sub_pc = message_filters.Subscriber(
            self, PointCloud2,
            '/zed/zed_node/point_cloud/cloud_registered',
            qos_profile=qos_profile_sensor_data)

        self.sub_mask = message_filters.Subscriber(
            self, Image,
            topics.topic_names["segmented_image"],
            qos_profile=qos_be)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_pc, self.sub_mask], queue_size=2, slop=0.05)
        self.ts.registerCallback(self.sync_callback)

        self.pub_pc = self.create_publisher(
            PointCloud2, '/zed/pointcloud/road_boundaries', 1)

        self._axis_offset = None  # cached after first message
        self.get_logger().info(
            f"Boundary masker ready — spoofing axis '{SPOOF_AXIS}' = {SPOOF_VALUE}m")

    def sync_callback(self, pc_msg: PointCloud2, mask_msg: Image):
        if pc_msg.height == 1:
            self.get_logger().warn_once("Unordered point cloud — skipping")
            return

        # Cache field offset
        if self._axis_offset is None:
            self._axis_offset = next(
                (f.offset for f in pc_msg.fields if f.name == SPOOF_AXIS), None)
            if self._axis_offset is None:
                self.get_logger().error(
                    f"Field '{SPOOF_AXIS}' not found in point cloud. "
                    f"Available: {[f.name for f in pc_msg.fields]}")
                return

        cv_mask = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
        if cv_mask.shape != (pc_msg.height, pc_msg.width):
            cv_mask = cv2.resize(
                cv_mask, (pc_msg.width, pc_msg.height),
                interpolation=cv2.INTER_NEAREST)

        # Wall strip: erode road inward so wall sits inside road edge
        kernel = np.ones((BOUNDARY_KERNEL_PX, BOUNDARY_KERNEL_PX), np.uint8)
        road_eroded = cv2.erode(cv_mask, kernel, iterations=BOUNDARY_ITERATIONS)
        boundary_pixels = (cv_mask == 255) & (road_eroded == 0)

        cloud_data = np.frombuffer(pc_msg.data, dtype=np.uint8).copy()
        axis_view = np.ndarray(
            shape=(pc_msg.height, pc_msg.width),
            dtype=np.float32,
            buffer=cloud_data,
            offset=self._axis_offset,
            strides=(pc_msg.row_step, pc_msg.point_step))

        # Only spoof where depth is valid — NaN points won't mark anyway
        valid = np.isfinite(axis_view)
        spoof = boundary_pixels & valid
        axis_view[spoof] = SPOOF_VALUE

        pc_msg.data = cloud_data.tobytes()
        self.pub_pc.publish(pc_msg)


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

if __name__ == '__main__':
    main()
