
import numpy as np
import open3d as o3d
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")

        self.sub = self.create_subscription(
            PointCloud2,
            "/lidar/roi",
            self.callback,
            qos_profile_sensor_data
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2, "/lidar/obstacles",
            qos_profile_sensor_data
        )

        self.box_pub = self.create_publisher(
            MarkerArray, "/lidar/boxes", 10
        )

        self.frame_count = 0
        self.scan_parts = []
        self.scan_start_ns = None
        self.scan_window_ns = 200_000_000  # 200 ms



    def callback(self, msg):
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        points = point_cloud2.read_points_numpy(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=True
        ).astype(np.float64)

        if (
            self.scan_start_ns is None
            or stamp_ns < self.scan_start_ns
        ):
            self.scan_parts.clear()
            self.scan_start_ns = stamp_ns

        if len(points) > 0:
            self.scan_parts.append(points)

        elapsed = stamp_ns - self.scan_start_ns

        if elapsed < self.scan_window_ns:
            return

        # Merge point clouds collected during this window
        if not self.scan_parts:
            self.scan_start_ns = stamp_ns
            return

        xyz = np.concatenate(
            self.scan_parts,
            axis=0
        )

        self.scan_parts.clear()
        self.scan_start_ns = stamp_ns

        self.frame_count += 1

        if len(xyz) < 100:
            return

        # Existing Open3D processing continues below


        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(xyz)

        # Reduce point count
        cloud = cloud.voxel_down_sample(0.15)
        xyz = np.asarray(cloud.points)

        # Ground candidate points
        candidates = xyz[xyz[:, 2] < -0.5]

        if len(candidates) < 100:
            self.get_logger().warn("Not enough ground candidates")
            return

        ground_cloud = o3d.geometry.PointCloud()
        ground_cloud.points = o3d.utility.Vector3dVector(candidates)

        # RANSAC
        plane, _ = ground_cloud.segment_plane(
            distance_threshold=0.18,
            ransac_n=3,
            num_iterations=150
        )

        normal = np.array(plane[:3])
        norm = np.linalg.norm(normal)

        if abs(normal[2]) / norm < 0.9:
            self.get_logger().warn("Ground plane not horizontal")
            return

        distances = np.abs(xyz @ normal + plane[3]) / norm
        ground_mask = (
            (distances < 0.18) &
            (xyz[:, 2] < -0.5)
        )

        obstacles = xyz[~ground_mask]

        # Diagnostic information
        if self.frame_count % 25 == 0:
            self.get_logger().info(
                f"ROI: {len(xyz)} | "
                f"Ground candidates: {len(candidates)} | "
                f"Ground: {np.sum(ground_mask)} | "
                f"Non-ground: {len(obstacles)} | "
                f"Z range: [{xyz[:, 2].min():.2f}, "
                f"{xyz[:, 2].max():.2f}]"
            )


        # Publish non-ground points
        obstacle_msg = point_cloud2.create_cloud_xyz32(
            msg.header,
            obstacles.astype(np.float32)
        )
        self.cloud_pub.publish(obstacle_msg)

        # DBSCAN

        if len(obstacles) < 10:
            boxes = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            boxes.markers.append(clear)
            self.box_pub.publish(boxes)
            return

        obs_cloud = o3d.geometry.PointCloud()
        obs_cloud.points = o3d.utility.Vector3dVector(obstacles)

        labels = np.asarray(
            obs_cloud.cluster_dbscan(
                eps=0.7,
                min_points=10
            )
        )

        boxes = MarkerArray()

        # Remove boxes from the previous frame
        clear = Marker()
        clear.action = Marker.DELETEALL
        boxes.markers.append(clear)

        num_clusters = max(0, int(labels.max()) + 1) if len(labels) else 0

        for cluster_id in range(num_clusters):
            cluster = obstacles[labels == cluster_id]

            if len(cluster) < 10:
                continue

            minimum = cluster.min(axis=0)
            maximum = cluster.max(axis=0)

            center = (minimum + maximum) / 2.0
            size = maximum - minimum

            marker = Marker()
            marker.header = msg.header
            marker.ns = "lidar_obstacles"
            marker.id = cluster_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = max(float(size[0]), 0.05)
            marker.scale.y = max(float(size[1]), 0.05)
            marker.scale.z = max(float(size[2]), 0.05)

            marker.color.r = 1.0
            marker.color.g = 0.4
            marker.color.b = 0.0
            marker.color.a = 0.3

            boxes.markers.append(marker)
            self.get_logger().info(
    f"Box {cluster_id}: "
    f"x={center[0]:.2f}, "
    f"y={center[1]:.2f}, "
    f"z={center[2]:.2f} | "
    f"size={size.round(2)}"
)

        self.box_pub.publish(boxes)

        self.get_logger().info(
            f"Non-ground: {len(obstacles)} | "
            f"Clusters: {num_clusters}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
