
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class RoiFilter(Node):
    def __init__(self):
        super().__init__("roi_filter")

        self.declare_parameter(
            "input_topic",
            "/carla/ego/lidar/point_cloud"
        )

        input_topic = self.get_parameter(
            "input_topic"
        ).value

        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            PointCloud2,
            "/lidar/roi",
            qos_profile_sensor_data
        )

        self.counter = 0

        self.get_logger().info(
            f"Listening to {input_topic}"
        )

    def callback(self, msg):
        # Read XYZ coordinates
        xyz = point_cloud2.read_points_numpy(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=True
        )

        # Keep points inside our region
        mask = (
            (xyz[:, 0] >= 0.0) &
            (xyz[:, 0] <= 30.0) &
            (np.abs(xyz[:, 1]) <= 5.0) &
            (xyz[:, 2] >= -2.0) &
            (xyz[:, 2] <= 2.0)
        )

        filtered_xyz = xyz[mask].astype(np.float32)

        output = point_cloud2.create_cloud_xyz32(
            msg.header,
            filtered_xyz
        )

        self.pub.publish(output)

        self.counter += 1

        if self.counter % 30 == 0:
            self.get_logger().info(
                f"Input: {len(xyz)} | "
                f"Output: {len(filtered_xyz)}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = RoiFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
