#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthSensorNode(Node):
    def __init__(self):
        super().__init__('depth_sensor')

        # Coordinates of the pixel of interest
        self.pixel_x = 100
        self.pixel_y = 50

        self.bridge = CvBridge()

        # Subscribe to the depth image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.callback,
            10
        )

    def callback(self, data):
        try:
            # Convert the ROS image message to an OpenCV/NumPy image
            depth_image = self.bridge.imgmsg_to_cv2(
                data,
                desired_encoding='passthrough'
            )

            # Extract the depth value of the pixel of interest
            depth = depth_image[self.pixel_y][self.pixel_x] * 0.001  # Convert from millimeters to meters

            # Print the distance of the pixel of interest
            self.get_logger().info(
                f'Distance of pixel ({self.pixel_x},{self.pixel_y}): {depth:.3f} meters'
            )

        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = DepthSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()