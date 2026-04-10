#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

PIXEL_X = 100
PIXEL_Y = 50
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
MM_TO_M = 0.001


class DepthSensorNode(Node):
    def __init__(self):
        super().__init__("depth_sensor")
        self.bridge = CvBridge()

        # Create the subscription with a QoS profile depth of 10
        self.subscription = self.create_subscription(
            Image, DEPTH_TOPIC, self.callback, 10
        )
        self.get_logger().info("Depth sensor node initialized.")

    def convert_depth_image(self, data):
        return self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def extract_depth_at_pixel(self, depth_image, pixel_x=PIXEL_X, pixel_y=PIXEL_Y):
        depth_mm = depth_image[pixel_y][pixel_x]
        return depth_mm * MM_TO_M

    def format_depth_message(self, pixel_x, pixel_y, depth_m):
        return "Distance of pixel ({},{}): {:.3f} meters".format(
            pixel_x, pixel_y, depth_m
        )

    def process_depth_message(self, data, pixel_x=PIXEL_X, pixel_y=PIXEL_Y):
        depth_image = self.convert_depth_image(data)
        depth_m = self.extract_depth_at_pixel(depth_image, pixel_x, pixel_y)
        return self.format_depth_message(pixel_x, pixel_y, depth_m)

    def callback(self, data):
        message = self.process_depth_message(data)
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)

    node = DepthSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

