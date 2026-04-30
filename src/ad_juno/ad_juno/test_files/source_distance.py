#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


PIXEL_X = 100
PIXEL_Y = 50
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
MM_TO_M = 0.001


def convert_depth_image(data, bridge):
    return bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")


def extract_depth_at_pixel(depth_image, pixel_x=PIXEL_X, pixel_y=PIXEL_Y):
    depth_mm = depth_image[pixel_y][pixel_x]
    return depth_mm * MM_TO_M


def format_depth_message(pixel_x, pixel_y, depth_m):
    return "Distanza del pixel ({},{}): {:.3f} metri".format(pixel_x, pixel_y, depth_m)


def process_depth_message(data, bridge, pixel_x=PIXEL_X, pixel_y=PIXEL_Y):
    depth_image = convert_depth_image(data, bridge)
    depth_m = extract_depth_at_pixel(depth_image, pixel_x, pixel_y)
    return format_depth_message(pixel_x, pixel_y, depth_m)


class DepthSensorNode(Node):
    def __init__(self):
        super().__init__('depth_sensor')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.callback,
            10,
        )

    def callback(self, data):
        try:
            message = process_depth_message(data, self.bridge, PIXEL_X, PIXEL_Y)
            self.get_logger().info(message)
        except Exception as e:
            self.get_logger().error(f"Error processing depth message: {e}")


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