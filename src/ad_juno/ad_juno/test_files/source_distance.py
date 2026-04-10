#!/usr/bin/env python

import rospy
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


def callback(data, bridge=None, pixel_x=PIXEL_X, pixel_y=PIXEL_Y):
    if bridge is None:
        bridge = CvBridge()
    message = process_depth_message(data, bridge, pixel_x, pixel_y)
    print(message)


def main():
    rospy.Subscriber(
        DEPTH_TOPIC,
        Image,
        callback,
    )
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('depth_sensor', anonymous=True)
    main()