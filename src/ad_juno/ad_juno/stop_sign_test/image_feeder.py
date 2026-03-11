import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageFeeder(Node):
    def __init__(self):
        super().__init__('image_feeder')
        # Send image to ROS topic
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0) # 0: Webcam, "video.mp4" : Video

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    feeder = ImageFeeder()
    # Saniyede 30 kare gönder
    timer_period = 1.0 / 30.0 
    feeder.create_timer(timer_period, feeder.publish_frame)
    rclpy.spin(feeder)
    feeder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()