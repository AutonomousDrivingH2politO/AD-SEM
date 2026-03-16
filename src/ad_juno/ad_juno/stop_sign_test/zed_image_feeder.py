import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix # RTK için NavSatFix eklendi
from cv_bridge import CvBridge
import pyzed.sl  as sl # ZED SDK
import cv2

class ZedImageFeeder(Node):
    def __init__(self):
        super().__init__('zed_image_feeder')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        # RTK/GPS verisi için yeni bir publisher (isteğe bağlı)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.bridge = CvBridge()

        # 1. ZED Kamera Ayarları
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.sdk_verbose = True
        
        # 2. Kamerayı Aç
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("ZED Kamera açılamadı!")
            exit()

        self.image_zed = sl.Mat()
        self.sensors_data = sl.SensorsData() # RTK/Sensör verileri için

        # Saniyede 30 kare gönder
        self.timer = self.create_timer(1.0 / 30.0, self.publish_data)

    def publish_data(self):
        # Kareyi yakala
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Görüntüyü al ve publish et
            self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
            frame = self.image_zed.get_data()
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgra8") # ZED genelde 4 kanallı (BGRA) döner
            self.publisher_.publish(img_msg)

            # 3. RTK / Sensör Verilerini Al (ZED üzerinde RTK/GPS varsa)
            self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT)
            gnss_data = self.sensors_data.get_gnss_data()
            
            if gnss_data.status == sl.GNSS_STATUS.OK:
                gps_msg = NavSatFix()
                gps_msg.latitude = gnss_data.location.latitude
                gps_msg.longitude = gnss_data.location.longitude
                gps_msg.altitude = gnss_data.location.altitude
                self.gps_pub.publish(gps_msg)

    def __del__(self):
        self.zed.close()

def main():
    rclpy.init()
    node = ZedImageFeeder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()