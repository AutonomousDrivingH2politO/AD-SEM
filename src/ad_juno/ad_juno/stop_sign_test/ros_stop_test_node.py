import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class SimpleStopNode(Node):
    def __init__(self):
        super().__init__('simple_stop_node')
        
        # Modeli yükle
        self.model = YOLO("yolo12x.pt")
        self.bridge = CvBridge()
        
        # Parametre (Liderinin istediği 10 saniye)
        self.declare_parameter('wait_duration', 10.0)
        self.wait_duration = self.get_parameter('wait_duration').value
        
        self.is_waiting = False
        self.timer = None

        # Kamera görüntüsünü dinle (Topic adını ekibine göre kontrol et)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Fren durumunu yayınla
        self.stop_pub = self.create_publisher(Bool, '/stop_status', 10)
        
        self.get_logger().info("ROS Stop Test Node Başlatıldı!")

    def image_callback(self, msg):
        # Eğer zaten 10 saniyelik bekleme içindeysek işlem yapma
        if self.is_waiting:
            return

        # Görüntüyü OpenCV formatına çevir
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # YOLO ile tara
        results = self.model.predict(frame, conf=0.6, verbose=False)
        
        stop_sign_detected = False
        for result in results:
            for box in result.boxes:
                # YOLO sınıflarında 'stop sign' genellikle 11. indekstir
                if int(box.cls[0]) == 11: 
                    stop_sign_detected = True
                    break

        if stop_sign_detected:
            self.trigger_stop()

    def trigger_stop(self):
        self.is_waiting = True
        self.get_logger().info("DUR TABELASI GÖRDÜM! Fren yapılıyor...")
        
        # Fren mesajı gönder
        self.stop_pub.publish(Bool(data=True))
        
        # 10 Saniyelik Timer Başlat (Liderinin commit mantığı)
        self.timer = self.create_timer(self.wait_duration, self.resume_driving)

    def resume_driving(self):
        self.is_waiting = False
        self.get_logger().info(f"{self.wait_duration} saniye doldu. Devam ediliyor.")
        
        # Freni bırak
        self.stop_pub.publish(Bool(data=False))
        
        # Timer'ı kapat
        if self.timer:
            self.timer.cancel()
            self.timer = None

def main():
    rclpy.init()
    node = SimpleStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()