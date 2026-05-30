#!/usr/bin/env python3

import os
import sys
import cv2
import rclpy
from rclpy.node import Node
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy,
    HistoryPolicy,
)

from shared_objects.stop_utils_new_v1 import analysis

ZED2_TOPIC = '/zed/zed_node/rgb/color/rect/image'

QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
"""
class StopSignTestNode(Node):
    def __init__(self):
        super().__init__('stop_sign_test_node')
        self.bridge = CvBridge()

        self.create_subscription(Image, ZED2_TOPIC, self.image_callback, QOS)

        self.get_logger().info(f"Listening: {ZED2_TOPIC}")
        self.get_logger().info("Started. Ctrl+C to stop.")

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # YOLO analizine gönder
        detected, output_frame, details = analysis(img)

        confirmed = details['confirmed']

        # ── YENİ EKLENEN: Görsel Hata Ayıklama Çizgisi (ROI Sınırı) ──
        # Sahada %60'lık kesimin nereye denk geldiğini canlı görmek için
        h, w = output_frame.shape[:2]
        crop_y2 = int(h * 0.60)
        
        # Sınır çizgisini çiz (Mavi renkli)
        cv2.line(output_frame, (0, crop_y2), (w, crop_y2), (255, 0, 0), 2)
        cv2.putText(output_frame, "YOLO ISLEM SINIRI (Bu cizginin alti çöpe atiliyor)", 
                    (10, crop_y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        if detected:
            for det in confirmed:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                self.get_logger().info(
                    f"✅ STOP SIGN! Box=({x1},{y1},{x2},{y2}) "
                    f"C={det['conf']:.2f}"
                )
        else:
            self.get_logger().info("❌ No stop sign.")

        cv2.imshow('Stop Sign Test', output_frame)
        cv2.waitKey(1)


"""


# --- KAYIT KLASÖRLERİ (Masaüstüne kaydeder) ---
DESKTOP       = os.path.expanduser("~/Desktop")
CONFIRMED_DIR = os.path.join(DESKTOP, "stop_sign_detections", "full_frames")
PROCESSED_DIR = os.path.join(DESKTOP, "stop_sign_detections", "processed_only")

class StopSignTestNode(Node):
    def __init__(self):
        super().__init__('stop_sign_test_node')
        self.bridge = CvBridge()
        self.counter = 0

        # Klasörleri otomatik oluştur
        os.makedirs(CONFIRMED_DIR, exist_ok=True)
        os.makedirs(PROCESSED_DIR, exist_ok=True)

        self.create_subscription(Image, ZED2_TOPIC, self.image_callback, QOS)

        self.get_logger().info(f"Listening: {ZED2_TOPIC}")
        self.get_logger().info(f"Tam kareler şuraya kaydediliyor: {CONFIRMED_DIR}")
        self.get_logger().info(f"İşlenen (%60) kısımlar şuraya: {PROCESSED_DIR}")
        self.get_logger().info("Started. Ctrl+C to stop.")

    def _ts(self):
        # Dosya isimlerinin benzersiz olması için saat, dakika, saniye ve milisaniye
        return datetime.now().strftime("%H%M%S_%f")[:10] 

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # YOLO analizine gönder
        detected, output_frame, details = analysis(img)
        confirmed = details['confirmed']

        # --- GÖRSEL HATA AYIKLAMA (Mavi Çizgi) ---
        h, w = output_frame.shape[:2]
        crop_y2 = int(h * 0.60)
        
        # Mavi çizgiyi çiz
        cv2.line(output_frame, (0, crop_y2), (w, crop_y2), (255, 0, 0), 2)
        cv2.putText(output_frame, "YOLO ISLEM SINIRI", 
                    (10, crop_y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # SADECE YOLO'nun işlediği (kırpılmış üst %60) alanı resim olarak ayır
        processed_part = output_frame[0:crop_y2, 0:w]

        ts = self._ts()

        if detected:
            for det in confirmed:
                x1, y1, x2, y2 = [int(c) for c in det['box']]
                self.get_logger().info(
                    f"✅ STOP SIGN! Box=({x1},{y1},{x2},{y2}) C={det['conf']:.2f}"
                )
            
            # --- KAYDETME İŞLEMİ ---
            # 1. Tam kareyi (mavi çizgili ve kutulu haliyle genel perspektif için) kaydet
            full_path = os.path.join(CONFIRMED_DIR, f"{ts}_{self.counter:04d}_full.jpg")
            cv2.imwrite(full_path, output_frame)

            # 2. Sadece işlenen kısmı (YOLO'nun gözünden görmek için) kaydet
            proc_path = os.path.join(PROCESSED_DIR, f"{ts}_{self.counter:04d}_processed.jpg")
            cv2.imwrite(proc_path, processed_part)
            
            self.counter += 1
        else:
            self.get_logger().info("❌ No stop sign.")

        # Ekranda canlı göster
        cv2.imshow('Stop Sign Test', output_frame)
        cv2.waitKey(1)



def main():
    rclpy.init(args=sys.argv)
    node = StopSignTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
