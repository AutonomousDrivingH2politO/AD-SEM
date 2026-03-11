import cv2
from ultralytics import YOLO
import time

# 1. Modeli Yükle (Elindeki en güncel olanı yaz, örn: yolo12x.pt)
model = YOLO("yolov8s.pt") 
if model is None:
    print("Model yüklenemedi. Lütfen model dosyasını kontrol edin.")
    exit(1)

# 2. Test Kaynağı (0 yazarsan webcam açılır, "video.mp4" yazarsan videoyu okur)
source = "stopsign2.mp4"  # Değiştirilebilir: "0" veya başka bir video dosyası
cap = cv2.VideoCapture(source)

print("Test başlıyor... Çıkmak için 'q' tuşuna basın.")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Zamanı ölçmeye başla
    start_time = time.time()

    # Model tahmini yap (Confidence 0.5 yapalım ki emin olduklarını göstersin)
    results = model.predict(frame, conf=0.5, verbose=False)

    # Geçen süreyi hesapla
    end_time = time.time()
    fps = 1 / (end_time - start_time)

    # Sonuçları ekrana çizdir
    annotated_frame = results[0].plot()
    
    # Sol üste FPS yazdır
    cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (20, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("YOLO Saf Performans Testi", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()