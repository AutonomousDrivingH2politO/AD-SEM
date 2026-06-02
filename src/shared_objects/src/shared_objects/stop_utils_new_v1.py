'''
# Stop Sign Detection Utilities
# Single model: YOLOv12m (COCO class 11)
# Slicing 
#Ignore ground
import os
import cv2
import numpy as np
import torch
import torchvision
from ultralytics import YOLO

DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Stop Utils - Using device: {DEVICE}")

SHOW_INTERNAL_VISUALS = True

DESKTOP        = os.path.expanduser("~/Desktop")
MODEL_PATH     = os.path.join(DESKTOP, "models", "yolo12m.pt")
CONF_THRESHOLD = 0.40
STOP_CLASS_ID  = 11   # COCO stop sign

# --- Slicing ---
SLICE_ROWS    = 2     
SLICE_COLS    = 6
OVERLAP_RATIO = 0.15 

# --- NMS ---
NMS_IOU_THRESHOLD = 0.45

# --- Geometry Filters ---
MIN_ROI_SIDE       = 10   
MIN_ASPECT_RATIO   = 0.50
MAX_ASPECT_RATIO   = 1.60
MAX_BOX_AREA_RATIO = 0.08

# --- Görsel Alarm ---
ALARM_COLOR      = (0, 0, 255)
ALARM_THICKNESS  = 10
ALARM_FONT       = cv2.FONT_HERSHEY_SIMPLEX
ALARM_FONT_SCALE = 1.0
ALARM_TEXT       = "STOP SIGN!"

model = None
try:
    model = YOLO(MODEL_PATH).to(DEVICE)
    print(f"Model loaded: {MODEL_PATH}")
except Exception as e:
    print(f"ERROR loading model: {e}")

def slice_frame_new(frame, rows, cols, overlap_ratio):
    height, width = frame.shape[:2]
    if height == 0 or width == 0:
        return []
    slice_h   = height // rows
    slice_w   = width  // cols
    overlap_h = int(slice_h * overlap_ratio)
    overlap_w = int(slice_w * overlap_ratio)
    slices_meta = []
    for i in range(rows):
        for j in range(cols):
            ys = max(0,      i * slice_h - overlap_h // 2)
            ye = min(height, (i+1) * slice_h + overlap_h // 2)
            xs = max(0,      j * slice_w - overlap_w // 2)
            xe = min(width,  (j+1) * slice_w + overlap_w // 2)
            if i == 0:        ys = 0
            if i == rows - 1: ye = height
            if j == 0:        xs = 0
            if j == cols - 1: xe = width
            sliced_img = frame[ys:ye, xs:xe]
            if sliced_img.size == 0 or sliced_img.shape[0] < 10 or sliced_img.shape[1] < 10:
                continue
            slices_meta.append({
                'image': sliced_img,
                'coords_on_original': (xs, ys, xe, ye)
            })
    return slices_meta

def analysis(frame_original):
    empty_details = {'confirmed': [], 'rejected': []}

    if model is None:
        return False, frame_original, empty_details

    output_frame = frame_original
    orig_h, orig_w = frame_original.shape[:2]
    frame_area = orig_w * orig_h

    crop_y2 = int(orig_h * 0.60)
    frame_cropped = frame_original[0:crop_y2, 0:orig_w]

    slices_meta = slice_frame_new(frame_cropped, SLICE_ROWS, SLICE_COLS, OVERLAP_RATIO)
    
    batch_slice_images = []
    valid_slices_meta  = []
    for s in slices_meta:
        img = s['image']
        if img is not None and img.shape[0] > 0 and img.shape[1] > 0:
            batch_slice_images.append(img)
            valid_slices_meta.append(s)
            
    if not batch_slice_images:
        return False, frame_original, empty_details

    results = model.predict(
        batch_slice_images,
        conf=CONF_THRESHOLD,
        classes=[STOP_CLASS_ID],
        device=DEVICE,
        verbose=False
    )

    all_boxes, all_scores, all_info = [], [], []
    for i, res in enumerate(results):
        sx, sy, _, _ = valid_slices_meta[i]['coords_on_original']
        for box in res.boxes:
            x1s, y1s, x2s, y2s = box.xyxy[0].cpu().numpy().astype(int)
            
            x1f = max(0,      x1s + sx)
            y1f = max(0,      y1s + sy) 
            x2f = min(orig_w, x2s + sx)
            y2f = min(orig_h, y2s + sy)
            
            if x2f > x1f and y2f > y1f:
                all_boxes.append([x1f, y1f, x2f, y2f])
                all_scores.append(box.conf.item())
                all_info.append({
                    'box': [x1f, y1f, x2f, y2f],
                    'conf': box.conf.item()
                })

    detections = []
    if all_boxes:
        boxes_t  = torch.tensor(all_boxes,  dtype=torch.float32, device=DEVICE)
        scores_t = torch.tensor(all_scores, dtype=torch.float32, device=DEVICE)
        for idx in torchvision.ops.nms(boxes_t, scores_t, NMS_IOU_THRESHOLD).cpu().numpy():
            detections.append(all_info[idx])

    confirmed = []
    for det in detections:
        x1, y1, x2, y2 = [int(c) for c in det['box']]
        roi_w = x2 - x1
        roi_h = y2 - y1

        if roi_w < MIN_ROI_SIDE or roi_h < MIN_ROI_SIDE:
            continue
        aspect_ratio = roi_w / roi_h if roi_h > 0 else 0
        if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
            continue
        if (roi_w * roi_h) / frame_area > MAX_BOX_AREA_RATIO:
            continue

        confirmed.append({'box': det['box'], 'conf': det['conf']})

    stop_sign_found = len(confirmed) > 0
    if stop_sign_found and SHOW_INTERNAL_VISUALS:
        output_frame = frame_original.copy()
        for det in confirmed:
            x1, y1, x2, y2 = [int(c) for c in det['box']]
            cv2.rectangle(output_frame, (x1, y1), (x2, y2), ALARM_COLOR, 2)
            cv2.putText(output_frame, f"C:{det['conf']:.2f}",
                        (x1, max(y1-10, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, ALARM_COLOR, 2)
        h, w = output_frame.shape[:2]
        cv2.rectangle(output_frame, (0,0), (w-1,h-1), ALARM_COLOR, ALARM_THICKNESS)
        text_size = cv2.getTextSize(ALARM_TEXT, ALARM_FONT, ALARM_FONT_SCALE, 2)[0]
        cv2.putText(output_frame, ALARM_TEXT,
                    ((w - text_size[0])//2, ALARM_THICKNESS + text_size[1] + 10),
                    ALARM_FONT, ALARM_FONT_SCALE, (255,255,255), 2, cv2.LINE_AA)

    return stop_sign_found, output_frame, {
        'confirmed': confirmed,
        'rejected':  []
    }'''


# Stop Sign Detection Utilities
# Tek model: YOLOv12m (COCO class 11)
# Ignores sky
import os
import cv2
import numpy as np
import torch
import torchvision
from ultralytics import YOLO

DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Stop Utils - Using device: {DEVICE}")

SHOW_INTERNAL_VISUALS = True

# --- Model Yol Yapılandırması (Dinamik Masaüstü Konumu) ---
DESKTOP        = os.path.expanduser("~/Desktop")
MODEL_PATH     = os.path.join(DESKTOP, "models", "yolo12m.pt")
CONF_THRESHOLD = 0.40
STOP_CLASS_ID  = 11   # COCO stop sign

# --- Slicing (Dilimleme) ---
SLICE_ROWS    = 2
SLICE_COLS    = 6
OVERLAP_RATIO = 0.15   

# --- NMS ---
NMS_IOU_THRESHOLD = 0.45

# --- Geometri Filtreleri ---
MIN_ROI_SIDE       = 10   
MIN_ASPECT_RATIO   = 0.50
MAX_ASPECT_RATIO   = 1.60
MAX_BOX_AREA_RATIO = 0.08

# --- Görsel Alarm ---
ALARM_COLOR      = (0, 0, 255)
ALARM_THICKNESS  = 10
ALARM_FONT       = cv2.FONT_HERSHEY_SIMPLEX
ALARM_FONT_SCALE = 1.0
ALARM_TEXT       = "STOP SIGN!"

model = None
try:
    model = YOLO(MODEL_PATH).to(DEVICE)
    print(f"Model loaded: {MODEL_PATH}")
except Exception as e:
    print(f"ERROR loading model: {e}")

def slice_frame_new(frame, rows, cols, overlap_ratio):
    height, width = frame.shape[:2]
    if height == 0 or width == 0:
        return []
    slice_h   = height // rows
    slice_w   = width  // cols
    overlap_h = int(slice_h * overlap_ratio)
    overlap_w = int(slice_w * overlap_ratio)
    slices_meta = []
    for i in range(rows):
        for j in range(cols):
            ys = max(0,      i * slice_h - overlap_h // 2)
            ye = min(height, (i+1) * slice_h + overlap_h // 2)
            xs = max(0,      j * slice_w - overlap_w // 2)
            xe = min(width,  (j+1) * slice_w + overlap_w // 2)
            if i == 0:        ys = 0
            if i == rows - 1: ye = height
            if j == 0:        xs = 0
            if j == cols - 1: xe = width
            sliced_img = frame[ys:ye, xs:xe]
            if sliced_img.size == 0 or sliced_img.shape[0] < 10 or sliced_img.shape[1] < 10:
                continue
            slices_meta.append({
                'image': sliced_img,
                'coords_on_original': (xs, ys, xe, ye)
            })
    return slices_meta

def analysis(frame_original):
    empty_details = {'confirmed': [], 'rejected': []}

    if model is None:
        return False, frame_original, empty_details

    output_frame = frame_original
    orig_h, orig_w = frame_original.shape[:2]
    frame_area = orig_w * orig_h


    crop_y1 = int(orig_h * 0.40)
    frame_cropped = frame_original[crop_y1:orig_h, 0:orig_w]

    slices_meta = slice_frame_new(frame_cropped, SLICE_ROWS, SLICE_COLS, OVERLAP_RATIO)
    
    batch_slice_images = []
    valid_slices_meta  = []
    for s in slices_meta:
        img = s['image']
        if img is not None and img.shape[0] > 0 and img.shape[1] > 0:
            batch_slice_images.append(img)
            valid_slices_meta.append(s)
            
    if not batch_slice_images:
        return False, frame_original, empty_details

    results = model.predict(
        batch_slice_images,
        conf=CONF_THRESHOLD,
        classes=[STOP_CLASS_ID],
        device=DEVICE,
        verbose=False
    )

    all_boxes, all_scores, all_info = [], [], []
    for i, res in enumerate(results):
        sx, sy, _, _ = valid_slices_meta[i]['coords_on_original']
        for box in res.boxes:
            x1s, y1s, x2s, y2s = box.xyxy[0].cpu().numpy().astype(int)
            
            x1f = max(0,      x1s + sx)
            y1f = max(0,      y1s + sy + crop_y1) 
            x2f = min(orig_w, x2s + sx)
            y2f = min(orig_h, y2s + sy + crop_y1)
            
            if x2f > x1f and y2f > y1f:
                all_boxes.append([x1f, y1f, x2f, y2f])
                all_scores.append(box.conf.item())
                all_info.append({
                    'box': [x1f, y1f, x2f, y2f],
                    'conf': box.conf.item()
                })

    detections = []
    if all_boxes:
        boxes_t  = torch.tensor(all_boxes,  dtype=torch.float32, device=DEVICE)
        scores_t = torch.tensor(all_scores, dtype=torch.float32, device=DEVICE)
        for idx in torchvision.ops.nms(boxes_t, scores_t, NMS_IOU_THRESHOLD).cpu().numpy():
            detections.append(all_info[idx])

    confirmed = []
    for det in detections:
        x1, y1, x2, y2 = [int(c) for c in det['box']]
        roi_w = x2 - x1
        roi_h = y2 - y1

        if roi_w < MIN_ROI_SIDE or roi_h < MIN_ROI_SIDE:
            continue
        aspect_ratio = roi_w / roi_h if roi_h > 0 else 0
        if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
            continue
        if (roi_w * roi_h) / frame_area > MAX_BOX_AREA_RATIO:
            continue

        confirmed.append({'box': det['box'], 'conf': det['conf']})

    stop_sign_found = len(confirmed) > 0
    if stop_sign_found and SHOW_INTERNAL_VISUALS:
        output_frame = frame_original.copy()
        for det in confirmed:
            x1, y1, x2, y2 = [int(c) for c in det['box']]
            cv2.rectangle(output_frame, (x1, y1), (x2, y2), ALARM_COLOR, 2)
            cv2.putText(output_frame, f"C:{det['conf']:.2f}",
                        (x1, max(y1-10, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, ALARM_COLOR, 2)
        h, w = output_frame.shape[:2]
        cv2.rectangle(output_frame, (0,0), (w-1,h-1), ALARM_COLOR, ALARM_THICKNESS)
        text_size = cv2.getTextSize(ALARM_TEXT, ALARM_FONT, ALARM_FONT_SCALE, 2)[0]
        cv2.putText(output_frame, ALARM_TEXT,
                    ((w - text_size[0])//2, ALARM_THICKNESS + text_size[1] + 10),
                    ALARM_FONT, ALARM_FONT_SCALE, (255,255,255), 2, cv2.LINE_AA)

    return stop_sign_found, output_frame, {
        'confirmed': confirmed,
        'rejected':  []
    }