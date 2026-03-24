'''#This module provides functions for detecting and verifying stop signs in images using a two-stage approach.
#The first stage uses a detector model to find potential stop sign candidates, and the second stage uses a segmentaion model to verify those candidates.


import cv2
import numpy as np
import torch
import time
from ultralytics import YOLO
import torchvision # For NMS

# --- Configuration ---
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Stop Utils - Using device: {DEVICE}")

SHOW_INTERNAL_VISUALS = True

# --- Stage 1: Detector Model ---
DETECTOR_MODEL_PATH = "/home/bafe/Desktop/ROS2_training/weights (8).pt"
DETECTOR_CONF_THRESHOLD = 0.20
DETECTOR_STOP_SIGN_CLASS_ID = 0 # Assuming 'stop sign' is class ID 11 in the detector model
INFERENCE_SIZE_DETECTOR = (640, 640)

# --- Stage 2: Verifier Model ---
VERIFIER_MODEL_PATH = "/home/bafe/Desktop/ROS2_training/yolo12x.pt"
VERIFIER_CONF_THRESHOLD = 0.50
VERIFIER_STOP_SIGN_CLASS_NAME = "stop sign"

# --- Slicing Configuration ---
SLICE_ROWS = 2
SLICE_COLS = 4
OVERLAP_RATIO = 0.1

# --- NMS ---
NMS_IOU_THRESHOLD = 0.45

# --- Visual Alarm Parameters ---
ALARM_COLOR = (0, 0, 255)
ALARM_THICKNESS = 10
ALARM_FONT = cv2.FONT_HERSHEY_SIMPLEX
ALARM_FONT_SCALE = 1.0
ALARM_TEXT = "STOP SIGN!"

# --- Load Models ---
detector_model = YOLO(DETECTOR_MODEL_PATH).to(DEVICE)
verifier_model = None
verifier_stop_sign_id = None

try:
    print(f"Loading detector model from: {DETECTOR_MODEL_PATH}")
    print(f"Detector model '{DETECTOR_MODEL_PATH}' loaded successfully.")
    if not hasattr(detector_model, 'names') or not detector_model.names:
        detector_model.names = {DETECTOR_STOP_SIGN_CLASS_ID: VERIFIER_STOP_SIGN_CLASS_NAME}
    elif DETECTOR_STOP_SIGN_CLASS_ID not in detector_model.names:
        print(f"Warning: Detector class ID {DETECTOR_STOP_SIGN_CLASS_ID} not in names.")

    print(f"Loading verifier model from: {VERIFIER_MODEL_PATH}")
    verifier_model = YOLO(VERIFIER_MODEL_PATH).to(DEVICE)
    print(f"Verifier model '{VERIFIER_MODEL_PATH}' loaded successfully.")
    try:
        verifier_stop_sign_id = [k for k, v in verifier_model.names.items() if v.lower() == VERIFIER_STOP_SIGN_CLASS_NAME.lower()][0]
        print(f"Verifier '{VERIFIER_STOP_SIGN_CLASS_NAME}' class ID: {verifier_stop_sign_id}")
    except IndexError:
        print(f"ERROR: Class '{VERIFIER_STOP_SIGN_CLASS_NAME}' not in verifier names: {verifier_model.names}")
        verifier_model = None
    except Exception as e:
        print(f"Error loading verifier model names: {e}")
        verifier_model = None
except Exception as e:
    print(f"Error loading models: {e}")


def slice_frame_new(frame, rows, cols, overlap_ratio):
    height, width = frame.shape[:2]
    if height == 0 or width == 0: return []
    slice_height_base = height // rows
    slice_width_base = width // cols
    overlap_h = int(slice_height_base * overlap_ratio)
    overlap_w = int(slice_width_base * overlap_ratio)
    slices_meta = []
    for i in range(rows):
        for j in range(cols):
            y_start_slice, y_end_slice = i * slice_height_base, (i + 1) * slice_height_base
            x_start_slice, x_end_slice = j * slice_width_base, (j + 1) * slice_width_base
            y_start_overlap, y_end_overlap = max(0, y_start_slice - overlap_h // 2), min(height, y_end_slice + overlap_h // 2)
            x_start_overlap, x_end_overlap = max(0, x_start_slice - overlap_w // 2), min(width, x_end_slice + overlap_w // 2)
            if i == 0: y_start_overlap = 0
            if i == rows - 1: y_end_overlap = height
            if j == 0: x_start_overlap = 0
            if j == cols - 1: x_end_overlap = width
            sliced_img = frame[y_start_overlap:y_end_overlap, x_start_overlap:x_end_overlap]
            if sliced_img.size == 0 or sliced_img.shape[0] < 10 or sliced_img.shape[1] < 10: continue
            slices_meta.append({
                'image': sliced_img,
                'coords_on_original': (x_start_overlap, y_start_overlap, x_end_overlap, y_end_overlap)
            })
    return slices_meta


def analysis(frame_original):
    """
    Returns: (bool: stop_sign_found, frame_output, details)
    details = {
        'confirmed': [ {box, detector_conf, verifier_conf} ],   # passed both stages
        'rejected':  [ {box, detector_conf} ],                   # passed stage1, failed stage2
    }
    """
    empty_details = {'confirmed': [], 'rejected': []}

    if detector_model is None or verifier_model is None or verifier_stop_sign_id is None:
        return False, frame_original, empty_details

    output_frame = frame_original
    original_height, original_width = frame_original.shape[:2]

    slices_meta = slice_frame_new(frame_original, SLICE_ROWS, SLICE_COLS, OVERLAP_RATIO)
    batch_slice_images, valid_slices_meta = [], []
    for s_meta in slices_meta:
        if s_meta['image'] is not None and s_meta['image'].shape[0] > 0 and s_meta['image'].shape[1] > 0:
            batch_slice_images.append(s_meta['image'])
            valid_slices_meta.append(s_meta)
    if not batch_slice_images:
        return False, frame_original, empty_details

    # --- Stage 1: Detector ---
    detector_results = detector_model.predict(batch_slice_images, imgsz=INFERENCE_SIZE_DETECTOR,
                                              conf=DETECTOR_CONF_THRESHOLD, classes=[DETECTOR_STOP_SIGN_CLASS_ID],
                                              device=DEVICE, verbose=False)
    all_boxes_coords_nms, all_scores_nms, all_info_nms = [], [], []
    for i, res_slice in enumerate(detector_results):
        meta = valid_slices_meta[i]
        sx, sy, _, _ = meta['coords_on_original']
        for box in res_slice.boxes:
            x1s, y1s, x2s, y2s = box.xyxy[0].cpu().numpy().astype(int)
            x1f = max(0, x1s + sx)
            y1f = max(0, y1s + sy)
            x2f = min(original_width, x2s + sx)
            y2f = min(original_height, y2s + sy)
            if x2f > x1f and y2f > y1f:
                all_boxes_coords_nms.append([x1f, y1f, x2f, y2f])
                all_scores_nms.append(box.conf.item())
                all_info_nms.append({'box_on_full_frame': [x1f, y1f, x2f, y2f], 'detector_conf': box.conf.item()})

    potential_detections = []
    if all_boxes_coords_nms:
        boxes_t = torch.tensor(all_boxes_coords_nms, dtype=torch.float32, device=DEVICE)
        scores_t = torch.tensor(all_scores_nms, dtype=torch.float32, device=DEVICE)
        indices = torchvision.ops.nms(boxes_t, scores_t, NMS_IOU_THRESHOLD).cpu().numpy()
        for idx in indices:
            potential_detections.append(all_info_nms[idx])

    # --- Stage 2: Verifier ---
    confirmed_stop_signs_details = []
    rejected_detections = []
    rois, rois_orig_info = [], []

    if potential_detections:
        for det_info in potential_detections:
            x1, y1, x2, y2 = [int(c) for c in det_info['box_on_full_frame']]
            roi = frame_original[y1:y2, x1:x2]
            if roi.size > 0 and roi.shape[0] > 10 and roi.shape[1] > 10:
                rois.append(roi)
                rois_orig_info.append(det_info)

    if rois:
        verifier_results = verifier_model.predict(rois, conf=VERIFIER_CONF_THRESHOLD,
                                                  classes=[verifier_stop_sign_id], device=DEVICE, verbose=False)
        for i, res_roi in enumerate(verifier_results):
            orig_info = rois_orig_info[i]
            max_v_conf = 0.0
            found_in_roi = False
            for box_v in res_roi.boxes:
                found_in_roi = True
                max_v_conf = max(max_v_conf, box_v.conf.item())
                break
            if max_v_conf >= VERIFIER_CONF_THRESHOLD:
                confirmed_stop_signs_details.append({
                    'box': orig_info['box_on_full_frame'],
                    'detector_conf': orig_info['detector_conf'],
                    'verifier_conf': max_v_conf
                })
            else:
                rejected_detections.append({
                    'box': orig_info['box_on_full_frame'],
                    'detector_conf': orig_info['detector_conf']
                })

    stop_sign_found_this_frame = len(confirmed_stop_signs_details) > 0

    if stop_sign_found_this_frame and SHOW_INTERNAL_VISUALS:
        output_frame = frame_original.copy()
        for det in confirmed_stop_signs_details:
            x1, y1, x2, y2 = [int(c) for c in det['box']]
            cv2.rectangle(output_frame, (x1, y1), (x2, y2), ALARM_COLOR, 2)
            label = f"D:{det['detector_conf']:.2f} V:{det['verifier_conf']:.2f}"
            cv2.putText(output_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ALARM_COLOR, 2)

        h_alarm, w_alarm = output_frame.shape[:2]
        cv2.rectangle(output_frame, (0, 0), (w_alarm - 1, h_alarm - 1), ALARM_COLOR, ALARM_THICKNESS)
        text_size_alarm = cv2.getTextSize(ALARM_TEXT, ALARM_FONT, ALARM_FONT_SCALE, 2)[0]
        text_x = (w_alarm - text_size_alarm[0]) // 2
        text_y = ALARM_THICKNESS + text_size_alarm[1] + 10
        cv2.putText(output_frame, ALARM_TEXT, (text_x, text_y),
                    ALARM_FONT, ALARM_FONT_SCALE, (255, 255, 255), 2, cv2.LINE_AA)

    details = {
        'confirmed': confirmed_stop_signs_details,
        'rejected': rejected_detections
    }

    return stop_sign_found_this_frame, output_frame, details
'''

# Stop Sign Detection and Verification Utilities
# This module provides functions for detecting and verifying stop signs in images using a two-stage approach.
# The first stage uses a detector model to find potential stop sign candidates, and the second stage uses a classifier model to verify those candidates.
import cv2
import numpy as np
import torch
import torchvision  # For NMS
from torchvision import transforms, models
import torch.nn as nn
from ultralytics import YOLO

# --- Configuration ---
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Stop Utils - Using device: {DEVICE}")

SHOW_INTERNAL_VISUALS = True

# --- Stage 1: Detector Model ---
DETECTOR_MODEL_PATH = "/home/bylogix/Desktop/best_detector_model.pt"
DETECTOR_CONF_THRESHOLD = 0.20
DETECTOR_STOP_SIGN_CLASS_ID = 0
INFERENCE_SIZE_DETECTOR = (1280, 1280)

# --- Stage 2: EfficientNet-B2 Classifier ---
CLASSIFIER_MODEL_PATH = "/home/bylogix/Desktop/best_classifier_model.pt"
CLASSIFIER_THRESHOLD = 0.70
CLASSIFIER_IMG_SIZE = 224

# --- Slicing Configuration ---
SLICE_ROWS = 3
SLICE_COLS = 6
OVERLAP_RATIO = 0.1

# --- NMS ---
NMS_IOU_THRESHOLD = 0.45

# --- Visual Alarm Parameters ---
ALARM_COLOR = (0, 0, 255)
ALARM_THICKNESS = 10
ALARM_FONT = cv2.FONT_HERSHEY_SIMPLEX
ALARM_FONT_SCALE = 1.0
ALARM_TEXT = "STOP SIGN!"

# Stage 1: YOLO detector
detector_model = None
try:
    print(f"Loading detector model from: {DETECTOR_MODEL_PATH}")
    detector_model = YOLO(DETECTOR_MODEL_PATH).to(DEVICE)
    print(f"Detector model loaded successfully.")
except Exception as e:
    print(f"ERROR loading detector model: {e}")

# Stage 2: EfficientNet-B2 classifier
classifier_model = None
try:
    print(f"Loading classifier model from: {CLASSIFIER_MODEL_PATH}")

    def _build_efficientnet():
        m = models.efficientnet_b2(weights=None)
        in_features = m.classifier[1].in_features
        m.classifier = nn.Sequential(
            nn.Dropout(0.3),
            nn.Linear(in_features, 1),
            nn.Sigmoid()
        )
        return m

    classifier_model = _build_efficientnet()
    classifier_model.load_state_dict(torch.load(CLASSIFIER_MODEL_PATH, map_location=DEVICE))
    classifier_model.to(DEVICE)
    classifier_model.eval()
    print(f"Classifier model loaded successfully.")
except Exception as e:
    print(f"ERROR loading classifier model: {e}")

classifier_transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((CLASSIFIER_IMG_SIZE, CLASSIFIER_IMG_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])


def slice_frame_new(frame, rows, cols, overlap_ratio):
    height, width = frame.shape[:2]
    if height == 0 or width == 0:
        return []
    slice_height_base = height // rows
    slice_width_base  = width  // cols
    overlap_h = int(slice_height_base * overlap_ratio)
    overlap_w = int(slice_width_base  * overlap_ratio)
    slices_meta = []
    for i in range(rows):
        for j in range(cols):
            y_start_slice = i * slice_height_base
            y_end_slice   = (i + 1) * slice_height_base
            x_start_slice = j * slice_width_base
            x_end_slice   = (j + 1) * slice_width_base

            y_start = max(0,      y_start_slice - overlap_h // 2)
            y_end   = min(height, y_end_slice   + overlap_h // 2)
            x_start = max(0,      x_start_slice - overlap_w // 2)
            x_end   = min(width,  x_end_slice   + overlap_w // 2)

            if i == 0:        y_start = 0
            if i == rows - 1: y_end   = height
            if j == 0:        x_start = 0
            if j == cols - 1: x_end   = width

            sliced_img = frame[y_start:y_end, x_start:x_end]
            if sliced_img.size == 0 or sliced_img.shape[0] < 10 or sliced_img.shape[1] < 10:
                continue
            slices_meta.append({
                'image': sliced_img,
                'coords_on_original': (x_start, y_start, x_end, y_end)
            })
    return slices_meta


def analysis(frame_original):
    """
    2 STAGE STOP SIGN DETECTION & VERIFICATION

    Returns:
        (bool, frame, details)
        details = {
            'confirmed': [ {box, detector_conf, classifier_score} ],
            'rejected':  [ {box, detector_conf} ],
        }
    """
    empty_details = {'confirmed': [], 'rejected': []}

    if detector_model is None or classifier_model is None:
        return False, frame_original, empty_details

    output_frame = frame_original
    original_height, original_width = frame_original.shape[:2]

    slices_meta = slice_frame_new(frame_original, SLICE_ROWS, SLICE_COLS, OVERLAP_RATIO)
    batch_slice_images = []
    valid_slices_meta  = []
    for s_meta in slices_meta:
        img = s_meta['image']
        if img is not None and img.shape[0] > 0 and img.shape[1] > 0:
            batch_slice_images.append(img)
            valid_slices_meta.append(s_meta)

    if not batch_slice_images:
        return False, frame_original, empty_details

    # Stage 1: YOLO Detector
    detector_results = detector_model.predict(
        batch_slice_images,
        imgsz=INFERENCE_SIZE_DETECTOR,
        conf=DETECTOR_CONF_THRESHOLD,
        classes=[DETECTOR_STOP_SIGN_CLASS_ID],
        device=DEVICE,
        verbose=False
    )

    all_boxes_coords = []
    all_scores       = []
    all_info         = []

    for i, res_slice in enumerate(detector_results):
        meta = valid_slices_meta[i]
        sx, sy, _, _ = meta['coords_on_original']
        for box in res_slice.boxes:
            x1s, y1s, x2s, y2s = box.xyxy[0].cpu().numpy().astype(int)
            x1f = max(0,               x1s + sx)
            y1f = max(0,               y1s + sy)
            x2f = min(original_width,  x2s + sx)
            y2f = min(original_height, y2s + sy)
            if x2f > x1f and y2f > y1f:
                all_boxes_coords.append([x1f, y1f, x2f, y2f])
                all_scores.append(box.conf.item())
                all_info.append({
                    'box_on_full_frame': [x1f, y1f, x2f, y2f],
                    'detector_conf': box.conf.item()
                })

    potential_detections = []
    if all_boxes_coords:
        boxes_t  = torch.tensor(all_boxes_coords, dtype=torch.float32, device=DEVICE)
        scores_t = torch.tensor(all_scores,       dtype=torch.float32, device=DEVICE)
        indices  = torchvision.ops.nms(boxes_t, scores_t, NMS_IOU_THRESHOLD).cpu().numpy()
        for idx in indices:
            potential_detections.append(all_info[idx])

    # Stage 2: EfficientNet-B2 Classifier
    confirmed_stop_signs = []
    rejected_detections  = []

    for det_info in potential_detections:
        x1, y1, x2, y2 = [int(c) for c in det_info['box_on_full_frame']]
        roi = frame_original[y1:y2, x1:x2]

        if roi.size == 0 or roi.shape[0] < 10 or roi.shape[1] < 10:
            rejected_detections.append({
                'box': det_info['box_on_full_frame'],
                'detector_conf': det_info['detector_conf']
            })
            continue

        roi_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        inp = classifier_transform(roi_rgb).unsqueeze(0).to(DEVICE)

        with torch.no_grad():
            score = classifier_model(inp).item()

        if score >= CLASSIFIER_THRESHOLD:
            confirmed_stop_signs.append({
                'box':              det_info['box_on_full_frame'],
                'detector_conf':    det_info['detector_conf'],
                'classifier_score': score
            })
        else:
            rejected_detections.append({
                'box':           det_info['box_on_full_frame'],
                'detector_conf': det_info['detector_conf']
            })

    stop_sign_found = len(confirmed_stop_signs) > 0

    if stop_sign_found and SHOW_INTERNAL_VISUALS:
        output_frame = frame_original.copy()
        for det in confirmed_stop_signs:
            x1, y1, x2, y2 = [int(c) for c in det['box']]
            cv2.rectangle(output_frame, (x1, y1), (x2, y2), ALARM_COLOR, 2)
            label = f"D:{det['detector_conf']:.2f} C:{det['classifier_score']:.2f}"
            cv2.putText(output_frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, ALARM_COLOR, 2)

        h, w = output_frame.shape[:2]
        cv2.rectangle(output_frame, (0, 0), (w - 1, h - 1), ALARM_COLOR, ALARM_THICKNESS)
        text_size = cv2.getTextSize(ALARM_TEXT, ALARM_FONT, ALARM_FONT_SCALE, 2)[0]
        text_x = (w - text_size[0]) // 2
        text_y = ALARM_THICKNESS + text_size[1] + 10
        cv2.putText(output_frame, ALARM_TEXT, (text_x, text_y),
                    ALARM_FONT, ALARM_FONT_SCALE, (255, 255, 255), 2, cv2.LINE_AA)

    details = {
        'confirmed': confirmed_stop_signs,
        'rejected':  rejected_detections
    }

    return stop_sign_found, output_frame, details
