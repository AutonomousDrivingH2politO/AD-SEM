import cv2
import numpy as np
import torch
import time
from ultralytics import YOLO
import torchvision # For NMS

# --- Configuration ---
# Device Configuration
# Prioritize CUDA for Jetson Nano, fallback to CPU if not available
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Stop Utils - Using device: {DEVICE}")

# --- Stage 1: Detector Model ---
DETECTOR_MODEL_PATH = "weights (5).pt"
DETECTOR_CONF_THRESHOLD = 0.20
DETECTOR_STOP_SIGN_CLASS_ID = 0
INFERENCE_SIZE_DETECTOR = (640, 640) # Consider (320,320) or (416,416) for Jetson speed

# --- Stage 2: Verifier Model ---
VERIFIER_MODEL_PATH = "yolo12l.pt" # Make sure this is a valid path or model name (e.g., yolov8s.pt)
VERIFIER_CONF_THRESHOLD = 0.50
VERIFIER_STOP_SIGN_CLASS_NAME = "stop sign"

# --- Slicing Configuration ---
SLICE_ROWS = 2
SLICE_COLS = 4
OVERLAP_RATIO = 0.1

# --- NMS ---
NMS_IOU_THRESHOLD = 0.45

# --- Load Models ---
detector_model = None
verifier_model = None
verifier_stop_sign_id = None # Will be populated after loading verifier_model

try:
    print(f"Loading detector model from: {DETECTOR_MODEL_PATH}")
    detector_model = YOLO(DETECTOR_MODEL_PATH).to(DEVICE)
    print(f"Detector model '{DETECTOR_MODEL_PATH}' loaded successfully.")
    if not hasattr(detector_model, 'names') or not detector_model.names:
        print(f"Warning: Detector model names not found. Assuming class {DETECTOR_STOP_SIGN_CLASS_ID} is '{VERIFIER_STOP_SIGN_CLASS_NAME}'.")
        detector_model.names = {DETECTOR_STOP_SIGN_CLASS_ID: VERIFIER_STOP_SIGN_CLASS_NAME}
    elif DETECTOR_STOP_SIGN_CLASS_ID not in detector_model.names:
        print(f"Warning: Detector model class ID {DETECTOR_STOP_SIGN_CLASS_ID} not found in names: {detector_model.names}.")
    # print(f"Detector model classes: {detector_model.names}") # Less verbose

    print(f"Loading verifier model from: {VERIFIER_MODEL_PATH}")
    verifier_model = YOLO(VERIFIER_MODEL_PATH).to(DEVICE)
    print(f"Verifier model '{VERIFIER_MODEL_PATH}' loaded successfully.")
    try:
        # Dynamically find the class ID for "stop sign" in the verifier model
        verifier_stop_sign_id = [k for k, v in verifier_model.names.items() if v.lower() == VERIFIER_STOP_SIGN_CLASS_NAME.lower()][0]
        print(f"Verifier '{VERIFIER_STOP_SIGN_CLASS_NAME}' class ID: {verifier_stop_sign_id}")
    except IndexError:
        print(f"ERROR: Class name '{VERIFIER_STOP_SIGN_CLASS_NAME}' not found in verifier model names: {verifier_model.names}")
        verifier_model = None # Disable verifier if class not found
        print("Verifier model will be disabled.")
    except Exception as e_load_verifier_names:
        print(f"An unexpected error occurred loading verifier model names: {e_load_verifier_names}")
        verifier_model = None
        print("Verifier model will be disabled.")

except Exception as e:
    print(f"Error loading models: {e}")
    # Models will remain None, analysis will skip processing.

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
    Performs two-stage stop sign detection.
    Returns: (bool: stop_sign_found, original_input_frame, list_of_confirmed_boxes_dicts)
    Each dict in list: {'box': [x1,y1,x2,y2], 'detector_conf': float, 'verifier_conf': float}
    """
    if detector_model is None or verifier_model is None or verifier_stop_sign_id is None:
        # print("Stop_utils: One or more models not loaded. Skipping analysis.") # Can be noisy
        return False, frame_original, []

    original_height, original_width = frame_original.shape[:2]

    slices_meta = slice_frame_new(frame_original, SLICE_ROWS, SLICE_COLS, OVERLAP_RATIO)
    batch_slice_images = []
    valid_slices_meta = []
    for s_meta in slices_meta:
        if s_meta['image'] is not None and s_meta['image'].shape[0] > 0 and s_meta['image'].shape[1] > 0:
            batch_slice_images.append(s_meta['image'])
            valid_slices_meta.append(s_meta)
    
    if not batch_slice_images:
        return False, frame_original, []

    # --- Stage 1: Detector Inference ---
    detector_results = detector_model.predict(batch_slice_images,
                                              imgsz=INFERENCE_SIZE_DETECTOR,
                                              conf=DETECTOR_CONF_THRESHOLD,
                                              classes=[DETECTOR_STOP_SIGN_CLASS_ID],
                                              device=DEVICE,
                                              verbose=False)

    all_boxes_coords_for_nms = []
    all_boxes_scores_for_nms = []
    all_boxes_info_for_nms = []

    for i, result_per_slice in enumerate(detector_results):
        slice_meta = valid_slices_meta[i]
        slice_x_start, slice_y_start, _, _ = slice_meta['coords_on_original']
        for box in result_per_slice.boxes:
            x1_s, y1_s, x2_s, y2_s = box.xyxy[0].cpu().numpy().astype(int)
            x1_f, y1_f = x1_s + slice_x_start, y1_s + slice_y_start
            x2_f, y2_f = x2_s + slice_x_start, y2_s + slice_y_start
            x1_f, y1_f = max(0, x1_f), max(0, y1_f)
            x2_f, y2_f = min(original_width, x2_f), min(original_height, y2_f)
            if x2_f > x1_f and y2_f > y1_f:
                all_boxes_coords_for_nms.append([x1_f, y1_f, x2_f, y2_f])
                all_boxes_scores_for_nms.append(box.conf.item())
                all_boxes_info_for_nms.append({
                    'box_on_full_frame': [x1_f, y1_f, x2_f, y2_f],
                    'detector_conf': box.conf.item()
                })
    
    potential_detections_for_verifier = []
    if all_boxes_coords_for_nms:
        boxes_tensor = torch.tensor(all_boxes_coords_for_nms, dtype=torch.float32, device=DEVICE)
        scores_tensor = torch.tensor(all_boxes_scores_for_nms, dtype=torch.float32, device=DEVICE)
        indices = torchvision.ops.nms(boxes_tensor, scores_tensor, NMS_IOU_THRESHOLD)
        indices_cpu = indices.cpu().numpy()
        for idx in indices_cpu:
            potential_detections_for_verifier.append(all_boxes_info_for_nms[idx])
    
    # --- Stage 2: Verifier Inference ---
    confirmed_stop_sign_boxes = [] # List to store dicts of confirmed boxes
    rois_for_verifier = []
    original_box_info_for_rois = []

    if potential_detections_for_verifier:
        for det_info in potential_detections_for_verifier:
            x1, y1, x2, y2 = [int(c) for c in det_info['box_on_full_frame']]
            roi = frame_original[y1:y2, x1:x2]
            if roi.size > 0 and roi.shape[0] > 10 and roi.shape[1] > 10:
                rois_for_verifier.append(roi)
                original_box_info_for_rois.append(det_info)
    
    if rois_for_verifier:
        verifier_batch_results = verifier_model.predict(rois_for_verifier,
                                                        conf=VERIFIER_CONF_THRESHOLD,
                                                        classes=[verifier_stop_sign_id],
                                                        device=DEVICE,
                                                        verbose=False)
        for i, verifier_result_per_roi in enumerate(verifier_batch_results):
            original_info = original_box_info_for_rois[i]
            max_verifier_conf_for_this_roi = 0.0
            found_stop_in_roi_by_verifier = False
            for box_in_roi in verifier_result_per_roi.boxes:
                found_stop_in_roi_by_verifier = True
                max_verifier_conf_for_this_roi = max(max_verifier_conf_for_this_roi, box_in_roi.conf.item())
                break
            if found_stop_in_roi_by_verifier:
                confirmed_stop_sign_boxes.append({
                    'box': original_info['box_on_full_frame'], # [x1,y1,x2,y2]
                    'detector_conf': original_info['detector_conf'],
                    'verifier_conf': max_verifier_conf_for_this_roi
                })

    stop_sign_actually_found_this_frame = len(confirmed_stop_sign_boxes) > 0
    return stop_sign_actually_found_this_frame, frame_original, confirmed_stop_sign_boxes

"""
# --- Example Usage (for testing stop_utils.py directly) ---
if True:
    # --- Parameters for the test block ---
    SHOW_VISUALS = True  # Set to False to disable cv2.imshow for pure performance testing
    VIDEO_SOURCE = 0   # 0 for webcam, or "path/to/your/video.mp4"
    # VIDEO_SOURCE = "your_test_video.mp4"
    
    # Visual alarm parameters (only used in this test block)
    ALARM_COLOR = (0, 0, 255)
    ALARM_THICKNESS = 10
    ALARM_FONT = cv2.FONT_HERSHEY_SIMPLEX
    ALARM_FONT_SCALE = 1.0
    ALARM_TEXT = "STOP SIGN!"

    import os # For os.path.exists in test block
    try:
        source_int = int(VIDEO_SOURCE)
        cap = cv2.VideoCapture(source_int)
        print(f"Test: Attempting to open webcam ID: {source_int}")
    except ValueError:
        cap = cv2.VideoCapture(VIDEO_SOURCE)
        print(f"Test: Attempting to open video file: {VIDEO_SOURCE}")

    if not cap.isOpened():
        print(f"Test Error: Could not open video source '{VIDEO_SOURCE}'.")
        if isinstance(VIDEO_SOURCE, str) and not os.path.exists(VIDEO_SOURCE):
             print(f"Test Error: File not found: {VIDEO_SOURCE}")
        exit()
    print(f"Test: Successfully opened video source: {VIDEO_SOURCE}")

    if SHOW_VISUALS:
        cv2.namedWindow('Stop Utils Test Output', cv2.WINDOW_NORMAL)
    
    frame_count = 0
    total_processing_time = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            if not isinstance(VIDEO_SOURCE, int): # If it's a video file, try to loop
                print("Test: End of video, restarting.")
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = cap.read()
                if not ret: break
            else: # Webcam stream ended
                print("Test: End of webcam stream or error.")
                break
        if frame is None or frame.size == 0: continue

        loop_start_time = time.time()

        # --- Call the analysis function ---
        # It returns: (bool_detected, original_frame_unchanged, list_of_box_dicts)
        stop_sign_detected, original_frame, detected_boxes_info = analysis(frame)

        processing_time = time.time() - loop_start_time
        total_processing_time += processing_time
        frame_count += 1
        fps = 1.0 / processing_time if processing_time > 0 else 0

        if stop_sign_detected:
            print(f"Frame {frame_count}: Stop sign DETECTED! (FPS: {fps:.1f}) Boxes: {len(detected_boxes_info)}")
        # else: # Can be noisy
            # print(f"Frame {frame_count}: No stop sign. (FPS: {fps:.1f})")


        if SHOW_VISUALS:
            # Create a copy for drawing if we are showing visuals
            display_frame = original_frame.copy()

            if stop_sign_detected:
                # Draw bounding boxes for confirmed detections
                for det_info in detected_boxes_info:
                    x1, y1, x2, y2 = [int(c) for c in det_info['box']]
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), ALARM_COLOR, 2)
                    label_text = f"D:{det_info['detector_conf']:.2f} V:{det_info['verifier_conf']:.2f}"
                    cv2.putText(display_frame, label_text, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, ALARM_COLOR, 2)
                
                # Draw global alarm signal
                h_alarm, w_alarm = display_frame.shape[:2]
                cv2.rectangle(display_frame, (0,0), (w_alarm-1, h_alarm-1), ALARM_COLOR, ALARM_THICKNESS)
                text_size_alarm = cv2.getTextSize(ALARM_TEXT, ALARM_FONT, ALARM_FONT_SCALE, 2)[0]
                text_x_alarm = (w_alarm - text_size_alarm[0]) // 2
                text_y_alarm_alt = ALARM_THICKNESS + text_size_alarm[1] + 10
                cv2.putText(display_frame, ALARM_TEXT, (text_x_alarm, text_y_alarm_alt),
                           ALARM_FONT, ALARM_FONT_SCALE, (255,255,255), 2, cv2.LINE_AA)

            cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Stop Utils Test Output', display_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else: # If not showing visuals, add a manual break for non-video files after some frames for testing
            if not isinstance(VIDEO_SOURCE, int) and frame_count > 200: # e.g., process 200 frames of a video
                # print("Test: Processed 200 frames, exiting.")
                # break
                pass # Or let it run through the whole video


    if frame_count > 0:
        avg_fps_overall = frame_count / total_processing_time if total_processing_time > 0 else 0
        print(f"\nTest Summary: Processed {frame_count} frames.")
        print(f"Average FPS (overall processing): {avg_fps_overall:.2f}")

    cap.release()
    if SHOW_VISUALS:
        cv2.destroyAllWindows()
    print("Test: Exiting stop_utils test.")

"""
