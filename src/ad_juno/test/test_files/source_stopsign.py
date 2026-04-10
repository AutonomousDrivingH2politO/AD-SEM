#!/usr/bin/env python3
import cv2


WINDOW_NAME = "Stop Sign Camera Test"
CAMERA_INDEX = 4
WAIT_KEY_DELAY_MS = 1
QUIT_KEY = ord("q")


def create_video_capture(camera_index: int = CAMERA_INDEX):
    return cv2.VideoCapture(camera_index)


def read_frame(video_capture):
    return video_capture.read()


def process_frame(frame, analysis_fn=None):
    """
    Run the two-stage stop-sign analysis on a frame.

    If analysis_fn is not provided, the function performs a lazy import.
    This avoids loading heavy model code during unit tests.

    Returns:
        detected (bool)
        output_frame
        details (dict)
    """
    if analysis_fn is None:
        from shared_objects.stop_utils_new_v1 import analysis
        analysis_fn = analysis
    return analysis_fn(frame)


def build_detection_messages(details, detected: bool):
    messages = []

    confirmed = details.get("confirmed", [])
    rejected = details.get("rejected", [])

    if detected:
        for det in confirmed:
            x1, y1, x2, y2 = [int(c) for c in det["box"]]
            detector_conf = det.get("detector_conf", -1.0)

            if "verifier_conf" in det:
                messages.append(
                    f"✅ STOP SIGN DETECTED! "
                    f"Box=({x1},{y1},{x2},{y2}) "
                    f"D_conf={detector_conf:.2f} "
                    f"V_conf={det['verifier_conf']:.2f} "
                    f"→ Successful at Stage1 and Stage2"
                )
            elif "classifier_score" in det:
                messages.append(
                    f"✅ STOP SIGN DETECTED! "
                    f"Box=({x1},{y1},{x2},{y2}) "
                    f"D_conf={detector_conf:.2f} "
                    f"C_score={det['classifier_score']:.2f} "
                    f"→ Successful at Stage1 and Stage2"
                )
            else:
                messages.append(
                    f"✅ STOP SIGN DETECTED! "
                    f"Box=({x1},{y1},{x2},{y2}) "
                    f"D_conf={detector_conf:.2f}"
                )

    elif rejected:
        for det in rejected:
            x1, y1, x2, y2 = [int(c) for c in det["box"]]
            detector_conf = det.get("detector_conf", -1.0)
            messages.append(
                f"⚠️ Stage1 passed but Stage2 failed. "
                f"Box=({x1},{y1},{x2},{y2}) "
                f"D_conf={detector_conf:.2f}"
            )
    else:
        messages.append("❌ No stop sign.")

    return messages


def print_detection_messages(messages):
    for msg in messages:
        print(msg)


def show_frame(frame, window_name: str = WINDOW_NAME):
    cv2.imshow(window_name, frame)


def should_quit(wait_key_delay_ms: int = WAIT_KEY_DELAY_MS, quit_key: int = QUIT_KEY):
    return cv2.waitKey(wait_key_delay_ms) & 0xFF == quit_key


def release_resources(video_capture):
    video_capture.release()
    cv2.destroyAllWindows()


def run_camera_loop(camera_index: int = CAMERA_INDEX, analysis_fn=None):
    video = create_video_capture(camera_index)

    if not video.isOpened():
        print(f"Failed to open camera index {camera_index}")
        release_resources(video)
        return

    try:
        while True:
            ret, frame = read_frame(video)

            if not ret:
                print("Failed to grab frame")
                break

            detected, output_frame, details = process_frame(frame, analysis_fn=analysis_fn)
            messages = build_detection_messages(details, detected)
            print_detection_messages(messages)
            show_frame(output_frame, WINDOW_NAME)

            if should_quit():
                break
    finally:
        release_resources(video)


def main():
    run_camera_loop(CAMERA_INDEX)


if __name__ == "__main__":
    main()