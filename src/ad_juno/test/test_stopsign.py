#!/usr/bin/env python3
import pytest
from unittest.mock import MagicMock, patch, call

import test_sources.source_stopsign as stop_sign_camera_test

def test_constants_are_correct():
    assert stop_sign_camera_test.WINDOW_NAME == "Stop Sign Camera Test"
    assert stop_sign_camera_test.CAMERA_INDEX == 4
    assert stop_sign_camera_test.WAIT_KEY_DELAY_MS == 1
    assert stop_sign_camera_test.QUIT_KEY == ord("q")


def test_create_video_capture_uses_correct_camera_index():
    with patch("stop_sign_camera_test.cv2.VideoCapture") as mock_capture:
        cap = stop_sign_camera_test.create_video_capture(4)

    mock_capture.assert_called_once_with(4)
    assert cap is mock_capture.return_value


def test_read_frame_calls_video_read():
    fake_video = MagicMock()
    fake_video.read.return_value = (True, "frame")

    result = stop_sign_camera_test.read_frame(fake_video)

    fake_video.read.assert_called_once()
    assert result == (True, "frame")


def test_process_frame_calls_analysis():
    fake_frame = MagicMock()

    with patch("stop_sign_camera_test.analysis", return_value=(True, "out", {"confirmed": [], "rejected": []})) as mock_analysis:
        result = stop_sign_camera_test.process_frame(fake_frame)

    mock_analysis.assert_called_once_with(fake_frame)
    assert result == (True, "out", {"confirmed": [], "rejected": []})


def test_build_detection_messages_for_classifier_confirmed():
    details = {
        "confirmed": [
            {
                "box": [10, 20, 30, 40],
                "detector_conf": 0.91,
                "classifier_score": 0.88,
            }
        ],
        "rejected": [],
    }

    messages = stop_sign_camera_test.build_detection_messages(details, detected=True)

    assert len(messages) == 1
    assert "STOP SIGN DETECTED!" in messages[0]
    assert "Box=(10,20,30,40)" in messages[0]
    assert "D_conf=0.91" in messages[0]
    assert "C_score=0.88" in messages[0]


def test_build_detection_messages_for_segmentation_confirmed():
    details = {
        "confirmed": [
            {
                "box": [1, 2, 3, 4],
                "detector_conf": 0.75,
                "verifier_conf": 0.80,
            }
        ],
        "rejected": [],
    }

    messages = stop_sign_camera_test.build_detection_messages(details, detected=True)

    assert len(messages) == 1
    assert "V_conf=0.80" in messages[0]


def test_build_detection_messages_for_rejected():
    details = {
        "confirmed": [],
        "rejected": [
            {
                "box": [5, 6, 7, 8],
                "detector_conf": 0.62,
            }
        ],
    }

    messages = stop_sign_camera_test.build_detection_messages(details, detected=False)

    assert len(messages) == 1
    assert "Stage1 passed but Stage2 failed." in messages[0]
    assert "Box=(5,6,7,8)" in messages[0]
    assert "D_conf=0.62" in messages[0]


def test_build_detection_messages_for_no_detection():
    details = {"confirmed": [], "rejected": []}

    messages = stop_sign_camera_test.build_detection_messages(details, detected=False)

    assert messages == ["❌ No stop sign."]


def test_print_detection_messages_prints_all():
    messages = ["msg1", "msg2"]

    with patch("builtins.print") as mock_print:
        stop_sign_camera_test.print_detection_messages(messages)

    assert mock_print.call_args_list == [call("msg1"), call("msg2")]


def test_show_frame_calls_imshow():
    fake_frame = MagicMock()

    with patch("stop_sign_camera_test.cv2.imshow") as mock_imshow:
        stop_sign_camera_test.show_frame(fake_frame, "Win")

    mock_imshow.assert_called_once_with("Win", fake_frame)


def test_should_quit_returns_true_for_q():
    with patch("stop_sign_camera_test.cv2.waitKey", return_value=ord("q")) as mock_waitkey:
        result = stop_sign_camera_test.should_quit()

    mock_waitkey.assert_called_once_with(1)
    assert result is True


def test_should_quit_returns_false_for_other_key():
    with patch("stop_sign_camera_test.cv2.waitKey", return_value=ord("a")):
        result = stop_sign_camera_test.should_quit()

    assert result is False


def test_release_resources_calls_release_and_destroy():
    fake_video = MagicMock()

    with patch("stop_sign_camera_test.cv2.destroyAllWindows") as mock_destroy:
        stop_sign_camera_test.release_resources(fake_video)

    fake_video.release.assert_called_once()
    mock_destroy.assert_called_once()


def test_run_camera_loop_handles_camera_open_failure():
    fake_video = MagicMock()
    fake_video.isOpened.return_value = False

    with patch("stop_sign_camera_test.create_video_capture", return_value=fake_video), \
         patch("stop_sign_camera_test.release_resources") as mock_release, \
         patch("builtins.print") as mock_print:

        stop_sign_camera_test.run_camera_loop(4)

    mock_release.assert_called_once_with(fake_video)
    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert any("Failed to open camera index 4" in text for text in printed)


def test_run_camera_loop_breaks_on_failed_frame_read():
    fake_video = MagicMock()
    fake_video.isOpened.return_value = True

    with patch("stop_sign_camera_test.create_video_capture", return_value=fake_video), \
         patch("stop_sign_camera_test.read_frame", return_value=(False, None)) as mock_read, \
         patch("stop_sign_camera_test.release_resources") as mock_release, \
         patch("builtins.print") as mock_print:

        stop_sign_camera_test.run_camera_loop(4)

    mock_read.assert_called_once_with(fake_video)
    mock_release.assert_called_once_with(fake_video)
    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "Failed to grab frame" in printed


def test_run_camera_loop_processes_one_frame_and_quits():
    fake_video = MagicMock()
    fake_video.isOpened.return_value = True
    fake_frame = MagicMock()
    fake_output = MagicMock()
    fake_details = {
        "confirmed": [
            {
                "box": [10, 20, 30, 40],
                "detector_conf": 0.9,
                "classifier_score": 0.85,
            }
        ],
        "rejected": [],
    }

    with patch("stop_sign_camera_test.create_video_capture", return_value=fake_video), \
         patch("stop_sign_camera_test.read_frame", return_value=(True, fake_frame)) as mock_read, \
         patch("stop_sign_camera_test.process_frame", return_value=(True, fake_output, fake_details)) as mock_process, \
         patch("stop_sign_camera_test.print_detection_messages") as mock_print_msgs, \
         patch("stop_sign_camera_test.show_frame") as mock_show, \
         patch("stop_sign_camera_test.should_quit", return_value=True) as mock_quit, \
         patch("stop_sign_camera_test.release_resources") as mock_release:

        stop_sign_camera_test.run_camera_loop(4)

    mock_read.assert_called_once_with(fake_video)
    mock_process.assert_called_once_with(fake_frame)
    mock_print_msgs.assert_called_once()
    mock_show.assert_called_once_with(fake_output, stop_sign_camera_test.WINDOW_NAME)
    mock_quit.assert_called_once()
    mock_release.assert_called_once_with(fake_video)


def test_main_calls_run_camera_loop():
    with patch("stop_sign_camera_test.run_camera_loop") as mock_run:
        stop_sign_camera_test.main()

    mock_run.assert_called_once_with(stop_sign_camera_test.CAMERA_INDEX)