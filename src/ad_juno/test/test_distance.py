#!/usr/bin/env python3

import pytest
import numpy as np
from unittest.mock import MagicMock, patch

import ad_juno.test_files.source_distance as depth_sensor


# -------------------------------------------------------------
# Constants
# -------------------------------------------------------------

def test_constants_are_correct():
    assert depth_sensor.PIXEL_X == 100
    assert depth_sensor.PIXEL_Y == 50
    assert depth_sensor.DEPTH_TOPIC == "/camera/depth/image_rect_raw"
    assert depth_sensor.MM_TO_M == 0.001


# -------------------------------------------------------------
# Image conversion
# -------------------------------------------------------------

def test_convert_depth_image_calls_bridge_correctly():
    fake_data = MagicMock()
    fake_bridge = MagicMock()
    fake_image = np.zeros((100, 200), dtype=np.uint16)
    fake_bridge.imgmsg_to_cv2.return_value = fake_image

    result = depth_sensor.convert_depth_image(fake_data, fake_bridge)

    fake_bridge.imgmsg_to_cv2.assert_called_once_with(
        fake_data,
        desired_encoding="passthrough"
    )
    assert result is fake_image


# -------------------------------------------------------------
# Depth extraction
# -------------------------------------------------------------

def test_extract_depth_at_pixel_converts_mm_to_m():
    depth_image = np.zeros((100, 200), dtype=np.uint16)
    depth_image[50][100] = 2500  # mm

    result = depth_sensor.extract_depth_at_pixel(depth_image, 100, 50)

    assert result == 2.5


def test_extract_depth_at_pixel_uses_given_coordinates():
    depth_image = np.zeros((10, 10), dtype=np.uint16)
    depth_image[3][4] = 1234

    result = depth_sensor.extract_depth_at_pixel(depth_image, 4, 3)

    assert result == 1.234


def test_extract_depth_at_pixel_zero_depth():
    depth_image = np.zeros((100, 200), dtype=np.uint16)

    result = depth_sensor.extract_depth_at_pixel(depth_image, 100, 50)

    assert result == 0.0


def test_extract_depth_at_pixel_out_of_bounds_raises():
    depth_image = np.zeros((5, 5), dtype=np.uint16)

    with pytest.raises(IndexError):
        depth_sensor.extract_depth_at_pixel(depth_image, 100, 50)


# -------------------------------------------------------------
# Message formatting
# -------------------------------------------------------------

def test_format_depth_message():
    result = depth_sensor.format_depth_message(100, 50, 2.3456)
    assert result == "Distanza del pixel (100,50): 2.346 metri"


def test_format_depth_message_zero():
    result = depth_sensor.format_depth_message(0, 0, 0.0)
    assert result == "Distanza del pixel (0,0): 0.000 metri"


# -------------------------------------------------------------
# End-to-end processing
# -------------------------------------------------------------

def test_process_depth_message_happy_path():
    fake_data = MagicMock()
    fake_bridge = MagicMock()

    depth_image = np.zeros((100, 200), dtype=np.uint16)
    depth_image[50][100] = 1750
    fake_bridge.imgmsg_to_cv2.return_value = depth_image

    result = depth_sensor.process_depth_message(fake_data, fake_bridge, 100, 50)

    assert result == "Distanza del pixel (100,50): 1.750 metri"


def test_process_depth_message_uses_custom_pixel():
    fake_data = MagicMock()
    fake_bridge = MagicMock()

    depth_image = np.zeros((20, 20), dtype=np.uint16)
    depth_image[7][9] = 4200
    fake_bridge.imgmsg_to_cv2.return_value = depth_image

    result = depth_sensor.process_depth_message(fake_data, fake_bridge, 9, 7)

    assert result == "Distanza del pixel (9,7): 4.200 metri"


# -------------------------------------------------------------
# Callback
# -------------------------------------------------------------

def test_callback_uses_given_bridge_and_prints_message():
    fake_data = MagicMock()
    fake_bridge = MagicMock()

    with patch("depth_sensor.process_depth_message", return_value="depth ok") as mock_process, \
         patch("builtins.print") as mock_print:

        depth_sensor.callback(fake_data, bridge=fake_bridge, pixel_x=10, pixel_y=20)

    mock_process.assert_called_once_with(fake_data, fake_bridge, 10, 20)
    mock_print.assert_called_once_with("depth ok")


def test_callback_creates_bridge_if_none_given():
    fake_data = MagicMock()
    fake_bridge = MagicMock()

    with patch("depth_sensor.CvBridge", return_value=fake_bridge) as mock_cvbridge, \
         patch("depth_sensor.process_depth_message", return_value="depth ok") as mock_process, \
         patch("builtins.print"):

        depth_sensor.callback(fake_data)

    mock_cvbridge.assert_called_once()
    mock_process.assert_called_once_with(
        fake_data,
        fake_bridge,
        depth_sensor.PIXEL_X,
        depth_sensor.PIXEL_Y,
    )


# -------------------------------------------------------------
# ROS wiring
# -------------------------------------------------------------

def test_main_subscribes_to_correct_topic_and_spins():
    with patch("depth_sensor.rospy.Subscriber") as mock_subscriber, \
         patch("depth_sensor.rospy.spin") as mock_spin:

        depth_sensor.main()

    mock_subscriber.assert_called_once_with(
        "/camera/depth/image_rect_raw",
        depth_sensor.Image,
        depth_sensor.callback,
    )
    mock_spin.assert_called_once()