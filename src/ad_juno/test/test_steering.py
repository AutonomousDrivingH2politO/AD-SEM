import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import pytest
from unittest.mock import MagicMock, patch

from ad_juno.steering_node import (
    SteeringNode,
    TRANSMISSION_RATIO,
    MAX_ANGLE,
    STEP_ANGLE,
)

POS_INF = float("inf")
NEG_INF = float("-inf")

# ----------------------------------------------------------------
# ROS 2 setup
# ----------------------------------------------------------------


@pytest.fixture(scope="module", autouse=True)
def ros_init():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def steering_node():
    mock_stepper = MagicMock()
    mock_topics = MagicMock()
    mock_topics.topic_names = {
        "steering": "/test/steering",
        "stop": "/test/stop",
    }

    # Use a patch to ensure we don't actually create folders on the dev machine
    with (
        patch("ad_juno.steering_node.Stepper", return_value=mock_stepper),
        patch("ad_juno.steering_node.Topics", return_value=mock_topics),
        patch(
            "ad_juno.steering_node.SteeringNode.set_debug_folders",
            return_value="/tmp/test_logs",
        ),
        patch(
            "ad_juno.steering_node.SteeringNode._save_debug"
        ),  # Prevent I/O during tests
    ):
        # Optional: Force debug to False via parameter overrides if supported by your setup
        node = SteeringNode()
        yield node, mock_stepper
        node.destroy_node()


# ----------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------
def create_valid_angle_msg(angle):
    msg = Float64()
    msg.data = float(angle)
    return msg


# ----------------------------------------------------------------
# Tests
# ----------------------------------------------------------------


def test_callback_step_angle(steering_node):
    node, mock_stepper = steering_node
    msg = create_valid_angle_msg(5.0)
    node.callback_stepangle(msg)
    assert node.latest_angle == 5.0
    expected = node.convert_angle_to_position(5.0)
    mock_stepper.move_stepper.assert_called_once_with(expected)


def test_convert_angle_to_position(steering_node):
    angle = 5.0
    node, _ = steering_node
    assert STEP_ANGLE > 0
    assert TRANSMISSION_RATIO > 0
    steps = (angle / STEP_ANGLE) * 256
    expected = -int(steps * TRANSMISSION_RATIO)
    assert node.convert_angle_to_position(angle) == expected


def test_convert_angle_to_position_negative(steering_node):
    node, _ = steering_node
    angle = -5.0
    steps = (angle / STEP_ANGLE) * 256
    expected = -int(steps * TRANSMISSION_RATIO)
    assert node.convert_angle_to_position(angle) == expected


def test_convert_angle_to_position_zero(steering_node):
    node, _ = steering_node
    assert node.convert_angle_to_position(0.0) == 0


def test_convert_angle_to_position_raises_on_inf(steering_node):
    node, _ = steering_node
    with pytest.raises(OverflowError):
        node.convert_angle_to_position(float("inf"))


def test_convert_angle_to_position_raises_on_negative_inf(steering_node):
    node, _ = steering_node
    with pytest.raises(OverflowError):
        node.convert_angle_to_position(float("-inf"))


def test_convert_angle_to_position_raises_on_nan(steering_node):
    node, _ = steering_node
    with pytest.raises(ValueError):
        node.convert_angle_to_position(float("nan"))


def test_convert_angle_to_position_raises_on_none(steering_node):
    node, _ = steering_node
    with pytest.raises(TypeError):
        node.convert_angle_to_position(None)


def test_convert_angle_to_position_raises_on_string(steering_node):
    node, _ = steering_node
    with pytest.raises(TypeError):
        node.convert_angle_to_position("abc")


def test_callback_step_angle_clamps_inf_to_max(steering_node):
    node, mock_stepper = steering_node
    msg = create_valid_angle_msg(float("inf"))

    node.callback_stepangle(msg)

    assert node.latest_angle == MAX_ANGLE
    expected = node.convert_angle_to_position(MAX_ANGLE)
    mock_stepper.move_stepper.assert_called_once_with(expected)


def test_callback_step_angle_clamps_negative_inf_to_min(steering_node):
    node, mock_stepper = steering_node
    msg = create_valid_angle_msg(float("-inf"))

    node.callback_stepangle(msg)

    assert node.latest_angle == -MAX_ANGLE
    expected = node.convert_angle_to_position(-MAX_ANGLE)
    mock_stepper.move_stepper.assert_called_once_with(expected)


def test_callback_step_angle_raises_on_nan(steering_node):
    node, _ = steering_node
    msg = create_valid_angle_msg(float("nan"))

    with pytest.raises(ValueError):
        node.callback_stepangle(msg)


def test_callback_step_angle_raises_on_none(steering_node):
    node, _ = steering_node
    msg = MagicMock()
    msg.data = None

    with pytest.raises(TypeError):
        node.callback_stepangle(msg)


def test_callback_step_angle_raises_on_string(steering_node):
    node, _ = steering_node
    msg = MagicMock()
    msg.data = "abc"

    with pytest.raises(TypeError):
        node.callback_stepangle(msg)


def test_callback_step_angle_does_nothing_when_stopped(steering_node):
    node, mock_stepper = steering_node
    node._stopped = True
    msg = create_valid_angle_msg(5.0)

    node.callback_stepangle(msg)

    assert node.latest_angle == 0.0
    mock_stepper.move_stepper.assert_not_called()


def test_callback_stop_sets_true(steering_node):
    node, _ = steering_node
    msg = Bool()
    msg.data = True

    node.callback_stop(msg)

    assert node._stopped is True


def test_callback_stop_sets_false(steering_node):
    node, _ = steering_node
    node._stopped = True
    msg = Bool()
    msg.data = False

    node.callback_stop(msg)

    assert node._stopped is False


def test_callback_step_angle_at_max_boundary(steering_node):
    node, mock_stepper = steering_node
    msg = create_valid_angle_msg(MAX_ANGLE)

    node.callback_stepangle(msg)

    assert node.latest_angle == MAX_ANGLE
    expected = node.convert_angle_to_position(MAX_ANGLE)
    mock_stepper.move_stepper.assert_called_once_with(expected)


def test_callback_step_angle_at_min_boundary(steering_node):
    node, mock_stepper = steering_node
    msg = create_valid_angle_msg(-MAX_ANGLE)

    node.callback_stepangle(msg)

    assert node.latest_angle == -MAX_ANGLE
    expected = node.convert_angle_to_position(-MAX_ANGLE)
    mock_stepper.move_stepper.assert_called_once_with(expected)


def test_callback_step_angle_raises_if_move_stepper_fails(steering_node):
    node, mock_stepper = steering_node
    mock_stepper.move_stepper.side_effect = RuntimeError("motor failure")
    msg = create_valid_angle_msg(5.0)

    with pytest.raises(RuntimeError, match="motor failure"):
        node.callback_stepangle(msg)

