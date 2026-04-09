#!/usr/bin/env python3
import pytest
from unittest.mock import MagicMock, patch, call

import test_files.source_one_motor as trinamic_demo


@pytest.fixture
def fake_motor():
    motor = MagicMock()

    motor.drive_settings = MagicMock()
    motor.linear_ramp = MagicMock()

    class ENUM:
        MicrostepResolution256Microsteps = 256

    class AP:
        RelativePositioningOption = 99

    motor.ENUM = ENUM
    motor.AP = AP
    motor.actual_position = 123
    motor.target_position = 456
    return motor


@pytest.fixture
def fake_module(fake_motor):
    module = MagicMock()
    module.motors = [fake_motor]
    return module


@pytest.fixture
def fake_interface():
    return MagicMock()


# -------------------------------------------------------------
# Constants / simple helpers
# -------------------------------------------------------------

def test_connection_string_is_correct():
    assert trinamic_demo.CONNECTION_STRING == "--interface socketcan_tmcl"


def test_get_motor_returns_index_zero(fake_module):
    result = trinamic_demo.get_motor(fake_module, 0)
    assert result is fake_module.motors[0]


def test_show_library_info_calls_pytrinamic():
    with patch("trinamic_demo.pytrinamic.show_info") as mock_show:
        trinamic_demo.show_library_info()
    mock_show.assert_called_once()


def test_create_connection_manager_uses_correct_string():
    with patch("trinamic_demo.ConnectionManager") as MockCM:
        manager = trinamic_demo.create_connection_manager()

    MockCM.assert_called_once_with("--interface socketcan_tmcl")
    assert manager is MockCM.return_value


def test_create_module_uses_interface(fake_interface):
    with patch("trinamic_demo.TMCM1260") as MockTMCM:
        module = trinamic_demo.create_module(fake_interface)

    MockTMCM.assert_called_once_with(fake_interface)
    assert module is MockTMCM.return_value


# -------------------------------------------------------------
# Drive settings
# -------------------------------------------------------------

def test_configure_drive_settings(fake_motor):
    trinamic_demo.configure_drive_settings(fake_motor)

    assert fake_motor.drive_settings.max_current == 50
    assert fake_motor.drive_settings.standby_current == 0
    assert fake_motor.drive_settings.boost_current == 0
    assert fake_motor.drive_settings.microstep_resolution == 256


def test_configure_linear_ramp(fake_motor):
    trinamic_demo.configure_linear_ramp(fake_motor)

    assert fake_motor.linear_ramp.max_velocity == 20000
    assert fake_motor.linear_ramp.max_acceleration == 40000


# -------------------------------------------------------------
# Preparation
# -------------------------------------------------------------

def test_prepare_motor_sets_parameters_and_resets_position(fake_motor):
    with patch("trinamic_demo.time.sleep") as mock_sleep:
        trinamic_demo.prepare_motor(fake_motor)

    mock_sleep.assert_called_once_with(1.0)
    fake_motor.set_axis_parameter.assert_called_once_with(
        fake_motor.AP.RelativePositioningOption,
        1,
    )
    assert fake_motor.actual_position == 0


def test_prepare_motor_prints(fake_motor):
    with patch("trinamic_demo.time.sleep"), \
         patch("builtins.print") as mock_print:
        trinamic_demo.prepare_motor(fake_motor)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "Preparing parameters..." in printed


# -------------------------------------------------------------
# Rotate / stop
# -------------------------------------------------------------

def test_rotate_and_stop_calls_rotate_stop_and_sleeps(fake_motor):
    with patch("trinamic_demo.time.sleep") as mock_sleep:
        trinamic_demo.rotate_and_stop(fake_motor)

    fake_motor.rotate.assert_called_once_with(20000)
    fake_motor.stop.assert_called_once()
    assert mock_sleep.call_args_list == [call(5), call(2)]


def test_rotate_and_stop_order(fake_motor):
    tracker = MagicMock()
    tracker.attach_mock(fake_motor.rotate, "rotate")
    tracker.attach_mock(fake_motor.stop, "stop")

    with patch("trinamic_demo.time.sleep"):
        trinamic_demo.rotate_and_stop(fake_motor)

    assert tracker.mock_calls == [
        call.rotate(20000),
        call.stop(),
    ]


# -------------------------------------------------------------
# Waiting loop
# -------------------------------------------------------------

def test_wait_until_position_reached_exits_immediately(fake_motor):
    fake_motor.get_position_reached.return_value = True

    with patch("trinamic_demo.time.sleep") as mock_sleep:
        trinamic_demo.wait_until_position_reached(fake_motor)

    fake_motor.get_position_reached.assert_called_once()
    mock_sleep.assert_not_called()


def test_wait_until_position_reached_polls_until_true(fake_motor):
    fake_motor.get_position_reached.side_effect = [False, False, True]

    with patch("trinamic_demo.time.sleep") as mock_sleep, \
         patch("builtins.print") as mock_print:
        trinamic_demo.wait_until_position_reached(fake_motor, poll_interval_s=0.2)

    assert fake_motor.get_position_reached.call_count == 3
    assert mock_sleep.call_count == 2
    assert mock_sleep.call_args_list == [call(0.2), call(0.2)]

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert any("target position:" in text for text in printed)


# -------------------------------------------------------------
# Motion helpers
# -------------------------------------------------------------

def test_move_farther_calls_move_by_and_wait(fake_motor):
    with patch("trinamic_demo.wait_until_position_reached") as mock_wait, \
         patch("builtins.print") as mock_print:
        trinamic_demo.move_farther(fake_motor)

    fake_motor.move_by.assert_called_once_with(fake_motor.actual_position, 10000)
    mock_wait.assert_called_once_with(fake_motor)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "Doubling moved distance." in printed
    assert "Furthest point reached." in printed


def test_move_back_to_zero_calls_sleep_move_to_and_wait(fake_motor):
    with patch("trinamic_demo.wait_until_position_reached") as mock_wait, \
         patch("trinamic_demo.time.sleep") as mock_sleep, \
         patch("builtins.print") as mock_print:
        trinamic_demo.move_back_to_zero(fake_motor)

    mock_sleep.assert_called_once_with(3)
    fake_motor.move_to.assert_called_once_with(0, 20000)
    mock_wait.assert_called_once_with(fake_motor)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "Moving back to 0..." in printed
    assert "Reached position 0." in printed


# -------------------------------------------------------------
# End-to-end orchestration
# -------------------------------------------------------------

def test_run_demo_calls_stages_in_order(fake_motor):
    tracker = MagicMock()

    with patch("trinamic_demo.prepare_motor") as mock_prepare, \
         patch("trinamic_demo.rotate_and_stop") as mock_rotate, \
         patch("trinamic_demo.move_farther") as mock_move_farther, \
         patch("trinamic_demo.move_back_to_zero") as mock_move_back:

        tracker.attach_mock(mock_prepare, "prepare_motor")
        tracker.attach_mock(mock_rotate, "rotate_and_stop")
        tracker.attach_mock(mock_move_farther, "move_farther")
        tracker.attach_mock(mock_move_back, "move_back_to_zero")

        trinamic_demo.run_demo(fake_motor)

    assert tracker.mock_calls == [
        call.prepare_motor(fake_motor),
        call.rotate_and_stop(fake_motor),
        call.move_farther(fake_motor),
        call.move_back_to_zero(fake_motor),
    ]


# -------------------------------------------------------------
# Main / context manager
# -------------------------------------------------------------

def test_main_runs_full_flow(fake_interface, fake_module, fake_motor):
    fake_manager = MagicMock()
    fake_manager.connect.return_value.__enter__.return_value = fake_interface
    fake_manager.connect.return_value.__exit__.return_value = None

    with patch("trinamic_demo.show_library_info") as mock_info, \
         patch("trinamic_demo.create_connection_manager", return_value=fake_manager) as mock_create_manager, \
         patch("trinamic_demo.create_module", return_value=fake_module) as mock_create_module, \
         patch("trinamic_demo.get_motor", return_value=fake_motor) as mock_get_motor, \
         patch("trinamic_demo.run_demo") as mock_run_demo, \
         patch("builtins.print") as mock_print:

        trinamic_demo.main()

    mock_info.assert_called_once()
    mock_create_manager.assert_called_once()
    fake_manager.connect.assert_called_once()
    mock_create_module.assert_called_once_with(fake_interface)
    mock_get_motor.assert_called_once_with(fake_module, 0)
    mock_run_demo.assert_called_once_with(fake_motor)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "\nReady." in printed


def test_main_propagates_run_demo_error(fake_interface, fake_module, fake_motor):
    fake_manager = MagicMock()
    fake_manager.connect.return_value.__enter__.return_value = fake_interface
    fake_manager.connect.return_value.__exit__.return_value = None

    with patch("trinamic_demo.show_library_info"), \
         patch("trinamic_demo.create_connection_manager", return_value=fake_manager), \
         patch("trinamic_demo.create_module", return_value=fake_module), \
         patch("trinamic_demo.get_motor", return_value=fake_motor), \
         patch("trinamic_demo.run_demo", side_effect=RuntimeError("motor fault")):

        with pytest.raises(RuntimeError, match="motor fault"):
            trinamic_demo.main()