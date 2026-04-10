#!/usr/bin/env python3
import pytest
from unittest.mock import MagicMock, patch, call

import test_files.source_brake_tester as brake_motor


@pytest.fixture
def fake_motor():
    return MagicMock()


def test_modules_are_correct():
    assert brake_motor.modules["brake"] == 1
    assert brake_motor.modules["steering"] == 3


def test_create_brake_motor_calls_stepper_with_correct_args():
    with patch("brake_motor.Stepper") as MockStepper:
        motor = brake_motor.create_brake_motor()

        MockStepper.assert_called_once_with(
            "can",
            1_000_000,
            1,
            50_000,
            140_000,
            140_000,
            0,
            50_000,
            50_000,
        )
        assert motor is MockStepper.return_value


def test_run_brake_test_calls_sleep_brake_and_disconnect(fake_motor):
    with (
        patch("brake_motor.create_brake_motor", return_value=fake_motor),
        patch("brake_motor.time.sleep") as mock_sleep,
    ):
        brake_motor.run_brake_test(delay=3)

        mock_sleep.assert_called_once_with(3)
        fake_motor.brake.assert_called_once()
        fake_motor.disconnect_motor.assert_called_once()


def test_run_brake_test_disconnects_even_if_brake_fails(fake_motor):
    fake_motor.brake.side_effect = RuntimeError("CAN timeout")

    with (
        patch("brake_motor.create_brake_motor", return_value=fake_motor),
        patch("brake_motor.time.sleep"),
    ):
        brake_motor.run_brake_test(delay=0)

    fake_motor.brake.assert_called_once()
    fake_motor.disconnect_motor.assert_called_once()


def test_run_brake_test_call_order(fake_motor):
    tracker = MagicMock()
    tracker.attach_mock(fake_motor.brake, "brake")
    tracker.attach_mock(fake_motor.disconnect_motor, "disconnect_motor")

    with (
        patch("brake_motor.create_brake_motor", return_value=fake_motor),
        patch("brake_motor.time.sleep"),
    ):
        brake_motor.run_brake_test(delay=0)

    assert tracker.mock_calls == [
        call.brake(),
        call.disconnect_motor(),
    ]


def test_run_brake_test_prints_error_when_brake_fails(fake_motor):
    fake_motor.brake.side_effect = Exception("motor fault")

    with (
        patch("brake_motor.create_brake_motor", return_value=fake_motor),
        patch("brake_motor.time.sleep"),
        patch("builtins.print") as mock_print,
    ):
        brake_motor.run_brake_test(delay=0)

    printed_texts = [args[0] for args, _ in mock_print.call_args_list]
    assert any("[ERROR] motor fault" in text for text in printed_texts)
