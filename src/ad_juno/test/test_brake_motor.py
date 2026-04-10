#!/usr/bin/env python3
import pytest
from unittest.mock import MagicMock, patch, call

import ad_juno.test_files.source_brake_tester as brake_motor


@pytest.fixture
def fake_motor():
    return MagicMock()


def test_modules_are_correct():
    assert brake_motor.modules["brake"] == 1
    assert brake_motor.modules["steering"] == 3


def test_create_brake_motor_calls_stepper_with_correct_args():
    fake_stepper_cls = MagicMock()

    motor = brake_motor.create_brake_motor(stepper_cls=fake_stepper_cls)

    fake_stepper_cls.assert_called_once_with(
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
    assert motor is fake_stepper_cls.return_value


def test_run_brake_test_calls_sleep_brake_and_disconnect(fake_motor):
    fake_stepper_cls = MagicMock(return_value=fake_motor)

    with patch("ad_juno.test_files.source_brake_tester.time.sleep") as mock_sleep:
        brake_motor.run_brake_test(delay=3, stepper_cls=fake_stepper_cls)

    mock_sleep.assert_called_once_with(3)
    fake_motor.brake.assert_called_once()
    fake_motor.disconnect_motor.assert_called_once()


def test_run_brake_test_disconnects_even_if_brake_fails(fake_motor):
    fake_motor.brake.side_effect = RuntimeError("CAN timeout")
    fake_stepper_cls = MagicMock(return_value=fake_motor)

    with patch("ad_juno.test_files.source_brake_tester.time.sleep"):
        brake_motor.run_brake_test(delay=0, stepper_cls=fake_stepper_cls)

    fake_motor.brake.assert_called_once()
    fake_motor.disconnect_motor.assert_called_once()


def test_run_brake_test_call_order(fake_motor):
    fake_stepper_cls = MagicMock(return_value=fake_motor)
    tracker = MagicMock()
    tracker.attach_mock(fake_motor.brake, "brake")
    tracker.attach_mock(fake_motor.disconnect_motor, "disconnect_motor")

    with patch("ad_juno.test_files.source_brake_tester.time.sleep"):
        brake_motor.run_brake_test(delay=0, stepper_cls=fake_stepper_cls)

    assert tracker.mock_calls == [
        call.brake(),
        call.disconnect_motor(),
    ]


def test_run_brake_test_prints_error_when_brake_fails(fake_motor):
    fake_motor.brake.side_effect = Exception("motor fault")
    fake_stepper_cls = MagicMock(return_value=fake_motor)

    with patch("ad_juno.test_files.source_brake_tester.time.sleep"), \
         patch("builtins.print") as mock_print:
        brake_motor.run_brake_test(delay=0, stepper_cls=fake_stepper_cls)

    printed_texts = [args[0] for args, _ in mock_print.call_args_list]
    assert any("[ERROR] motor fault" in text for text in printed_texts)