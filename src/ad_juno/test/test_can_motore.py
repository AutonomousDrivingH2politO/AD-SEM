#!/usr/bin/env python3
import pytest
from unittest.mock import MagicMock, patch, call

import src.test_files.source_can_motore as trinamic_motor


@pytest.fixture
def fake_connection():
    return MagicMock()


@pytest.fixture
def fake_motor():
    motor = MagicMock()
    motor.AP.MaxVelocity = 4
    motor.AP.MaxAcceleration = 5
    return motor


@pytest.fixture
def fake_module(fake_motor):
    module = MagicMock()
    module.motors = [fake_motor]
    return module


def test_constants_are_correct():
    assert trinamic_motor.CONNECTION_STRING == "--interface socketcan_tmcl --port can1"
    assert trinamic_motor.MODULE_ID == 2
    assert trinamic_motor.MOTOR_INDEX == 0
    assert trinamic_motor.MAX_VELOCITY == 2000
    assert trinamic_motor.MAX_ACCELERATION == 1000
    assert trinamic_motor.ROTATE_VELOCITY == 5000
    assert trinamic_motor.RUN_TIME_S == 5


def test_create_connection_uses_correct_connection_string():
    with patch("trinamic_motor.ConnectionManager") as MockCM:
        instance = MockCM.return_value
        fake_connection = MagicMock()
        instance.connect.return_value = fake_connection

        connection = trinamic_motor.create_connection()

        MockCM.assert_called_once_with("--interface socketcan_tmcl --port can1")
        instance.connect.assert_called_once()
        assert connection is fake_connection


def test_setup_motor_creates_module_with_correct_id(fake_connection, fake_module):
    with patch("trinamic_motor.TMCM1260", return_value=fake_module) as MockTMCM:
        motor = trinamic_motor.setup_motor(fake_connection)

        MockTMCM.assert_called_once_with(fake_connection, module_id=2)
        assert motor is fake_module.motors[0]


def test_setup_motor_sets_axis_parameters(fake_connection, fake_motor, fake_module):
    with patch("trinamic_motor.TMCM1260", return_value=fake_module):
        motor = trinamic_motor.setup_motor(fake_connection)

        motor.set_axis_parameter.assert_any_call(fake_motor.AP.MaxVelocity, 2000)
        motor.set_axis_parameter.assert_any_call(fake_motor.AP.MaxAcceleration, 1000)


def test_setup_motor_uses_motor_index_zero(fake_connection, fake_motor, fake_module):
    with patch("trinamic_motor.TMCM1260", return_value=fake_module):
        motor = trinamic_motor.setup_motor(fake_connection)

        assert motor is fake_module.motors[0]


def test_run_motor_sequence_calls_rotate_sleep_stop(fake_motor):
    with patch("trinamic_motor.time.sleep") as mock_sleep:
        trinamic_motor.run_motor_sequence(fake_motor, run_time_s=5)

    fake_motor.rotate.assert_called_once_with(5000)
    mock_sleep.assert_called_once_with(5)
    fake_motor.stop.assert_called_once()


def test_run_motor_sequence_order(fake_motor):
    tracker = MagicMock()
    tracker.attach_mock(fake_motor.rotate, "rotate")
    tracker.attach_mock(fake_motor.stop, "stop")

    with patch("trinamic_motor.time.sleep"):
        trinamic_motor.run_motor_sequence(fake_motor, run_time_s=0)

    assert tracker.mock_calls == [
        call.rotate(5000),
        call.stop(),
    ]


def test_run_motor_sequence_prints_messages(fake_motor):
    with patch("trinamic_motor.time.sleep"), \
         patch("builtins.print") as mock_print:
        trinamic_motor.run_motor_sequence(fake_motor, run_time_s=0)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "Il motore sta ruotando" in printed
    assert "Il motore si è fermato" in printed


def test_main_closes_connection_on_success(fake_connection):
    fake_motor = MagicMock()

    with patch("trinamic_motor.create_connection", return_value=fake_connection), \
         patch("trinamic_motor.setup_motor", return_value=fake_motor), \
         patch("trinamic_motor.run_motor_sequence"):
        trinamic_motor.main()

    fake_connection.close.assert_called_once()


def test_main_closes_connection_if_run_motor_sequence_fails(fake_connection):
    fake_motor = MagicMock()

    with patch("trinamic_motor.create_connection", return_value=fake_connection), \
         patch("trinamic_motor.setup_motor", return_value=fake_motor), \
         patch("trinamic_motor.run_motor_sequence", side_effect=RuntimeError("driver fault")):
        with pytest.raises(RuntimeError, match="driver fault"):
            trinamic_motor.main()

    fake_connection.close.assert_called_once()


def test_main_calls_setup_and_run_in_order(fake_connection):
    fake_motor = MagicMock()
    tracker = MagicMock()

    with patch("trinamic_motor.create_connection", return_value=fake_connection), \
         patch("trinamic_motor.setup_motor", return_value=fake_motor) as mock_setup, \
         patch("trinamic_motor.run_motor_sequence") as mock_run:

        tracker.attach_mock(mock_setup, "setup_motor")
        tracker.attach_mock(mock_run, "run_motor_sequence")

        trinamic_motor.main()

    assert tracker.mock_calls == [
        call.setup_motor(fake_connection),
        call.run_motor_sequence(fake_motor),
    ]