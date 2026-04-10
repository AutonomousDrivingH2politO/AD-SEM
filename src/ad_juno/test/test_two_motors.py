#!/usr/bin/env python3
import pytest
from unittest.mock import MagicMock, patch, call

import ad_juno.test_files.source_two_motors as dual_motor_test


@pytest.fixture
def fake_connection():
    return MagicMock()


@pytest.fixture
def fake_steering():
    return MagicMock()


@pytest.fixture
def fake_brake():
    return MagicMock()


def test_constants_are_correct():
    assert dual_motor_test.MODULES["steering"] == 3
    assert dual_motor_test.MODULES["brake"] == 1
    assert dual_motor_test.CONNECTION_STRING == "--interface socketcan_tmcl --port can1 --data-rate 1000000"
    assert dual_motor_test.ROTATE_SPEED == 20000
    assert dual_motor_test.STEP_DELAY_S == 1


def test_create_connection_uses_correct_string():
    with patch("dual_motor_test.ConnectionManager") as MockManager:
        manager_instance = MockManager.return_value
        fake_conn = MagicMock()
        manager_instance.connect.return_value = fake_conn

        result = dual_motor_test.create_connection()

        MockManager.assert_called_once_with(
            "--interface socketcan_tmcl --port can1 --data-rate 1000000"
        )
        manager_instance.connect.assert_called_once()
        assert result is fake_conn


def test_create_motor_uses_correct_module_id(fake_connection):
    with patch("dual_motor_test.TMCM1260") as MockTMCM:
        motor = dual_motor_test.create_motor(fake_connection, 3)

        MockTMCM.assert_called_once_with(fake_connection, module_id=3)
        assert motor is MockTMCM.return_value


def test_create_steering_motor_uses_steering_id(fake_connection):
    with patch("dual_motor_test.create_motor") as mock_create_motor:
        dual_motor_test.create_steering_motor(fake_connection)

        mock_create_motor.assert_called_once_with(
            fake_connection,
            dual_motor_test.MODULES["steering"],
        )


def test_create_brake_motor_uses_brake_id(fake_connection):
    with patch("dual_motor_test.create_motor") as mock_create_motor:
        dual_motor_test.create_brake_motor(fake_connection)

        mock_create_motor.assert_called_once_with(
            fake_connection,
            dual_motor_test.MODULES["brake"],
        )


def test_run_single_cycle_calls_correct_sequence(fake_steering, fake_brake):
    tracker = MagicMock()
    tracker.attach_mock(fake_steering.rotate, "steering_rotate")
    tracker.attach_mock(fake_steering.stop, "steering_stop")
    tracker.attach_mock(fake_brake.rotate, "brake_rotate")
    tracker.attach_mock(fake_brake.stop, "brake_stop")

    with patch("dual_motor_test.time.sleep") as mock_sleep:
        dual_motor_test.run_single_cycle(fake_steering, fake_brake, delay_s=1)

    assert tracker.mock_calls == [
        call.steering_rotate(0, 20000),
        call.steering_stop(0),
        call.brake_rotate(0, 20000),
        call.brake_stop(0),
    ]

    assert mock_sleep.call_args_list == [call(1), call(1), call(1), call(1)]


def test_run_single_cycle_uses_custom_delay(fake_steering, fake_brake):
    with patch("dual_motor_test.time.sleep") as mock_sleep:
        dual_motor_test.run_single_cycle(fake_steering, fake_brake, delay_s=0.5)

    assert mock_sleep.call_args_list == [call(0.5), call(0.5), call(0.5), call(0.5)]


def test_main_loop_closes_connections_when_interrupted(fake_connection, fake_steering, fake_brake):
    conn_br = MagicMock()
    conn_st = MagicMock()

    with patch(
        "dual_motor_test.create_connection",
        side_effect=[conn_br, conn_st],
    ) as mock_create_connection, \
         patch("dual_motor_test.create_steering_motor", return_value=fake_steering) as mock_create_steering, \
         patch("dual_motor_test.create_brake_motor", return_value=fake_brake) as mock_create_brake, \
         patch("dual_motor_test.run_single_cycle", side_effect=KeyboardInterrupt):

        with pytest.raises(KeyboardInterrupt):
            dual_motor_test.main_loop()

    assert mock_create_connection.call_count == 2
    mock_create_steering.assert_called_once_with(conn_st)
    mock_create_brake.assert_called_once_with(conn_br)
    conn_st.close.assert_called_once()
    conn_br.close.assert_called_once()


def test_main_loop_closes_connections_if_cycle_fails(fake_steering, fake_brake):
    conn_br = MagicMock()
    conn_st = MagicMock()

    with patch(
        "dual_motor_test.create_connection",
        side_effect=[conn_br, conn_st],
    ), patch("dual_motor_test.create_steering_motor", return_value=fake_steering), \
       patch("dual_motor_test.create_brake_motor", return_value=fake_brake), \
       patch("dual_motor_test.run_single_cycle", side_effect=RuntimeError("motor fault")):

        with pytest.raises(RuntimeError, match="motor fault"):
            dual_motor_test.main_loop()

    conn_st.close.assert_called_once()
    conn_br.close.assert_called_once()