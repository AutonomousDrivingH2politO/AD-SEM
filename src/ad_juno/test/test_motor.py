#!/usr/bin/env python3
import pytest
from unittest.mock import MagicMock, patch, call

import test_files.source_motor as tmcl_config

@pytest.fixture
def fake_connection():
    return MagicMock()


@pytest.fixture
def fake_brake():
    motor = MagicMock()

    class AP:
        MaxVelocity = 1
        MaxAcceleration = 2
        V1 = 3
        StartVelocity = 4
        StopVelocity = 5
        RampWaitTime = 6
        MaxCurrent = 7
        StandbyCurrent = 8
        SG2Threshold = 9
        SG2FilterEnable = 10
        SmartEnergyStallVelocity = 11
        SmartEnergyHysteresis = 12
        SmartEnergyHysteresisStart = 13
        SECUS = 14
        SmartEnergyThresholdSpeed = 15

    motor.AP = AP
    return motor


# -------------------------------------------------------------
# Constants
# -------------------------------------------------------------

def test_modules_are_correct():
    assert tmcl_config.MODULES["steering"] == 3
    assert tmcl_config.MODULES["brake"] == 1


def test_connection_string_is_correct():
    assert tmcl_config.CONNECTION_STRING == "--interface socketcan_tmcl --port can1 --data-rate 1000000"


def test_brake_config_contains_expected_values():
    assert tmcl_config.BRAKE_CONFIG["MaxVelocity"] == 50000
    assert tmcl_config.BRAKE_CONFIG["MaxAcceleration"] == 140000
    assert tmcl_config.BRAKE_CONFIG["V1"] == 0
    assert tmcl_config.BRAKE_CONFIG["StartVelocity"] == 1000
    assert tmcl_config.BRAKE_CONFIG["StopVelocity"] == 1000
    assert tmcl_config.BRAKE_CONFIG["RampWaitTime"] == 0
    assert tmcl_config.BRAKE_CONFIG["MaxCurrent"] == 200
    assert tmcl_config.BRAKE_CONFIG["StandbyCurrent"] == 100
    assert tmcl_config.BRAKE_CONFIG["SG2Threshold"] == 11
    assert tmcl_config.BRAKE_CONFIG["SG2FilterEnable"] == 0
    assert tmcl_config.BRAKE_CONFIG["SmartEnergyStallVelocity"] == 0
    assert tmcl_config.BRAKE_CONFIG["SmartEnergyHysteresis"] == 15
    assert tmcl_config.BRAKE_CONFIG["SmartEnergyHysteresisStart"] == 0
    assert tmcl_config.BRAKE_CONFIG["SECUS"] == 1
    assert tmcl_config.BRAKE_CONFIG["SmartEnergyThresholdSpeed"] == 7_999_774


# -------------------------------------------------------------
# Connection / module creation
# -------------------------------------------------------------

def test_create_connection_uses_correct_string():
    with patch("tmcl_config.ConnectionManager") as MockManager:
        manager_instance = MockManager.return_value
        fake_connection = MagicMock()
        manager_instance.connect.return_value = fake_connection

        result = tmcl_config.create_connection()

        MockManager.assert_called_once_with(
            "--interface socketcan_tmcl --port can1 --data-rate 1000000"
        )
        manager_instance.connect.assert_called_once()
        assert result is fake_connection


def test_create_module_uses_correct_module_id(fake_connection):
    with patch("tmcl_config.TMCM1260") as MockTMCM:
        module = tmcl_config.create_module(fake_connection, 1)

        MockTMCM.assert_called_once_with(fake_connection, module_id=1)
        assert module is MockTMCM.return_value


# -------------------------------------------------------------
# Axis config application
# -------------------------------------------------------------

def test_apply_axis_config_sets_all_parameters(fake_brake):
    config = {
        "MaxVelocity": 50000,
        "MaxAcceleration": 140000,
        "V1": 0,
    }

    tmcl_config.apply_axis_config(fake_brake, config)

    fake_brake.set_axis_parameter.assert_any_call(fake_brake.AP.MaxVelocity, 50000)
    fake_brake.set_axis_parameter.assert_any_call(fake_brake.AP.MaxAcceleration, 140000)
    fake_brake.set_axis_parameter.assert_any_call(fake_brake.AP.V1, 0)
    assert fake_brake.set_axis_parameter.call_count == 3


def test_apply_axis_config_order(fake_brake):
    config = {
        "MaxVelocity": 50000,
        "MaxAcceleration": 140000,
        "V1": 0,
    }

    tmcl_config.apply_axis_config(fake_brake, config)

    assert fake_brake.set_axis_parameter.call_args_list == [
        call(fake_brake.AP.MaxVelocity, 50000),
        call(fake_brake.AP.MaxAcceleration, 140000),
        call(fake_brake.AP.V1, 0),
    ]


def test_apply_axis_config_raises_if_param_missing(fake_brake):
    config = {"NonExistingParam": 123}

    with pytest.raises(AttributeError):
        tmcl_config.apply_axis_config(fake_brake, config)


# -------------------------------------------------------------
# Brake setup
# -------------------------------------------------------------

def test_setup_brake_creates_brake_module_and_applies_config(fake_connection):
    fake_brake = MagicMock()

    with patch("tmcl_config.create_module", return_value=fake_brake) as mock_create_module, \
         patch("tmcl_config.apply_axis_config") as mock_apply_config:

        result = tmcl_config.setup_brake(fake_connection)

    mock_create_module.assert_called_once_with(fake_connection, tmcl_config.MODULES["brake"])
    mock_apply_config.assert_called_once_with(fake_brake, tmcl_config.BRAKE_CONFIG)
    assert result is fake_brake


# -------------------------------------------------------------
# Main
# -------------------------------------------------------------

def test_main_closes_connection_on_success(fake_connection):
    with patch("tmcl_config.create_connection", return_value=fake_connection), \
         patch("tmcl_config.setup_brake") as mock_setup:

        tmcl_config.main()

    mock_setup.assert_called_once_with(fake_connection)
    fake_connection.close.assert_called_once()


def test_main_closes_connection_if_setup_fails(fake_connection):
    with patch("tmcl_config.create_connection", return_value=fake_connection), \
         patch("tmcl_config.setup_brake", side_effect=RuntimeError("setup failed")):

        with pytest.raises(RuntimeError, match="setup failed"):
            tmcl_config.main()

    fake_connection.close.assert_called_once()