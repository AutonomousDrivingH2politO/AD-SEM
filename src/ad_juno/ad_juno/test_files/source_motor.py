#!/usr/bin/env python3
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260

MODULES = {
    "steering": 3,
    "brake": 1,
}

CONNECTION_STRING = "--interface socketcan_tmcl --port can1 --data-rate 1000000"

BRAKE_CONFIG = {
    "MaxVelocity": 50000,
    "MaxAcceleration": 140000,
    "V1": 0,
    "StartVelocity": 1_000,
    "StopVelocity": 1_000,
    "RampWaitTime": 0,
    "MaxCurrent": 200,
    "StandbyCurrent": 100,
    "SG2Threshold": 11,
    "SG2FilterEnable": 0,
    "SmartEnergyStallVelocity": 0,
    "SmartEnergyHysteresis": 15,
    "SmartEnergyHysteresisStart": 0,
    "SECUS": 1,
    "SmartEnergyThresholdSpeed": 7_999_774,
}


def create_connection(connection_string: str = CONNECTION_STRING):
    manager = ConnectionManager(connection_string)
    return manager.connect()


def create_module(connection, module_id: int):
    return TMCM1260(connection, module_id=module_id)


def apply_axis_config(motor, config: dict):
    for param_name, value in config.items():
        motor.set_axis_parameter(getattr(motor.AP, param_name), value)


def setup_brake(connection):
    brake = create_module(connection, MODULES["brake"])
    apply_axis_config(brake, BRAKE_CONFIG)
    return brake


def main():
    connection = create_connection()
    try:
        setup_brake(connection)
    finally:
        connection.close()


if __name__ == "__main__":
    main()