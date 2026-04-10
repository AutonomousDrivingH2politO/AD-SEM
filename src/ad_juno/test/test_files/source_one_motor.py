#!/usr/bin/env python3
################################################################################
# Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
# (now owned by Analog Devices Inc.),
#
# Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
# proprietary & confidential to Analog Devices, Inc. and its licensors.
################################################################################

import time
import pytrinamic
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260


CONNECTION_STRING = "--interface socketcan_tmcl"
ROTATE_VELOCITY = 20000
ROTATE_TIME_S = 5
POST_STOP_DELAY_S = 2
FURTHEST_POINT_DELAY_S = 3
POLL_INTERVAL_S = 0.2
MOVE_BY_VELOCITY = 10000
MOVE_TO_ZERO_VELOCITY = 20000


def show_library_info():
    pytrinamic.show_info()


def create_connection_manager(connection_string: str = CONNECTION_STRING):
    return ConnectionManager(connection_string)


def create_module(interface):
    return TMCM1260(interface)


def get_motor(module, index: int = 0):
    return module.motors[index]


def configure_drive_settings(motor):
    motor.drive_settings.max_current = 50
    motor.drive_settings.standby_current = 0
    motor.drive_settings.boost_current = 0
    motor.drive_settings.microstep_resolution = (
        motor.ENUM.MicrostepResolution256Microsteps
    )


def configure_linear_ramp(motor):
    motor.linear_ramp.max_velocity = 20000
    motor.linear_ramp.max_acceleration = 40000


def prepare_motor(motor):
    print("Preparing parameters...")
    configure_drive_settings(motor)
    print(motor.drive_settings)
    configure_linear_ramp(motor)
    print(motor.linear_ramp)

    time.sleep(1.0)

    motor.set_axis_parameter(motor.AP.RelativePositioningOption, 1)
    motor.actual_position = 0


def rotate_and_stop(motor):
    print("Rotating...")
    motor.rotate(ROTATE_VELOCITY)
    time.sleep(ROTATE_TIME_S)

    print("Stopping...")
    motor.stop()

    print("ActualPosition = {}".format(motor.actual_position))
    time.sleep(POST_STOP_DELAY_S)


def wait_until_position_reached(motor, poll_interval_s: float = POLL_INTERVAL_S):
    while not motor.get_position_reached():
        print(
            "target position: "
            + str(motor.target_position)
            + " actual position: "
            + str(motor.actual_position)
        )
        time.sleep(poll_interval_s)


def move_farther(motor):
    print("Doubling moved distance.")
    motor.move_by(motor.actual_position, MOVE_BY_VELOCITY)
    wait_until_position_reached(motor)
    print("Furthest point reached.")
    print("ActualPosition = {}".format(motor.actual_position))


def move_back_to_zero(motor):
    time.sleep(FURTHEST_POINT_DELAY_S)
    print("Moving back to 0...")
    motor.move_to(0, MOVE_TO_ZERO_VELOCITY)
    wait_until_position_reached(motor)
    print("Reached position 0.")


def run_demo(motor):
    prepare_motor(motor)
    rotate_and_stop(motor)
    move_farther(motor)
    move_back_to_zero(motor)


def main():
    show_library_info()

    with create_connection_manager().connect() as my_interface:
        print(my_interface)

        module = create_module(my_interface)
        print(module)

        motor = get_motor(module, 0)
        run_demo(motor)

    print("\nReady.")


if __name__ == "__main__":
    main()