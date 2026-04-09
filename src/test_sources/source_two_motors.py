#!/usr/bin/env python3
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260
import time

MODULES = {
    "steering": 3,
    "brake": 1,
}

CONNECTION_STRING = "--interface socketcan_tmcl --port can1 --data-rate 1000000"
ROTATE_SPEED = 20000
STEP_DELAY_S = 1


def create_connection(connection_string: str = CONNECTION_STRING):
    manager = ConnectionManager(connection_string)
    return manager.connect()


def create_motor(connection, module_id: int):
    return TMCM1260(connection, module_id=module_id)


def create_steering_motor(connection):
    return create_motor(connection, MODULES["steering"])


def create_brake_motor(connection):
    return create_motor(connection, MODULES["brake"])


def run_single_cycle(steering, brake, delay_s: float = STEP_DELAY_S):
    steering.rotate(0, ROTATE_SPEED)
    time.sleep(delay_s)
    steering.stop(0)
    time.sleep(delay_s)

    brake.rotate(0, ROTATE_SPEED)
    time.sleep(delay_s)
    brake.stop(0)
    time.sleep(delay_s)


def main_loop():
    interface_br = create_connection()
    interface_st = create_connection()

    try:
        steering = create_steering_motor(interface_st)
        brake = create_brake_motor(interface_br)

        while True:
            run_single_cycle(steering, brake)
    finally:
        interface_st.close()
        interface_br.close()


if __name__ == "__main__":
    main_loop()