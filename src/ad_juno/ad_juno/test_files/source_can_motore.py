#!/usr/bin/env python3
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260
import time


CONNECTION_STRING = "--interface socketcan_tmcl --port can1"
MODULE_ID = 2
MOTOR_INDEX = 0
MAX_VELOCITY = 2000
MAX_ACCELERATION = 1000
ROTATE_VELOCITY = 5000
RUN_TIME_S = 5


def create_connection():
    connection_manager = ConnectionManager(CONNECTION_STRING)
    return connection_manager.connect()


def setup_motor(connection):
    module = TMCM1260(connection, module_id=MODULE_ID)
    motor = module.motors[MOTOR_INDEX]
    motor.set_axis_parameter(motor.AP.MaxVelocity, MAX_VELOCITY)
    motor.set_axis_parameter(motor.AP.MaxAcceleration, MAX_ACCELERATION)
    return motor


def run_motor_sequence(motor, run_time_s=RUN_TIME_S):
    motor.rotate(ROTATE_VELOCITY)
    print("Il motore sta ruotando")
    time.sleep(run_time_s)
    motor.stop()
    print("The engine is stopped")


def main():
    connection = create_connection()
    try:
        motor = setup_motor(connection)
        run_motor_sequence(motor)
    finally:
        connection.close()


if __name__ == "__main__":
    main()