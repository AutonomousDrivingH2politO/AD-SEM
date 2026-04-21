#!/usr/bin/env python3
import time
import sys

# Import your new Stepper class (assuming the file is named motor.py)
from Motor import Stepper


def main():
    print("Initializing Stepper Module...")

    # Initialize the stepper using the same parameters you use in stop_node.py
    try:
        stepper = Stepper(
            interface="can",
            data_rate=1000000,
            module_id=1,
            max_velocity=80_000,
            max_acc=50_000,
            MaxDeceleration=50_000,
            V1=100_000,
            A1=50_000,
            D1=50_000,
        )
    except Exception as e:
        print(f"Failed to connect to the motor: {e}")
        sys.exit(1)

    step_angle = 1.8
    microsteps = 256

    print("\n--- Stepper Value Search Ready ---")

    while True:
        try:
            move = (
                input(
                    "\nCommands:\n [+] Move Positive\n [-] Move Negative\n [p] Set Position to 0\n [s] Tune StallGuard Value\n [c] Tune Max Current\n [q] Quit\n> "
                )
                .strip()
                .lower()
            )

            if move == "+":
                steps = input("Enter steps to move (default 10000): ")
                steps = int(steps) if steps else 10000

                # Using absolute positioning relative to current actual position
                current_pos = stepper.get_actual_position()
                stepper.move_to_position(current_pos + steps)

                # Wait for the motor to reach the target position
                while not stepper.motor.get_position_reached():
                    time.sleep(0.1)

                pos = stepper.get_actual_position()
                angle = (pos / microsteps) * step_angle
                print(f"Position is {pos} | Angle is {angle:.2f}°")

            elif move == "-":
                steps = input("Enter steps to move (default 10000): ")
                steps = int(steps) if steps else 10000

                current_pos = stepper.get_actual_position()
                stepper.move_to_position(current_pos - steps)

                while not stepper.motor.get_position_reached():
                    time.sleep(0.1)

                pos = stepper.get_actual_position()
                angle = (pos / microsteps) * step_angle
                print(f"Position is {pos} | Angle is {angle:.2f}°")

            elif move == "p":
                # Zero out both firmware counter and encoder
                stepper.motor.set_axis_parameter(stepper.motor.AP.ActualPosition, 0)
                stepper.motor.set_axis_parameter(stepper.motor.AP.EncoderPosition, 0)
                stepper.virtual_position = 0
                print(
                    f"Position reset. Actual: {stepper.get_actual_position()}, Encoder: {stepper.get_encoder_position()}"
                )

            elif move == "s":
                current_sg = stepper.motor.get_axis_parameter(
                    stepper.motor.AP.SG2Threshold
                )
                print(f"Current StallGuard Threshold is: {current_sg}")
                sg_val = input("Enter new StallGuard Threshold (e.g., 11 to 63): ")
                if sg_val.isdigit():
                    stepper.motor.set_axis_parameter(
                        stepper.motor.AP.SG2Threshold, int(sg_val)
                    )
                    print(f"StallGuard threshold updated to {sg_val}")

            elif move == "c":
                current_c = stepper.motor.get_axis_parameter(
                    stepper.motor.AP.MaxCurrent
                )
                print(f"Current Max Current is: {current_c}")
                current_val = input("Enter new Max Current (e.g., 160, 200, 255): ")
                if current_val.isdigit():
                    stepper.motor.set_axis_parameter(
                        stepper.motor.AP.MaxCurrent, int(current_val)
                    )
                    print(f"Max Current updated to {current_val}")

            elif move == "q":
                print("Quitting value search...")
                break

            else:
                print("Unrecognized command.")

        except KeyboardInterrupt:
            print("\nKeyboard Interrupt detected. Stopping motor.")
            stepper.motor.stop()
            break
        except Exception as e:
            print(f"\nAn error occurred: {e}")

    # Safely close connection on exit
    stepper.disconnect_motor()


if __name__ == "__main__":
    main()
