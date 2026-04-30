#!/usr/bin/env python3
import time

modules = {
    "steering": 3,
    "brake": 1
}


def create_brake_motor(stepper_cls=None):
    if stepper_cls is None:
        from shared_objects.Motor import Stepper
        stepper_cls = Stepper

    return stepper_cls(
        "can",
        1_000_000,
        modules["brake"],
        50_000,
        140_000,
        140_000,
        0,
        50_000,
        50_000,
    )


def run_brake_test(delay=3, stepper_cls=None):
    brake_motor = create_brake_motor(stepper_cls=stepper_cls)
    try:
        print(f"Brake test starts in {delay} seconds…")
        time.sleep(delay)
        brake_motor.brake()
        print("Brake sequence completed successfully.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        brake_motor.disconnect_motor()
        print("Motor disconnected. Test complete.")


if __name__ == "__main__":
    run_brake_test()