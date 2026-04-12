#!/usr/bin/env python3
# import RPi.GPIO as GPIO
import Jetson.GPIO as GPIO
import time

INPUT_PIN = 18
POLL_INTERVAL_S = 0.1


def setup_gpio(pin: int) -> None:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def cleanup_gpio() -> None:
    GPIO.cleanup()


def read_pin(pin: int) -> int:
    return GPIO.input(pin)


def format_state_message(state: bool) -> str:
    return f"Initial state: {'HIGH (3.3V)' if state else 'LOW (0V)'}"


def handle_state_change(new_state: bool) -> None:
    if new_state:
        print("POWER DETECTED! (3.3V on pin)\n")
        print("System would be ENGAGED")
    else:
        print("Power removed (0V on pin)\n")
        print("System would be DISENGAGED")


def process_pin_state(last_state: bool, current_state: bool) -> bool:
    if current_state != last_state:
        handle_state_change(current_state)
        return current_state
    return last_state


def monitor_loop(pin: int, poll_interval_s: float) -> None:
    last_state = read_pin(pin)
    print(format_state_message(last_state))
    while True:
        current_state = read_pin(pin)
        last_state = process_pin_state(last_state, current_state)
        time.sleep(poll_interval_s)


def main() -> None:
    print(f"Monitoring GPIO pin {INPUT_PIN} for 3.3V power...")
    print("Press CTRL+C to exit")
    setup_gpio(INPUT_PIN)
    try:
        monitor_loop(pin=INPUT_PIN, poll_interval_s=POLL_INTERVAL_S)
    except KeyboardInterrupt:
        print("\nExiting program")
    finally:
        cleanup_gpio()


if __name__ == "__main__":
    main()
