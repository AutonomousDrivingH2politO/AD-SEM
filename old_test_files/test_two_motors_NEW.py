#!/usr/bin/env python3
"""
test_motors.py

Standalone test for the Stepper class (no ROS required).
  - Brake motor   (module_id=1): mirrors Stepper.brake() with 70k retract
  - Steering motor (module_id=3): sweeps to full-lock using the same
    angle->position conversion as SteeringNode
"""

import time
from shared_objects.Motor import Stepper

# --- Steering geometry (must match steering_node.py) -----------------------
TRANSMISSION_RATIO = 14
MAX_ANGLE = 10          # degrees
STEP_ANGLE = 1.8        # degrees per full step
MICROSTEPS = 256

def angle_to_position(angle_deg: float) -> int:
    steps = (angle_deg / STEP_ANGLE) * MICROSTEPS
    return -int(steps * TRANSMISSION_RATIO)

HOME_POSITION = 0
SETTLE_TIME = 2.0
TIMEOUT = 15.0
POLL_DT = 0.1

COMMON_CFG = dict(
    interface="can",
    data_rate=1000000,
    max_velocity=50_000,
    max_acc=32_015,
    MaxDeceleration=50_000,
    V1=50_000,
    A1=32_830,
    D1=31_898,
)

# --- Diagnostics -----------------------------------------------------------
AP_ENCODER_MODE      = 201
AP_ENCODER_POSITION  = 209
AP_ENCODER_PRESCALER = 210
AP_MAX_DEVIATION     = 212


def print_encoder_config(motor, name: str):
    print(f"[{name}] --- Encoder config ---")
    for ap_id, label in [
        (AP_ENCODER_MODE,      "EncoderMode (201)"),
        (AP_ENCODER_PRESCALER, "EncoderPrescaler (210)"),
        (AP_MAX_DEVIATION,     "MaxDeviation (212)"),
    ]:
        try:
            val = motor.motor.get_axis_parameter(ap_id)
            print(f"[{name}]   {label} = {val}")
        except Exception as e:
            print(f"[{name}]   {label} = <not readable: {e}>")


def _wait_reached(motor, name: str, tag: str) -> bool:
    time.sleep(POLL_DT)
    t0 = time.time()
    while not motor.motor.get_position_reached():
        if time.time() - t0 > TIMEOUT:
            print(f"[{name}] ❌ TIMEOUT during {tag}")
            return False
        time.sleep(POLL_DT)
    return True


def test_direction_and_scale(motor, name: str, probe_ticks: int = 5000):
    """Sanity-check direction convention and microstep<->encoder scale."""
    print(f"[{name}] --- Direction & scale probe (±{probe_ticks}) ---")

    def read_pair():
        act = motor._to_signed_32(
            motor.motor.get_axis_parameter(motor.motor.AP.ActualPosition)
        )
        enc = motor._to_signed_32(
            motor.motor.get_axis_parameter(motor.motor.AP.EncoderPosition)
        )
        return act, enc

    act0, enc0 = read_pair()
    print(f"[{name}]   start: actual={act0}, encoder={enc0}")

    motor.move_to_position(act0 + probe_ticks)
    if not _wait_reached(motor, name, "+probe"): return
    time.sleep(0.3)
    act1, enc1 = read_pair()
    d_act_p = act1 - act0
    d_enc_p = enc1 - enc0
    ratio_p = (d_enc_p / d_act_p) if d_act_p else float("nan")
    print(f"[{name}]   +move: Δactual={d_act_p}, Δencoder={d_enc_p}, enc/act={ratio_p:.3f}")

    motor.move_to_position(act0 - probe_ticks)
    if not _wait_reached(motor, name, "-probe"): return
    time.sleep(0.3)
    act2, enc2 = read_pair()
    d_act_n = act2 - act1
    d_enc_n = enc2 - enc1
    ratio_n = (d_enc_n / d_act_n) if d_act_n else float("nan")
    print(f"[{name}]   -move: Δactual={d_act_n}, Δencoder={d_enc_n}, enc/act={ratio_n:.3f}")

    motor.move_to_position(act0)
    _wait_reached(motor, name, "return")

    same_sign_p = (d_act_p > 0) == (d_enc_p > 0)
    same_sign_n = (d_act_n > 0) == (d_enc_n > 0)
    if same_sign_p and same_sign_n:
        print(f"[{name}]   ✅ Encoder direction matches commanded direction")
    else:
        print(f"[{name}]   ⚠️  Encoder direction INVERTED vs commanded "
              f"(+:{same_sign_p}, -:{same_sign_n})")

    for tag, r in [("+", ratio_p), ("-", ratio_n)]:
        if 0.9 <= abs(r) <= 1.1:
            print(f"[{name}]   ✅ {tag} scale ≈ 1:1")
        else:
            print(f"[{name}]   ⚠️  {tag} scale = {r:.3f} — EncoderPrescaler (AP 210) "
                  f"likely not set; actual vs encoder are NOT in the same units")


def zero_positions(motor, name: str):
    ap = motor.motor.AP
    motor.motor.set_axis_parameter(ap.ActualPosition, 0)
    motor.motor.set_axis_parameter(ap.EncoderPosition, 0)
    motor.virtual_position = 0
    motor.last_encoder_position = 0
    motor.unwrapped_position = 0
    actual = motor.motor.get_actual_position()
    enc = motor.motor.get_axis_parameter(ap.EncoderPosition)
    print(f"[{name}] Zeroed -> actual={actual}, encoder={enc}")


def test_brake(name: str, module_id: int):
    """Mirrors Stepper.brake(): 70k retract, 5s hold w/ drift check, return."""
    print(f"\n========== Testing {name} (module_id={module_id}) ==========")
    motor = None
    try:
        motor = Stepper(module_id=module_id, **COMMON_CFG)
        zero_positions(motor, name)
        print_encoder_config(motor, name)
        test_direction_and_scale(motor, name, probe_ticks=5000)

        print(f"[{name}] Starting timed brake test...")
        retract_ticks = 70000
        brake_duration = 5.0
        align_threshold = 200

        ap = motor.motor.AP

        # Step 1: move to brake position (start from ActualPosition, like brake())
        start_pos = motor._to_signed_32(motor.motor.get_axis_parameter(ap.ActualPosition))
        target_pos = start_pos - retract_ticks
        print(f"[{name}] Moving from {start_pos} → {target_pos}")
        motor.move_to_position(target_pos)
        _wait_reached(motor, name, "brake target")

        # Snapshot both counters AFTER the move completes — drift = relative slip
        act_at_hold_start = motor._to_signed_32(motor.motor.get_axis_parameter(ap.ActualPosition))
        enc_at_hold_start = motor._to_signed_32(motor.motor.get_axis_parameter(ap.EncoderPosition))
        print(f"[{name}] Hold start: actual={act_at_hold_start}, encoder={enc_at_hold_start}")

        # Step 2: monitor alignment during brake
        print(f"[{name}] Holding brake for {brake_duration:.1f}s with live re-alignment...")
        t_start = time.time()
        while (time.time() - t_start) < brake_duration:
            try:
                actual_pos = motor._to_signed_32(motor.motor.get_axis_parameter(ap.ActualPosition))
                encoder_pos = motor._to_signed_32(motor.motor.get_axis_parameter(ap.EncoderPosition))
                act_delta = actual_pos  - act_at_hold_start
                enc_delta = encoder_pos - enc_at_hold_start
                drift = abs(act_delta - enc_delta)
                print(f"[{name}] actual={actual_pos}, encoder={encoder_pos}, "
                      f"Δact={act_delta}, Δenc={enc_delta}, drift={drift}")
                if drift > align_threshold:
                    print(f"[{name}] Drift detected ({drift}>{align_threshold}). "
                          f"Reissuing move to {target_pos}")
                    motor.move_to_position(target_pos)
                time.sleep(0.5)
            except Exception as e:
                print(f"[{name}] CAN timeout during drift check: {e}")

        # Step 3: return
        print(f"[{name}] Returning to start position: {start_pos}")
        motor.move_to_position(start_pos)
        _wait_reached(motor, name, "return home")
        print(f"[{name}] ✅ Timed brake test completed.")

    except Exception as e:
        print(f"[{name}] ❌ ERROR: {e}")
    finally:
        if motor is not None:
            try:
                motor.disconnect_motor()
            except Exception as e:
                print(f"[{name}] disconnect error: {e}")


def test_steering(name: str, module_id: int):
    """Sweeps steering through the same targets the ROS node would command."""
    print(f"\n========== Testing {name} (module_id={module_id}) ==========")
    motor = None
    try:
        motor = Stepper(module_id=module_id, **COMMON_CFG)
        zero_positions(motor, name)
        print_encoder_config(motor, name)
        test_direction_and_scale(motor, name, probe_ticks=5000)

        ap = motor.motor.AP

        # Match steering_node: use move_stepper and real angle->position conversion
        sweep_angles = [0.0, MAX_ANGLE / 2, MAX_ANGLE, 0.0, -MAX_ANGLE / 2, -MAX_ANGLE, 0.0]
        for angle in sweep_angles:
            target = angle_to_position(angle)
            print(f"[{name}] angle={angle:+.1f}° -> target={target}")
            motor.move_stepper(target)

            t0 = time.time()
            reached = False
            while time.time() - t0 < TIMEOUT:
                if motor.motor.get_position_reached():
                    reached = True
                    break
                time.sleep(POLL_DT)
            if not reached:
                print(f"[{name}] ❌ TIMEOUT reaching angle {angle}")
                continue

            time.sleep(SETTLE_TIME)
            actual = motor._to_signed_32(motor.motor.get_axis_parameter(ap.ActualPosition))
            encoder = motor._to_signed_32(motor.motor.get_axis_parameter(ap.EncoderPosition))
            print(f"[{name}]   reached: actual={actual}, encoder={encoder}, "
                  f"err(actual-target)={actual - target}")

        print(f"[{name}] Returning to HOME={HOME_POSITION}")
        motor.move_stepper(HOME_POSITION)
        _wait_reached(motor, name, "home")
        print(f"[{name}] Done.")

    except Exception as e:
        print(f"[{name}] ❌ ERROR: {e}")
    finally:
        if motor is not None:
            try:
                motor.disconnect_motor()
            except Exception as e:
                print(f"[{name}] disconnect error: {e}")


def main():
    
    #test_brake("BRAKE", module_id=1)
    time.sleep(1.0)
    test_steering("STEERING", module_id=3)
    print("\nAll tests finished.")


if __name__ == "__main__":
    main()
