#!/usr/bin/env python3
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260

import time
import struct


class Stepper:
    """
    High-level controller for TMCM-1260 stepper motor over CAN or USB.
    Handles configuration, absolute positioning, braking, and encoder sync.
    """

    def __init__(self, interface, data_rate, module_id,
                 max_velocity, max_acc, MaxDeceleration, V1, A1, D1):

        # Setup communication interface
        if interface == "can":
            communication = "socketcan_tmcl"
        elif interface == "usb":
            communication = "usb_tmcl"
        else:
            raise ValueError(f"Unsupported interface: {interface}")

        # Connect to the module
        self.interface = ConnectionManager(
            f"--interface {communication} --port can1 --data-rate {data_rate}"
        ).connect()

        self.module = TMCM1260(self.interface, module_id=module_id)
        self.motor = self.module.motors[0]
        self.module_id = module_id

        # Configure motor motion parameters
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity, max_velocity)
        self.motor.set_axis_parameter(self.motor.AP.MaxAcceleration, max_acc)
        self.motor.set_axis_parameter(self.motor.AP.MaxDeceleration, MaxDeceleration)
        self.motor.set_axis_parameter(self.motor.AP.V1, V1)
        self.motor.set_axis_parameter(self.motor.AP.A1, A1)
        self.motor.set_axis_parameter(self.motor.AP.D1, D1)
        self.motor.set_axis_parameter(self.motor.AP.StartVelocity, 1000)
        self.motor.set_axis_parameter(self.motor.AP.StopVelocity, 1000)
        self.motor.set_axis_parameter(self.motor.AP.RampWaitTime, 0)
        self.motor.set_axis_parameter(self.motor.AP.MaxCurrent, 200)
        self.motor.set_axis_parameter(self.motor.AP.StandbyCurrent, 100)
        self.motor.set_axis_parameter(self.motor.AP.SG2Threshold, 11)
        self.motor.set_axis_parameter(self.motor.AP.SG2FilterEnable, 0)
        # For disabling StallGuard2 change 181(SmartEnergyStallVelocity) to 7999774 to enable from velocity
        # 0-7999774
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyStallVelocity, 7999774)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresis, 15)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresisStart, 0)
        self.motor.set_axis_parameter(self.motor.AP.SECUS, 1)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyThresholdSpeed, 7999774)

        # Driver settings
        self.motor.drive_settings.boost_current = 0
        self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution256Microsteps

        # Zero both coordinate systems so firmware step counter and encoder
        # share the same origin. The firmware is open-loop (step pulses only),
        # so the encoder is ground truth. Both must be in the same reference
        # frame for slip detection and position sync to work correctly.
        self.motor.set_axis_parameter(self.motor.AP.ActualPosition, 0)
        self.motor.set_axis_parameter(self.motor.AP.EncoderPosition, 0)
        self.virtual_position = 0

        print(f"[Stepper] Initialized on port can1 id={module_id}")
        print(self.motor.drive_settings)

    def move_to_position(self, target: int):
        """
        Move to an absolute position and update internal virtual position tracker.
        """
        self.motor.move_to(target)
        self.virtual_position = target

    def _to_signed_32(self, value):
        """Convert unsigned 32-bit CAN reading to signed."""
        return struct.unpack('>i', struct.pack('>I', value & 0xFFFFFFFF))[0]

    def get_encoder_position(self) -> int:
        """Read current encoder position as a signed 32-bit value."""
        return self._to_signed_32(
            self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        )

    def get_actual_position(self) -> int:
        """Read current actual (commanded) position as a signed 32-bit value."""
        return self._to_signed_32(
            self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        )

    def _sync_position_to_encoder(self):
        """
        Sync the firmware's open-loop step counter to the encoder's actual
        position. Without this, move_to(target) is a no-op after slip because
        the firmware already believes it is at target.

        After calling this, the firmware sees the real position and move_to()
        will generate the correct number of corrective steps.
        """
        encoder_pos = self.get_encoder_position()
        self.motor.set_axis_parameter(self.motor.AP.ActualPosition, encoder_pos)

    def brake(self, retract_ticks: int = 70000, brake_duration: float = 5.0,
              slip_threshold: int = 200):
        """
        Executes a timed braking maneuver:
        1. Retracts by retract_ticks from current position.
        2. Holds brake position — monitors encoder at 1Hz for slip.
           If slip exceeds threshold, syncs firmware position to encoder
           and reissues move_to so the ramp generator drives corrective steps.
        3. Returns to the original start position.
        """
        print("[Stepper] Starting timed brake...")

        # --- Step 1: Move to brake position ---
        start_pos = self.get_actual_position()
        target_pos = start_pos - retract_ticks
        print(f"[Stepper] Moving from {start_pos} -> {target_pos}")
        self.move_to_position(target_pos)

        while not self.motor.get_position_reached():
            time.sleep(0.1)

        # --- Step 2: Hold with slip correction ---
        # Monitor at 10Hz — just enough to catch slip without flooding CAN.
        # The hardware holding torque does the real work; this only intervenes
        # when external load overcomes it and the shaft physically slips.
        print(f"[Stepper] Holding brake for {brake_duration:.1f}s "
              f"(slip threshold={slip_threshold} ticks)")

        t_start = time.time()
        while (time.time() - t_start) < brake_duration:
            try:
                encoder_pos = self.get_encoder_position()
                slip = abs(target_pos - encoder_pos)

                if slip > slip_threshold:
                    # Firmware thinks it's at target (open-loop). Sync to
                    # encoder so it sees the real discrepancy, then reissue.
                    print(f"[Stepper] Slip detected ({slip} ticks). "
                          f"Syncing position and reissuing move_to({target_pos})")
                    self._sync_position_to_encoder()
                    self.move_to_position(target_pos)

                time.sleep(0.1)
            except Exception as e:
                print(f"[Stepper] CAN error during hold: {e}")
                time.sleep(0.1)

        # --- Step 3: Return to original position ---
        # Sync before return in case slip accumulated during hold
        self._sync_position_to_encoder()
        print(f"[Stepper] Returning to start position: {start_pos}")
        self.move_to_position(start_pos)

        while not self.motor.get_position_reached():
            time.sleep(0.1)

        print("[Stepper] Timed brake completed.")

    def move_stepper(self, step):
        """
        Moves the stepper to an absolute position.
        """
        print(f"[Stepper] Moving to position: {step}")
        self.motor.move_to(step)

    def disconnect_motor(self):
        """
        Stops the motor and closes the communication interface.
        """
        self.motor.stop()
        self.interface.close()
        print("[Stepper] Disconnected")

