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

    def __init__(
        self,
        interface,
        module_id,
        max_velocity,
        max_acc,
        MaxDeceleration,
        V1,
        A1,
        D1,
        data_rate=1000000,
        port=None,
    ):

        if interface == "can":
            communication = "socketcan_tmcl"
            _port = port or "can1"
            connection_str = (
                f"--interface {communication} --port {_port} --data-rate {data_rate}"
            )
        elif interface == "usb":
            communication = "usb_tmcl"
            _port = port or "/dev/ttyACM0"
            # USB-TMCL does not use --data-rate; the baud rate is fixed by the module
            connection_str = f"--interface {communication} --port {_port}"
        else:
            raise ValueError(
                f"Unsupported interface: {interface!r}. Use 'can' or 'usb'."
            )

        self.interface = ConnectionManager(connection_str).connect()

        self.module = TMCM1260(self.interface, module_id=module_id)
        self.motor = self.module.motors[0]
        self.module_id = module_id

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
        # Standby current: reduce heat when idle, but keep enough to resist back-drive
        self.motor.set_axis_parameter(self.motor.AP.StandbyCurrent, 100)

        self.motor.set_axis_parameter(self.motor.AP.SG2Threshold, 11)
        self.motor.set_axis_parameter(self.motor.AP.SG2FilterEnable, 0)
        # StallGuard disabled during braking to avoid false stall detection
        # under the sustained load of holding the brake rope
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyStallVelocity, 7999774)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresis, 15)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresisStart, 0)
        self.motor.set_axis_parameter(self.motor.AP.SECUS, 1)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyThresholdSpeed, 7999774)

        self.motor.drive_settings.boost_current = 0
        self.motor.drive_settings.microstep_resolution = (
            self.motor.ENUM.MicrostepResolution256Microsteps
        )

        self.motor.set_axis_parameter(self.motor.AP.ActualPosition, 0)
        self.motor.set_axis_parameter(self.motor.AP.EncoderPosition, 0)
        self.virtual_position = 0

        print(
            f"[Stepper] Initialized on {_port!r} (interface={interface}, id={module_id})"
        )
        print(self.motor.drive_settings)

    def move_to_position(self, target: int):
        self.motor.move_to(target)
        self.virtual_position = target

    def _to_signed_32(self, value):
        return struct.unpack(">i", struct.pack(">I", value & 0xFFFFFFFF))[0]

    def get_encoder_position(self) -> int:
        return self._to_signed_32(
            self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        )

    def get_actual_position(self) -> int:
        return self._to_signed_32(
            self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        )

    def _sync_position_to_encoder(self):
        encoder_pos = self.get_encoder_position()
        self.motor.set_axis_parameter(self.motor.AP.ActualPosition, encoder_pos)

    def brake(
        self,
        retract_ticks: int = 70000,
        brake_duration: float = 5.0,
        slip_threshold: int = 200,
        approach_velocity: int = 20000,
    ):
        """
        Braking sequence:
        1. Slow approach to take up rope slack without slamming the mechanism.
        2. Hold position under load with aggressive slip correction at 10Hz.
        3. Return to start.

        Args:
            retract_ticks:    How far to wind the rope (must be correct sign for
                              your geometry — positive winds onto disk if motor
                              turns clockwise to pull rope).
            brake_duration:   Seconds to hold the brake.
            slip_threshold:   Encoder ticks of slip before corrective action.
            approach_velocity: Reduced velocity for the initial rope-take-up
                               move, to avoid shock-loading the rope.
        """
        print("[Stepper] Starting timed brake...")

        start_pos = self.get_actual_position()
        target_pos = start_pos + retract_ticks

        # --- Step 1: Slow approach to avoid rope jerk ---
        # Temporarily reduce velocity so the rope is taken up smoothly.
        # A sudden high-speed pull causes the rope to snap taut and rebound,
        # which looks exactly like slip and can damage the mechanism.
        original_velocity = self.motor.get_axis_parameter(self.motor.AP.MaxVelocity)
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity, approach_velocity)

        print(
            f"[Stepper] Slow approach {start_pos} -> {target_pos} "
            f"at v={approach_velocity}"
        )
        self.move_to_position(target_pos)

        while not self.motor.get_position_reached():
            time.sleep(0.05)

        # Restore full velocity for any subsequent moves
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity, original_velocity)

        # --- Step 2: Active hold with slip correction ---
        # Poll at 10Hz. On slip, sync firmware counter and reissue move_to,
        # then wait for position_reached before resuming the hold loop.
        # This prevents stacking redundant move commands.
        print(
            f"[Stepper] Holding brake for {brake_duration:.1f}s "
            f"(slip threshold={slip_threshold} ticks)"
        )

        t_start = time.time()
        while (time.time() - t_start) < brake_duration:
            try:
                encoder_pos = self.get_encoder_position()
                slip = abs(target_pos - encoder_pos)

                if slip > slip_threshold:
                    print(
                        f"[Stepper] Slip detected ({slip} ticks). "
                        f"Correcting -> {target_pos}"
                    )
                    self._sync_position_to_encoder()
                    self.move_to_position(target_pos)

                    # Wait for correction to complete before re-checking.
                    # Without this, the loop fires move_to repeatedly on every
                    # iteration while the motor is still responding, which can
                    # cause the ramp generator to reset mid-move and stall.
                    correction_timeout = time.time() + 3.0
                    while (
                        not self.motor.get_position_reached()
                        and time.time() < correction_timeout
                    ):
                        time.sleep(0.05)

                    if not self.motor.get_position_reached():
                        print(
                            "[Stepper] WARNING: Correction move timed out — "
                            "load may exceed motor torque capacity."
                        )

                time.sleep(0.1)
            except Exception as e:
                print(f"[Stepper] CAN error during hold: {e}")
                time.sleep(0.1)

        # --- Step 3: Sync then return ---
        self._sync_position_to_encoder()
        print(f"[Stepper] Returning to start: {start_pos}")
        self.move_to_position(start_pos)

        while not self.motor.get_position_reached():
            time.sleep(0.1)

        print("[Stepper] Timed brake completed.")

    def move_stepper(self, step):
        print(f"[Stepper] Moving to position: {step}")
        self.motor.move_to(step)

    def disconnect_motor(self):
        self.motor.stop()
        self.interface.close()
        print("[Stepper] Disconnected")
