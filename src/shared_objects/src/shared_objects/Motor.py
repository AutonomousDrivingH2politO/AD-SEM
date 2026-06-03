#!/usr/bin/env python3
"""
stepper.py — High-level controller for TMCM-1260 stepper motor over CAN or USB.

Two construction modes, selected by whether `params` is provided:

  1. Legacy (params=None) — positional velocity/acceleration args, matching the
     original constructor. Used for the steering module whose values are set
     directly in code.

        steering = Stepper(
            interface="can", module_id=2,
            max_velocity=80_000, max_acc=50_000, MaxDeceleration=50_000,
            V1=60_000, A1=50_000, D1=50_000,
        )

  2. Params-dict / JSON (params=<dict>) — all axis parameters come from a dict,
     typically loaded from a tuned JSON file via Stepper.from_json(). Used for
     the brake module.

        brake = Stepper.from_json(
            "params_brake.json", interface="can", module_id=1,
        )
        # or equivalently:
        brake = Stepper(interface="can", module_id=1, params=my_dict)
"""

import json
import struct
import time

from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260


# ---------------------------------------------------------------------------
# Mapping: params.json key  →  AP attribute name on motor.AP
# ---------------------------------------------------------------------------
# Keys absent from the JSON are simply skipped; an unknown key raises a warning.
# ---------------------------------------------------------------------------
_PARAM_MAP: dict[str, str | list[str]] = {
    # ── Ramp generator ──────────────────────────────────────────────────────
    "StartVelocity":    "StartVelocity",
    "StopVelocity":     "StopVelocity",
    "MaxVelocity":      "MaxVelocity",
    "MaxAcceleration":  "MaxAcceleration",
    "MaxDeceleration":  "MaxDeceleration",
    "V1":               "V1",
    "A1":               "A1",
    "D1":               "D1",
    # ── Current ─────────────────────────────────────────────────────────────
    "MaxCurrent":       "MaxCurrent",
    "StandbyCurrent":   "StandbyCurrent",
    # ── Microstepping ────────────────────────────────────────────────────────
    "MicrostepResolution": "MicrostepResolution",
    # ── StallGuard2 ──────────────────────────────────────────────────────────
    "SG2Threshold":     "SG2Threshold",
    # StopOnStallVel: velocity threshold below which stall-stop is active.
    # Set to 0 to disable stall-stop entirely.
    # NOTE: no longer writes to SmartEnergyThresholdSpeed — use
    # CoolStepThresholdSpeed for that register independently.
    "StopOnStallVel":   "SmartEnergyStallVelocity",
    # ── CoolStep (SmartEnergy) ───────────────────────────────────────────────
    # CoolStepThresholdSpeed → TCOOLTHRS (AP 182).
    #   0          = CoolStep disabled (motor never triggers the threshold).
    #   large val  = CoolStep active across all speeds (e.g. 7_999_774).
    # This is now a separate key from StopOnStallVel so the two thresholds
    # can be tuned independently.
    "CoolStepThresholdSpeed": "SmartEnergyThresholdSpeed",
    # CoolStepMin  → SEMIN  (lower current-reduction threshold, AP 172)
    # CoolStepMax  → SEMAX  (upper hysteresis,                  AP 170)
    # CoolStepUpStep   → SECUS  (current increment step size,   AP 171)
    # CoolStepDownStep → SECDS  (current decrement step size,   AP 169)
    # CoolStepIMin → SEIMIN (minimum CoolStep current,          AP 168)
    "CoolStepMin":      "SmartEnergyHysteresisStart",
    "CoolStepMax":      "SmartEnergyHysteresis",
    "CoolStepUpStep":   "SECUS",
    "CoolStepDownStep": "SECDS",
    "CoolStepIMin":     "SEIMIN",
    # ── StealthChop / PWM ────────────────────────────────────────────────────
    # Exposed here so the JSON can document intent, but PWMMode is also
    # enforced in _FIXED_PARAMS below to guarantee StealthChop stays off.
    "PWMMode":          "PWMMode",
    "SmartEnergyThresholdSpeed": "SmartEnergyThresholdSpeed",
}

# Parameters that are always forced to a fixed value regardless of the JSON,
# because they control low-level safety behaviour we never want overridden.
_FIXED_PARAMS: dict[str, int] = {
    "SG2FilterEnable": 0,   # keep SG2 responsive; filter adds 4-cycle lag
    "RampWaitTime":    0,   # no dwell between ramp phases
    # StealthChop must be OFF on the brake motor: the back-EMF model it uses
    # reduces holding torque at low speed, which is exactly when the brake is
    # under load.  PWMMode=0 forces SpreadCycle (full torque, deterministic
    # current control) regardless of what the JSON says.
    #"PWMMode":         0,
}


class Stepper:
    """
    High-level controller for TMCM-1260 stepper motor over CAN or USB.
    Handles configuration, absolute positioning, braking, and encoder sync.
    """

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(
        self,
        interface: str,
        module_id: int,
        # ── Legacy positional args (used when params=None) ─────────────────
        max_velocity: int = 0,
        max_acc: int = 0,
        MaxDeceleration: int = 0,
        V1: int = 0,
        A1: int = 0,
        D1: int = 0,
        # ── Params-dict path (overrides all positional args when provided) ─
        params: dict | None = None,
        # ── Connection args ────────────────────────────────────────────────
        data_rate: int = 1_000_000,
        port: str | None = None,
    ):
        """
        Args:
            interface:       "can" or "usb".
            module_id:       CAN/USB module address (1 = brake, 2 = steering).
            max_velocity:    [Legacy] MaxVelocity axis parameter.
            max_acc:         [Legacy] MaxAcceleration axis parameter.
            MaxDeceleration: [Legacy] MaxDeceleration axis parameter.
            V1:              [Legacy] V1 axis parameter.
            A1:              [Legacy] A1 axis parameter.
            D1:              [Legacy] D1 axis parameter.
            params:          When provided, ALL axis parameters are taken from
                             this dict (legacy positional args are ignored).
                             Typically loaded via Stepper.from_json().
            data_rate:       CAN bus data rate in bps (ignored for USB).
            port:            Override the default port ("can1" / "/dev/ttyACM0").
        """
        # ── Connection ────────────────────────────────────────────────────
        if interface == "can":
            _port = port or "can1"
            conn_str = (
                f"--interface socketcan_tmcl --port {_port} --data-rate {data_rate}"
            )
        elif interface == "usb":
            _port = port or "/dev/ttyACM0"
            conn_str = f"--interface usb_tmcl --port {_port}"
        else:
            raise ValueError(
                f"Unsupported interface: {interface!r}. Use 'can' or 'usb'."
            )

        self.interface = ConnectionManager(conn_str).connect()
        self.module = TMCM1260(self.interface, module_id=module_id)
        self.motor = self.module.motors[0]
        self.module_id = module_id

        # ── Apply parameters ──────────────────────────────────────────────
        if params is not None:
            self._apply_params(params)
        else:
            # Legacy path: mirror the original hardcoded constructor exactly.
            self.motor.set_axis_parameter(self.motor.AP.MaxVelocity,      max_velocity)
            self.motor.set_axis_parameter(self.motor.AP.MaxAcceleration,  max_acc)
            self.motor.set_axis_parameter(self.motor.AP.MaxDeceleration,  MaxDeceleration)
            self.motor.set_axis_parameter(self.motor.AP.V1,               V1)
            self.motor.set_axis_parameter(self.motor.AP.A1,               A1)
            self.motor.set_axis_parameter(self.motor.AP.D1,               D1)
            self.motor.set_axis_parameter(self.motor.AP.StartVelocity,    1000)
            self.motor.set_axis_parameter(self.motor.AP.StopVelocity,     10000)
            self.motor.set_axis_parameter(self.motor.AP.MaxCurrent,       200)
            self.motor.set_axis_parameter(self.motor.AP.StandbyCurrent,   100)
            self.motor.set_axis_parameter(self.motor.AP.SG2Threshold,     11)
            self.motor.set_axis_parameter(self.motor.AP.SmartEnergyStallVelocity,  7999774)
            self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresis,     15)
            self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresisStart, 0)
            self.motor.set_axis_parameter(self.motor.AP.SECUS,            1)
            self.motor.set_axis_parameter(self.motor.AP.SmartEnergyThresholdSpeed, 7999774)
            self.motor.drive_settings.boost_current = 0
            self.motor.drive_settings.microstep_resolution = (
                self.motor.ENUM.MicrostepResolution256Microsteps
            )

        # ── Fixed safety overrides (always applied last, both paths) ──────
        for ap_name, value in _FIXED_PARAMS.items():
            ap_id = getattr(self.motor.AP, ap_name, None)
            if ap_id is not None:
                self.motor.set_axis_parameter(ap_id, value)
                print(f"[Stepper] FIXED  AP.{ap_name} = {value}")

        # ── Zero position counters ────────────────────────────────────────
        self.motor.set_axis_parameter(self.motor.AP.ActualPosition,  0)
        self.motor.set_axis_parameter(self.motor.AP.EncoderPosition, 0)
        self.virtual_position = 0

        print(
            f"[Stepper] Initialized on {_port!r} "
            f"(interface={interface}, module_id={module_id}, "
            f"mode={'params-dict' if params is not None else 'legacy'})"
        )
        print(self.motor.drive_settings)

    def _apply_params(self, params: dict) -> None:
        for key, value in params.items():
            mapping = _PARAM_MAP.get(key)
            if mapping is None:
                # Fallback: treat the key itself as a raw AP attribute name.
                if hasattr(self.motor.AP, key):
                    mapping = key
                else:
                    print(f"[Stepper] WARNING: unknown param key {key!r} — skipped.")
                    continue
            targets: list[str] = [mapping] if isinstance(mapping, str) else mapping

            for ap_name in targets:
                ap_id = getattr(self.motor.AP, ap_name, None)
                if ap_id is None:
                    print(
                        f"[Stepper] WARNING: AP.{ap_name} not found on this "
                        "module — skipped."
                    )
                    continue
                try:
                    self.motor.set_axis_parameter(ap_id, value)
                    print(f"[Stepper]   AP.{ap_name} = {value}")
                except:
                    print(f"[Stepper] Name=AP.{ap_name}|ID={ap_id} couldn't be applied")

    # ------------------------------------------------------------------
    # Alternate constructor — load from a JSON file
    # ------------------------------------------------------------------

    @classmethod
    def from_json(
        cls,
        json_path: str,
        interface: str,
        module_id: int,
        data_rate: int = 1_000_000,
        port: str | None = None,
    ) -> "Stepper":
        """
        Create a Stepper whose axis parameters are loaded from a JSON file.

        Example:
            brake = Stepper.from_json(
                "params_brake.json",
                interface="can",
                module_id=1,
            )
        """
        with open(json_path) as fh:
            params = json.load(fh)

        print(f"[Stepper] Loaded params from {json_path!r}")
        return cls(
            interface=interface,
            module_id=module_id,
            params=params,
            data_rate=data_rate,
            port=port,
        )

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def move_to_position(self, target: int) -> None:
        self.motor.move_to(target)
        self.virtual_position = target

    def move_stepper(self, step: int) -> None:
        print(f"[Stepper] Moving to position: {step}")
        self.motor.move_to(step)

    # ------------------------------------------------------------------
    # Position / encoder helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _to_signed_32(value: int) -> int:
        return struct.unpack(">i", struct.pack(">I", value & 0xFFFFFFFF))[0]

    def get_encoder_position(self) -> int:
        return self._to_signed_32(
            self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        )

    def get_actual_position(self) -> int:
        return self._to_signed_32(
            self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        )

    def _sync_position_to_encoder(self) -> None:
        encoder_pos = self.get_encoder_position()
        self.motor.set_axis_parameter(self.motor.AP.ActualPosition, encoder_pos)

    # ------------------------------------------------------------------
    # Braking sequence
    # ------------------------------------------------------------------

    def brake(
        self,
        retract_ticks: int = -24_500,
        brake_duration: float = 5.0,
        slip_threshold: int = 400,
        log=None,
    ) -> None:
        """
        Braking sequence:
          1. Approach      — move to target at the configured MaxVelocity.
          2. Active hold   — 10 Hz slip-correction loop for brake_duration seconds.
          3. Return        — release and go back to start position.

        Args:
            retract_ticks:     Encoder ticks to wind the rope (sign = direction).
            brake_duration:    Seconds to hold under load.
            slip_threshold:    Ticks of slip before issuing a corrective move.
            log:               Optional callable log(event: str, **fields) for
                               per-attempt logging (e.g. stop_node._log_csv).
        """
        def emit(event, **fields):
            if log is not None:
                log(event, **fields)

        print("[Stepper] Starting timed brake...")

        self.motor.set_axis_parameter(self.motor.AP.StandbyCurrent,   128)
        print(f" Setting to StandbyCurrent : {self.motor.get_axis_parameter(self.motor.AP.StandbyCurrent)}")
        start_pos = self.get_actual_position()
        target_pos = start_pos + retract_ticks
        corrections = 0
        emit("brake_approach", actual_pos=start_pos, target_pos=target_pos)

        # ── Step 1: Approach at the configured MaxVelocity ────────────────
        print(f"[Stepper] Approach {start_pos} -> {target_pos}")
        self.move_to_position(target_pos)

        while not self.motor.get_position_reached():
            time.sleep(0.05)

        emit(
            "brake_hold_start",
            actual_pos=self.get_actual_position(),
            encoder_pos=self.get_encoder_position(),
            target_pos=target_pos,
        )

        # ── Step 2: Active hold with slip correction ──────────────────────
        print(
            f"[Stepper] Holding brake for {brake_duration:.1f}s "
            f"(slip_threshold={slip_threshold} ticks)"
        )

        t_start = time.time()
        while (time.time() - t_start) < brake_duration:
            try:
                encoder_pos = self.get_encoder_position()
                slip = abs(target_pos - encoder_pos)

                if slip > slip_threshold:
                    corrections += 1
                    print(
                        f"[Stepper] Slip detected ({slip} ticks). "
                        f"Correcting -> {target_pos}"
                    )
                    emit(
                        "slip_detected",
                        slip=slip,
                        encoder_pos=encoder_pos,
                        target_pos=target_pos,
                        corrections=corrections,
                    )
                    self._sync_position_to_encoder()
                    self.move_to_position(target_pos)

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

            except Exception as exc:
                print(f"[Stepper] CAN error during hold: {exc}")
                time.sleep(0.1)

        emit(
            "brake_hold_end",
            actual_pos=self.get_actual_position(),
            encoder_pos=self.get_encoder_position(),
            target_pos=target_pos,
            corrections=corrections,
        )

        # ── Step 3: Sync then return ──────────────────────────────────────
        self._sync_position_to_encoder()
        print(f"[Stepper] Returning to start: {start_pos}")
        self.move_to_position(start_pos)

        while not self.motor.get_position_reached():
            time.sleep(0.1)

        print("[Stepper] Timed brake completed.")

        self.motor.set_axis_parameter(self.motor.AP.StandbyCurrent,   8)
        emit("brake_done", actual_pos=self.get_actual_position())
        print(f" Setting back to StandbyCurrent is : {self.motor.get_axis_parameter(self.motor.AP.StandbyCurrent)}")
    # ------------------------------------------------------------------
    # Teardown
    # ------------------------------------------------------------------

    def disconnect_motor(self) -> None:
        self.motor.stop()
        self.interface.close()
        print("[Stepper] Disconnected")
