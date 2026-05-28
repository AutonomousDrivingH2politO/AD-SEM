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
# Two keys (StopOnStallVel) are applied to two AP registers because TMC uses
# separate threshold registers for stall detection vs. CoolStep.
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
    # Value is passed directly as an integer (0–8 → 1–256 µsteps per TMC enum)
    "MicrostepResolution": "MicrostepResolution",
    # ── StallGuard2 ──────────────────────────────────────────────────────────
    "SG2Threshold":     "SG2Threshold",
    # StopOnStallVel=0 effectively disables stall-stop on both threshold regs.
    # Use a large value (e.g. 7999774) to disable while keeping CoolStep active.
    "StopOnStallVel":   ["SmartEnergyStallVelocity", "SmartEnergyThresholdSpeed"],
    # ── CoolStep (SmartEnergy) ───────────────────────────────────────────────
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
}

# Parameters that are always forced to a fixed value regardless of the JSON,
# because they control low-level safety behaviour we never want overridden.
_FIXED_PARAMS: dict[str, int] = {
    "SG2FilterEnable": 0,   # keep SG2 responsive; filter adds 4-cycle lag
    "RampWaitTime":    0,   # no dwell between ramp phases
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
            # Params-dict path: every value comes from the dict.
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
        """Iterate over the params dict, resolve AP names, and apply each value."""
        for key, value in params.items():
            mapping = _PARAM_MAP.get(key)

            if mapping is None:
                print(f"[Stepper] WARNING: unknown param key {key!r} — skipped.")
                continue

            # A key may map to more than one AP register (e.g. StopOnStallVel).
            targets: list[str] = [mapping] if isinstance(mapping, str) else mapping

            for ap_name in targets:
                ap_id = getattr(self.motor.AP, ap_name, None)
                if ap_id is None:
                    print(
                        f"[Stepper] WARNING: AP.{ap_name} not found on this "
                        "module — skipped."
                    )
                    continue
                self.motor.set_axis_parameter(ap_id, value)
                print(f"[Stepper]   AP.{ap_name} = {value}")

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
        retract_ticks: int = -20_000,
        brake_duration: float = 5.0,
        slip_threshold: int = 400,
        approach_velocity: int = 30_000,
    ) -> None:
        """
        Braking sequence:
          1. Slow approach — take up rope slack without shock-loading.
          2. Active hold   — 10 Hz slip-correction loop for brake_duration seconds.
          3. Return        — release and go back to start position.

        Args:
            retract_ticks:     Encoder ticks to wind the rope (sign = direction).
            brake_duration:    Seconds to hold under load.
            slip_threshold:    Ticks of slip before issuing a corrective move.
            approach_velocity: Reduced velocity used for the initial take-up move.
        """
        print("[Stepper] Starting timed brake...")

        start_pos = self.get_actual_position()
        target_pos = start_pos + retract_ticks

        # ── Step 1: Slow approach ─────────────────────────────────────────
        # Temporarily reduce MaxVelocity so the rope is taken up smoothly.
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

        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity, original_velocity)

        # ── Step 2: Active hold with slip correction ──────────────────────
        # Poll at 10 Hz. On slip: sync firmware counter, reissue move_to,
        # then wait for position_reached before resuming — prevents stacking
        # redundant move commands that reset the ramp generator mid-move.
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
                    print(
                        f"[Stepper] Slip detected ({slip} ticks). "
                        f"Correcting -> {target_pos}"
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

        # ── Step 3: Sync then return ──────────────────────────────────────
        self._sync_position_to_encoder()
        print(f"[Stepper] Returning to start: {start_pos}")
        self.move_to_position(start_pos)

        while not self.motor.get_position_reached():
            time.sleep(0.1)

        print("[Stepper] Timed brake completed.")

    # ------------------------------------------------------------------
    # Teardown
    # ------------------------------------------------------------------

    def disconnect_motor(self) -> None:
        self.motor.stop()
        self.interface.close()
        print("[Stepper] Disconnected")
