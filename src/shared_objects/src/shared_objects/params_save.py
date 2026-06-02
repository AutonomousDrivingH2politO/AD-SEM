#!/usr/bin/env python3
"""
params_save.py
──────────────
Read **all 84** TMCM-1260 axis parameters from a live motor (typically
configured via TMCL-IDE) and dump the writable ones to a JSON file that
stepper.py's ``Stepper.from_json()`` can load.

Workflow
────────
  1. Open TMCL-IDE, connect to the motor, tune any parameter in the UI.
  2. **Disconnect TMCL-IDE** so it releases the CAN/USB bus.
  3. Run this tool to read the current values back from the motor.
  4. Save to JSON (interactive `s`, or one-shot `--save <file>`).
  5. In your HAL:
         brake = Stepper.from_json("params_brake.json",
                                   interface="can", module_id=1)

Coverage & safety
─────────────────
- All 84 APs are READ and DISPLAYED (grouped: Ramp, Position, Encoder, Current,
  StallGuard, Microstep, Chopper, PWM, DcStep, Reference, Status, Misc).
- The saved JSON contains only WRITABLE config parameters. Read-only state
  params (``DrvStatusFlags``, ``LoadValue``, ``EncoderPosition``, …) and
  motion-command targets (``TargetPosition``, ``TargetVelocity``) are
  deliberately excluded — loading a target into the HAL would command the
  motor to move.
- Where ``stepper.py``'s ``_PARAM_MAP`` defines a friendly key (e.g.
  ``StopOnStallVel`` for ``SmartEnergyStallVelocity``), the JSON uses that
  friendly key. All other writable params are saved under their raw AP name.

The current HAL warns about and skips JSON keys it doesn't recognise. See the
note at the bottom of this file for a 4-line patch to ``_apply_params`` that
makes the HAL accept all raw AP names as a fallback.

Usage
─────
    python params_save.py                                # interactive, CAN can1, id 1
    python params_save.py --interface usb                # USB instead
    python params_save.py --interface can --port can0 --id 2
    python params_save.py --save params_brake.json       # one-shot dump & exit
    python params_save.py --compare params_brake.json    # one-shot diff vs file
"""

import argparse
import json
import struct
import sys
from pathlib import Path

try:
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel
    from rich.prompt import Prompt
    from rich import box

    HAS_RICH = True
    console = Console()
except ImportError:
    HAS_RICH = False

try:
    from pytrinamic.connections import ConnectionManager
    from pytrinamic.modules import TMCM1260
except ImportError:
    sys.exit("ERROR: pytrinamic not installed.  Run:  pip install pytrinamic")


# ─── Complete AP table (84 params) ───────────────────────────────────────────
# Each row: (raw_ap_name, ap_id, group). Order within a group is logical
# (related items adjacent) rather than alphabetical.
AP_TABLE: list[tuple[str, int, str]] = [
    # ── Ramp & speed ─────────────────────────────────────────────────────────
    ("StartVelocity", 19, "Ramp"),
    ("StopVelocity", 20, "Ramp"),
    ("MaxVelocity", 4, "Ramp"),
    ("MaxAcceleration", 5, "Ramp"),
    ("MaxDeceleration", 17, "Ramp"),
    ("V1", 16, "Ramp"),
    ("A1", 15, "Ramp"),
    ("D1", 18, "Ramp"),
    ("RampWaitTime", 21, "Ramp"),
    ("HighSpeedTheshold", 22, "Ramp"),  # firmware misspelling preserved
    ("MinDcStepSpeed", 23, "Ramp"),
    ("PWMThresholdSpeed", 186, "Ramp"),
    # ── Position & motion state ──────────────────────────────────────────────
    ("TargetPosition", 0, "Position"),
    ("ActualPosition", 1, "Position"),
    ("TargetVelocity", 2, "Position"),
    ("ActualVelocity", 3, "Position"),
    ("PositionReachedFlag", 8, "Position"),
    ("MeasuredSpeed", 29, "Position"),
    # ── Encoder ──────────────────────────────────────────────────────────────
    ("EncoderPosition", 209, "Encoder"),
    ("MaxEncoderDeviation", 212, "Encoder"),
    ("EncoderClearOnNull", 210, "Encoder"),
    ("AbsoluteResolverValue", 215, "Encoder"),
    ("ExternalEncoderPosition", 216, "Encoder"),
    ("ExternalEncoderResolution", 217, "Encoder"),
    ("ExternalEncoderMaxDeviation", 218, "Encoder"),
    # ── Current ──────────────────────────────────────────────────────────────
    ("MaxCurrent", 6, "Current"),
    ("StandbyCurrent", 7, "Current"),
    ("GlobalCurrentScaler", 178, "Current"),
    ("SmartEnergyActualCurrent", 180, "Current"),
    # ── StallGuard & CoolStep ────────────────────────────────────────────────
    ("SG2Threshold", 174, "StallGuard"),
    ("SG2FilterEnable", 173, "StallGuard"),
    ("SmartEnergyStallVelocity", 181, "StallGuard"),
    ("SmartEnergyThresholdSpeed", 182, "StallGuard"),
    ("SmartEnergyHysteresis", 170, "StallGuard"),
    ("SmartEnergyHysteresisStart", 172, "StallGuard"),
    ("SECUS", 171, "StallGuard"),
    ("SECDS", 169, "StallGuard"),
    ("SEIMIN", 168, "StallGuard"),
    ("LoadValue", 206, "StallGuard"),
    # ── Microstepping ────────────────────────────────────────────────────────
    ("MicrostepResolution", 140, "Microstep"),
    ("FullstepResolution", 202, "Microstep"),
    ("Intpol", 160, "Microstep"),
    ("DoubleEdgeSteps", 161, "Microstep"),
    # ── Chopper (SpreadCycle) ────────────────────────────────────────────────
    ("ChopperBlankTime", 162, "Chopper"),
    ("ChopperHysteresisStart", 166, "Chopper"),
    ("ChopperHysteresisEnd", 165, "Chopper"),
    ("ChopperSynchronization", 185, "Chopper"),
    ("ConstantTOffMode", 163, "Chopper"),
    ("DisableFastDecayComparator", 164, "Chopper"),
    ("TOff", 167, "Chopper"),
    ("RandomTOffMode", 184, "Chopper"),
    ("HighSpeedChopperMode", 27, "Chopper"),
    ("HighSpeedFullstepMode", 28, "Chopper"),
    # ── StealthChop / PWM ────────────────────────────────────────────────────
    ("PWMMode", 190, "PWM"),
    ("PWMAmplitude", 188, "PWM"),
    ("PWMAutoscale", 192, "PWM"),
    ("PWMFrequency", 191, "PWM"),
    ("PWMGrad", 187, "PWM"),
    ("PWMScale", 189, "PWM"),
    # ── dcStep ───────────────────────────────────────────────────────────────
    ("DcStepStallGuard", 33, "DcStep"),
    ("DcStepTime", 32, "DcStep"),
    # ── Reference / homing / limit switches ──────────────────────────────────
    ("ReferenceSearchMode", 193, "Reference"),
    ("ReferenceSearchSpeed", 194, "Reference"),
    ("ReferenceSwitchSpeed", 195, "Reference"),
    ("ReferenceSwitchDistance", 196, "Reference"),
    ("LastReferenceSwitchPosition", 197, "Reference"),
    ("HomeSwitch", 9, "Reference"),
    ("LeftEndstop", 11, "Reference"),
    ("RightEndstop", 10, "Reference"),
    ("LeftLimitSwitchDisable", 13, "Reference"),
    ("LeftLimitSwitchPolarity", 25, "Reference"),
    ("RightLimitSwitchDisable", 12, "Reference"),
    ("RightLimitSwitchPolarity", 24, "Reference"),
    ("SwapLimitSwitches", 14, "Reference"),
    ("SoftStop", 26, "Reference"),
    # ── Driver status ────────────────────────────────────────────────────────
    ("DrvStatusFlags", 208, "Status"),
    ("ExtendedErrorFlags", 207, "Status"),
    # ── Misc / driver behaviour ──────────────────────────────────────────────
    ("FreewheelingMode", 204, "Misc"),
    ("PowerDownDelay", 214, "Misc"),
    ("PowerDownRamp", 31, "Misc"),
    ("RelativePositioningOption", 127, "Misc"),
    ("ReverseShaft", 251, "Misc"),
    ("StepDirectionMode", 254, "Misc"),
    ("UnitMode", 255, "Misc"),
]
assert len(AP_TABLE) == 84, f"AP_TABLE has {len(AP_TABLE)} entries, expected 84"

# Group order for display (matches insertion order in AP_TABLE)
GROUP_ORDER = list(dict.fromkeys(g for _, _, g in AP_TABLE))

# ─── Read-only / unsafe-to-load state ────────────────────────────────────────
# These are read and displayed for context but excluded from saved JSON.
#
# Critical safety items:
#   TargetPosition / TargetVelocity — writing these COMMANDS the motor to
#       move. Including them in a config file would silently turn a HAL
#       startup into an unintended motion.
#   ActualPosition / EncoderPosition — writing these resets internal
#       counters. Not dangerous, but they're runtime state, not config.
#
# Read-only registers (writing is ignored or errors):
#   DrvStatusFlags, ExtendedErrorFlags, PositionReachedFlag, MeasuredSpeed,
#   ActualVelocity, LoadValue, SmartEnergyActualCurrent, PWMScale,
#   AbsoluteResolverValue, LastReferenceSwitchPosition.
#
# Inputs (switch state, not configuration):
#   HomeSwitch, LeftEndstop, RightEndstop.
STATE_PARAMS: set[str] = {
    # Targets — DO NOT load (would cause motion)
    "TargetPosition",
    "TargetVelocity",
    # Position/velocity state
    "ActualPosition",
    "ActualVelocity",
    "EncoderPosition",
    "ExternalEncoderPosition",
    "AbsoluteResolverValue",
    "LastReferenceSwitchPosition",
    "MeasuredSpeed",
    # Read-only status / flags
    "PositionReachedFlag",
    "DrvStatusFlags",
    "ExtendedErrorFlags",
    "LoadValue",
    "SmartEnergyActualCurrent",
    "PWMScale",
    # Switch inputs
    "HomeSwitch",
    "LeftEndstop",
    "RightEndstop",
}

# ─── Friendly JSON key aliases (HAL-compatible names) ────────────────────────
# When saving, params on the left are renamed to the keys on the right so
# Stepper.from_json() picks them up via its _PARAM_MAP. All other params are
# saved under their raw AP name.
FRIENDLY_KEY: dict[str, str] = {
    "SmartEnergyStallVelocity": "StopOnStallVel",
    "SmartEnergyThresholdSpeed": "CoolStepThresholdSpeed",
    "SmartEnergyHysteresisStart": "CoolStepMin",
    "SmartEnergyHysteresis": "CoolStepMax",
    "SECUS": "CoolStepUpStep",
    "SECDS": "CoolStepDownStep",
    "SEIMIN": "CoolStepIMin",
}

# Raw AP names that the current HAL's _PARAM_MAP recognises directly. Used
# only to colour the display so you can tell at a glance what the HAL will
# accept without a fallback patch.
HAL_NATIVE_KEYS: set[str] = {
    "StartVelocity",
    "StopVelocity",
    "MaxVelocity",
    "MaxAcceleration",
    "MaxDeceleration",
    "V1",
    "A1",
    "D1",
    "MaxCurrent",
    "StandbyCurrent",
    "MicrostepResolution",
    "SG2Threshold",
    "PWMMode",
    "SmartEnergyThresholdSpeed",
}


# ─── Helpers ─────────────────────────────────────────────────────────────────
def _s32(v: int) -> int:
    """Convert an unsigned 32-bit value to signed (e.g. for SGT)."""
    return struct.unpack(">i", struct.pack(">I", v & 0xFFFFFFFF))[0]


def read_one(motor, ap_id: int) -> "int | None":
    """Read one AP by ID. Returns None on any read failure."""
    try:
        return _s32(motor.get_axis_parameter(ap_id))
    except Exception:
        return None


def read_all(motor) -> dict[str, "int | None"]:
    """Read every AP in AP_TABLE. Returns {raw_ap_name: value_or_None}."""
    return {name: read_one(motor, ap_id) for name, ap_id, _ in AP_TABLE}


def to_json_payload(values: dict[str, "int | None"]) -> dict[str, int]:
    """
    Filter and rename a {raw_name → value} dict for safe HAL loading:
      - Skip STATE_PARAMS (unsafe or meaningless to load).
      - Skip None values (AP not readable on this module).
      - Rename to friendly keys where FRIENDLY_KEY defines one.
    """
    payload: dict[str, int] = {}
    for name, _id, _g in AP_TABLE:
        if name in STATE_PARAMS:
            continue
        v = values.get(name)
        if v is None:
            continue
        payload[FRIENDLY_KEY.get(name, name)] = v
    return payload


def cprint(msg: str, style: str = "") -> None:
    if HAS_RICH:
        console.print(msg, style=style)
    else:
        import re

        print(re.sub(r"\[/?[a-z_ ]*\]", "", msg))


def ask(prompt: str, default=None) -> str:
    if HAS_RICH:
        return Prompt.ask(prompt, default=str(default) if default is not None else "")
    suffix = f" [{default}]" if default is not None else ""
    raw = input(f"{prompt}{suffix}: ").strip()
    return raw if raw else (str(default) if default is not None else "")


# ─── Display ─────────────────────────────────────────────────────────────────
def _kind_for(name: str) -> str:
    if name in STATE_PARAMS:
        return "state"
    if name in FRIENDLY_KEY or name in HAL_NATIVE_KEYS:
        return "hal"
    return "raw"


def _style_for_kind(kind: str) -> str:
    return {
        "state": "dim",
        "hal": "",  # default (highlighted by yellow value column)
        "raw": "white",
    }.get(kind, "")


def show_params(motor, values: dict | None = None) -> dict:
    """Display all 84 params, grouped, with state/HAL/raw markers."""
    if values is None:
        values = read_all(motor)

    if HAS_RICH:
        # One table per group, stacked vertically. Keeps each section
        # visually scannable instead of one giant 84-row wall.
        for group in GROUP_ORDER:
            group_rows = [row for row in AP_TABLE if row[2] == group]
            t = Table(
                title=f"[bold cyan]{group}[/bold cyan]",
                box=box.SIMPLE,
                show_header=True,
                header_style="bold",
                expand=False,
            )
            t.add_column("Name", width=30)
            t.add_column("AP", justify="right", width=4, style="dim")
            t.add_column("Value", justify="right", width=12, style="yellow")
            t.add_column("Kind", width=6, style="dim")
            t.add_column("JSON key", style="dim")
            for name, ap_id, _ in group_rows:
                v = values[name]
                kind = _kind_for(name)
                style = _style_for_kind(kind)
                json_key = "—" if kind == "state" else FRIENDLY_KEY.get(name, name)
                t.add_row(
                    name,
                    str(ap_id),
                    "—" if v is None else str(v),
                    kind,
                    json_key,
                    style=style or None,
                )
            console.print(t)
    else:
        print(f"\n{'═' * 86}")
        print("  TMCM-1260 Axis Parameters (84 total)")
        print(f"{'═' * 86}")
        for group in GROUP_ORDER:
            print(f"\n  [{group}]")
            print(f"  {'-' * 80}")
            print(f"  {'Name':<30} {'AP':>4}  {'Value':>10}  {'Kind':<5}  JSON key")
            for name, ap_id, g in AP_TABLE:
                if g != group:
                    continue
                v = values[name]
                kind = _kind_for(name)
                json_key = "—" if kind == "state" else FRIENDLY_KEY.get(name, name)
                v_str = "—" if v is None else str(v)
                print(f"  {name:<30} {ap_id:>4}  {v_str:>10}  {kind:<5}  {json_key}")
        print(f"\n{'═' * 86}")
        print(
            "  Legend: hal = HAL recognises directly | "
            "raw = saved under raw AP name | state = read-only/excluded"
        )
        print(f"{'═' * 86}\n")

    # Surface the HAL override for PWMMode if the motor disagrees.
    pwm = values.get("PWMMode")
    if pwm is not None and pwm != 0:
        cprint(
            f"  ⚠  PWMMode = {pwm} on the motor (StealthChop enabled), but the "
            "HAL forces PWMMode = 0 (SpreadCycle) on load via _FIXED_PARAMS. "
            "The saved value will be ignored at HAL startup.",
            "yellow",
        )

    return values


# ─── Save ────────────────────────────────────────────────────────────────────
def save_params(motor, path: str | None = None) -> str | None:
    if path is None:
        path = (
            ask("  Save to file", default="params_brake.json").strip()
            or "params_brake.json"
        )

    values = read_all(motor)
    payload = to_json_payload(values)

    Path(path).write_text(json.dumps(payload, indent=2) + "\n")

    # Reporting
    skipped_state = sorted(n for n in STATE_PARAMS if values.get(n) is not None)
    skipped_unreadable = [
        n for n, _, _ in AP_TABLE if values.get(n) is None and n not in STATE_PARAMS
    ]
    hal_count = sum(
        1
        for n, _, _ in AP_TABLE
        if n not in STATE_PARAMS
        and (n in HAL_NATIVE_KEYS or n in FRIENDLY_KEY)
        and values.get(n) is not None
    )
    raw_count = len(payload) - hal_count

    cprint(
        f"  ✓  Wrote {len(payload)} parameters to {path}  "
        f"([bold]{hal_count}[/bold] HAL-recognised, [bold]{raw_count}[/bold] raw)",
        "green",
    )
    if skipped_state:
        cprint(
            f"  ℹ  Skipped {len(skipped_state)} state/read-only params "
            f"(targets, status, encoders): {', '.join(skipped_state)}",
            "cyan",
        )
    if skipped_unreadable:
        cprint(
            f"  ⚠  Skipped {len(skipped_unreadable)} unreadable params "
            f"(AP not present): {', '.join(skipped_unreadable)}",
            "yellow",
        )
    return path


# ─── Compare ─────────────────────────────────────────────────────────────────
def compare_to_file(motor, path: str | None = None) -> None:
    if path is None:
        path = ask("  Compare against file", default="params_brake.json").strip()
        if not path:
            return

    try:
        file_vals = json.loads(Path(path).read_text())
    except FileNotFoundError:
        cprint(f"  ✗  File not found: {path}", "red")
        return
    except json.JSONDecodeError as e:
        cprint(f"  ✗  Bad JSON: {e}", "red")
        return

    live_raw = read_all(motor)
    live_payload = to_json_payload(live_raw)  # matches the keyspace of saved files
    diffs = 0

    if HAS_RICH:
        t = Table(
            box=box.SIMPLE_HEAVY,
            header_style="bold cyan",
            title=f"Live motor  vs  {path}",
        )
        t.add_column("Key", style="bold white", width=30)
        t.add_column("Motor", justify="right", width=12)
        t.add_column("File", justify="right", width=12)
        t.add_column("Δ", justify="right", width=10)

        # Walk in AP_TABLE order so related items stay adjacent; then any
        # extra keys present only in the file go at the bottom.
        seen: set[str] = set()
        for name, _id, _g in AP_TABLE:
            if name in STATE_PARAMS:
                continue
            json_key = FRIENDLY_KEY.get(name, name)
            seen.add(json_key)
            live = live_payload.get(json_key)
            file_v = file_vals.get(json_key)
            if live is None and file_v is None:
                continue
            if live == file_v:
                t.add_row(json_key, str(live), str(file_v), "·", style="dim")
            else:
                diffs += 1
                if isinstance(live, int) and isinstance(file_v, int):
                    delta = f"{live - file_v:+d}"
                else:
                    delta = "≠"
                t.add_row(
                    json_key,
                    "—" if live is None else str(live),
                    "—" if file_v is None else str(file_v),
                    delta,
                    style="yellow",
                )
        # Keys present in file but not in our table at all
        extras = sorted(set(file_vals) - seen)
        for k in extras:
            diffs += 1
            t.add_row(k, "—", str(file_vals[k]), "file-only", style="magenta")
        console.print(t)
    else:
        print(f"\n  Live motor  vs  {path}")
        print(f"  {'Key':<30} {'Motor':>10} {'File':>10}  Δ")
        seen = set()
        for name, _id, _g in AP_TABLE:
            if name in STATE_PARAMS:
                continue
            json_key = FRIENDLY_KEY.get(name, name)
            seen.add(json_key)
            live = live_payload.get(json_key)
            file_v = file_vals.get(json_key)
            if live is None and file_v is None:
                continue
            marker = "·" if live == file_v else "≠"
            if live != file_v:
                diffs += 1
            print(
                f"  {json_key:<30} {('—' if live is None else live):>10} "
                f"{('—' if file_v is None else file_v):>10}  {marker}"
            )
        for k in sorted(set(file_vals) - seen):
            diffs += 1
            print(f"  {k:<30} {'—':>10} {file_vals[k]:>10}  file-only")

    cprint(
        f"  ✓  Identical." if diffs == 0 else f"  ⚠  {diffs} difference(s).",
        "green" if diffs == 0 else "yellow",
    )


# ─── Menu ────────────────────────────────────────────────────────────────────
MENU = [
    ("v", "View all 84 parameters (reads fresh from motor)"),
    ("s", "Save writable parameters to JSON (state params excluded)"),
    ("c", "Compare current parameters with a JSON file"),
    ("q", "Quit"),
]


def print_menu() -> None:
    if HAS_RICH:
        from rich.text import Text

        body = Text()
        for i, (k, desc) in enumerate(MENU):
            body.append(f"  [{k}]  ", style="bold cyan")
            body.append(desc)
            if i < len(MENU) - 1:
                body.append("\n")
        console.print(
            Panel(
                body,
                title="[bold cyan]Param Save — TMCM-1260 (84 APs)[/bold cyan]",
                border_style="cyan",
                expand=False,
            )
        )
    else:
        print("\n" + "═" * 64)
        print("  Param Save — TMCM-1260 (84 APs)")
        print("═" * 64)
        for k, desc in MENU:
            print(f"  [{k}]  {desc}")
        print("═" * 64)


def run_interactive(motor) -> None:
    while True:
        print_menu()
        choice = ask("\n  >").strip().lower()
        if choice in ("v", "view", "r", "read", "refresh"):
            show_params(motor)
        elif choice in ("s", "save"):
            save_params(motor)
        elif choice in ("c", "compare", "diff"):
            compare_to_file(motor)
        elif choice in ("q", "quit", "exit"):
            cprint("\n  Bye.\n", "dim")
            return
        elif choice == "":
            continue
        else:
            cprint(f"  ✗  Unknown: {choice!r}", "red")


# ─── Entry point ─────────────────────────────────────────────────────────────
def main() -> None:
    p = argparse.ArgumentParser(
        description="Dump all 84 TMCM-1260 axis parameters from a live motor to JSON."
    )
    p.add_argument("--interface", default="can", choices=["can", "usb"])
    p.add_argument("--port", default=None)
    p.add_argument("--id", type=int, default=1, dest="module_id")
    p.add_argument("--data-rate", type=int, default=1_000_000)
    p.add_argument(
        "--save",
        default=None,
        metavar="JSON",
        help="One-shot: read motor params and save to this file, then exit.",
    )
    p.add_argument(
        "--compare",
        default=None,
        metavar="JSON",
        help="One-shot: compare live motor params against this file, then exit.",
    )
    args = p.parse_args()

    port = args.port or ("can1" if args.interface == "can" else "/dev/ttyACM0")
    if args.interface == "can":
        conn_str = (
            f"--interface socketcan_tmcl --port {port} --data-rate {args.data_rate}"
        )
    else:
        conn_str = f"--interface usb_tmcl --port {port}"

    cprint(
        f"\n  Connecting via [bold]{args.interface.upper()}[/bold] on {port}…",
        "cyan",
    )
    iface = ConnectionManager(conn_str).connect()
    module = TMCM1260(iface, module_id=args.module_id)
    motor = module.motors[0]
    cprint(f"  ✓  Connected (module id={args.module_id})\n", "green")

    try:
        if args.save:
            show_params(motor)
            save_params(motor, args.save)
        elif args.compare:
            compare_to_file(motor, args.compare)
        else:
            show_params(motor)
            run_interactive(motor)
    finally:
        iface.close()
        cprint("  Interface closed.", "dim")


if __name__ == "__main__":
    main()


# ─────────────────────────────────────────────────────────────────────────────
# OPTIONAL HAL PATCH (stepper.py) — load raw AP names without warnings
# ─────────────────────────────────────────────────────────────────────────────
# The dumps produced here include params under their raw AP names (e.g.
# "ChopperBlankTime", "TOff", "Intpol") that the current HAL's _PARAM_MAP
# doesn't list. The HAL warns and skips them. To make the HAL accept any
# raw AP name that exists on motor.AP as a fallback, replace the head of
# Stepper._apply_params with:
#
#     def _apply_params(self, params: dict) -> None:
#         for key, value in params.items():
#             mapping = _PARAM_MAP.get(key)
#             if mapping is None:
#                 # Fallback: treat the key itself as a raw AP attribute name.
#                 if hasattr(self.motor.AP, key):
#                     mapping = key
#                 else:
#                     print(f"[Stepper] WARNING: unknown param key {key!r} — skipped.")
#                     continue
#             targets: list[str] = [mapping] if isinstance(mapping, str) else mapping
#             # ... rest unchanged
#
# Safety notes if you apply the patch:
#   - This tool already excludes TargetPosition/TargetVelocity from saves, so
#     enabling the fallback won't accidentally cause motion at HAL startup.
#   - _FIXED_PARAMS still wins for SG2FilterEnable/RampWaitTime/PWMMode, so
#     the brake's StealthChop-off invariant is preserved regardless of what's
#     in the JSON.
