#!/usr/bin/env python3
"""
brake_tuner.py
────────────────
Quick brake-tuning tool for the TMCM-1260.
Edit speed / acceleration params and fire a brake test without restarting.

Usage
─────
    python stepper_tuner.py                        # CAN, port can1,  id 1
    python stepper_tuner.py --interface usb
    python stepper_tuner.py --interface can --port can0 --id 2
    python stepper_tuner.py --log results.csv      # append test results here
"""

import argparse
import csv
import json
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    from rich.console import Console
    from rich.table import Table
    from rich import box
    from rich.prompt import Prompt
    from rich.panel import Panel

    HAS_RICH = True
    console = Console()
except ImportError:
    HAS_RICH = False

try:
    from pytrinamic.connections import ConnectionManager
    from pytrinamic.modules import TMCM1260
except ImportError:
    sys.exit("ERROR: pytrinamic not installed.  Run:  pip install pytrinamic")


# ─── Parameters exposed for editing ──────────────────────────────────────────
#  (name, AP attribute, unit, description)

PARAMS = [
    ("StartVelocity",      "StartVelocity",      "pps",  "Speed at ramp start (launch speed)"),
    ("StopVelocity",       "StopVelocity",       "pps",  "Speed at ramp end (near stop)"),
    ("MaxVelocity",        "MaxVelocity",        "pps",  "Top cruising speed"),
    ("MaxAcceleration",    "MaxAcceleration",    "pps²", "Acceleration to MaxVelocity"),
    ("MaxDeceleration",    "MaxDeceleration",    "pps²", "Deceleration from MaxVelocity"),
    ("V1",                 "V1",                 "pps",  "Intermediate ramp speed"),
    ("A1",                 "A1",                 "pps²", "Acceleration up to V1"),
    ("D1",                 "D1",                 "pps²", "Deceleration from V1"),
    ("MicrostepResolution","MicrostepResolution","µstep","Microstep resolution (1/2/4/8/16/32/64/128/256)"),
        # ── StallGuard2 ─────────────────────────────────────────────────────────
    ("SG2Threshold",          "SG2Threshold",          "",     "StallGuard2 sensitivity (-64..+63, lower = more sensitive)"),
    ("SG2FilterEnable",       "SG2FilterEnable",       "0/1",  "StallGuard2 filter (0=fast, 1=stable over 4 fullsteps)"),
    ("SmartEnergyStallVelocity","SmartEnergyStallVelocity","pps", "Min velocity for stall detect (very high = disabled)"),
]

# Valid microstep resolutions for the TMCM-1260
VALID_MICROSTEPS = {1, 2, 4, 8, 16, 32, 64, 128, 256}

# Position-reached polling interval and default timeout
POLL_INTERVAL = 0.05          # seconds
MOVE_TIMEOUT  = 30.0          # seconds  — override per call if needed
ENC_SANITY_MARGIN = 500_000   # ticks; encoder reads outside ±this from target
                               # are flagged as suspect before trusting them


# ─── Helpers ──────────────────────────────────────────────────────────────────

def _s32(v: int) -> int:
    """Reinterpret a 32-bit unsigned value as signed."""
    return struct.unpack(">i", struct.pack(">I", v & 0xFFFFFFFF))[0]


def get_ap(motor, attr: str) -> int:
    return _s32(motor.get_axis_parameter(getattr(motor.AP, attr)))


def set_ap(motor, attr: str, value: int) -> None:
    motor.set_axis_parameter(getattr(motor.AP, attr), value)


def wait_for_position(motor, timeout: float = MOVE_TIMEOUT) -> bool:
    """
    Poll until position is reached or *timeout* seconds elapse.
    Returns True on success, False on timeout.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if motor.get_position_reached():
            return True
        time.sleep(POLL_INTERVAL)
    return False


def snapshot_params(motor) -> dict:
    """Return a dict of all tunable param names → current values."""
    return {name: get_ap(motor, attr) for name, attr, *_ in PARAMS}


def cprint(msg: str, style: str = "") -> None:
    if HAS_RICH:
        console.print(msg, style=style)
    else:
        import re
        print(re.sub(r"\[/?[a-z_ ]*\]", "", msg))


def ask(prompt: str, default=None) -> str:
    if HAS_RICH:
        raw = Prompt.ask(prompt, default=str(default) if default is not None else "")
        return raw if raw is not None else ""
    suffix = f" [{default}]" if default is not None else ""
    raw = input(f"{prompt}{suffix}: ").strip()
    return raw if raw else (str(default) if default is not None else "")


def ask_int(prompt: str, default=None) -> int:
    while True:
        raw = ask(prompt, default)
        try:
            return int(raw)
        except ValueError:
            cprint(f"  ✗  '{raw}' is not an integer.", "red")


def ask_float(prompt: str, default=None) -> float:
    while True:
        raw = ask(prompt, default)
        try:
            return float(raw)
        except ValueError:
            cprint(f"  ✗  '{raw}' is not a number.", "red")


# ─── Display ──────────────────────────────────────────────────────────────────

def show_params(motor) -> None:
    if HAS_RICH:
        t = Table(
            box=box.SIMPLE_HEAVY,
            show_lines=False,
            header_style="bold cyan",
            title="Speed / Acceleration Parameters",
        )
        t.add_column("#",           style="dim",        width=4)
        t.add_column("Name",        style="bold white", width=22)
        t.add_column("Value",       style="yellow",     width=12, justify="right")
        t.add_column("Unit",        style="dim green",  width=7)
        t.add_column("Description", style="dim")
        for i, (name, attr, unit, desc) in enumerate(PARAMS, 1):
            t.add_row(str(i), name, str(get_ap(motor, attr)), unit, desc)
        console.print(t)
    else:
        print(f"\n{'─' * 72}")
        print("  Speed / Acceleration Parameters")
        print(f"{'─' * 72}")
        print(f"  {'#':<3}  {'Name':<22}  {'Value':>10}  {'Unit':<7}  Description")
        print(f"{'─' * 72}")
        for i, (name, attr, unit, desc) in enumerate(PARAMS, 1):
            print(f"  {i:<3}  {name:<22}  {get_ap(motor, attr):>10}  {unit:<7}  {desc}")
        print(f"{'─' * 72}\n")


# ─── Parameter snapshot save / load ──────────────────────────────────────────

def save_params(motor) -> None:
    path = ask("  Save to file", default="params.json").strip() or "params.json"
    data = snapshot_params(motor)
    Path(path).write_text(json.dumps(data, indent=2))
    cprint(f"  ✓  Parameters saved to {path}", "green")


def load_params(motor) -> None:
    path = ask("  Load from file", default="params.json").strip() or "params.json"
    try:
        data = json.loads(Path(path).read_text())
    except FileNotFoundError:
        cprint(f"  ✗  File not found: {path}", "red")
        return
    except json.JSONDecodeError as e:
        cprint(f"  ✗  Bad JSON: {e}", "red")
        return

    for name, attr, unit, _ in PARAMS:
        if name in data:
            value = int(data[name])
            if name == "MicrostepResolution" and value not in VALID_MICROSTEPS:
                cprint(
                    f"  ✗  Skipping MicrostepResolution={value}: "
                    f"must be one of {sorted(VALID_MICROSTEPS)}.",
                    "red",
                )
                continue
            set_ap(motor, attr, value)
            readback = get_ap(motor, attr)
            cprint(f"  ✓  {name} = {readback} {unit}", "green")
        else:
            cprint(f"  ⚠  {name} not found in file, skipped.", "yellow")


# ─── Edit ─────────────────────────────────────────────────────────────────────

def edit_param(motor) -> None:
    show_params(motor)
    idx = ask_int(f"  Parameter # to change (1-{len(PARAMS)}, 0=cancel)", default=0)
    if idx == 0:
        return
    if not 1 <= idx <= len(PARAMS):
        cprint(f"  ✗  No parameter #{idx}.", "red")
        return

    name, attr, unit, desc = PARAMS[idx - 1]
    current = get_ap(motor, attr)

    # ── MicrostepResolution gets special handling ──────────────────────────
    if name == "MicrostepResolution":
        sorted_ms = sorted(VALID_MICROSTEPS)
        cprint(
            f"\n  [bold]{name}[/bold] — {desc}  (now: {current} µstep)\n"
            f"  Valid values: {sorted_ms}",
        )
        cprint(
            "  [bold yellow]⚠  Changing microstep resolution rescales all position "
            "counts. Home the axis afterwards.[/bold yellow]"
        )
        while True:
            new_val = ask_int(f"  New value [{unit}]", default=current)
            if new_val in VALID_MICROSTEPS:
                break
            cprint(
                f"  ✗  {new_val} is not a valid microstep resolution. "
                f"Choose from: {sorted_ms}",
                "red",
            )
        if new_val == current:
            cprint("  No change.", "dim")
            return
        set_ap(motor, attr, new_val)
        readback = get_ap(motor, attr)
        cprint(f"  ✓  {name} = {readback} µstep", "green")
        return
       # ── All other params ───────────────────────────────────────────────────
    cprint(f"\n  [bold]{name}[/bold] — {desc}  (now: {current} {unit})")
    new_val = ask_int(f"  New value [{unit}]", default=current)

    if new_val == current:
        cprint("  No change.", "dim")
        return
    if name == "SG2Threshold" and not -64 <= new_val <= 63:
        cprint(f"  ✗  {new_val} out of range. SG2Threshold must be -64..+63.", "red")
        return
    if name == "SG2FilterEnable" and new_val not in (0, 1):
        cprint(f"  ✗  SG2FilterEnable must be 0 or 1.", "red")
        return
 
    set_ap(motor, attr, new_val)
    readback = get_ap(motor, attr)
    cprint(f"  ✓  {name} = {readback} {unit}", "green")


# ─── Brake test ───────────────────────────────────────────────────────────────

# Params we override during the three-phase approach; saved/restored as a unit.
_PHASED_PARAMS = ("MaxVelocity", "MaxAcceleration", "MaxDeceleration", "V1", "A1", "D1")


def brake_test(motor, log_path: str | None = None) -> None:
    cprint("\n  [bold cyan]Brake Test[/bold cyan]")
    retract   = ask_int("  Retract ticks   (rope wind distance)", default=70_000)
    duration  = ask_float("  Hold duration   (seconds)",          default=5.0)
    slip_thr  = ask_int("  Slip threshold  (encoder ticks)",      default=200)
    approach  = ask_int("  Approach velocity (pps)",              default=20_000)

    max_vel = get_ap(motor, "MaxVelocity")
    if approach > max_vel:
        cprint(
            f"  ⚠  Approach velocity ({approach}) > MaxVelocity ({max_vel}); "
            f"clamping to {max_vel}.",
            "yellow",
        )
        approach = max_vel

    start_pos  = _s32(motor.get_axis_parameter(motor.AP.ActualPosition))
    target_pos = start_pos + retract

    # ── Three-phase approach: slow → fast → slow ──────────────────────────────
    # Snapshot everything we modify so we can restore it no matter what.
    saved = {name: get_ap(motor, name) for name in _PHASED_PARAMS}

    # Split the retract into thirds.  Any rounding remainder is absorbed by the
    # final phase so the end target is unchanged.
    third = retract // 3
    wp1   = start_pos + third
    wp2   = start_pos + 2 * third
    wp3   = target_pos

    def apply_slow() -> None:
        """Phase 1 / 3 settings: MaxVelocity = user-entered approach,
        everything else at the user-configured ('slow') values."""
        set_ap(motor, "MaxVelocity",     approach)
        set_ap(motor, "MaxAcceleration", saved["MaxAcceleration"])
        set_ap(motor, "MaxDeceleration", saved["MaxDeceleration"])
        set_ap(motor, "V1",              saved["V1"])
        set_ap(motor, "A1",              saved["A1"])
        set_ap(motor, "D1",              saved["D1"])

    def apply_fast() -> None:
        """Middle-phase settings: every speed/accel doubled."""
        set_ap(motor, "MaxVelocity",     approach * 3)
        set_ap(motor, "MaxAcceleration", saved["MaxAcceleration"] * 3)
        set_ap(motor, "MaxDeceleration", saved["MaxDeceleration"] * 3)
        set_ap(motor, "V1",              saved["V1"] * 2)
        set_ap(motor, "A1",              saved["A1"] * 2)
        set_ap(motor, "D1",              saved["D1"] * 2)

    cprint(
        f"\n  Approaching {start_pos} → {target_pos} in 3 phases "
        f"(slow → fast → slow)…",
        "cyan",
    )

    try:
        # Phase 1 — slow
        apply_slow()
        cprint(f"  Phase 1/3 (slow): {start_pos} → {wp1} @ {approach} pps…", "cyan")
        motor.move_to(wp1)
        if not wait_for_position(motor, timeout=MOVE_TIMEOUT):
            cprint("  ✗  Phase 1 timed out — aborting.", "red")
            return

        # Phase 2 — fast (doubled)
        apply_slow()
        cprint(f"  Phase 2/3 (fast): {wp1} → {wp2} @ {approach * 2} pps…", "cyan")
        motor.move_to(wp2)
        if not wait_for_position(motor, timeout=MOVE_TIMEOUT):
            cprint("  ✗  Phase 2 timed out — aborting.", "red")
            return

        # Phase 3 — slow again
        apply_slow()
        cprint(f"  Phase 3/3 (slow): {wp2} → {wp3} @ {approach} pps…", "cyan")
        motor.move_to(wp3)
        if not wait_for_position(motor, timeout=MOVE_TIMEOUT):
            cprint("  ✗  Phase 3 timed out — aborting.", "red")
            return
    finally:
        # Always restore the original values, even if something blew up above.
        for name, val in saved.items():
            set_ap(motor, name, val)

    # ── Hold phase ────────────────────────────────────────────────────────────
    cprint(f"  Holding for {duration}s  (slip threshold={slip_thr} ticks)…", "cyan")
    slip_events: list[dict] = []
    t0 = time.time()

    while time.time() - t0 < duration:
        try:
            enc  = _s32(motor.get_axis_parameter(motor.AP.EncoderPosition))
            slip = abs(target_pos - enc)

            # Sanity-check the encoder reading before acting on it
            if slip > ENC_SANITY_MARGIN:
                cprint(
                    f"  ⚠  Encoder jump {slip} ticks is suspiciously large — "
                    "skipping correction this cycle.",
                    "yellow",
                )
                time.sleep(0.1)
                continue

            if slip > slip_thr:
                elapsed = round(time.time() - t0, 3)
                cprint(f"  ⚠  Slip {slip} ticks at t={elapsed}s → correcting", "yellow")
                slip_events.append({"t": elapsed, "slip_ticks": slip})

                motor.set_axis_parameter(motor.AP.ActualPosition, enc)
                motor.move_to(target_pos)
                if not wait_for_position(motor, timeout=3.0):
                    cprint("  ✗  Slip correction timed out.", "red")

        except Exception as e:
            cprint(f"  CAN error: {e}", "red")

        time.sleep(0.1)

    total_slip = sum(e["slip_ticks"] for e in slip_events)
    cprint(
        f"\n  Hold complete: {len(slip_events)} slip event(s), "
        f"{total_slip} total tick-slip.",
        "cyan" if not slip_events else "yellow",
    )

    # ── Return ────────────────────────────────────────────────────────────────
    motor.set_axis_parameter(
        motor.AP.ActualPosition,
        _s32(motor.get_axis_parameter(motor.AP.EncoderPosition)),
    )
    cprint(f"  Returning to {start_pos}…", "cyan")
    motor.move_to(start_pos)
    if not wait_for_position(motor, timeout=MOVE_TIMEOUT):
        cprint("  ✗  Return move timed out.", "red")
    cprint("  ✓  Brake test done.", "green")

    # ── Log results ───────────────────────────────────────────────────────────
    if log_path:
        params_at_test = snapshot_params(motor)
        row = {
            "timestamp":    datetime.now().isoformat(timespec="seconds"),
            "retract_ticks": retract,
            "hold_duration": duration,
            "slip_threshold": slip_thr,
            "approach_pps":  approach,
            "slip_events":   len(slip_events),
            "total_slip_ticks": total_slip,
            **params_at_test,
        }
        write_header = not Path(log_path).exists()
        with open(log_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=row.keys())
            if write_header:
                writer.writeheader()
            writer.writerow(row)
        cprint(f"  ✓  Results appended to {log_path}", "dim green")


# ─── Menu ─────────────────────────────────────────────────────────────────────
#
# Each entry: (primary_key, aliases_tuple, description)
# Aliases are shown in the Rich panel and accepted by the dispatcher.
# Keep aliases short and mnemonic; the first char of the description is a
# natural choice when it doesn't clash.

MENU = [
    ("e", ("edit",),          "Edit a parameter"),
    ("v", ("view",),          "View all parameters"),
    ("b", ("brake", "test"),  "Run brake test"),
    ("s", ("save",),          "Save parameters to file"),
    ("l", ("load",),          "Load parameters from file"),
    ("q", ("quit", "exit"),   "Quit"),
]

# Flat lookup: every accepted key/alias → primary key
_DISPATCH: dict[str, str] = {}
for _pk, _aliases, _ in MENU:
    _DISPATCH[_pk] = _pk
    for _a in _aliases:
        _DISPATCH[_a] = _pk


def _alias_hint(aliases: tuple[str, ...]) -> str:
    """Return a dim comma-separated alias list, e.g. '  edit'."""
    return ", ".join(aliases)


def print_menu() -> None:
    if HAS_RICH:
        from rich.text import Text

        body = Text()
        rows = []
        for pk, aliases, desc in MENU:
            row = Text()
            row.append(f"  [{pk}]", style="bold cyan")
            row.append(f"  {desc}")
            if aliases:
                row.append(f"  ({_alias_hint(aliases)})", style="dim")
            rows.append(row)
        for i, row in enumerate(rows):
            body.append_text(row)
            if i < len(rows) - 1:
                body.append("\n")
        console.print(
            Panel(
                body,
                title="[bold cyan]Brake Tuner[/bold cyan]",
                border_style="cyan",
                expand=False,
            )
        )
    else:
        print("\n" + "═" * 44)
        print("  Brake Tuner — TMCM-1260")
        print("═" * 44)
        for pk, aliases, desc in MENU:
            hint = f"  ({_alias_hint(aliases)})" if aliases else ""
            print(f"  [{pk}]  {desc}{hint}")
        print("═" * 44)


def run(motor, log_path: str | None = None) -> None:
    while True:
        print_menu()
        choice = ask("  >").strip().lower()
        action = _DISPATCH.get(choice)

        if   action == "e": edit_param(motor)
        elif action == "v": show_params(motor)
        elif action == "b": brake_test(motor, log_path=log_path)
        elif action == "s": save_params(motor)
        elif action == "l": load_params(motor)
        elif action == "q":
            cprint("\n  Bye.\n", "dim")
            break
        else:
            cprint(f"  ✗  Unknown: '{choice}'", "red")


# ─── Entry point ──────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser(description="TMCM-1260 brake tuner")
    p.add_argument("--interface", default="can", choices=["can", "usb"])
    p.add_argument("--port",      default=None)
    p.add_argument("--id",        type=int, default=1, dest="module_id")
    p.add_argument("--data-rate", type=int, default=1_000_000)
    p.add_argument("--log",       default=None, metavar="CSV",
                   help="CSV file to append brake-test results to")
    args = p.parse_args()

    port = args.port or ("can1" if args.interface == "can" else "/dev/ttyACM0")

    if args.interface == "can":
        conn_str = (
            f"--interface socketcan_tmcl --port {port} --data-rate {args.data_rate}"
        )
    else:
        conn_str = f"--interface usb_tmcl --port {port}"

    cprint(f"\n  Connecting via [bold]{args.interface.upper()}[/bold] on {port}…", "cyan")
    iface  = ConnectionManager(conn_str).connect()
    module = TMCM1260(iface, module_id=args.module_id)
    motor  = module.motors[0]
    cprint(f"  ✓  Connected (module id={args.module_id})\n", "green")

    try:
        run(motor, log_path=args.log)
    finally:
        motor.stop()
        iface.close()
        cprint("  Interface closed.", "dim")


if __name__ == "__main__":
    main()
