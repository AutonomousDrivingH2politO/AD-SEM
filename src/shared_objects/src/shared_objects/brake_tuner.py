#!/usr/bin/env python3
"""
brake_tuner.py
────────────────
Quick brake-tuning tool for the TMCM-1260.
Edit speed / acceleration params and fire a brake test without restarting.

Usage
─────
    python brake_tuner.py                          # CAN, port can1, id 1
    python brake_tuner.py --interface usb
    python brake_tuner.py --interface can --port can0 --id 2
    python brake_tuner.py --log results.csv        # append test results here
"""

import argparse
import csv
import json
import struct
import sys
import threading
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


# ─── Axis-parameter numbers ──────────────────────────────────────────────────
# Raw AP numbers used by both PARAMS and the StallGuard monitor.
# Verify against your TMCM-1260 firmware manual if anything looks off — newer
# firmware revisions moved SGT to AP 202 and SG result to AP 212 (this file).

# Current
AP_MAX_CURRENT     = 6    # Run current, 0-255
AP_STANDBY_CURRENT = 7    # Standby / holding current, 0-255

# StallGuard2 / stop-on-stall
SG_AP_RESULT       = 212  # SG2 result / actual load (read-only)
SG_AP_THRESHOLD    = 202  # SGT — threshold the driver compares against (-64..+63)
SG_AP_STOP_VEL     = 181  # Stop-on-stall velocity threshold (0 disables)

# CoolStep (smartEnergy) — CoolStepMin (SEMIN) > 0 enables CoolStep
CS_AP_I_MIN        = 168  # SEIMIN: min current (0 = ½ MaxCurrent, 1 = ¼ MaxCurrent)
CS_AP_DOWN_STEP    = 169  # SEDN:   current down-step exponent (0..3)
CS_AP_MAX          = 170  # SEMAX:  upper hysteresis width (0..15)
CS_AP_UP_STEP      = 171  # SEUP:   current up-step exponent (0..3)
CS_AP_MIN          = 172  # SEMIN:  lower hysteresis (0..15, 0 = CoolStep OFF)


# ─── Parameters exposed for editing ──────────────────────────────────────────
# Each row: (display_name, attr_or_ap, unit, description)
#   attr_or_ap is either a string (looked up via motor.AP.<name>) or an int
#   (used directly as a raw axis-parameter number).

PARAMS = [
    # Ramp / speed
    ("StartVelocity",       "StartVelocity",       "pps",   "Speed at ramp start (launch speed)"),
    ("StopVelocity",        "StopVelocity",        "pps",   "Speed at ramp end (near stop)"),
    ("MaxVelocity",         "MaxVelocity",         "pps",   "Top cruising speed"),
    ("MaxAcceleration",     "MaxAcceleration",     "pps²",  "Acceleration to MaxVelocity"),
    ("MaxDeceleration",     "MaxDeceleration",     "pps²",  "Deceleration from MaxVelocity"),
    ("V1",                  "V1",                  "pps",   "Intermediate ramp speed"),
    ("A1",                  "A1",                  "pps²",  "Acceleration up to V1"),
    ("D1",                  "D1",                  "pps²",  "Deceleration from V1"),
    ("MicrostepResolution", "MicrostepResolution", "µstep", "Microstep resolution (1/2/4/8/16/32/64/128/256)"),

    # Current
    ("MaxCurrent",          AP_MAX_CURRENT,        "0-255", "Run current (peak)"),
    ("StandbyCurrent",      AP_STANDBY_CURRENT,    "0-255", "Standby / holding current"),

    # StallGuard2
    ("SG2Threshold",        SG_AP_THRESHOLD,       "±",     "StallGuard2 threshold SGT (-64..+63, lower = more sensitive)"),
    ("StopOnStallVel",      SG_AP_STOP_VEL,        "pps",   "Stop-on-stall velocity threshold (0 disables)"),

    # CoolStep (smartEnergy) — CoolStepMin > 0 enables CoolStep
    ("CoolStepMin",         CS_AP_MIN,             "0-15",  "SEMIN — set > 0 to enable CoolStep (0 = OFF)"),
    ("CoolStepMax",         CS_AP_MAX,             "0-15",  "SEMAX — upper hysteresis width"),
    ("CoolStepUpStep",      CS_AP_UP_STEP,         "0-3",   "SEUP — current up-step exponent"),
    ("CoolStepDownStep",    CS_AP_DOWN_STEP,       "0-3",   "SEDN — current down-step exponent"),
    ("CoolStepIMin",        CS_AP_I_MIN,           "0/1",   "SEIMIN — min current (0=½ MaxCurrent, 1=¼)"),
]

VALID_MICROSTEPS = {1, 2, 4, 8, 16, 32, 64, 128, 256}

# Inclusive range checks used by edit_param. None = unbounded on that side.
PARAM_RANGES: dict[str, tuple[int | None, int | None]] = {
    "MaxCurrent":       (0, 255),
    "StandbyCurrent":   (0, 255),
    "SG2Threshold":     (-64, 63),
    "StopOnStallVel":   (0, None),
    "CoolStepMin":      (0, 15),
    "CoolStepMax":      (0, 15),
    "CoolStepUpStep":   (0, 3),
    "CoolStepDownStep": (0, 3),
    "CoolStepIMin":     (0, 1),
}

POLL_INTERVAL = 0.05  # seconds
MOVE_TIMEOUT = 30.0  # seconds — override per call if needed
ENC_SANITY_MARGIN = 500_000  # ticks; encoder reads outside ±this from target

# ─── StallGuard monitor configuration ─────────────────────────────────────────

SG_POLL_HZ = 10  # default polls per second for the monitor thread

# ─── Global rig state & Thread Locks ──────────────────────────────────────────

_rig: dict = {"zero_pos": None}

bus_lock = threading.Lock()  # Prevents CAN/USB collisions
print_lock = threading.Lock()  # Prevents console interleaving
ui_busy = threading.Event()  # Suppresses SG printing during input prompts


# ─── StallGuard monitor ───────────────────────────────────────────────────────


class StallGuardMonitor:
    """
    Background thread that continuously polls the StallGuard result register
    and prints it to stdout without blocking the interactive menu.
    """

    def __init__(self, motor, poll_hz: float = SG_POLL_HZ) -> None:
        self._motor = motor
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None

        self._history: list[int] = []
        self._history_len = 20
        self.stall_count = 0
        self.set_poll_hz(poll_hz)

    def set_poll_hz(self, hz: float) -> None:
        self._interval = max(0.05, 1.0 / max(hz, 1e-9))

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            cprint("  ⚠  StallGuard monitor already running.", "yellow")
            return
        self._stop_evt.clear()
        self.stall_count = 0
        self._history.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="sg-monitor"
        )
        self._thread.start()
        cprint(
            f"  ✓  StallGuard monitor started  "
            f"(AP {SG_AP_RESULT}, {1 / self._interval:.0f} Hz)",
            "green",
        )

    def stop(self) -> None:
        if not self._thread or not self._thread.is_alive():
            cprint("  ⚠  StallGuard monitor is not running.", "yellow")
            return
        self._stop_evt.set()
        self._thread.join(timeout=2.0)
        with print_lock:
            sys.stdout.write("\r" + " " * 80 + "\r")
            sys.stdout.flush()
        cprint(
            f"  ✓  StallGuard monitor stopped  (stall events seen: {self.stall_count})",
            "green",
        )

    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def _read_sg(self) -> tuple[int, int]:
        with bus_lock:
            result = _s32(self._motor.get_axis_parameter(SG_AP_RESULT))
            threshold = _s32(self._motor.get_axis_parameter(SG_AP_THRESHOLD))
        return result, threshold

    @staticmethod
    def _sparkline(values: list[int], width: int = 10) -> str:
        bars = "▁▂▃▄▅▆▇█"
        if not values:
            return " " * width
        lo, hi = min(values), max(values)
        span = hi - lo or 1
        chars = [bars[int((v - lo) / span * (len(bars) - 1))] for v in values[-width:]]
        return "".join(chars).ljust(width)

    def _run(self) -> None:
        err_count = 0
        while not self._stop_evt.is_set():
            try:
                sg, thr = self._read_sg()
                err_count = 0

                self._history.append(sg)
                if len(self._history) > self._history_len:
                    self._history.pop(0)

                stalled = sg == 0
                if stalled:
                    self.stall_count += 1

                # Only print if we aren't currently asking the user for input
                if not ui_busy.is_set():
                    spark = self._sparkline(self._history)
                    status = " [STALL]" if stalled else ""
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                    line = (
                        f"\r  [SG] {ts}  result={sg:>4}  threshold={thr:>4}"
                        f"  {spark}{status}   "
                    )
                    with print_lock:
                        sys.stdout.write(line)
                        sys.stdout.flush()

            except Exception as exc:
                err_count += 1
                if err_count <= 3 and not ui_busy.is_set():
                    with print_lock:
                        sys.stdout.write(f"\r  [SG] read error: {exc}   ")
                        sys.stdout.flush()

            self._stop_evt.wait(self._interval)


# ─── Helpers ──────────────────────────────────────────────────────────────────


def _s32(v: int) -> int:
    return struct.unpack(">i", struct.pack(">I", v & 0xFFFFFFFF))[0]


def _resolve_ap(motor, attr) -> int:
    """attr may be a string (looked up in motor.AP) or a raw int AP number."""
    if isinstance(attr, int):
        return attr
    return getattr(motor.AP, attr)


def get_ap(motor, attr) -> int:
    with bus_lock:
        return _s32(motor.get_axis_parameter(_resolve_ap(motor, attr)))


def set_ap(motor, attr, value: int) -> None:
    with bus_lock:
        motor.set_axis_parameter(_resolve_ap(motor, attr), value)


def wait_for_position(motor, timeout: float = MOVE_TIMEOUT) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        with bus_lock:
            reached = motor.get_position_reached()
        if reached:
            return True
        time.sleep(POLL_INTERVAL)
    return False


def snapshot_params(motor) -> dict:
    return {name: get_ap(motor, attr) for name, attr, *_ in PARAMS}


def cprint(msg: str, style: str = "") -> None:
    with print_lock:
        sys.stdout.write("\r" + " " * 80 + "\r")
        sys.stdout.flush()
        if HAS_RICH:
            console.print(msg, style=style)
        else:
            import re

            print(re.sub(r"\[/?[a-z_ ]*\]", "", msg))


def ask(prompt: str, default=None, pause_monitor=True) -> str:
    if pause_monitor:
        ui_busy.set()
    try:
        with print_lock:
            sys.stdout.write("\r" + " " * 80 + "\r")
            sys.stdout.flush()
        if HAS_RICH:
            raw = Prompt.ask(
                prompt, default=str(default) if default is not None else ""
            )
            return raw if raw is not None else ""
        suffix = f" [{default}]" if default is not None else ""
        raw = input(f"{prompt}{suffix}: ").strip()
        return raw if raw else (str(default) if default is not None else "")
    finally:
        if pause_monitor:
            ui_busy.clear()


def ask_int(prompt: str, default=None, pause_monitor=True) -> int:
    while True:
        raw = ask(prompt, default, pause_monitor)
        try:
            return int(raw)
        except ValueError:
            cprint(f"  ✗  '{raw}' is not an integer.", "red")


def ask_float(prompt: str, default=None, pause_monitor=True) -> float:
    while True:
        raw = ask(prompt, default, pause_monitor)
        try:
            return float(raw)
        except ValueError:
            cprint(f"  ✗  '{raw}' is not a number.", "red")


# ─── Display ──────────────────────────────────────────────────────────────────


def show_params(motor) -> None:
    # Fetch parameters first to avoid holding locks longer than needed
    rows = []
    for i, (name, attr, unit, desc) in enumerate(PARAMS, 1):
        rows.append((str(i), name, str(get_ap(motor, attr)), unit, desc))

    zero_str = (
        str(_rig["zero_pos"]) if _rig["zero_pos"] is not None else "[dim]not set[/dim]"
    )

    with print_lock:
        sys.stdout.write("\r" + " " * 80 + "\r")
        sys.stdout.flush()
        if HAS_RICH:
            t = Table(
                box=box.SIMPLE_HEAVY,
                show_lines=False,
                header_style="bold cyan",
                title="Speed / Acceleration Parameters",
            )
            t.add_column("#", style="dim", width=4)
            t.add_column("Name", style="bold white", width=22)
            t.add_column("Value", style="yellow", width=12, justify="right")
            t.add_column("Unit", style="dim green", width=7)
            t.add_column("Description", style="dim")
            for r in rows:
                t.add_row(*r)
            console.print(t)
            console.print(f"  Zero point: {zero_str}")
        else:
            print(f"\n{'─' * 72}")
            print("  Speed / Acceleration Parameters")
            print(f"{'─' * 72}")
            print(f"  {'#':<3}  {'Name':<22}  {'Value':>10}  {'Unit':<7}  Description")
            print(f"{'─' * 72}")
            for r in rows:
                print(f"  {r[0]:<3}  {r[1]:<22}  {r[2]:>10}  {r[3]:<7}  {r[4]}")
            import re

            # Extract regex replacement to a variable to prevent f-string backslash errors (< Python 3.12)
            clean_zero_str = re.sub(r"\[/?[a-z_ ]*\]", "", zero_str)
            print(f"\n  Zero point: {clean_zero_str}")

            print(f"{'─' * 72}\n")


# ─── Parameter snapshot save / load ──────────────────────────────────────────


def save_params(motor) -> None:
    path = ask("  Save to file", default="params.json").strip() or "params.json"
    data = snapshot_params(motor)
    if _rig["zero_pos"] is not None:
        data["_zero_pos"] = _rig["zero_pos"]
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
                continue
            # Clamp values that have a declared range so a stale or hand-edited
            # JSON file can't push the driver into an invalid state.
            if name in PARAM_RANGES:
                lo, hi = PARAM_RANGES[name]
                if lo is not None and value < lo:
                    cprint(f"  ⚠  {name}={value} below min ({lo}), clamping.", "yellow")
                    value = lo
                if hi is not None and value > hi:
                    cprint(f"  ⚠  {name}={value} above max ({hi}), clamping.", "yellow")
                    value = hi
            set_ap(motor, attr, value)
            readback = get_ap(motor, attr)
            cprint(f"  ✓  {name} = {readback} {unit}", "green")
        else:
            cprint(f"  ⚠  {name} not found in file, skipped.", "yellow")

    if "_zero_pos" in data:
        _rig["zero_pos"] = int(data["_zero_pos"])
        cprint(f"  ✓  Zero point restored: {_rig['zero_pos']} ticks", "green")


# ─── Edit ─────────────────────────────────────────────────────────────────────


def _ask_int_in_range(prompt: str, default: int, lo: int | None, hi: int | None) -> int:
    """Ask for an int and re-prompt until it lies in [lo, hi] (inclusive, None=unbounded)."""
    while True:
        new_val = ask_int(prompt, default=default)
        ok = (lo is None or new_val >= lo) and (hi is None or new_val <= hi)
        if ok:
            return new_val
        lo_str = str(lo) if lo is not None else "−∞"
        hi_str = str(hi) if hi is not None else "+∞"
        cprint(f"  ✗  Must be in range [{lo_str}..{hi_str}]", "red")


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

    if name == "MicrostepResolution":
        sorted_ms = sorted(VALID_MICROSTEPS)
        while True:
            new_val = ask_int(f"  New value [{unit}]", default=current)
            if new_val in VALID_MICROSTEPS:
                break
            cprint(f"  ✗  Choose from: {sorted_ms}", "red")
        if new_val == current:
            return
        set_ap(motor, attr, new_val)
        cprint(f"  ✓  {name} = {get_ap(motor, attr)} µstep", "green")
        return

    cprint(f"\n  [bold]{name}[/bold] — {desc}  (now: {current} {unit})")

    if name in PARAM_RANGES:
        lo, hi = PARAM_RANGES[name]
        new_val = _ask_int_in_range(f"  New value [{unit}]", current, lo, hi)
    else:
        new_val = ask_int(f"  New value [{unit}]", default=current)

    if new_val == current:
        return

    set_ap(motor, attr, new_val)
    cprint(f"  ✓  {name} = {get_ap(motor, attr)} {unit}", "green")

    # Friendly hint for CoolStep enable/disable transitions
    if name == "CoolStepMin":
        if new_val == 0 and current != 0:
            cprint("  ℹ  CoolStep is now DISABLED (SEMIN = 0).", "cyan")
        elif new_val != 0 and current == 0:
            cprint("  ℹ  CoolStep is now ENABLED (SEMIN > 0).", "cyan")


# ─── Zero point ───────────────────────────────────────────────────────────────


def set_zero(motor) -> None:
    with bus_lock:
        pos = _s32(motor.get_axis_parameter(motor.AP.ActualPosition))
    _rig["zero_pos"] = pos
    cprint(f"  ✓  Zero point set at tick {pos}  (save params to persist)", "green")


def goto_zero(motor) -> bool:
    if _rig["zero_pos"] is None:
        cprint("  ✗  No zero point set. Use [z] first.", "red")
        return False

    target = _rig["zero_pos"]
    with bus_lock:
        current = _s32(motor.get_axis_parameter(motor.AP.ActualPosition))

    if current == target:
        cprint(f"  ✓  Already at zero point ({target} ticks).", "green")
        return True

    cprint(f"  Moving to zero point: {current} → {target} ticks…", "cyan")

    with bus_lock:
        motor.move_to(target)

    if not wait_for_position(motor, timeout=MOVE_TIMEOUT):
        cprint("  ✗  Move to zero point timed out.", "red")
        return False

    cprint(f"  ✓  At zero point ({target} ticks).", "green")
    return True


# ─── StallGuard Tuning & Control ──────────────────────────────────────────────


def sg_toggle(monitor: StallGuardMonitor) -> None:
    if monitor.running:
        monitor.stop()
    else:
        hz = ask_float("  Poll rate (Hz)", default=SG_POLL_HZ)
        monitor.set_poll_hz(hz)
        monitor.start()


def sg_show_once(motor) -> None:
    try:
        with bus_lock:
            sg = _s32(motor.get_axis_parameter(SG_AP_RESULT))
            thr = _s32(motor.get_axis_parameter(SG_AP_THRESHOLD))
        stalled = sg == 0
        status = "  ← [bold red]STALL[/bold red]" if stalled else ""
        cprint(
            f"  StallGuard result={sg}  threshold={thr}{status}",
            "red" if stalled else "cyan",
        )
    except Exception as exc:
        cprint(f"  ✗  Could not read StallGuard: {exc}", "red")


def tune_stallguard_threshold(motor) -> None:
    cprint("\n  [bold cyan]StallGuard2 Threshold (SGT) Tuner[/bold cyan]")
    cprint("  Goal: find the SGT where SG ≥ 100 with a slack string")
    cprint("        and SG ≈ 0 when the string is at your safe working tension.")

    with bus_lock:
        original_sgt = _s32(motor.get_axis_parameter(SG_AP_THRESHOLD))
    vel = ask_int("  Test velocity (pps)", default=5000)

    # ── PHASE 1: no-load sweep
    cprint("\n  [bold yellow][Phase 1/2] NO-LOAD SWEEP[/bold yellow]")
    cprint("  Make sure the string is completely slack before continuing.")
    ask("  Press Enter when ready…")

    with bus_lock:
        motor.rotate(vel)
    time.sleep(1.5)

    no_load_sg: dict[int, int] = {}
    cprint("  Sweeping SGT from +10 down to -10 and sampling SG…", "dim")
    for sgt in range(10, -11, -1):
        with bus_lock:
            motor.set_axis_parameter(SG_AP_THRESHOLD, sgt)
        time.sleep(0.15)
        with bus_lock:
            sg = _s32(motor.get_axis_parameter(SG_AP_RESULT))
        no_load_sg[sgt] = sg
        cprint(f"    SGT={sgt:>3}  →  SG (no load) = {sg}", "dim")

    with bus_lock:
        motor.stop()
    time.sleep(0.5)

    candidates = [sgt for sgt, sg in no_load_sg.items() if sg >= 100]

    if not candidates:
        cprint("\n  ✗  No SGT value produced SG ≥ 100 under no-load conditions.", "red")
        with bus_lock:
            motor.set_axis_parameter(SG_AP_THRESHOLD, original_sgt)
        return

    cprint(f"\n  ✓  Candidate SGT values (no-load SG ≥ 100): {candidates}", "green")

    # ── PHASE 2: safe-tension sweep
    cprint("\n  [bold yellow][Phase 2/2] SAFE-TENSION SWEEP[/bold yellow]")
    cprint("  The motor will spin again.  Once it has ramped up, apply your")
    cprint("  DESIRED MAXIMUM SAFE TENSION to the string.")
    ask("  Press Enter to start the motor…")

    with bus_lock:
        motor.rotate(vel)
    time.sleep(1.5)

    cprint("\n  [bold red]>>> APPLY SAFE TENSION NOW — hold it steady <<<[/bold red]")
    time.sleep(1.0)

    safe_tension_sg: dict[int, int] = {}

    cprint("  Sampling candidates under safe tension…", "dim")
    for sgt in candidates:
        with bus_lock:
            motor.set_axis_parameter(SG_AP_THRESHOLD, sgt)
        time.sleep(0.2)
        with bus_lock:
            sg = _s32(motor.get_axis_parameter(SG_AP_RESULT))
        safe_tension_sg[sgt] = sg
        cprint(f"    SGT={sgt:>3}  →  SG (safe tension) = {sg}", "dim")

    with bus_lock:
        motor.stop()
    time.sleep(0.5)

    usable = {sgt: sg for sgt, sg in safe_tension_sg.items() if sg >= 0}

    if not usable:
        cprint(
            "\n  ✗  Every candidate SGT triggered a stall at the applied tension.",
            "yellow",
        )
        with bus_lock:
            motor.set_axis_parameter(SG_AP_THRESHOLD, original_sgt)
        return

    min_sg = min(usable.values())
    best_sgt = min(
        (sgt for sgt, sg in usable.items() if sg == min_sg), key=lambda s: abs(s)
    )

    cprint(
        f"\n  ✓  Best SGT = {best_sgt}  (no-load SG = {no_load_sg[best_sgt]}, safe-tension SG = {safe_tension_sg[best_sgt]})",
        "green",
    )

    with bus_lock:
        motor.set_axis_parameter(SG_AP_THRESHOLD, best_sgt)
    cprint("  Use [s] to save this setting to your params file.", "cyan")


# ─── Brake test ───────────────────────────────────────────────────────────────

_PHASED_PARAMS = ("MaxVelocity", "MaxAcceleration", "MaxDeceleration", "V1", "A1", "D1")


def brake_test(
    motor, log_path: str | None = None, monitor: "StallGuardMonitor | None" = None
) -> None:
    cprint("\n  [bold cyan]Brake Test[/bold cyan]")

    # Auto-manage StallGuard monitor
    monitor_was_running = monitor is not None and monitor.running
    if monitor is not None and not monitor.running:
        cprint(
            "  ℹ  StallGuard monitor will run during the brake test.\n"
            "     SG readings appear on the line below — they don't block input.",
            "cyan",
        )
        monitor.start()

    if _rig["zero_pos"] is not None:
        raw = ask(
            f"  Home to zero point ({_rig['zero_pos']} ticks) before test? [y/N]",
            default="n",
        )
        if raw.strip().lower() in ("y", "yes"):
            if not goto_zero(motor):
                cprint("  ✗  Could not reach zero point — aborting test.", "red")
                if not monitor_was_running and monitor is not None and monitor.running:
                    monitor.stop()
                return

    retract = ask_int("  Retract ticks   (rope wind distance)", default=70_000)
    duration = ask_float("  Hold duration   (seconds)", default=5.0)
    slip_thr = ask_int("  Slip threshold  (encoder ticks)", default=200)
    approach = ask_int("  Approach velocity (pps)", default=20_000)

    max_vel = get_ap(motor, "MaxVelocity")
    if approach > max_vel:
        cprint(
            f"  ⚠  Approach velocity ({approach}) > MaxVelocity ({max_vel}); clamping to {max_vel}.",
            "yellow",
        )
        approach = max_vel

    with bus_lock:
        start_pos = _s32(motor.get_axis_parameter(motor.AP.ActualPosition))
    target_pos = start_pos + retract

    saved = {name: get_ap(motor, name) for name in _PHASED_PARAMS}

    third = retract // 3
    wp1 = start_pos + third
    wp2 = start_pos + 2 * third
    wp3 = target_pos

    def apply_slow() -> None:
        set_ap(motor, "MaxVelocity", approach)
        set_ap(motor, "MaxAcceleration", saved["MaxAcceleration"])
        set_ap(motor, "MaxDeceleration", saved["MaxDeceleration"])
        set_ap(motor, "V1", saved["V1"])
        set_ap(motor, "A1", saved["A1"])
        set_ap(motor, "D1", saved["D1"])

    def apply_fast() -> None:
        set_ap(motor, "MaxVelocity", approach * 3)
        set_ap(motor, "MaxAcceleration", saved["MaxAcceleration"] * 3)
        set_ap(motor, "MaxDeceleration", saved["MaxDeceleration"] * 3)
        set_ap(motor, "V1", saved["V1"] * 2)
        set_ap(motor, "A1", saved["A1"] * 2)
        set_ap(motor, "D1", saved["D1"] * 2)

    cprint(
        f"\n  Approaching {start_pos} → {target_pos} in 3 phases (slow → fast → slow)…",
        "cyan",
    )

    try:
        apply_fast()
        cprint(f"  Phase 1/3 (slow): {start_pos} → {wp1} @ {approach} pps…", "cyan")
        with bus_lock:
            motor.move_to(wp1)
        if not wait_for_position(motor, timeout=MOVE_TIMEOUT):
            cprint("  ✗  Phase 1 timed out — aborting.", "red")
            return

        apply_fast()
        cprint(f"  Phase 2/3 (fast): {wp1} → {wp2} @ {approach * 3} pps…", "cyan")
        with bus_lock:
            motor.move_to(wp2)
        if not wait_for_position(motor, timeout=MOVE_TIMEOUT):
            cprint("  ✗  Phase 2 timed out — aborting.", "red")
            return

        apply_fast()
        cprint(f"  Phase 3/3 (slow): {wp2} → {wp3} @ {approach} pps…", "cyan")
        with bus_lock:
            motor.move_to(wp3)
        if not wait_for_position(motor, timeout=MOVE_TIMEOUT):
            cprint("  ✗  Phase 3 timed out — aborting.", "red")
            return
    finally:
        for name, val in saved.items():
            set_ap(motor, name, val)

    cprint(f"  Holding for {duration}s  (slip threshold={slip_thr} ticks)…", "cyan")
    slip_events: list[dict] = []
    t0 = time.time()

    while time.time() - t0 < duration:
        try:
            with bus_lock:
                enc = _s32(motor.get_axis_parameter(motor.AP.EncoderPosition))
            slip = abs(target_pos - enc)

            if slip > ENC_SANITY_MARGIN:
                time.sleep(0.1)
                continue

            if slip > slip_thr:
                elapsed = round(time.time() - t0, 3)
                cprint(f"  ⚠  Slip {slip} ticks at t={elapsed}s → correcting", "yellow")
                slip_events.append({"t": elapsed, "slip_ticks": slip})

                with bus_lock:
                    motor.set_axis_parameter(motor.AP.ActualPosition, enc)
                    motor.move_to(target_pos)

                if not wait_for_position(motor, timeout=3.0):
                    cprint("  ✗  Slip correction timed out.", "red")
                else:
                    with bus_lock:
                        enc_after = _s32(
                            motor.get_axis_parameter(motor.AP.EncoderPosition)
                        )
                        motor.set_axis_parameter(motor.AP.ActualPosition, enc_after)

        except Exception as e:
            cprint(f"  CAN error: {e}", "red")

        time.sleep(0.1)

    total_slip = sum(e["slip_ticks"] for e in slip_events)
    cprint(
        f"\n  Hold complete: {len(slip_events)} slip event(s), {total_slip} total tick-slip.",
        "cyan" if not slip_events else "yellow",
    )

    if not monitor_was_running and monitor is not None and monitor.running:
        monitor.stop()

    with bus_lock:
        enc_final = _s32(motor.get_axis_parameter(motor.AP.EncoderPosition))
        motor.set_axis_parameter(motor.AP.ActualPosition, enc_final)

    cprint(f"  Returning to {start_pos}…", "cyan")

    with bus_lock:
        motor.move_to(start_pos)

    if not wait_for_position(motor, timeout=MOVE_TIMEOUT):
        cprint("  ✗  Return move timed out.", "red")
    cprint("  ✓  Brake test done.", "green")

    if log_path:
        params_at_test = snapshot_params(motor)
        row = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "zero_pos": _rig["zero_pos"],
            "retract_ticks": retract,
            "hold_duration": duration,
            "slip_threshold": slip_thr,
            "approach_pps": approach,
            "slip_events": len(slip_events),
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

MENU = [
    ("e", ("edit",), "Edit a parameter"),
    ("v", ("view",), "View all parameters"),
    ("b", ("brake", "test"), "Run brake test"),
    ("z", ("zero", "setzero"), "Set current position as zero point"),
    ("0", ("home", "gohome"), "Move to saved zero point"),
    ("s", ("save",), "Save parameters to file"),
    ("l", ("load",), "Load parameters from file"),
    ("t", ("tune", "sgt"), "Tune StallGuard threshold for safe tension"),
    ("g", ("sg", "stallguard"), "Toggle StallGuard monitor  (background thread)"),
    ("r", ("read", "sgread"), "Read StallGuard once"),
    ("q", ("quit", "exit"), "Quit"),
]

_DISPATCH: dict[str, str] = {}
for _pk, _aliases, _ in MENU:
    _DISPATCH[_pk] = _pk
    for _a in _aliases:
        _DISPATCH[_a] = _pk


def _alias_hint(aliases: tuple[str, ...]) -> str:
    return ", ".join(aliases)


def print_menu(monitor: StallGuardMonitor) -> None:
    sg_status = (
        "[bold green]● running[/bold green]"
        if monitor.running
        else "[dim]○ stopped[/dim]"
    )
    zero_status = (
        f"[bold green]● {_rig['zero_pos']} ticks[/bold green]"
        if _rig["zero_pos"] is not None
        else "[dim]○ not set[/dim]"
    )

    with print_lock:
        sys.stdout.write("\r" + " " * 80 + "\r")
        sys.stdout.flush()
        if HAS_RICH:
            from rich.text import Text

            rows = []
            for pk, aliases, desc in MENU:
                row = Text()
                row.append(f"  [{pk}]", style="bold cyan")
                if pk == "g":
                    row.append(f"  {desc}  ")
                    row.append_text(Text.from_markup(f"  {sg_status}"))
                elif pk == "z":
                    row.append(f"  {desc}  ")
                    row.append_text(Text.from_markup(f"  {zero_status}"))
                else:
                    row.append(f"  {desc}")
                if aliases:
                    row.append(f"  ({_alias_hint(aliases)})", style="dim")
                rows.append(row)

            body = Text()
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
            running_str = "● running" if monitor.running else "○ stopped"
            zero_str = (
                f"● {_rig['zero_pos']} ticks"
                if _rig["zero_pos"] is not None
                else "○ not set"
            )
            print("\n" + "═" * 56)
            print("  Brake Tuner — TMCM-1260")
            print("═" * 56)
            for pk, aliases, desc in MENU:
                hint = f"  ({_alias_hint(aliases)})" if aliases else ""
                extra = ""
                if pk == "g":
                    extra = f"  [{running_str}]"
                elif pk == "z":
                    extra = f"  [{zero_str}]"
                print(f"  [{pk}]  {desc}{extra}{hint}")
            print("═" * 56)


def run(motor, log_path: str | None = None) -> None:
    monitor = StallGuardMonitor(motor, poll_hz=SG_POLL_HZ)

    try:
        while True:
            if not monitor.running:
                print_menu(monitor)
            # Do NOT pause the monitor at the main menu waiting prompt
            choice = ask("\n  >", pause_monitor=False).strip().lower()
            action = _DISPATCH.get(choice)

            if action == "e":
                edit_param(motor)
            elif action == "v":
                show_params(motor)
            elif action == "b":
                brake_test(motor, log_path=log_path, monitor=monitor)
            elif action == "z":
                set_zero(motor)
            elif action == "0":
                goto_zero(motor)
            elif action == "s":
                save_params(motor)
            elif action == "l":
                load_params(motor)
            elif action == "t":
                tune_stallguard_threshold(motor)
            elif action == "g":
                sg_toggle(monitor)
            elif action == "r":
                sg_show_once(motor)
            elif action == "q":
                cprint("\n  Bye.\n", "dim")
                break
            elif choice == "":
                pass
            else:
                cprint(f"  ✗  Unknown: '{choice}'", "red")
    finally:
        if monitor.running:
            monitor.stop()


# ─── Entry point ──────────────────────────────────────────────────────────────


def main() -> None:
    p = argparse.ArgumentParser(description="TMCM-1260 brake tuner")
    p.add_argument("--interface", default="can", choices=["can", "usb"])
    p.add_argument("--port", default=None)
    p.add_argument("--id", type=int, default=1, dest="module_id")
    p.add_argument("--data-rate", type=int, default=1_000_000)
    p.add_argument(
        "--log",
        default=None,
        metavar="CSV",
        help="CSV file to append brake-test results to",
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
        f"\n  Connecting via [bold]{args.interface.upper()}[/bold] on {port}…", "cyan"
    )
    iface = ConnectionManager(conn_str).connect()
    module = TMCM1260(iface, module_id=args.module_id)
    motor = module.motors[0]
    cprint(f"  ✓  Connected (module id={args.module_id})\n", "green")

    try:
        run(motor, log_path=args.log)
    finally:
        with bus_lock:
            motor.stop()
        iface.close()
        cprint("  Interface closed.", "dim")


if __name__ == "__main__":
    main()
