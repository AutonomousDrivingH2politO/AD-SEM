#!/usr/bin/env python3
"""
brake_tuner.py
────────────────
StallGuard-aware brake-tuning tool for the TMCM-1260.

What's different from the previous version:
  • Adds StallGuard params (SGT, filter, stop-on-stall velocity, standby current)
  • Adds a homing routine — either via STOP_L/HOME switch or via a soft
    StallGuard touch against the brake-released mechanical stop
  • Brake test engages the brake using stop-on-stall in the final phase,
    so the motor stops automatically when the brake reaches its hard
    mechanical limit (no over-pull, no jamming)
  • Hold phase raises standby current to run current so the brake stays
    pressed firmly for the full hold duration
  • Logs SG value and stall flag, not just slip ticks

Usage (unchanged):
    python brake_tuner.py
    python brake_tuner.py --interface usb
    python brake_tuner.py --log results.csv
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


# ─── TMCM-1260 axis-parameter numbers ────────────────────────────────────────
# Source: TMCM-1260 TMCL Firmware Manual V1.46, §4 (Axis Parameters).
# We use raw AP numbers for the StallGuard / hold parameters because
# pytrinamic attribute names vary between releases — these numbers never do.

AP_ACTUAL_POSITION       = 1
AP_MAX_CURRENT           = 6     # run current (0..255)
AP_STANDBY_CURRENT       = 7     # standby current (0..255) — critical for hold
AP_SG2_FILTER_ENABLE     = 173   # 0=off (fast), 1=on (filtered, recommended)
AP_SG2_THRESHOLD         = 174   # SGT, signed, range -64..+63
AP_STOP_ON_STALL         = 181   # velocity threshold; above this, SG=0 stops motor; 0 disables
AP_LOAD_VALUE            = 206   # SG_RESULT, 0..1023, read-only
AP_EXT_ERROR_FLAGS       = 207   # bit 0 = StallGuard error (cleared on read)
AP_DRIVER_ERROR_FLAGS    = 208   # bit 0 = stall detected
AP_ENCODER_POSITION      = 209   # built-in SensOstep encoder
AP_POWER_DOWN_DELAY      = 214   # in 10 ms units, 0..417


# ─── Tunable parameters exposed in the menu ──────────────────────────────────
#  (name, AP number, unit, description)

PARAMS = [
    # Ramp / speed
    ("StartVelocity",         19,  "pps",     "Speed at ramp start (launch speed)"),
    ("StopVelocity",          20,  "pps",     "Speed at ramp end (near stop)"),
    ("MaxVelocity",           4,   "pps",     "Top cruising speed"),
    ("MaxAcceleration",       5,   "pps²",    "Acceleration to MaxVelocity"),
    ("MaxDeceleration",       17,  "pps²",    "Deceleration from MaxVelocity"),
    ("V1",                    16,  "pps",     "Intermediate ramp speed"),
    ("A1",                    15,  "pps²",    "Acceleration up to V1"),
    ("D1",                    18,  "pps²",    "Deceleration from V1"),
    ("MicrostepResolution",   140, "µstep",   "Microstep resolution (0..8 → 1..256)"),
    # Current
    ("RunCurrent",            AP_MAX_CURRENT,      "0..255", "Motor run current (0..255 ≈ 0..6 A RMS)"),
    ("StandbyCurrent",        AP_STANDBY_CURRENT,  "0..255", "Motor standby current (raise this for stronger hold)"),
    # StallGuard
    ("SG2Threshold",          AP_SG2_THRESHOLD,    "SGT",    "StallGuard2 threshold (-64..+63, try ±10 first)"),
    ("SG2FilterEnable",       AP_SG2_FILTER_ENABLE,"0/1",    "0=fast (every full step), 1=filtered (every electrical period)"),
    ("StopOnStallVelocity",   AP_STOP_ON_STALL,    "pps",    "Above this speed, motor stops when SG=0 (0=disabled)"),
]

# AP 140 holds microstep resolution as an enum 0..8 (= 1, 2, 4, … 256 µsteps)
VALID_MICROSTEP_CODES = {0, 1, 2, 3, 4, 5, 6, 7, 8}
MICROSTEP_LABEL = {0: 1, 1: 2, 2: 4, 3: 8, 4: 16, 5: 32, 6: 64, 7: 128, 8: 256}

# Polling / safety
POLL_INTERVAL     = 0.05
MOVE_TIMEOUT      = 30.0
ENC_SANITY_MARGIN = 500_000


# ─── Low-level helpers ───────────────────────────────────────────────────────

def _s32(v: int) -> int:
    """Reinterpret a 32-bit unsigned value as signed."""
    return struct.unpack(">i", struct.pack(">I", v & 0xFFFFFFFF))[0]


def get_ap(motor, ap: int) -> int:
    return _s32(motor.get_axis_parameter(ap))


def set_ap(motor, ap: int, value: int) -> None:
    motor.set_axis_parameter(ap, value)


def wait_for_position(motor, timeout: float = MOVE_TIMEOUT,
                      stall_aborts: bool = False) -> tuple[bool, bool]:
    """
    Poll until position is reached or *timeout* seconds elapse.

    Returns (reached, stalled).
      reached — True if position_reached fired
      stalled — True if stop-on-stall fired during the move
                (only checked when stall_aborts=True)
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if motor.get_position_reached():
            return (True, False)
        if stall_aborts:
            # AP 207 bit 0 latches the StallGuard event; reading clears it.
            errflags = get_ap(motor, AP_EXT_ERROR_FLAGS)
            if errflags & 0x01:
                return (False, True)
        time.sleep(POLL_INTERVAL)
    return (False, False)


def read_sg(motor) -> int:
    """Current StallGuard2 load value (0..1023). Higher = more reserve, 0 = stall."""
    return get_ap(motor, AP_LOAD_VALUE)


def snapshot_params(motor) -> dict:
    return {name: get_ap(motor, ap) for name, ap, *_ in PARAMS}


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


def ask_yes_no(prompt: str, default: bool = True) -> bool:
    dflt = "y" if default else "n"
    while True:
        raw = ask(prompt + " [y/n]", default=dflt).strip().lower()
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False
        cprint("  enter y or n.", "red")


# ─── Display ──────────────────────────────────────────────────────────────────

def show_params(motor) -> None:
    if HAS_RICH:
        t = Table(box=box.SIMPLE_HEAVY, header_style="bold cyan",
                  title="Tunable Parameters")
        t.add_column("#", style="dim", width=4)
        t.add_column("Name", style="bold white", width=22)
        t.add_column("AP",   style="dim",   width=4, justify="right")
        t.add_column("Value", style="yellow", width=12, justify="right")
        t.add_column("Unit",  style="dim green", width=8)
        t.add_column("Description", style="dim")
        for i, (name, ap, unit, desc) in enumerate(PARAMS, 1):
            val = get_ap(motor, ap)
            display_val = (f"{val} (={MICROSTEP_LABEL.get(val, '?')})"
                           if name == "MicrostepResolution" else str(val))
            t.add_row(str(i), name, str(ap), display_val, unit, desc)
        console.print(t)
        # Live readings
        sg   = read_sg(motor)
        pos  = get_ap(motor, AP_ACTUAL_POSITION)
        enc  = get_ap(motor, AP_ENCODER_POSITION)
        cprint(f"  Live  SG={sg}   pos={pos}   enc={enc}", "dim cyan")
    else:
        print("\n" + "─" * 80)
        print(f"  {'#':<3}  {'Name':<22}  {'AP':>4}  {'Value':>10}  {'Unit':<8}")
        print("─" * 80)
        for i, (name, ap, unit, _) in enumerate(PARAMS, 1):
            print(f"  {i:<3}  {name:<22}  {ap:>4}  {get_ap(motor, ap):>10}  {unit:<8}")
        print(f"  SG={read_sg(motor)}  pos={get_ap(motor, AP_ACTUAL_POSITION)}  "
              f"enc={get_ap(motor, AP_ENCODER_POSITION)}")


# ─── Parameter snapshot save / load ──────────────────────────────────────────

def save_params(motor) -> None:
    path = ask("  Save to file", default="params.json").strip() or "params.json"
    Path(path).write_text(json.dumps(snapshot_params(motor), indent=2))
    cprint(f"  ✓  Parameters saved to {path}", "green")


def load_params(motor) -> None:
    path = ask("  Load from file", default="params.json").strip() or "params.json"
    try:
        data = json.loads(Path(path).read_text())
    except FileNotFoundError:
        cprint(f"  ✗  File not found: {path}", "red"); return
    except json.JSONDecodeError as e:
        cprint(f"  ✗  Bad JSON: {e}", "red"); return

    for name, ap, unit, _ in PARAMS:
        if name not in data:
            cprint(f"  ⚠  {name} not in file, skipped.", "yellow"); continue
        value = int(data[name])
        if name == "MicrostepResolution" and value not in VALID_MICROSTEP_CODES:
            cprint(f"  ✗  Skipping MicrostepResolution={value} "
                   f"(valid codes: {sorted(VALID_MICROSTEP_CODES)})", "red"); continue
        set_ap(motor, ap, value)
        cprint(f"  ✓  {name} = {get_ap(motor, ap)} {unit}", "green")


# ─── Edit ─────────────────────────────────────────────────────────────────────

def edit_param(motor) -> None:
    show_params(motor)
    idx = ask_int(f"  Parameter # to change (1-{len(PARAMS)}, 0=cancel)", default=0)
    if idx == 0:
        return
    if not 1 <= idx <= len(PARAMS):
        cprint(f"  ✗  No parameter #{idx}.", "red"); return

    name, ap, unit, desc = PARAMS[idx - 1]
    current = get_ap(motor, ap)

    if name == "MicrostepResolution":
        cprint(f"\n  [bold]{name}[/bold] — {desc}  (now: {current} → "
               f"{MICROSTEP_LABEL.get(current, '?')} µsteps/full step)")
        cprint("  [bold yellow]⚠  Changing this rescales every position counter. "
               "Re-home afterwards.[/bold yellow]")
        while True:
            new_val = ask_int(f"  New code 0..8", default=current)
            if new_val in VALID_MICROSTEP_CODES:
                break
            cprint(f"  ✗  Must be one of {sorted(VALID_MICROSTEP_CODES)}.", "red")
    else:
        cprint(f"\n  [bold]{name}[/bold] (AP {ap}) — {desc}  (now: {current} {unit})")
        new_val = ask_int(f"  New value [{unit}]", default=current)

    if new_val == current:
        cprint("  No change.", "dim"); return
    set_ap(motor, ap, new_val)
    cprint(f"  ✓  {name} = {get_ap(motor, ap)} {unit}", "green")


# ─── Homing ───────────────────────────────────────────────────────────────────
#
# "Home" = cable parallel to ground.  Two strategies:
#
#   (1) manual — operator hand-positions, we zero AP 1 here.
#       Quickest; uses no sensor; trusts the operator.
#
#   (2) stallguard touch — back off in the brake-release direction until the
#       motor lightly stalls against the mechanical stop, then offset forward
#       by `home_offset` ticks and zero.  This is what to use for repeatable
#       reset after power-cycle.

def home_manual(motor) -> None:
    """Zero ActualPosition at the current physical location."""
    set_ap(motor, AP_ACTUAL_POSITION, 0)
    # Also re-sync encoder for cleanliness
    set_ap(motor, AP_ENCODER_POSITION, 0)
    cprint("  ✓  Current position is now home (pos=0, enc=0).", "green")


def home_stallguard(motor) -> None:
    """
    Stallguard-touch homing: drive in the brake-release direction at a low
    velocity with stop-on-stall armed.  When the cable goes slack and the
    motor bumps the released-brake stop, SG → 0 and motor halts.  Then we
    move forward by an operator-chosen offset and zero.
    """
    cprint("\n  [bold cyan]Stallguard touch-home[/bold cyan]")
    direction = ask_int("  Touch direction (-1 = brake-release, +1 = brake-engage)",
                        default=-1)
    if direction not in (-1, 1):
        cprint("  ✗  Must be -1 or +1.", "red"); return
    touch_vel  = ask_int("  Touch velocity (pps, low for sensitivity)", default=10_000)
    max_travel = ask_int("  Max travel before giving up (ticks)", default=200_000)
    offset     = ask_int("  Offset from touch point to home (ticks, signed)",
                         default=2_000)

    # Save state we'll modify so we can restore it.
    saved = {ap: get_ap(motor, ap)
             for ap in (AP_STOP_ON_STALL, 4)}  # 4 = MaxVelocity

    try:
        # Arm stop-on-stall just below touch_vel.  SG must already be tuned
        # well enough that a light bump triggers stall — for a sensitive touch
        # use a relatively high SGT (positive) and SG2FilterEnable=1.
        set_ap(motor, AP_STOP_ON_STALL, max(touch_vel - 1_000, 1))
        set_ap(motor, 4, touch_vel)  # cap top speed at touch_vel

        start = get_ap(motor, AP_ACTUAL_POSITION)
        target = start + direction * max_travel

        # Clear any latched StallGuard error before we begin.
        _ = get_ap(motor, AP_EXT_ERROR_FLAGS)

        cprint(f"  Touching… (start={start}, max target={target})", "cyan")
        motor.move_to(target)
        reached, stalled = wait_for_position(motor, timeout=MOVE_TIMEOUT,
                                             stall_aborts=True)
        if reached and not stalled:
            cprint("  ✗  Reached travel limit without stalling — increase max travel, "
                   "raise SGT (more sensitive), or check brake mechanics.", "red")
            return
        if not stalled:
            cprint("  ✗  Timed out without stall.", "red"); return

        touch_pos = get_ap(motor, AP_ACTUAL_POSITION)
        cprint(f"  Stalled at pos={touch_pos}, SG={read_sg(motor)}.", "green")

        # Move off the stop by `offset` and zero there.
        set_ap(motor, AP_STOP_ON_STALL, 0)  # disable while backing off
        home_pos = touch_pos + offset
        cprint(f"  Backing off to {home_pos}…", "cyan")
        motor.move_to(home_pos)
        wait_for_position(motor, timeout=MOVE_TIMEOUT)

        # Re-sync encoder to actual and zero both.
        set_ap(motor, AP_ACTUAL_POSITION, 0)
        set_ap(motor, AP_ENCODER_POSITION, 0)
        cprint("  ✓  Home set (pos=0, enc=0).", "green")
    finally:
        for ap, v in saved.items():
            set_ap(motor, ap, v)


def home_menu(motor) -> None:
    cprint("\n  [bold cyan]Home[/bold cyan]")
    cprint("  [m] manual zero (current physical position becomes home)")
    cprint("  [s] stallguard touch (drive into release stop, then offset)")
    choice = ask("  >", default="m").strip().lower()
    if   choice == "m": home_manual(motor)
    elif choice == "s": home_stallguard(motor)
    else: cprint(f"  ✗  Unknown: {choice!r}", "red")


# ─── Discover brake range ────────────────────────────────────────────────────

def discover_range(motor) -> None:
    """
    Drive slowly from current position toward the brake with stop-on-stall
    armed.  Reports the distance traveled when the brake's hard stop is
    reached.  Use ~90% of that as engage_dist for the brake test.

    Does not modify the home reference; returns to start when done.
    """
    cprint("\n  [bold cyan]Discover Brake Range[/bold cyan]")
    cprint(
        "  Drives slowly from your current position in the engage direction\n"
        "  until StallGuard fires, then reports the distance traveled.\n"
        "  Run this once after homing; use the reported number as engage_dist.",
        "dim",
    )

    direction = ask_int("  Engage direction (+1 = positive, -1 = negative)",
                        default=1)
    if direction not in (-1, 1):
        cprint("  ✗  Must be -1 or +1.", "red"); return

    velocity   = ask_int("  Discovery velocity (pps, keep low and steady)",
                         default=10_000)
    max_travel = ask_int("  Give-up distance (ticks)", default=300_000)

    # Sanity check: SG must be tuned, or this is useless.
    sgt    = get_ap(motor, AP_SG2_THRESHOLD)
    filt   = get_ap(motor, AP_SG2_FILTER_ENABLE)
    cprint(f"  SGT={sgt}  Filter={filt}  "
           f"(if you haven't tuned these yet, use [m]onitor first)", "dim")

    saved = {ap: get_ap(motor, ap) for ap in (4, AP_STOP_ON_STALL)}
    start = get_ap(motor, AP_ACTUAL_POSITION)

    try:
        set_ap(motor, 4, velocity)
        set_ap(motor, AP_STOP_ON_STALL, max(velocity - 1_000, 1))
        _ = get_ap(motor, AP_EXT_ERROR_FLAGS)  # clear latched flag

        target = start + direction * max_travel
        cprint(f"\n  Discovering… {start} → {target} @ {velocity} pps", "cyan")
        motor.move_to(target)

        deadline = time.time() + MOVE_TIMEOUT
        slack_end_pos: int | None = None
        sg_initial = read_sg(motor)
        result_distance: int | None = None

        while time.time() < deadline:
            if get_ap(motor, AP_EXT_ERROR_FLAGS) & 0x01:
                stall_pos = get_ap(motor, AP_ACTUAL_POSITION)
                result_distance = abs(stall_pos - start)
                cprint(f"\n  ✓  Brake hard-stop hit at pos={stall_pos}.", "green")
                cprint(f"      Distance from home:        {result_distance} ticks",
                       "bold green")
                cprint(f"      Recommended engage_dist:   "
                       f"{int(result_distance * 0.90)}  (90%)",
                       "bold green")
                if slack_end_pos is not None:
                    slack_dist = abs(slack_end_pos - start)
                    cprint(f"      Slack ended around:        {slack_dist} ticks "
                           f"({slack_dist * 100 // max(1, result_distance)}% in)",
                           "green")
                break
            if motor.get_position_reached():
                cprint(f"\n  ✗  Reached give-up distance ({max_travel} ticks) "
                       "without stalling.\n      Either the brake never engaged, "
                       "SGT is too low (insensitive),\n      or max travel is "
                       "too short.", "red")
                motor.stop()
                break

            sg = read_sg(motor)
            # Heuristic: first time SG drops well below its initial reading,
            # we've taken up the slack and are starting to load the cable.
            if slack_end_pos is None and sg_initial > 200 and sg < sg_initial * 0.6:
                slack_end_pos = get_ap(motor, AP_ACTUAL_POSITION)
            time.sleep(POLL_INTERVAL)
        else:
            cprint("\n  ✗  Discovery timed out.", "red")
            motor.stop()

        # Return to start, no zeroing.
        set_ap(motor, AP_STOP_ON_STALL, 0)
        cprint(f"  Returning to start pos={start}…", "cyan")
        motor.move_to(start)
        wait_for_position(motor, timeout=MOVE_TIMEOUT)

        if result_distance is not None:
            cprint(f"\n  → Set engage_dist ≈ {int(result_distance * 0.90)} "
                   "in the brake test menu.", "bold cyan")
    finally:
        for ap, v in saved.items():
            set_ap(motor, ap, v)


# ─── Brake test ───────────────────────────────────────────────────────────────

_PHASED_PARAMS = (4, 5, 17, 16, 15, 18)  # MaxVel, MaxAcc, MaxDec, V1, A1, D1


def brake_test(motor, log_path: str | None = None) -> None:
    cprint("\n  [bold cyan]Brake Test[/bold cyan]")
    use_home = ask_yes_no("  Treat current pos=0 as home? (no = re-home now)",
                          default=True)
    if not use_home:
        home_menu(motor)

    engage_dist = ask_int("  Engage distance (ticks from home to brake mech-stop)",
                          default=70_000)
    hold_dur    = ask_float("  Hold duration (s)", default=5.0)
    approach    = ask_int("  Phase-3 approach velocity (pps)", default=20_000)
    fast_mult   = ask_int("  Phase-2 fast multiplier (×approach)", default=3)
    hold_curr   = ask_int(
        "  Hold current (AP 7 standby override, 0..255; -1 = leave as configured)",
        default=240,
    )

    max_vel_now = get_ap(motor, 4)  # AP 4 = MaxVelocity
    if approach > max_vel_now:
        cprint(f"  ⚠  Approach ({approach}) > MaxVelocity ({max_vel_now}); "
               f"clamping to {max_vel_now}.", "yellow")
        approach = max_vel_now

    start_pos  = get_ap(motor, AP_ACTUAL_POSITION)
    target_pos = start_pos + engage_dist
    third      = engage_dist // 3
    wp1, wp2, wp3 = start_pos + third, start_pos + 2 * third, target_pos

    # Snapshot every parameter we modify (incl. StallGuard / current state).
    saved = {ap: get_ap(motor, ap) for ap in _PHASED_PARAMS}
    saved[AP_STOP_ON_STALL]  = get_ap(motor, AP_STOP_ON_STALL)
    saved[AP_STANDBY_CURRENT] = get_ap(motor, AP_STANDBY_CURRENT)

    def apply_slow() -> None:
        """Phase 1/3: configured ramp, but MaxVelocity capped to approach."""
        for ap in _PHASED_PARAMS:
            set_ap(motor, ap, saved[ap])
        set_ap(motor, 4, approach)  # MaxVelocity = approach

    def apply_fast() -> None:
        """Phase 2: everything scaled up."""
        set_ap(motor, 4,  approach * fast_mult)
        set_ap(motor, 5,  saved[5]  * fast_mult)  # MaxAccel
        set_ap(motor, 17, saved[17] * fast_mult)  # MaxDecel
        set_ap(motor, 16, saved[16] * 2)          # V1
        set_ap(motor, 15, saved[15] * 2)          # A1
        set_ap(motor, 18, saved[18] * 2)          # D1

    sgt = get_ap(motor, AP_SG2_THRESHOLD)
    cprint(f"\n  SGT={sgt}  Filter={get_ap(motor, AP_SG2_FILTER_ENABLE)}  "
           f"Stop-on-stall vel={get_ap(motor, AP_STOP_ON_STALL)}",
           "dim")
    cprint(f"  Engaging {start_pos} → {target_pos} in 3 phases…", "cyan")

    stall_pos: int | None = None
    timed_out = False

    try:
        # ── Phases 1 and 2: stop-on-stall DISABLED.  We don't want a false
        #    trip while ramping up or traversing free travel.
        set_ap(motor, AP_STOP_ON_STALL, 0)

        apply_slow()
        cprint(f"  Phase 1/3 (slow): {start_pos} → {wp1} @ {approach} pps", "cyan")
        motor.move_to(wp1)
        reached, _ = wait_for_position(motor, timeout=MOVE_TIMEOUT)
        if not reached:
            cprint("  ✗  Phase 1 timed out — aborting.", "red"); timed_out = True
            return

        apply_fast()
        cprint(f"  Phase 2/3 (fast): {wp1} → {wp2} @ {approach * fast_mult} pps", "cyan")
        motor.move_to(wp2)
        reached, _ = wait_for_position(motor, timeout=MOVE_TIMEOUT)
        if not reached:
            cprint("  ✗  Phase 2 timed out — aborting.", "red"); timed_out = True
            return

        # ── Phase 3: ARM stop-on-stall.  This is the point of the whole test:
        #    we expect the brake's mechanical limit to trigger SG→0 before
        #    we reach wp3.  When it does, the motor stops right there.
        apply_slow()
        # Arm at slightly below approach so it's hot the whole phase.
        set_ap(motor, AP_STOP_ON_STALL, max(approach - 1_000, 1))
        _ = get_ap(motor, AP_EXT_ERROR_FLAGS)  # clear latched flag

        cprint(f"  Phase 3/3 (slow + stall-watch): {wp2} → {wp3} @ {approach} pps  "
               f"[SG live]", "cyan")
        motor.move_to(wp3)

        # Watch SG while moving.  This loop also handles the stall stop.
        deadline = time.time() + MOVE_TIMEOUT
        sg_min = 1023
        while time.time() < deadline:
            if get_ap(motor, AP_EXT_ERROR_FLAGS) & 0x01:
                stall_pos = get_ap(motor, AP_ACTUAL_POSITION)
                cprint(f"  ⏹  Stall detected at pos={stall_pos} "
                       f"(planned target was {wp3}).  Brake at max.", "green")
                break
            if motor.get_position_reached():
                cprint("  ⚠  Reached planned target without stalling. "
                       "Either the brake didn't engage, or SGT is too low "
                       "(insensitive).  Holding at target anyway.",
                       "yellow")
                stall_pos = wp3
                break
            sg_min = min(sg_min, read_sg(motor))
            time.sleep(POLL_INTERVAL)
        else:
            cprint("  ✗  Phase 3 timed out without reach or stall.", "red")
            timed_out = True
            return

        cprint(f"  Phase 3 SG min during move: {sg_min}", "dim")

        # ── Hold.  Sync ActualPosition to where we ACTUALLY stopped (not
        #    where we asked to go), then issue a move_to current position
        #    so the driver maintains active position control.  Bump standby
        #    current up so the brake stays pressed.
        set_ap(motor, AP_STOP_ON_STALL, 0)  # disengage so we don't keep tripping
        actual_now = get_ap(motor, AP_ACTUAL_POSITION)
        set_ap(motor, AP_ACTUAL_POSITION, actual_now)  # idempotent, defensive
        motor.move_to(actual_now)

        if hold_curr >= 0:
            set_ap(motor, AP_STANDBY_CURRENT, min(255, max(0, hold_curr)))

        cprint(f"  Holding {hold_dur:.1f}s at pos={actual_now}  "
               f"(standby_current={get_ap(motor, AP_STANDBY_CURRENT)})", "cyan")

        t0 = time.time()
        sg_samples = []
        while time.time() - t0 < hold_dur:
            sg_samples.append(read_sg(motor))
            time.sleep(0.1)
        sg_avg = sum(sg_samples) / max(1, len(sg_samples))
        cprint(f"  Hold complete.  Mean SG during hold: {sg_avg:.0f}", "green")

    finally:
        # Restore everything we touched.
        for ap, v in saved.items():
            set_ap(motor, ap, v)

    if timed_out:
        return

    # ── Return to home (= 0 by our convention) ──────────────────────────────
    cprint("  Returning to home (pos=0)…", "cyan")
    motor.move_to(0)
    reached, _ = wait_for_position(motor, timeout=MOVE_TIMEOUT)
    if not reached:
        cprint("  ✗  Return move timed out.", "red")
    else:
        cprint("  ✓  Brake test done.", "green")

    # ── Log ─────────────────────────────────────────────────────────────────
    if log_path:
        row = {
            "timestamp":       datetime.now().isoformat(timespec="seconds"),
            "engage_dist":     engage_dist,
            "hold_duration":   hold_dur,
            "approach_pps":    approach,
            "fast_mult":       fast_mult,
            "hold_current":    hold_curr,
            "stall_pos":       stall_pos if stall_pos is not None else "",
            "overshoot":       (stall_pos - target_pos) if stall_pos is not None else "",
            "mean_sg_hold":    round(sg_avg, 1) if sg_samples else "",
            **snapshot_params(motor),
        }
        write_header = not Path(log_path).exists()
        with open(log_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=row.keys())
            if write_header: w.writeheader()
            w.writerow(row)
        cprint(f"  ✓  Results appended to {log_path}", "dim green")


# ─── Live SG monitor (handy for SGT tuning) ──────────────────────────────────

def sg_monitor(motor) -> None:
    """Print SG / pos every 100 ms until Ctrl-C.  Useful while you hand-load
    the brake to find the right SGT."""
    cprint("\n  [bold cyan]SG monitor[/bold cyan] — Ctrl-C to stop.", "cyan")
    try:
        while True:
            sg  = read_sg(motor)
            pos = get_ap(motor, AP_ACTUAL_POSITION)
            spd = get_ap(motor, 3)  # AP 3 = actual speed
            bar = "█" * int(sg / 1023 * 40)
            print(f"\r  SG={sg:4d} {bar:<40}  pos={pos:>10}  v={spd:>7}  ",
                  end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


# ─── Menu ─────────────────────────────────────────────────────────────────────

MENU = [
    ("e", ("edit",),          "Edit a parameter"),
    ("v", ("view",),          "View all parameters"),
    ("h", ("home",),          "Home the axis"),
    ("d", ("discover",),      "Discover brake range (find engage_dist)"),
    ("b", ("brake", "test"),  "Run brake test"),
    ("m", ("monitor", "sg"),  "Live SG monitor (for tuning SGT)"),
    ("s", ("save",),          "Save parameters to file"),
    ("l", ("load",),          "Load parameters from file"),
    ("q", ("quit", "exit"),   "Quit"),
]

_DISPATCH: dict[str, str] = {}
for _pk, _aliases, _ in MENU:
    _DISPATCH[_pk] = _pk
    for _a in _aliases:
        _DISPATCH[_a] = _pk


def print_menu() -> None:
    if HAS_RICH:
        from rich.text import Text
        body = Text()
        for i, (pk, aliases, desc) in enumerate(MENU):
            row = Text()
            row.append(f"  [{pk}]", style="bold cyan")
            row.append(f"  {desc}")
            if aliases:
                row.append(f"  ({', '.join(aliases)})", style="dim")
            body.append_text(row)
            if i < len(MENU) - 1:
                body.append("\n")
        console.print(Panel(body, title="[bold cyan]Brake Tuner — TMCM-1260[/bold cyan]",
                            border_style="cyan", expand=False))
    else:
        print("\n" + "═" * 50)
        print("  Brake Tuner — TMCM-1260")
        print("═" * 50)
        for pk, aliases, desc in MENU:
            hint = f"  ({', '.join(aliases)})" if aliases else ""
            print(f"  [{pk}]  {desc}{hint}")
        print("═" * 50)


def run(motor, log_path: str | None = None) -> None:
    while True:
        print_menu()
        choice = ask("  >").strip().lower()
        action = _DISPATCH.get(choice)
        if   action == "e": edit_param(motor)
        elif action == "v": show_params(motor)
        elif action == "h": home_menu(motor)
        elif action == "d": discover_range(motor)
        elif action == "b": brake_test(motor, log_path=log_path)
        elif action == "m": sg_monitor(motor)
        elif action == "s": save_params(motor)
        elif action == "l": load_params(motor)
        elif action == "q":
            cprint("\n  Bye.\n", "dim"); break
        else:
            cprint(f"  ✗  Unknown: '{choice}'", "red")


# ─── Entry point ──────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser(description="TMCM-1260 brake tuner (StallGuard-aware)")
    p.add_argument("--interface", default="can", choices=["can", "usb"])
    p.add_argument("--port",      default=None)
    p.add_argument("--id",        type=int, default=1, dest="module_id")
    p.add_argument("--data-rate", type=int, default=1_000_000)
    p.add_argument("--log",       default=None, metavar="CSV",
                   help="CSV file to append brake-test results to")
    args = p.parse_args()

    port = args.port or ("can1" if args.interface == "can" else "/dev/ttyACM0")
    conn_str = (
        f"--interface socketcan_tmcl --port {port} --data-rate {args.data_rate}"
        if args.interface == "can"
        else f"--interface usb_tmcl --port {port}"
    )

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
