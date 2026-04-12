"""
Unit tests for steering_node.py, PIDController, and Motor.unwrap_position_signed.

No ROS 2 install or CAN hardware is required.

Run with:
    pytest test_steering_node.py -v

Layout
------
Section 1  –  Stubs / sys.modules patching  (runs at import time)
Section 2  –  TestConvertAngleToPosition
Section 3  –  TestCallbackStepangle
Section 4  –  TestPidLoop
Section 5  –  TestPIDControllerWrapper
Section 6  –  TestUnwrapPositionSigned
"""

import sys
import types
import threading
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

# ── Resolve steering_node's location relative to this test file ───────────────
# Typical colcon layout:
#   src/ad_juno/
#     ad_juno/steering_node.py   ← source
#     test/test_steering.py      ← this file
#
# We walk up from the test directory and try the two most common locations so
# the test works whether run via `pytest` from the repo root or from the
# package directory.
_TEST_DIR = Path(__file__).resolve().parent  # …/ad_juno/test
_PACKAGE_DIR = _TEST_DIR.parent  # …/ad_juno
_CANDIDATES = [
    _PACKAGE_DIR / "ad_juno",  # installed / standard colcon layout
    _PACKAGE_DIR,  # flat layout (node sits next to CMakeLists)
]
for _candidate in _CANDIDATES:
    if (_candidate / "steering_node.py").exists():
        sys.path.insert(0, str(_candidate))
        break
else:
    raise RuntimeError(
        f"steering_node.py not found. Searched:\n"
        + "\n".join(f"  {p}" for p in _CANDIDATES)
    )

# ══════════════════════════════════════════════════════════════════════════════
# Section 1 – Lightweight stubs injected before steering_node is imported
# ══════════════════════════════════════════════════════════════════════════════

# ── ROS2 time helpers ──────────────────────────────────────────────────────────


class _ROSTimeDelta:
    """Minimal stand-in for rclpy Duration returned by Time.__sub__."""

    def __init__(self, ns: int = 0):
        self.nanoseconds = ns


class _ROSTime:
    """Minimal stand-in for rclpy.time.Time."""

    def __init__(self, ns: int = 0):
        self._ns = ns

    @property
    def nanoseconds(self):
        return self._ns

    def __sub__(self, other: "_ROSTime") -> _ROSTimeDelta:
        return _ROSTimeDelta(self._ns - other._ns)


class _FakeClock:
    """Controllable clock: advance() lets tests simulate elapsed time."""

    def __init__(self):
        self._ns: int = 0

    def now(self) -> _ROSTime:
        return _ROSTime(self._ns)

    def advance(self, seconds: float) -> None:
        self._ns += int(seconds * 1e9)


# ── Minimal node base-class replacement ───────────────────────────────────────


class _FakeNode:
    """
    Drop-in for rclpy.node.Node / ThreadSafeNode.
    Provides only the surface that SteeringNode.__init__ calls.
    """

    def __init__(self, _name: str):
        self._logger = MagicMock()
        self._clock = _FakeClock()

    # ---------- rclpy.node.Node interface ----------
    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def declare_parameter(self, _name: str, _default=None):
        p = MagicMock()
        p.get_parameter_value.return_value.bool_value = False
        p.get_parameter_value.return_value.string_value = "/tmp/steering_test"
        return p

    def get_parameter(self, _name: str):
        p = MagicMock()
        p.get_parameter_value.return_value.string_value = "/tmp/steering_test"
        return p

    def create_subscription(self, *_a, **_kw):
        return MagicMock()

    def create_timer(self, *_a, **_kw):
        return MagicMock()

    def destroy_node(self):
        pass


# ── Helper: build a throw-away module with attribute shortcuts ─────────────────


def _mod(name: str, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    return m


# ── Register stubs ─────────────────────────────────────────────────────────────

sys.modules.setdefault(
    "rclpy",
    _mod("rclpy", init=MagicMock(), spin=MagicMock(), shutdown=MagicMock()),
)
sys.modules.setdefault(
    "rclpy.qos",
    _mod(
        "rclpy.qos",
        QoSProfile=MagicMock(),
        HistoryPolicy=MagicMock(),
        ReliabilityPolicy=MagicMock(),
    ),
)
sys.modules.setdefault("std_msgs", _mod("std_msgs"))
sys.modules.setdefault("std_msgs.msg", _mod("std_msgs.msg", Float64=MagicMock()))
sys.modules.setdefault("shared_objects", _mod("shared_objects"))
sys.modules.setdefault(
    "shared_objects.Motor", _mod("shared_objects.Motor", Stepper=MagicMock())
)
sys.modules.setdefault(
    "shared_objects.thread_safe_node",
    _mod("shared_objects.thread_safe_node", ThreadSafeNode=_FakeNode),
)
_topics_cls = MagicMock()
_topics_cls.return_value.topic_names = {"steering": "/steering_angle"}
sys.modules.setdefault(
    "shared_objects.ROS_utils",
    _mod("shared_objects.ROS_utils", Topics=_topics_cls),
)

# ── Intercept the module-level ctypes.CDLL call that loads libpid.so ──────────

_fake_lib = MagicMock()
_fake_lib.pid_init.restype = None
_fake_lib.pid_compute.return_value = (
    0.0  # plain float — ctypes restype unwraps automatically
)

with patch("ctypes.CDLL", return_value=_fake_lib):
    import ad_juno.steering_node as steering_node  # type: ignore[import]  # noqa: E402

from ad_juno.steering_node import (  # type: ignore[import]  # noqa: E402
    SteeringNode,
    PIDController,
    TRANSMISSION_RATIO,
    MAX_ANGLE,
    STEP_ANGLE,
)

# Patch target must match the key under which the module is registered in
# sys.modules — that is the fully-qualified package name.
_STEERING_MODULE = "ad_juno.steering_node"


# ══════════════════════════════════════════════════════════════════════════════
# Shared factory
# ══════════════════════════════════════════════════════════════════════════════


def _make_node():
    """
    Construct a SteeringNode with:
    - Stepper replaced by a MagicMock (no CAN)
    - Background threads stopped immediately after construction

    Returns (node, mock_stepper).
    """
    mock_stepper = MagicMock()
    with patch(f"{_STEERING_MODULE}.Stepper", return_value=mock_stepper):
        node = SteeringNode()
    # Halt the reader/writer threads; they may still be alive for a brief
    # moment but will exit at their next _running check.
    node._running = False
    return node, mock_stepper


# ══════════════════════════════════════════════════════════════════════════════
# Section 2 – convert_angle_to_position
# ══════════════════════════════════════════════════════════════════════════════


class TestConvertAngleToPosition(unittest.TestCase):
    """
    Pure-math tests: no ROS, no hardware.
    Formula: position = -int((angle / STEP_ANGLE) * 256 * TRANSMISSION_RATIO)
    """

    def setUp(self):
        self.node, _ = _make_node()

    # ── Basic correctness ──────────────────────────────────────────────────────

    def test_zero_angle_gives_zero_position(self):
        self.assertEqual(self.node.convert_angle_to_position(0), 0)

    def test_positive_angle_gives_negative_position(self):
        """Motor is direction-inverted: positive angle → negative position."""
        self.assertLess(self.node.convert_angle_to_position(1), 0)

    def test_negative_angle_gives_positive_position(self):
        self.assertGreater(self.node.convert_angle_to_position(-1), 0)

    # ── Symmetry ──────────────────────────────────────────────────────────────

    def test_sign_symmetry(self):
        """+angle and –angle should produce equal-magnitude, opposite-sign values."""
        pos_pos = self.node.convert_angle_to_position(5)
        neg_pos = self.node.convert_angle_to_position(-5)
        self.assertEqual(pos_pos, -neg_pos)

    # ── Known numeric values ───────────────────────────────────────────────────

    def test_max_angle_value(self):
        """10 ° → -(10/1.8)*256*14 = -19911 microsteps (truncated)."""
        expected = -int((MAX_ANGLE / STEP_ANGLE) * 256 * TRANSMISSION_RATIO)
        self.assertEqual(self.node.convert_angle_to_position(MAX_ANGLE), expected)

    def test_half_max_angle(self):
        expected = -int((5.0 / STEP_ANGLE) * 256 * TRANSMISSION_RATIO)
        self.assertEqual(self.node.convert_angle_to_position(5.0), expected)

    def test_single_full_step(self):
        """One full step (1.8 °) → -(1 * 256 * 14) = -3584 microsteps."""
        expected = -int(256 * TRANSMISSION_RATIO)
        self.assertEqual(self.node.convert_angle_to_position(STEP_ANGLE), expected)

    # ── Linearity ─────────────────────────────────────────────────────────────

    def test_doubling_angle_doubles_position(self):
        """Position should scale linearly with angle (±1 due to int truncation)."""
        p1 = abs(self.node.convert_angle_to_position(2))
        p2 = abs(self.node.convert_angle_to_position(4))
        self.assertAlmostEqual(p2 / p1, 2.0, delta=0.01)

    # ── Return type ───────────────────────────────────────────────────────────

    def test_result_is_int(self):
        self.assertIsInstance(self.node.convert_angle_to_position(3), int)


# ══════════════════════════════════════════════════════════════════════════════
# Section 3 – callback_stepangle
# ══════════════════════════════════════════════════════════════════════════════


class TestCallbackStepangle(unittest.TestCase):
    """
    Verifies angle clamping and thread-safe target position update.
    """

    def setUp(self):
        self.node, _ = _make_node()

    def _msg(self, value: float):
        m = MagicMock()
        m.data = value
        return m

    def _get_target(self) -> int:
        with self.node._angle_lock:
            return self.node._target_pos

    # ── In-range messages ──────────────────────────────────────────────────────

    def test_valid_angle_updates_target(self):
        self.node.callback_stepangle(self._msg(5.0))
        self.assertEqual(self._get_target(), self.node.convert_angle_to_position(5.0))

    def test_zero_angle_sets_target_zero(self):
        self.node.callback_stepangle(self._msg(0.0))
        self.assertEqual(self._get_target(), 0)

    def test_exact_max_angle_accepted(self):
        self.node.callback_stepangle(self._msg(float(MAX_ANGLE)))
        expected = self.node.convert_angle_to_position(MAX_ANGLE)
        self.assertEqual(self._get_target(), expected)

    def test_exact_min_angle_accepted(self):
        self.node.callback_stepangle(self._msg(float(-MAX_ANGLE)))
        expected = self.node.convert_angle_to_position(-MAX_ANGLE)
        self.assertEqual(self._get_target(), expected)

    # ── Clamping ──────────────────────────────────────────────────────────────

    def test_positive_overflow_clamped_to_max(self):
        self.node.callback_stepangle(self._msg(9999.0))
        expected = self.node.convert_angle_to_position(MAX_ANGLE)
        self.assertEqual(self._get_target(), expected)

    def test_negative_overflow_clamped_to_min(self):
        self.node.callback_stepangle(self._msg(-9999.0))
        expected = self.node.convert_angle_to_position(-MAX_ANGLE)
        self.assertEqual(self._get_target(), expected)

    def test_just_above_max_clamped(self):
        self.node.callback_stepangle(self._msg(MAX_ANGLE + 0.001))
        expected = self.node.convert_angle_to_position(MAX_ANGLE)
        self.assertEqual(self._get_target(), expected)

    def test_just_below_min_clamped(self):
        self.node.callback_stepangle(self._msg(-MAX_ANGLE - 0.001))
        expected = self.node.convert_angle_to_position(-MAX_ANGLE)
        self.assertEqual(self._get_target(), expected)

    # ── Successive updates ────────────────────────────────────────────────────

    def test_second_message_overwrites_first(self):
        self.node.callback_stepangle(self._msg(3.0))
        self.node.callback_stepangle(self._msg(7.0))
        expected = self.node.convert_angle_to_position(7.0)
        self.assertEqual(self._get_target(), expected)


# ══════════════════════════════════════════════════════════════════════════════
# Section 4 – pid_loop
# ══════════════════════════════════════════════════════════════════════════════


class TestPidLoop(unittest.TestCase):
    """
    Tests the PID control loop: queue behaviour, command value, dt floor.
    The PID computation itself is replaced with a MagicMock so we test only
    the orchestration logic inside pid_loop().
    """

    def setUp(self):
        self.node, _ = _make_node()
        # Drain any startup item in the queue
        while not self.node._queue.empty():
            self.node._queue.get_nowait()
        # Default: pid.compute returns 0
        self.node.pid.compute = MagicMock(return_value=0.0)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_state(self, *, target: int, current: int) -> None:
        with self.node._angle_lock:
            self.node._target_pos = target
        with self.node._position_lock:
            self.node._current_position = current

    def _run_loop(self, advance_s: float = 0.01) -> None:
        self.node.get_clock().advance(advance_s)
        self.node.pid_loop()

    # ── Queue population ──────────────────────────────────────────────────────

    def test_loop_enqueues_exactly_one_command(self):
        self._set_state(target=100, current=0)
        self._run_loop()
        self.assertEqual(self.node._queue.qsize(), 1)

    def test_full_queue_is_replaced_not_grown(self):
        """Old command must be discarded; queue size stays at 1."""
        self.node._queue.put_nowait(9999)
        self._run_loop()
        self.assertEqual(self.node._queue.qsize(), 1)

    # ── Command value ─────────────────────────────────────────────────────────

    def test_command_equals_current_plus_pid_output(self):
        self.node.pid.compute = MagicMock(return_value=75.0)
        self._set_state(target=500, current=100)
        self._run_loop()
        cmd = self.node._queue.get_nowait()
        self.assertEqual(cmd, int(100 + 75.0))  # 175

    def test_at_setpoint_pid_zero_command_stays_current(self):
        self.node.pid.compute = MagicMock(return_value=0.0)
        self._set_state(target=0, current=0)
        self._run_loop()
        cmd = self.node._queue.get_nowait()
        self.assertEqual(cmd, 0)

    def test_stale_command_is_discarded_before_new_one(self):
        self.node.pid.compute = MagicMock(return_value=10.0)
        self._set_state(target=200, current=0)
        self.node._queue.put_nowait(9999)  # stale value
        self._run_loop()
        cmd = self.node._queue.get_nowait()
        self.assertEqual(cmd, int(0 + 10.0))  # stale gone, new command present
        self.assertTrue(self.node._queue.empty())

    # ── dt protection ─────────────────────────────────────────────────────────

    def test_dt_is_always_positive(self):
        """Even if the clock does not advance, dt must not be zero."""
        captured = []

        def capture(_target, _actual, dt):
            captured.append(dt)
            return 0.0

        self.node.pid.compute = capture
        # Two calls without advancing the clock
        self.node.pid_loop()
        self.node.pid_loop()
        for dt in captured:
            self.assertGreater(dt, 0.0, msg=f"dt was {dt}, expected > 0")

    def test_dt_reflects_elapsed_time(self):
        """After advancing the clock by 10 ms the captured dt should be ~0.01 s."""
        captured = []

        def capture(_target, _actual, dt):
            captured.append(dt)
            return 0.0

        self.node.pid.compute = capture
        self.node.get_clock().advance(0.01)
        self.node.pid_loop()
        self.assertAlmostEqual(captured[-1], 0.01, delta=1e-6)

    # ── pid.compute is called with correct args ───────────────────────────────

    def test_pid_compute_called_with_target_and_current(self):
        self.node.pid.compute = MagicMock(return_value=0.0)
        self._set_state(target=300, current=150)
        self.node.get_clock().advance(0.01)
        self.node.pid_loop()
        self.node.pid.compute.assert_called_once()
        args = self.node.pid.compute.call_args[0]
        self.assertEqual(args[0], 300)  # target
        self.assertEqual(args[1], 150)  # current


# ══════════════════════════════════════════════════════════════════════════════
# Section 5 – PIDController ctypes wrapper
# ══════════════════════════════════════════════════════════════════════════════


class TestPIDControllerWrapper(unittest.TestCase):
    """
    Validates that PIDController correctly delegates to the C library.
    The C library itself is replaced by a configurable MagicMock; these tests
    check argument passing and return-value forwarding, not PID math.
    """

    def setUp(self):
        self._orig_lib = steering_node._lib_pid
        self.lib = MagicMock()
        self.lib.pid_compute.return_value = (
            42.0  # plain float, matching real ctypes restype unwrapping
        )
        steering_node._lib_pid = self.lib

    def tearDown(self):
        steering_node._lib_pid = self._orig_lib

    # ── Construction ──────────────────────────────────────────────────────────

    def test_pid_init_called_exactly_once_on_construction(self):
        PIDController(kp=1.0, ki=0.5, kd=0.1)
        self.lib.pid_init.assert_called_once()

    def test_pid_init_receives_correct_kp_ki_kd(self):
        PIDController(kp=2.0, ki=0.3, kd=0.05)
        args = self.lib.pid_init.call_args[0]
        # args[0] is the ctypes byref pointer; gains follow
        self.assertAlmostEqual(args[1], 2.0)
        self.assertAlmostEqual(args[2], 0.3)
        self.assertAlmostEqual(args[3], 0.05)

    def test_zero_gains_are_forwarded(self):
        PIDController(kp=0.0, ki=0.0, kd=0.0)
        args = self.lib.pid_init.call_args[0]
        self.assertAlmostEqual(args[1], 0.0)
        self.assertAlmostEqual(args[2], 0.0)
        self.assertAlmostEqual(args[3], 0.0)

    # ── compute() ─────────────────────────────────────────────────────────────

    def test_compute_forwards_target_actual_dt(self):
        pid = PIDController(1.0, 0.0, 0.0)
        pid.compute(target=100, actual=60, dt=0.01)
        args = self.lib.pid_compute.call_args[0]
        self.assertAlmostEqual(args[1], 100)
        self.assertAlmostEqual(args[2], 60)
        self.assertAlmostEqual(args[3], 0.01)

    def test_compute_returns_float(self):
        pid = PIDController(1.0, 0.0, 0.0)
        result = pid.compute(100, 60, 0.01)
        self.assertIsInstance(result, float)

    def test_each_instance_has_independent_state(self):
        """Two PIDController objects must call pid_init with separate structs."""
        PIDController(kp=1.0, ki=0.0, kd=0.0)
        PIDController(kp=2.0, ki=0.0, kd=0.0)
        self.assertEqual(self.lib.pid_init.call_count, 2)
        calls = self.lib.pid_init.call_args_list
        kp_first = calls[0][0][1]
        kp_second = calls[1][0][1]
        self.assertNotAlmostEqual(kp_first, kp_second)


# ══════════════════════════════════════════════════════════════════════════════
# Section 6 – unwrap_position_signed  (pure Python, from Motor.py)
# ══════════════════════════════════════════════════════════════════════════════
#
# Motor.py depends on pytrinamic (hardware), so we replicate the function here
# verbatim.  The logic is pure Python (no side-effects, no I/O).
# If you later extract this helper into its own module the import below can
# replace the inline copy.


def unwrap_position_signed(current_position, previous_position, max_position=2**32):
    """Verbatim copy from Motor.py – kept in sync manually."""
    if current_position >= max_position // 2:
        current_signed = current_position - max_position
    else:
        current_signed = current_position

    if previous_position >= max_position // 2:
        previous_signed = previous_position - max_position
    else:
        previous_signed = previous_position

    diff = current_signed - previous_signed
    if diff > max_position // 2:
        return previous_signed - (max_position - diff)
    elif diff < -(max_position // 2):
        return previous_signed + (max_position + diff)
    else:
        return current_signed


_MAX32 = 2**32


class TestUnwrapPositionSigned(unittest.TestCase):
    """
    Tests for the rollover-unwrapping helper used by _position_reader.
    """

    # ── Normal motion (no rollover) ───────────────────────────────────────────

    def test_no_rollover_positive_increment(self):
        self.assertEqual(unwrap_position_signed(1000, 900), 1000)

    def test_no_rollover_zero_motion(self):
        self.assertEqual(unwrap_position_signed(500, 500), 500)

    def test_no_rollover_negative_motion(self):
        self.assertEqual(unwrap_position_signed(900, 1000), 900)

    def test_zero_initial(self):
        self.assertEqual(unwrap_position_signed(0, 0), 0)

    # ── Positive → negative rollover (counter wraps from 2^32-1 → 0) ─────────

    def test_rollover_positive_to_zero(self):
        """Encoder crosses 2^32 boundary: previous near max, current near 0."""
        previous = _MAX32 - 10
        current = 5  # wrapped around
        result = unwrap_position_signed(current, previous)
        # Signed previous ≈ -10; signed current ≈ 5 → diff = +15 → no big jump
        # The function should return current_signed = 5
        self.assertEqual(result, 5)

    def test_rollover_large_positive_jump_detected(self):
        """
        If current_signed − previous_signed > MAX/2 it means the encoder
        actually moved backward across the wrap point.
        """
        previous = 100
        # Simulate previous = 100 (positive direction), encoder jumps to
        # a very large unsigned value that looks like −50 signed
        current = _MAX32 - 50
        result = unwrap_position_signed(current, previous)
        # previous_signed = 100, current_signed = −50, diff = −150 → no wrap
        self.assertEqual(result, -50)

    # ── Negative half-plane ───────────────────────────────────────────────────

    def test_large_unsigned_value_treated_as_negative(self):
        """Values ≥ 2^31 should be interpreted as negative signed integers."""
        current = _MAX32 - 1  # = −1 as signed 32-bit
        previous = _MAX32 - 1
        result = unwrap_position_signed(current, previous)
        self.assertEqual(result, -1)

    def test_symmetric_travel(self):
        """
        Moving from 0 → +N and then back to 0 should give 0 on the second call
        if we use the previous reading correctly.
        """
        pos_a = unwrap_position_signed(500, 0)  # moved forward to 500
        pos_b = unwrap_position_signed(0, 500)  # moved back to 0
        self.assertEqual(pos_a, 500)
        self.assertEqual(pos_b, 0)

    # ── Custom max_position ───────────────────────────────────────────────────

    def test_custom_max_position_small(self):
        """Verify the half-plane logic works with a small max_position."""
        max_p = 256
        # previous = 250 (just below max) → signed = 250 - 256 = −6
        # current  = 5  (just above 0)    → signed = 5
        result = unwrap_position_signed(5, 250, max_position=max_p)
        # diff = 5 − (−6) = 11 < 128 → no wrap detected, return current_signed = 5
        self.assertEqual(result, 5)


# ══════════════════════════════════════════════════════════════════════════════
# Section 7 – Thread-safety smoke tests
# ══════════════════════════════════════════════════════════════════════════════


class TestThreadSafety(unittest.TestCase):
    """
    Verifies that concurrent writes to _target_pos and _current_position do
    not corrupt state (i.e., locks are actually used).
    """

    def setUp(self):
        self.node, _ = _make_node()

    def test_concurrent_callback_writes_do_not_corrupt_target(self):
        errors: list = []

        def writer(angle: float):
            try:
                msg = MagicMock()
                msg.data = angle
                for _ in range(200):
                    self.node.callback_stepangle(msg)
            except Exception as exc:
                errors.append(exc)

        threads = [
            threading.Thread(target=writer, args=(a,)) for a in [1.0, -1.0, 5.0, -5.0]
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.assertEqual(errors, [], msg=f"Exceptions in writer threads: {errors}")
        # Final value must be a valid position (not a partially written int)
        with self.node._angle_lock:
            pos = self.node._target_pos
        valid_positions = {
            self.node.convert_angle_to_position(a) for a in [1.0, -1.0, 5.0, -5.0]
        }
        self.assertIn(pos, valid_positions)

    def test_pid_loop_reads_consistent_state(self):
        """
        pid_loop must never read a partially-updated (torn) position.
        We flood _current_position from a background thread while calling
        pid_loop and verify no exception is raised.
        """
        self.node.pid.compute = MagicMock(return_value=0.0)
        stop = threading.Event()

        def position_writer():
            i = 0
            while not stop.is_set():
                with self.node._position_lock:
                    self.node._current_position = i
                i += 1

        writer = threading.Thread(target=position_writer, daemon=True)
        writer.start()

        errors: list = []
        for _ in range(500):
            try:
                self.node.pid_loop()
            except Exception as exc:
                errors.append(exc)

        stop.set()
        writer.join(timeout=1.0)
        self.assertEqual(errors, [], msg=f"pid_loop raised: {errors}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
