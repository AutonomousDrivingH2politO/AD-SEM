#!/usr/bin/env python3
"""
Unit tests for ThreadSafeNode and its inner _TSLogger class.

All ROS2 / rclpy machinery is mocked so the test suite runs without a live
ROS2 installation or a running ROS2 context.
"""

import sys
import threading
import time
import unittest
from unittest.mock import MagicMock, call


# ---------------------------------------------------------------------------
# Patch rclpy and rclpy.node BEFORE importing the module under test so that
# the import itself never tries to touch a real ROS2 runtime.
# ---------------------------------------------------------------------------

_mock_rclpy = MagicMock()
_mock_node_module = MagicMock()


class _FakeNode:
    """Minimal stand-in for rclpy.node.Node."""

    def __init__(self, *args, **kwargs):
        self._fake_logger = MagicMock()

    def get_logger(self):
        return self._fake_logger


_mock_node_module.Node = _FakeNode
_mock_rclpy.node = _mock_node_module

sys.modules.setdefault("rclpy", _mock_rclpy)
sys.modules.setdefault("rclpy.node", _mock_node_module)

from shared_objects.thread_safe_node import ThreadSafeNode  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_node(name: str = "test_node") -> ThreadSafeNode:
    """Return a fresh ThreadSafeNode with its underlying mock logger."""
    return ThreadSafeNode(name)


def _ros_logger(node: ThreadSafeNode) -> MagicMock:
    """
    Extract the underlying mock ROS logger from a node.
    Accesses it through the public _TSLogger._logger attribute, which
    Pyright can resolve without needing to know about _FakeNode internals.
    """
    return node.get_logger()._logger  # type: ignore[return-value]


# ---------------------------------------------------------------------------
# Tests: _TSLogger basic delegation
# ---------------------------------------------------------------------------


class TestTSLoggerDelegation(unittest.TestCase):
    """_TSLogger must forward every level to the underlying ROS logger."""

    def setUp(self):
        self.node = _make_node()
        self.ts_logger = self.node.get_logger()
        self.ros_logger = _ros_logger(self.node)

    def test_debug_is_forwarded(self):
        self.ts_logger.debug("dbg")
        self.ros_logger.debug.assert_called_once_with("dbg")

    def test_info_is_forwarded(self):
        self.ts_logger.info("hello")
        self.ros_logger.info.assert_called_once_with("hello")

    def test_warn_is_forwarded(self):
        self.ts_logger.warn("careful")
        self.ros_logger.warn.assert_called_once_with("careful")

    def test_error_is_forwarded(self):
        self.ts_logger.error("oops")
        self.ros_logger.error.assert_called_once_with("oops")

    def test_fatal_is_forwarded(self):
        self.ts_logger.fatal("dead")
        self.ros_logger.fatal.assert_called_once_with("dead")

    def test_kwargs_are_forwarded(self):
        """Extra keyword arguments reach the underlying logger unchanged."""
        self.ts_logger.info("msg", throttle_duration_sec=1.0)
        self.ros_logger.info.assert_called_once_with("msg", throttle_duration_sec=1.0)

    def test_multiple_calls_are_all_forwarded(self):
        self.ts_logger.info("first")
        self.ts_logger.info("second")
        self.ts_logger.info("third")
        self.assertEqual(self.ros_logger.info.call_count, 3)
        self.ros_logger.info.assert_has_calls(
            [call("first"), call("second"), call("third")]
        )


# ---------------------------------------------------------------------------
# Tests: shutdown silences the logger
# ---------------------------------------------------------------------------


class TestTSLoggerShutdown(unittest.TestCase):
    """After shutdown(), log calls must be silently dropped."""

    def setUp(self):
        self.node = _make_node()
        self.ts_logger = self.node.get_logger()
        self.ros_logger = _ros_logger(self.node)

    def test_no_calls_after_shutdown(self):
        self.ts_logger.shutdown()
        self.ts_logger.debug("ignored")
        self.ts_logger.info("ignored")
        self.ts_logger.warn("ignored")
        self.ts_logger.error("ignored")
        self.ts_logger.fatal("ignored")
        self.ros_logger.debug.assert_not_called()
        self.ros_logger.info.assert_not_called()
        self.ros_logger.warn.assert_not_called()
        self.ros_logger.error.assert_not_called()
        self.ros_logger.fatal.assert_not_called()

    def test_calls_before_shutdown_are_forwarded(self):
        self.ts_logger.info("before")
        self.ts_logger.shutdown()
        self.ts_logger.info("after")
        self.ros_logger.info.assert_called_once_with("before")

    def test_double_shutdown_is_safe(self):
        """Calling shutdown twice must not raise."""
        self.ts_logger.shutdown()
        self.ts_logger.shutdown()

    def test_alive_flag_is_false_after_shutdown(self):
        self.assertTrue(self.ts_logger._alive)
        self.ts_logger.shutdown()
        self.assertFalse(self.ts_logger._alive)


# ---------------------------------------------------------------------------
# Tests: get_logger() returns the _TSLogger wrapper
# ---------------------------------------------------------------------------


class TestGetLogger(unittest.TestCase):
    def test_get_logger_returns_ts_logger(self):
        node = _make_node()
        self.assertIsInstance(node.get_logger(), ThreadSafeNode._TSLogger)

    def test_get_logger_is_same_instance_on_repeated_calls(self):
        node = _make_node()
        self.assertIs(node.get_logger(), node.get_logger())

    def test_ts_logger_wraps_ros_logger(self):
        node = _make_node()
        ts = node.get_logger()
        self.assertIsNotNone(ts._logger)


# ---------------------------------------------------------------------------
# Tests: thread safety
# ---------------------------------------------------------------------------


class TestTSLoggerThreadSafety(unittest.TestCase):
    THREADS = 20
    CALLS_PER_THREAD = 50

    def setUp(self):
        self.node = _make_node()
        self.ts_logger = self.node.get_logger()
        self.ros_logger = _ros_logger(self.node)

    def _spam_logger(self, barrier: threading.Barrier) -> None:
        barrier.wait()
        for i in range(self.CALLS_PER_THREAD):
            self.ts_logger.info(f"msg {i}")

    def test_concurrent_logging_does_not_raise(self):
        barrier = threading.Barrier(self.THREADS)
        threads = [
            threading.Thread(target=self._spam_logger, args=(barrier,))
            for _ in range(self.THREADS)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)

        expected = self.THREADS * self.CALLS_PER_THREAD
        self.assertEqual(self.ros_logger.info.call_count, expected)

    def test_shutdown_during_concurrent_logging_does_not_raise(self):
        """
        Shutdown fired mid-flight: no exception must escape any thread,
        and the total call count must not exceed the theoretical maximum.
        """
        errors: list[Exception] = []
        barrier = threading.Barrier(self.THREADS + 1)  # +1 for the killer thread

        def worker() -> None:
            barrier.wait()
            try:
                for _ in range(self.CALLS_PER_THREAD):
                    self.ts_logger.info("concurrent")
                    time.sleep(0)
            except Exception as exc:
                errors.append(exc)

        def killer() -> None:
            barrier.wait()
            time.sleep(0.001)
            self.ts_logger.shutdown()

        threads = [threading.Thread(target=worker) for _ in range(self.THREADS)]
        threads.append(threading.Thread(target=killer))
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)

        self.assertEqual(errors, [], f"Exceptions in threads: {errors}")
        max_possible = self.THREADS * self.CALLS_PER_THREAD
        self.assertLessEqual(self.ros_logger.info.call_count, max_possible)

    def test_concurrent_shutdown_calls_are_safe(self):
        """Multiple threads calling shutdown() simultaneously must not raise."""
        errors: list[Exception] = []
        barrier = threading.Barrier(self.THREADS)

        def shutdown_worker() -> None:
            barrier.wait()
            try:
                self.ts_logger.shutdown()
            except Exception as exc:
                errors.append(exc)

        threads = [
            threading.Thread(target=shutdown_worker) for _ in range(self.THREADS)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)

        self.assertEqual(errors, [], f"Exceptions in threads: {errors}")
        self.assertFalse(self.ts_logger._alive)


# ---------------------------------------------------------------------------
# Tests: _TSLogger initial state
# ---------------------------------------------------------------------------


class TestTSLoggerInit(unittest.TestCase):
    def test_logger_starts_alive(self):
        self.assertTrue(_make_node().get_logger()._alive)

    def test_lock_is_threading_lock(self):
        lock = _make_node().get_logger()._lock
        self.assertTrue(hasattr(lock, "acquire") and hasattr(lock, "release"))

    def test_inner_logger_is_stored(self):
        node = _make_node()
        ts = node.get_logger()
        self.assertIsNotNone(ts._logger)


if __name__ == "__main__":
    unittest.main()
