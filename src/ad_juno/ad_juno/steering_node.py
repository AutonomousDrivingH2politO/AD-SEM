#!/usr/bin/env python3
import rclpy
import time
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float64
from shared_objects.Motor import Stepper
from shared_objects.thread_safe_node import ThreadSafeNode
from shared_objects.ROS_utils import Topics
import ctypes
import os
import threading
import queue
from pathlib import Path

# Conversion parameters and limits
TRANSMISSION_RATIO = 14
MAX_ANGLE = 10
STEP_ANGLE = 1.8  # in degrees

try:
    _lib_pid = ctypes.CDLL(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "c_utils", "libpid.so")
    )
except OSError as err:
    raise FileNotFoundError(f"libpid.so not found: {err}") from err


class _CPID(ctypes.Structure):
    _fields_ = [
        ("kp", ctypes.c_double),
        ("ki", ctypes.c_double),
        ("kd", ctypes.c_double),
        ("integral", ctypes.c_double),
        ("last_error", ctypes.c_double),
    ]


_lib_pid.pid_init.argtypes = [
    ctypes.POINTER(_CPID),
    ctypes.c_double,
    ctypes.c_double,
    ctypes.c_double,
]
_lib_pid.pid_init.restype = None
_lib_pid.pid_compute.argtypes = [
    ctypes.POINTER(_CPID),
    ctypes.c_double,
    ctypes.c_double,
    ctypes.c_double,
]
_lib_pid.pid_compute.restype = ctypes.c_double


class PIDController:
    def __init__(self, kp, ki, kd):
        self._state = _CPID()
        _lib_pid.pid_init(ctypes.byref(self._state), kp, ki, kd)

    def compute(self, target, actual, dt):
        return _lib_pid.pid_compute(ctypes.byref(self._state), target, actual, dt)


class SteeringNode(ThreadSafeNode):
    def __init__(self):
        super().__init__("steering_node")

        # Initialize the stepper with the required parameters
        self.steering = Stepper(
            interface="can",
            data_rate=1000000,
            module_id=3,  # steering module
            max_velocity=80_000,
            max_acc=50_000,
            MaxDeceleration=50_000,  # example value — adjust as needed
            V1=100_000,
            A1=50_000,  # example value — adjust as needed
            D1=50_000,  # example value — adjust as needed
        )

        # Initialize the PID (gains need tuning for the actual system)
        self.pid = PIDController(kp=1.0, ki=0.0, kd=0.0)

        self._angle_lock = threading.Lock()
        self._target_pos = 0

        self._position_lock = threading.Lock()
        self._current_position = 0

        self._queue = queue.Queue(maxsize=1)  # only latest command
        self._running = True  # flag for thread
        self._read_thread = threading.Thread(
            target=self._position_reader, daemon=True, name="can_read"
        )
        self._write_thread = threading.Thread(
            target=self._command_writer, daemon=True, name="can_write"
        )
        self._read_thread.start()
        self._write_thread.start()

        self.declare_parameter(
            "debug_root",
            "/home/bylogix/AD-SEM/DEBUG",  # default path
        )
        self.debug = (
            self.declare_parameter("debug", False).get_parameter_value().bool_value
        )
        self.debug_root = Path(
            self.get_parameter("debug_root").get_parameter_value().string_value
        )
        self.debug_root.mkdir(parents=True, exist_ok=True)

        # QoS profile for the subscriber
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        topics = Topics()
        topic_names = topics.topic_names

        self.subscription = self.create_subscription(
            Float64, topic_names["steering"], self.callback_stepangle, qos_profile
        )

        # Start the clock before creating the timer so the first tick never
        # finds _last_time undefined
        self._last_time = self.get_clock().now()

        # Timer at 100 Hz for the PID control loop (dt = 0.01 s nominal)
        self.pid_timer = self.create_timer(0.01, self.pid_loop)

        self.get_logger().info("Steering node with PID at 100Hz initialized")

    def callback_stepangle(self, msg):
        """
        Callback that updates the target angle (in degrees) when a new
        message arrives. Clamps the angle and converts it to microsteps
        once here so the PID loop doesn't repeat the work every tick.
        """
        angle = max(-MAX_ANGLE, min(MAX_ANGLE, msg.data))
        position = self.convert_angle_to_position(angle)
        with self._angle_lock:
            self._target_pos = position
        if self.debug:
            self.get_logger().info(
                f"Received angle: {angle:.2f} → position: {position}"
            )

    def _position_reader(self):
        interval = 1.0 / 50  # 20ms
        while self._running:
            t0 = time.monotonic()

            try:
                pos = self.steering.update_unwrapped_position()
                with self._position_lock:
                    self._current_position = pos
            except Exception as e:
                self.get_logger().error(f"[can_read] position read failed: {e}")

            # Sleep for whatever is left of the 20ms window
            elapsed = time.monotonic() - t0
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)
            else:
                # CAN round-trip took longer than the interval — log but keep going
                self.get_logger().warn(
                    f"[can_read] overrun by {-remaining * 1000:.1f}ms"
                )

    def _command_writer(self):
        while self._running:
            try:
                cmd = self._queue.get(timeout=0.02)
            except queue.Empty:
                self._logger.info("No command ready continuing the loop")
                continue  # no command ready, loop and check _running

            try:
                self.steering.move_stepper(cmd)
            except Exception as e:
                self.get_logger().error(f"[can_write] move failed: {e}")

    def convert_angle_to_position(self, angle):
        """
        Converts a target angle (degrees) to an absolute motor position
        in microsteps.

        Each full step is STEP_ANGLE degrees; the driver uses 256
        microsteps per full step. The transmission ratio scales the
        result to the output shaft, and the sign is negated to match
        the physical steering direction.
        """
        steps = (angle / STEP_ANGLE) * 256
        return -int(steps * TRANSMISSION_RATIO)

    def pid_loop(self):
        now = self.get_clock().now()
        dt = max((now - self._last_time).nanoseconds * 1e-9, 1e-6)
        self._last_time = now

        # Both reads are just memory — no CAN, no blocking
        with self._position_lock:
            current_position = self._current_position
        with self._angle_lock:
            target_position = self._target_pos

        pid_output = self.pid.compute(target_position, current_position, dt)
        new_command = int(current_position + pid_output)

        # Enqueue command — drop stale one if writer hasn't consumed it yet
        if self._queue.full():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
        self._queue.put_nowait(new_command)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Steering node shutting down")
    finally:
        node._running = False
        node.get_logger().shutdown()
        node._read_thread.join(timeout=1.0)
        node._write_thread.join(timeout=1.0)
        node.steering.disconnect_motor()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
