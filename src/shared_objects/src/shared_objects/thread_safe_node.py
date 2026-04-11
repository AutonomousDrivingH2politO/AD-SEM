import rclpy
from rclpy.node import Node
import threading


class ThreadSafeNode(Node):
    """
    Node subclass that returns a thread-safe logger from get_logger().
    Drop-in replacement for rclpy.node.Node — no call sites need to change.

    Safe to call from any thread. The logger silently drops messages after
    shutdown() is called, preventing races during node destruction.
    """

    class _TSLogger:
        def __init__(self, ros_logger):
            self._logger = ros_logger
            self._lock = threading.Lock()
            self._alive = True

        def shutdown(self):
            with self._lock:
                self._alive = False

        def _call(self, fn, msg, *args, **kwargs):
            with self._lock:
                if self._alive:
                    fn(msg, *args, **kwargs)

        def debug(self, msg, **kwargs):
            self._call(self._logger.debug, msg, **kwargs)

        def info(self, msg, **kwargs):
            self._call(self._logger.info, msg, **kwargs)

        def warn(self, msg, **kwargs):
            self._call(self._logger.warn, msg, **kwargs)

        def error(self, msg, **kwargs):
            self._call(self._logger.error, msg, **kwargs)

        def fatal(self, msg, **kwargs):
            self._call(self._logger.fatal, msg, **kwargs)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ts_logger = self._TSLogger(super().get_logger())

    def get_logger(self):
        return self._ts_logger
