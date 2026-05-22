#!/usr/bin/env python3
"""Bridge /cmd_vel -> vehicle steer/throttle with hardware-timed rate limits."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float32, Bool
from shared_objects.ROS_utils import Topics
from rclpy.qos import (
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

MAX_STEER_DEG = 10.0
WHEELBASE = 1.6
MS_TO_KMH = 3.6
MIN_V_FOR_STEER = 0.3
STEER_ALPHA = 0.3
STEER_PERIOD_S = 0.14
SPEED_PERIOD_S = 1.0
CMD_VEL_STALE_S = 0.5

# FIX 4: how many timer ticks to spend ramping steer back to zero once the
# /cmd_vel stream goes stale (prevents the car holding a turn indefinitely).
STEER_RAMP_ZERO_ALPHA = 0.2  # decay factor per tick; tune to taste


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_node")

        topics = Topics()
        tn = topics.topic_names

        # FIX (medium): define the QoS profile once and actually pass it to the
        # subscription. The original code declared qos_profile but then passed
        # the literal `1` (depth only) to create_subscription, silently
        # discarding the RELIABLE setting.
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Bool, "/direct_steer_active", self._direct_steer_cb, qos_latched)

        self.steer_pub = self.create_publisher(Float64, tn["steering"], 1)
        self.speed_pub = self.create_publisher(Float32, tn["requested_speed"], 1)

        # FIX (medium): pass qos_profile instead of the bare integer 1
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, qos_profile)

        self._steer_target_deg = 0.0
        self._steer_filt_deg = 0.0
        self._speed_kmh = 0.0
        self._last_cmd_time = None
        self._direct_steer_active = False

        self.create_timer(STEER_PERIOD_S, self._steer_timer)
        self.create_timer(SPEED_PERIOD_S, self._speed_timer)

        self.get_logger().info(
            f"Bridge: /cmd_vel -> {tn['steering']} @ {1 / STEER_PERIOD_S:.1f}Hz, "
            f"{tn['requested_speed']} @ {1 / SPEED_PERIOD_S:.1f}Hz"
        )

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z

        if abs(v) < MIN_V_FOR_STEER:
            steer_deg = 0.0
        else:
            steer_deg = math.degrees(math.atan(WHEELBASE * omega / v))
        steer_deg = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer_deg))

        self._steer_target_deg = steer_deg
        self._speed_kmh = abs(v) * MS_TO_KMH
        self._last_cmd_time = self.get_clock().now().nanoseconds * 1e-9
    
    def _direct_steer_cb(self, msg:Bool) -> None:
        if self._direct_steer_active != msg.data:
            self.get_logger().info(f"Arbitration update: Direct Steer Active = {msg.data}")
        self._direct_steer_active = msg.data

    def _cmd_fresh(self) -> bool:
        if self._last_cmd_time is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - self._last_cmd_time) <= CMD_VEL_STALE_S

    def _steer_timer(self):
        if self._direct_steer_active:
            self._steer_filt_deg = 0.0
            return
        if not self._cmd_fresh():
            # FIX 4: when the /cmd_vel stream goes stale, decay the filtered
            # steer value back toward zero each tick instead of freezing it.
            # This prevents the vehicle from holding a turn angle after Nav2
            # stops publishing — e.g. after reaching a goal or on planner failure.
            if abs(self._steer_filt_deg) > 0.05:  # dead-band to avoid chatter
                self._steer_filt_deg *= 1.0 - STEER_RAMP_ZERO_ALPHA
                m = Float64()
                m.data = float(self._steer_filt_deg)
                self.steer_pub.publish(m)
                self.get_logger().debug(
                    f"cmd_vel stale — ramping steer to zero: {self._steer_filt_deg:.2f}°"
                )
            else:
                # Snap to exactly zero once we are close enough
                if self._steer_filt_deg != 0.0:
                    self._steer_filt_deg = 0.0
                    m = Float64()
                    m.data = 0.0
                    self.steer_pub.publish(m)
                    self.get_logger().info("cmd_vel stale — steer zeroed")
            return

        self._steer_filt_deg = (
            STEER_ALPHA * self._steer_target_deg
            + (1.0 - STEER_ALPHA) * self._steer_filt_deg
        )
        m = Float64()
        m.data = float(self._steer_filt_deg)
        self.steer_pub.publish(m)
        self.get_logger().info(
            f"Sent Command: Steering degree raw: {self._steer_target_deg} "
            f"and filtered: {self._steer_filt_deg}"
        )

    def _speed_timer(self):
        if self._direct_steer_active:
            self._speed_kmh =0.0
            return
        if not self._cmd_fresh():
            return
        m = Float32()
        m.data = float(self._speed_kmh)
        self.speed_pub.publish(m)
        self.get_logger().info(f"Sent Command: Speed {self._speed_kmh} km/h")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down obstacle_avoidance_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
