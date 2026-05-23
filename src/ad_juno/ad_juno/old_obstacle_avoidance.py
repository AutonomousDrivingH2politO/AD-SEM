#!/usr/bin/env python3
"""
Bridge between Nav2 MPPI controller for obstacles and the vehicle's steering/throttle.
Compatible to plus3 because of the active_pp boolean
Nav2 outputs:  geometry_msgs/Twist on /cmd_vel  (m/s, rad/s)
Vehicle needs: Float64 on commands/KalmanAngle  (degrees, ±10 max)
               Float32 on commands/speed         (km/h threshold)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float32, Bool
from shared_objects.ROS_utils import Topics
from nav_msgs.msg import OccupancyGrid
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


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_node")

        topics = Topics()
        tn = topics.topic_names

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            depth=10,  # RECONSIDER THIS
            reliability=ReliabilityPolicy.RELIABLE,
        )
        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Publishers
        self.steer_pub = self.create_publisher(Float64, tn["steering"], 1)
        self.speed_pub = self.create_publisher(Float32, tn["requested_speed"], 1)
        self.direct_steer_active = True

        # Nav2 controller_server publishes cmd_vel here by default
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 1)

        self.create_subscription(
            Bool, "/direct_steer_active", self.direct_steer_cb, qos_latched
        )

        self.get_logger().info(
            f"Obstacle avoidance bridge started: /cmd_vel -> {tn['steering']} + {tn['requested_speed']}"
        )

    def direct_steer_cb(self, msg: Bool) -> None:
        self.direct_steer_active = msg.data

    def cmd_vel_callback(self, msg: Twist):
        if self.direct_steer_active:
            return
        v = msg.linear.x  # m/s
        omega = msg.angular.z  # rad/s

        # Ackermann: steering_angle = atan(wheelbase * omega / v)
        if abs(v) < 0.3:
            steering_deg = 0.0
        else:
            steering_rad = math.atan(WHEELBASE * omega / v)
            steering_deg = math.degrees(steering_rad)

        # comply to physical limits
        steering_deg = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steering_deg))

        steer_msg = Float64()
        steer_msg.data = steering_deg
        self.steer_pub.publish(steer_msg)

        # Convert velocity to km/h speed threshold for throttle node
        speed_kmh = abs(v) * MS_TO_KMH
        speed_msg = Float32()
        speed_msg.data = float(speed_kmh)
        self.speed_pub.publish(speed_msg)
        self.get_logger().info(
            f"Sent Commands: Steering degree: {steering_deg} Speed: {speed_kmh} km/h"
        )


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
