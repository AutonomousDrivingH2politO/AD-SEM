#!/usr/bin/env python3
"""Publishes dynamic odometry and TF so Nav2 can drive on the bench."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class KinematicFakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.pub = self.create_publisher(Odometry, '/odometry/filtered/local', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update_and_publish)  # 20 Hz
        self.last_time = self.get_clock().now()

        # Vehicle state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Current velocity commands
        self.v = 0.0
        self.omega = 0.0

    def cmd_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 1. Integrate velocities to get new position
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt

        # Convert theta to quaternion (yaw to quaternion)
        qw = math.cos(self.theta / 2.0)
        qz = math.sin(self.theta / 2.0)

        # 2. Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.omega
        self.pub.publish(odom_msg)

        # 3. Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(KinematicFakeOdom())

if __name__ == '__main__':
    main()
