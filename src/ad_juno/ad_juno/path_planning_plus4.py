#!/usr/bin/env python3

import os
import math
import csv
import numpy as np
import cv2
from datetime import datetime

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from pyproj import Transformer
from shared_objects.utils_path import computing_lateral_distance, processing_mask
from shared_objects.ROS_utils import Topics, SHOW

ARRIVAL_THRESHOLD_M = 1.5


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_plus4')

        self.debug = self.declare_parameter('debug', False).value
        self.wheelbase = self.declare_parameter('wheelbase', 1.6).value
        self.gain = self.declare_parameter('gain', 0.0).value

        self.costmap_has_obstacles = False

        self.bridge = CvBridge()
        self.cv_image = None
        self.counter = 0

        topics = Topics()
        self.topic_names = topics.topic_names

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:32632", always_xy=True)
        self.datum_east = None
        self.datum_north = None
        self.datum_set = False
        self.waypoints = self._load_waypoints("waypoints.csv")
        self.index = 0

        self.datum_sub = self.create_subscription(
            NavSatFix, "gps/filtered/fix", self._datum_callback, 10
        )

        self.create_subscription(Image, self.topic_names["segmented_image"], self.image_callback, 1)
        self.create_subscription(Image, '/zed/zed_node/rgb/color/rect/image', self.original_image_callback, 1)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 1)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.steer_pub = self.create_publisher(Float64, self.topic_names["steering"], 1)
        self.bev_pub = self.create_publisher(Image, "/birds_eye_view", 10)

        self._latest_goal = None
        self._goal_timer = self.create_timer(0.1, self._goal_timer_callback)

        self._arrival_timer = self.create_timer(0.2, self._arrival_check_callback)
        self._arrival_timer.cancel()

        if self.debug:
            self.logs_folder, self.output_folder, self.frames_folder = self._set_debug_folders()

        self.get_logger().info("PathPlanningPlus3 initialized")

    def _goal_timer_callback(self):
        if self._latest_goal is None:
            return
        self.goal_pub.publish(self._latest_goal)

    def _datum_callback(self, msg: NavSatFix):
        if self.datum_set or msg.status.status < 1:
            return
        e, n = self.transformer.transform(msg.longitude, msg.latitude)
        self.datum_east = e
        self.datum_north = n
        self.datum_set = True
        self.get_logger().info(f"Datum set: E={e:.2f}, N={n:.2f}")
        self.destroy_subscription(self.datum_sub)
        self._publish_current_waypoint()
        self._arrival_timer.reset()

    def _load_waypoints(self, csv_path: str) -> np.ndarray:
        rows = []
        with open(csv_path, "r") as f:
            for row in csv.DictReader(f):
                rows.append(row)

        lats = [float(r["lat"]) for r in rows]
        lons = [float(r["lon"]) for r in rows]
        eastings, northings = self.transformer.transform(lons, lats)

        headings = []
        for i in range(len(rows)):
            if i < len(rows) - 1:
                headings.append(math.atan2(
                    northings[i + 1] - northings[i],
                    eastings[i + 1] - eastings[i]
                ))
            else:
                headings.append(headings[-1] if headings else 0.0)

        waypoints = np.column_stack([lats, lons, headings])
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {csv_path}")
        return waypoints

    def _latlon_to_pose(self, lat: float, lon: float, yaw_rad: float) -> PoseStamped:
        easting, northing = self.transformer.transform(lon, lat)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = easting - self.datum_east
        pose.pose.position.y = northing - self.datum_north
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        return pose

    def _publish_current_waypoint(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached — stopping.")
            self._latest_goal = None
            self._arrival_timer.cancel()
            return

        lat, lon, yaw = self.waypoints[self.index]
        self._latest_goal = self._latlon_to_pose(lat, lon, yaw)
        self.get_logger().info(
            f"Waypoint {self.index + 1}/{len(self.waypoints)}: "
            f"x={self._latest_goal.pose.position.x:.2f}, "
            f"y={self._latest_goal.pose.position.y:.2f}, "
            f"yaw={math.degrees(yaw):.1f}°"
        )

    def _arrival_check_callback(self):
        if self._latest_goal is None or self.index >= len(self.waypoints):
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception as e:
            self.get_logger().debug(f"TF map->base_link unavailable: {e}")
            return

        dx = self._latest_goal.pose.position.x - tf.transform.translation.x
        dy = self._latest_goal.pose.position.y - tf.transform.translation.y

        if math.hypot(dx, dy) < ARRIVAL_THRESHOLD_M:
            self.get_logger().info(
                f"Waypoint {self.index + 1}/{len(self.waypoints)} reached. Advancing."
            )
            self.index += 1
            self._publish_current_waypoint()

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap_has_obstacles = bool(
            np.any(np.array(msg.data, dtype=np.int8) > 0)
        )

    def original_image_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")

    def image_callback(self, data):
        if self.cv_image is None or not self.datum_set:
            return

        mask = self.bridge.imgmsg_to_cv2(data, "mono8")
        line_edges = processing_mask(mask, self.cv_image, show=False)
        self.bev_pub.publish(self.bridge.cv2_to_imgmsg(line_edges, encoding="mono8"))

        lateral_distance, longitudinal_distance, midpoints = computing_lateral_distance(
            line_edges, show=False
        )

        if SHOW and midpoints is not None:
            for p in midpoints[:-1]:
                cv2.circle(line_edges, tuple(p[::-1]), 2, (200, 200, 200), 3)
            cv2.circle(line_edges, tuple(midpoints[-1][::-1]), 2, (255, 255, 255), 5)

        if self.debug:
            self._save_debug(mask, line_edges, lateral_distance, longitudinal_distance)

        lane_lost = lateral_distance in (-np.inf, np.inf)

        if self.costmap_has_obstacles:
            if lane_lost:
                self.get_logger().warn("Obstacles present but lane lost — holding current waypoint goal")
                return
            self._publish_current_waypoint()
        else:
            if lane_lost:
                degree_steering_angle = -10.0 if lateral_distance == -np.inf else 10.0
                self.get_logger().warn(f"Lane lost, emergency steering angle {degree_steering_angle}")
            else:
                dist = (longitudinal_distance + self.gain) ** 2 + lateral_distance ** 2
                degree_steering_angle = math.degrees(
                    math.atan2(2 * self.wheelbase * lateral_distance, dist)
                )
            steer_msg = Float64()
            steer_msg.data = degree_steering_angle
            self.steer_pub.publish(steer_msg)

    def _set_debug_folders(self):
        debug_folder = os.path.join(os.getcwd(), "DEBUG")
        os.makedirs(debug_folder, exist_ok=True)
        folder = os.path.join(debug_folder, datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
        os.makedirs(folder, exist_ok=True)
        logs_folder = os.path.join(folder, "logs")
        output_folder = os.path.join(folder, "output")
        frames_folder = os.path.join(folder, "frames")
        for p in (logs_folder, output_folder, frames_folder):
            os.makedirs(p, exist_ok=True)
        return logs_folder, output_folder, frames_folder

    def _save_debug(self, mask, line_edges, lateral_distance, longitudinal_distance):
        resized_image = cv2.resize(self.cv_image, (540, 360))
        concatenated_image = np.hstack((
            cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB),
            cv2.cvtColor(cv2.resize(mask, (540, 360)), cv2.COLOR_GRAY2RGB),
            cv2.cvtColor(cv2.resize(line_edges, (540, 360)), cv2.COLOR_GRAY2BGR)
        ))
        cv2.imwrite(os.path.join(self.frames_folder, f"frame_{self.counter}.png"), self.cv_image)
        cv2.imwrite(os.path.join(self.output_folder, f"output_{self.counter}.png"), concatenated_image)
        with open(os.path.join(self.logs_folder, f"log_{self.counter}.txt"), "w") as log:
            log.write(
                f"{self.counter}: lateral={lateral_distance:.3f} "
                f"longitudinal={longitudinal_distance:.3f}\n"
            )
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down path_planning_plus4")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
