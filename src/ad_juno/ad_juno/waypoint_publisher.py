#!/usr/bin/env python3
import math
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformListener
from pyproj import Transformer
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

ARRIVAL_THRESHOLD_M = 1.5


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("waypoint_navigator")
        self._logger = self.get_logger()

        self.transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:32632", always_xy=True
        )

        self.datum_east = None
        self.datum_north = None
        self.datum_set = False

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.datum_sub = self.create_subscription(
            NavSatFix, "gps/filtered/fix", self._datum_callback, 10
        )

        self.waypoints = self._load_waypoints_raw("waypoints.csv")
        self.index = 0

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 1)
        self._latest_goal: PoseStamped | None = None
        self._goal_timer = self.create_timer(0.1, self._goal_timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._arrival_timer = self.create_timer(0.2, self._arrival_check_callback)
        self._arrival_timer.cancel()

        
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
        self._logger.info(f"Datum set: E={e:.2f}, N={n:.2f}")
        self.destroy_subscription(self.datum_sub)
        self._publish_current_waypoint()
        self._arrival_timer.reset()

    def _load_waypoints_raw(self, csv_path: str) -> np.ndarray:
        rows = []
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)

        lats = [float(r["lat"]) for r in rows]
        lons = [float(r["lon"]) for r in rows]

        eastings, northings = self.transformer.transform(lons, lats)
        headings = []
        for i in range(len(rows)):
            if i < len(rows) - 1:
                dx = eastings[i + 1] - eastings[i]
                dy = northings[i + 1] - northings[i]
                headings.append(math.atan2(dy, dx))
            else:
                headings.append(headings[-1] if headings else 0.0)

        waypoints = np.column_stack([lats, lons, headings])
        self._logger.info(f"Loaded {len(waypoints)} waypoints from {csv_path}")
        return waypoints

    def _latlon_to_pose(self, lat: float, lon: float, yaw_rad: float) -> PoseStamped:
        easting, northing = self.transformer.transform(lon, lat)
        x = easting - self.datum_east
        y = northing - self.datum_north

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        return pose

    def _publish_current_waypoint(self):
        if self.index >= len(self.waypoints):
            self._logger.info("All waypoints reached — stopping.")
            self._latest_goal = None
            self._arrival_timer.cancel()
            return

        lat, lon, yaw = self.waypoints[self.index]
        pose = self._latlon_to_pose(lat, lon, yaw)
        self._latest_goal = pose
        self._logger.info(
            f"Waypoint {self.index + 1}/{len(self.waypoints)}: "
            f"x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, "
            f"yaw={math.degrees(yaw):.1f}°"
        )

    def _arrival_check_callback(self):
        if self._latest_goal is None or self.index >= len(self.waypoints):
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception as e:
            self._logger.debug(f"TF map->base_link unavailable: {e}")
            return

        dx = self._latest_goal.pose.position.x - tf.transform.translation.x
        dy = self._latest_goal.pose.position.y - tf.transform.translation.y

        if math.hypot(dx, dy) < ARRIVAL_THRESHOLD_M:
            self._logger.info(
                f"Waypoint {self.index + 1}/{len(self.waypoints)} reached. Advancing."
            )
            self.index += 1
            self._publish_current_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
