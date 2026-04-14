import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import numpy as np
from pyproj import Transformer
import csv
import math


class WaypointNavigator(Node):  # FIX 1: class, not def
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._logger = self.get_logger()
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:32632", always_xy=True
        )

        self.datum_east = None
        self.datum_north = None
        self.datum_set = False

        self.datum_sub = self.create_subscription(
            NavSatFix,
            'gps/filtered/fix',
            self._datum_callback,
            10
        )

        # FIX 2: Store raw lat/lon/heading — don't convert yet (no datum)
        self.waypoints = self._load_waypoints_raw('waypoints.csv')
        self.index = 0

    def _load_waypoints_raw(self, csv_path):
        """Load CSV as raw lat/lon with computed heading (radians)."""
        rows = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)

        lats = [float(r['lat']) for r in rows]
        lons = [float(r['lon']) for r in rows]

        # Compute heading from consecutive points using UTM
        eastings, northings = self.transformer.transform(lons, lats)
        headings = []
        for i in range(len(rows)):
            if i < len(rows) - 1:
                dx = eastings[i + 1] - eastings[i]
                dy = northings[i + 1] - northings[i]
                headings.append(math.atan2(dy, dx))
            else:
                headings.append(headings[-1] if headings else 0.0)  # Repeat last heading or default to 0

        # Store as (lat, lon, yaw_rad)
        waypoints = np.column_stack([lats, lons, headings])
        self._logger.info(f"Loaded {len(waypoints)} waypoints from {csv_path}")
        return waypoints

    def _latlon_to_pose(self, lat, lon, yaw_rad):
        """Convert lat/lon + yaw to a PoseStamped in the map frame."""
        easting, northing = self.transformer.transform(lon, lat)
        x = easting - self.datum_east
        y = northing - self.datum_north

        # FIX 3: yaw is already in radians (atan2), use directly
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _datum_callback(self, msg):
        if not self.datum_set and msg.status.status >= 1:
            e, n = self.transformer.transform(msg.longitude, msg.latitude)
            self.datum_east = e
            self.datum_north = n
            self.datum_set = True
            self._logger.info(f"Datum set: E={e:.2f}, N={n:.2f}")
            self.destroy_subscription(self.datum_sub)
            self.send_next_goal()

    def send_next_goal(self):
        if self.index >= len(self.waypoints):
            self._logger.info("All waypoints reached!")
            return

        wp = self.waypoints[self.index]
        lat, lon, yaw = wp[0], wp[1], wp[2]
        pose = self._latlon_to_pose(lat, lon, yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self._logger.info(
            f"Waypoint {self.index + 1}/{len(self.waypoints)}: "
            f"x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, "
            f"yaw={math.degrees(yaw):.1f}°"
        )

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._logger.warn(f"Goal rejected! Skipping waypoint {self.index + 1}.")
            self.index += 1
            self.send_next_goal()
            return
        self._logger.info("Goal accepted, waiting for result...")
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future):
        status = future.result().status
        if status == 4:
            self._logger.info(f"Waypoint {self.index + 1} reached!")
        else:
            self._logger.warn(f"Waypoint {self.index + 1} failed (status {status}), skipping.")
        self.index += 1
        self.send_next_goal()

    def _feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self._logger.info(f"Waypoint {self.index + 1}/{len(self.waypoints)}: {dist:.2f} m remaining")


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