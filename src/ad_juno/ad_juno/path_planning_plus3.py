#!/usr/bin/env python3

import os
import math
import numpy as np
import cv2
from datetime import datetime

import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from shared_objects.new_utils_path import computing_lateral_distance, processing_mask
from shared_objects.ROS_utils import Topics
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64, Bool
from pathlib import Path
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
)


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__("path_planning_plus3")

        self.debug = self.declare_parameter("debug", True).value
        self.wheelbase = self.declare_parameter("wheelbase", 1.6).value
        self.gain = self.declare_parameter("gain", 1.1).value

        self.declare_parameter("debug_root", "/home/bylogix/AD-SEM/DEBUG")
        self.debug_root = Path(
            self.get_parameter("debug_root").get_parameter_value().string_value
        )

        self.costmap_has_obstacles = False
        self.goal_published = False
        self.bridge = CvBridge()
        self.cv_image = None
        self.counter = 0

        self._obstacle_frames = 0
        self._clear_frames = 0
        self._OBSTACLE_CONFIRM = 3
        self._CLEAR_CONFIRM = 5

        self._fwd_roi_m = 4.0
        self._lat_roi_m = 0.8
        self._cost_thresh = 99  # only LETHAL cells — ignores boundary wall inflation

        topics = Topics()
        self.topic_names = topics.topic_names

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_image = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(
            Image, self.topic_names["segmented_image"], self.image_callback, qos_image
        )
        self.create_subscription(
            Image,
            "/zed/zed_node/rgb/color/rect/image",
            self.original_image_callback,
            qos_image,
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.costmap_callback, 1
        )

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 1)
        self.steer_pub = self.create_publisher(Float64, self.topic_names["steering"], 1)
        self.bev_pub = self.create_publisher(Image, "/birds_eye_view", 10)
        self.state_pub = self.create_publisher(Bool, "/ad_juno/nav2_active", 1)

        if self.debug:
            self.logs_folder, self.output_folder, self.frames_folder = (
                self.set_debug_folders()
            )

        self.get_logger().info("PathPlanningPlus3 initialized")

    # ------------------------------------------------------------------
    def costmap_callback(self, msg: OccupancyGrid):
        res = msg.info.resolution
        w = msg.info.width
        h = msg.info.height
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((h, w))

        # Get robot pose in costmap frame to build a yaw-corrected ROI
        try:
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id,
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            robot_x = tf.transform.translation.x
            robot_y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        except Exception:
            # Fallback: assume robot at grid centre, heading along map X
            robot_x = ox + (w / 2) * res
            robot_y = oy + (h / 2) * res
            yaw = 0.0

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # Forward corridor corners in robot frame → map frame → grid indices
        corners_robot = [
            (0.0, -self._lat_roi_m),
            (0.0, self._lat_roi_m),
            (self._fwd_roi_m, self._lat_roi_m),
            (self._fwd_roi_m, -self._lat_roi_m),
        ]
        pts = []
        for rx, ry in corners_robot:
            mx = robot_x + cos_y * rx - sin_y * ry
            my = robot_y + sin_y * rx + cos_y * ry
            col = int((mx - ox) / res)
            row = int((my - oy) / res)
            pts.append([col, row])

        roi_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(roi_mask, [np.array(pts, dtype=np.int32)], 1)

        detected = bool(np.any(data[roi_mask == 1] >= self._cost_thresh))

        if detected:
            self._obstacle_frames += 1
            self._clear_frames = 0
            if self._obstacle_frames >= self._OBSTACLE_CONFIRM:
                self.costmap_has_obstacles = True
        else:
            self._clear_frames += 1
            self._obstacle_frames = 0
            if self._clear_frames >= self._CLEAR_CONFIRM:
                self.costmap_has_obstacles = False

        state_msg = Bool()
        state_msg.data = self.costmap_has_obstacles
        self.state_pub.publish(state_msg)

    # ------------------------------------------------------------------
    def set_debug_folders(self):
        try:
            self.debug_root.mkdir(parents=True, exist_ok=True)
            ts_folder = self.debug_root / (
                datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + "_plus3"
            )
            ts_folder.mkdir()
            logs = ts_folder / "logs"
            out = ts_folder / "output"
            frames = ts_folder / "frames"
            for p in (logs, out, frames):
                p.mkdir()
            self.get_logger().info(
                f"DEBUG =>\n  logs: {logs}\n  output: {out}\n  frames: {frames}"
            )
            return str(logs), str(out), str(frames)
        except Exception as e:
            self.get_logger().error(f"Failed to create debug folders: {e}")
            raise

    # ------------------------------------------------------------------
    def original_image_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")

    # ------------------------------------------------------------------
    def image_callback(self, data):
        if self.cv_image is None:
            return

        mask = self.bridge.imgmsg_to_cv2(data, "mono8")
        line_edges = processing_mask(mask, self.cv_image, show=False)

        self.bev_pub.publish(self.bridge.cv2_to_imgmsg(line_edges, encoding="mono8"))

        lateral_distance, longitudinal_distance, midpoints = computing_lateral_distance(
            line_edges, show=False
        )

        if lateral_distance == -np.inf:
            self.get_logger().warn("Lane lost — emergency steer -10°")
            degree_steering_angle = -10.0
        elif lateral_distance == np.inf:
            self.get_logger().warn("Lane lost — emergency steer +10°")
            degree_steering_angle = 10.0
        else:
            degree_steering_angle = None

        if self.costmap_has_obstacles:
            if lateral_distance in (-np.inf, np.inf):
                self.get_logger().warn("Obstacle present but lane lost — holding")
                return

            goal_camera = PoseStamped()
            goal_camera.header.stamp = self.get_clock().now().to_msg()
            goal_camera.header.frame_id = "zed_camera_link"
            goal_camera.pose.position.x = float(longitudinal_distance)
            goal_camera.pose.position.y = float(-lateral_distance)
            goal_camera.pose.orientation.w = 1.0

            try:
                goal_map = self.tf_buffer.transform(
                    goal_camera, "odom", timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except Exception as e:
                self.get_logger().warn(f"TF failed: {e}")
                return

            self.goal_pub.publish(goal_map)
            self.goal_published = True

        else:
            self.goal_published = False
            if degree_steering_angle is None:
                dist = (longitudinal_distance + self.gain) ** 2 + lateral_distance**2
                degree_steering_angle = math.degrees(
                    math.atan2(2 * self.wheelbase * lateral_distance, dist)
                )
            steer_msg = Float64()
            steer_msg.data = degree_steering_angle
            self.steer_pub.publish(steer_msg)

        if self.debug:
            self._save_debug(
                mask,
                line_edges,
                lateral_distance,
                longitudinal_distance,
                midpoints,
                degree_steering_angle,
            )

    # ------------------------------------------------------------------
    def _save_debug(
        self,
        mask,
        line_edges,
        lateral_distance,
        longitudinal_distance,
        midpoints,
        degree_steering_angle,
    ):
        annotated = line_edges.copy()  # never mutate the published BEV buffer
        if midpoints is not None:
            for p in midpoints[:-1]:
                cv2.circle(annotated, tuple(p[::-1]), 2, (200, 200, 200), 3)
            cv2.circle(annotated, tuple(midpoints[-1][::-1]), 2, (255, 255, 255), 5)

        resized_image = cv2.resize(self.cv_image, (540, 360))
        resized_mask = cv2.resize(mask, (540, 360))
        resized_edges = cv2.resize(annotated, (540, 360))

        concatenated = np.hstack(
            (
                cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB),
                cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2RGB),
                cv2.cvtColor(resized_edges, cv2.COLOR_GRAY2BGR),
            )
        )

        cv2.imwrite(
            os.path.join(self.frames_folder, f"frame_{self.counter}.png"), self.cv_image
        )
        cv2.imwrite(
            os.path.join(self.output_folder, f"output_{self.counter}.png"), concatenated
        )

        mode = (
            f"{degree_steering_angle:.3f}deg"
            if degree_steering_angle is not None
            else "nav2(obstacle_avoidance)"
        )
        with open(os.path.join(self.logs_folder, f"log_{self.counter}.txt"), "w") as f:
            f.write(
                f"{self.counter}: "
                f"lateral={lateral_distance:.3f} "
                f"longitudinal={longitudinal_distance:.3f} "
                f"steer={mode} "
                f"nav2_active={self.costmap_has_obstacles} "
                f"time={datetime.now().isoformat()}\n"
            )

        self.counter += 1


# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
