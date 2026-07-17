#!/usr/bin/env python3.10
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float32
from cv_bridge import CvBridge
import math
from datetime import datetime
#modify the directory of shared_objects
import os
import sys
from pathlib import Path
from nav_msgs.msg import OccupancyGrid
from shared_objects.ROS_utils import Topics, SHOW
from std_msgs.msg import Float64, Float32, Bool
from geometry_msgs.msg import TwistWithCovarianceStamped

from shared_objects.new_utils_midpoint_avoidance_path import computing_lateral_distance, processing_mask
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Initialize topics and node-related variables
topics = Topics()
topic_names = topics.topic_names

# Node class
class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        self.wheelbase = 1.6
        self.speed = 5.0
        self.gain = 1.1
        self.DEBUG = True


        self._in_obs_mode = False
        self.LETHAL_THRESHOLD = 90      # cells with cost >= this count as obstacle
        self.MIN_LETHAL_CELLS = 3       # need >= N lethal cells
        self._raw_obstacle_history = [False, False, False]
        self.stop_sign_detected = False
        self.gps_vel_kmh = 0.0
        self.ecu_speed = 0.0

        qos_profile_floats = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        qos_profile_images = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.steer_pub = self.create_publisher(Float64, topic_names['steering'], 10)
        self.obs_mode_pub = self.create_publisher(Bool, '/path_planning/obstacle_mode', 10)
        self.req_speed_pub = self.create_publisher(Float32, topic_names['requested_speed'], 10)
        self.bev_pub = self.create_publisher(Image, "/birds_eye_view", 10)
        self.image_sub = self.create_subscription(Image, topic_names['segmented_image'], self.image_callback, 10)
        self.original_image_sub = self.create_subscription(Image, topic_names['RGB_image'], self.original_image_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap/costmap', self.costmap_callback, 1)
        self.stop_sub = self.create_subscription(Bool, '/commands/stop', self.stop_callback, 10)
        self.gps_vel_sub = self.create_subscription(TwistWithCovarianceStamped, '/ublox_gps_node/fix_velocity', self.gps_vel_callback, 10)
        self.ecu_speed_sub = self.create_subscription(Float32, '/ECU/speed', self.ecu_speed_callback, 10)

        self.bridge = CvBridge()
        self.counter = 0
        self.cv_image = None 

        self.declare_parameter(
            'debug_root',
            '/home/bylogix/AD-SEM/DEBUG'   # default
        )
        self.debug_root = Path(
        self.get_parameter('debug_root').get_parameter_value().string_value
        )
        
        if self.DEBUG:
            self.logs_folder, self.output_folder, self.frames_folder = self.set_debug_folders()

        # Initial speed
        req_speed_msg = Float32()
        req_speed_msg.data = self.speed
        self.req_speed_pub.publish(req_speed_msg)
    
    def costmap_callback(self, msg: OccupancyGrid):
        data = np.frombuffer(bytes(msg.data), dtype=np.int8)  # faster than np.array on list
        raw = int(np.sum(data >= self.LETHAL_THRESHOLD)) >= self.MIN_LETHAL_CELLS
        self._raw_obstacle_history.append(raw)
        self._raw_obstacle_history.pop(0)
        
        # Only flip when all recent frames agree
        if all(self._raw_obstacle_history) and not self._in_obs_mode:
            self.get_logger().warn("OBSTACLE_AVOID - Filtering active")
            self._in_obs_mode = True
            req_speed_msg = Float32()
            req_speed_msg.data = 6.0
            self.req_speed_pub.publish(req_speed_msg)
            obs_mode_msg = Bool()
            obs_mode_msg.data = True
            self.obs_mode_pub.publish(obs_mode_msg)
        elif not any(self._raw_obstacle_history) and self._in_obs_mode:
            self.get_logger().info("LANE_FOLLOW - Filtering inactive")
            self._in_obs_mode = False
            req_speed_msg = Float32()
            req_speed_msg.data = 10.0
            self.req_speed_pub.publish(req_speed_msg)
            obs_mode_msg = Bool()
            obs_mode_msg.data = False
            self.obs_mode_pub.publish(obs_mode_msg)

    def set_debug_folders(self):
        try:
            # absolute, user-controlled root
            self.debug_root.mkdir(parents=True, exist_ok=True)

            # time-stamped run folder
            ts_folder = self.debug_root / datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
            ts_folder.mkdir()

            # sub-folders
            logs  = ts_folder / 'logs'
            out   = ts_folder / 'output'
            frames= ts_folder / 'frames'
            for p in (logs, out, frames):
                p.mkdir()

            # show where we’re writing
            self.get_logger().info(
                f'DEBUG output →\n  logs:   {logs}\n  output: {out}\n  frames: {frames}'
            )
            return str(logs), str(out), str(frames)

        except Exception as e:
            self.get_logger().error(f'Failed to create debug folders: {e}')
            raise

    def original_image_callback(self, data):
        # Store the RGB image
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")

    def stop_callback(self, msg: Bool):
        self.stop_sign_detected = msg.data

    def gps_vel_callback(self, msg: TwistWithCovarianceStamped):
        x = msg.twist.twist.linear.x
        y = msg.twist.twist.linear.y
        z = msg.twist.twist.linear.z
        self.gps_vel_kmh = math.sqrt(x**2 + y**2 + z**2) * 3.6

    def ecu_speed_callback(self, msg: Float32):
        self.ecu_speed = msg.data

    def image_callback(self, data):
        # Convert ROS Image message to OpenCV image
        mask = self.bridge.imgmsg_to_cv2(data, "mono8")
        if self.cv_image is None:
            self.get_logger().warn("Image not recieved from original_image_callback")
            return
        
        line_edges = processing_mask(mask, self.cv_image)

        birds_eye_msg = self.bridge.cv2_to_imgmsg(line_edges, encoding="mono8")
        
        self.bev_pub.publish(birds_eye_msg)
        
        # Calculate distances and midpoint
        lateral_distance, longitudinal_distance, midpoints = computing_lateral_distance(
            line_edges, 
            obstacle_seen=self._in_obs_mode, 
            show=SHOW
        )

        if lateral_distance == -np.inf:
            degree_steering_angle = -10.0
        elif lateral_distance == np.inf:
            degree_steering_angle = 10.0
        else:
            distance_to_waypoint = (longitudinal_distance + self.gain) ** 2 + lateral_distance ** 2
            degree_steering_angle = math.degrees(math.atan2(2 * self.wheelbase * lateral_distance, distance_to_waypoint))

        # Debug mode image display and logging
        if self.DEBUG:
            BG_DARK      = (24, 22, 20)
            TEXT_PRIMARY = (245, 245, 245)
            TEXT_MUTED   = (170, 170, 170)
            ACCENT_CYAN  = (235, 206, 135)
            ACCENT_RED   = (80, 80, 235)
            BORDER       = (90, 88, 84)
            SEPARATOR    = (60, 58, 56)

            if midpoints is not None:
                posm = midpoints[-1]
                midpoints = midpoints[:-1]
                cv2.drawMarker(line_edges, tuple(posm[::-1]), (255, 255, 255),
                            markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
                for p in midpoints:
                    cv2.circle(line_edges, tuple(p[::-1]), 3, (180, 180, 180), -1)

            PANEL_W, PANEL_H = 540, 360
            resized_image      = cv2.resize(self.cv_image, (PANEL_W, PANEL_H))
            resized_mask       = cv2.resize(mask,           (PANEL_W, PANEL_H))
            resized_line_edges = cv2.resize(line_edges,     (PANEL_W, PANEL_H))

            concatenated_image = np.hstack((
                cv2.cvtColor(resized_image,      cv2.COLOR_BGR2RGB),
                cv2.cvtColor(resized_mask,       cv2.COLOR_GRAY2RGB),
                cv2.cvtColor(resized_line_edges, cv2.COLOR_GRAY2BGR),
            ))
            H, W = concatenated_image.shape[:2]

            for i in (1, 2):
                cv2.line(concatenated_image, (i * PANEL_W, 0), (i * PANEL_W, H), SEPARATOR, 1)

            def draw_panel(img, p1, p2, color=BG_DARK, alpha=0.62, border=BORDER):
                overlay = img.copy()
                cv2.rectangle(overlay, p1, p2, color, -1)
                cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, dst=img)
                if border is not None:
                    cv2.rectangle(img, p1, p2, border, 1)

            def put_text(img, s, org, scale=0.55, color=TEXT_PRIMARY, thick=1):
                cv2.putText(img, s, org, cv2.FONT_HERSHEY_DUPLEX, scale, color, thick, cv2.LINE_AA)

            for i, title in enumerate(("CAMERA", "MASK", "LINE DETECTION")):
                (tw, th), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_DUPLEX, 0.45, 1)
                x, y = i * PANEL_W + 14, 26
                draw_panel(concatenated_image, (x - 8, y - th - 6), (x + tw + 8, y + 6), alpha=0.55)
                put_text(concatenated_image, title, (x, y), scale=0.45)

            current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            lines = [
                ("STEER",     f"{degree_steering_angle:+6.2f} deg", ACCENT_CYAN),
                ("LAT DIST",  f"{lateral_distance:+6.2f} m",        TEXT_PRIMARY),
                ("ECU SPEED", f"{self.ecu_speed:6.2f} km/h",        TEXT_PRIMARY),
                ("TIME",      current_time,                         TEXT_MUTED),
            ]
            pad_x, pad_y, line_h, label_w = 14, 12, 24, 110
            block_w = label_w + 200
            block_h = pad_y * 2 + line_h * len(lines)
            x0, y0 = 12, H - block_h - 12
            draw_panel(concatenated_image, (x0, y0), (x0 + block_w, y0 + block_h))
            for i, (label, value, color) in enumerate(lines):
                ly = y0 + pad_y + line_h * (i + 1) - 6
                put_text(concatenated_image, label, (x0 + pad_x, ly),
                        scale=0.42, color=TEXT_MUTED)
                put_text(concatenated_image, value, (x0 + pad_x + label_w, ly),
                        scale=0.55, color=color)

            if self.stop_sign_detected:
                badge = "STOP SIGN DETECTED"
                (bw, bh), _ = cv2.getTextSize(badge, cv2.FONT_HERSHEY_DUPLEX, 0.5, 1)
                bx2, by1 = W - 14, 14
                bx1, by2 = bx2 - bw - 44, by1 + bh + 14
                overlay = concatenated_image.copy()
                cv2.rectangle(overlay, (bx1, by1), (bx2, by2), ACCENT_RED, -1)
                cv2.addWeighted(overlay, 0.88, concatenated_image, 0.12, 0, dst=concatenated_image)
                cv2.circle(concatenated_image, (bx1 + 16, (by1 + by2) // 2), 5, (255, 255, 255), -1)
                put_text(concatenated_image, badge, (bx1 + 30, by2 - 9),
                        scale=0.5, color=(255, 255, 255))

            frame_path  = os.path.join(self.frames_folder, f"frame_{self.counter}.png")
            output_path = os.path.join(self.output_folder, f"output_{self.counter}.png")
            log_path    = os.path.join(self.logs_folder,   f"log_{self.counter}.txt")

            cv2.imwrite(frame_path, self.cv_image)
            cv2.imwrite(output_path, concatenated_image)
            with open(log_path, "w") as log:
                log.write(
                    f"{self.counter}: lateral_distance={lateral_distance:.4f} "
                    f"longitudinal_distance={longitudinal_distance:.4f} "
                    f"degree_steering_angle={degree_steering_angle:.4f}\n"
                )
            self.counter += 1        # Display concatenated image if SHOW is enabled
        if SHOW:
            resized_image = cv2.resize(self.cv_image, (540, 360))
            resized_mask = cv2.resize(mask, (540, 360))
            resized_line_edges = cv2.resize(line_edges, (540, 360))
            concatenated_image = np.hstack((cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB),
                                            cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2RGB),
                                            cv2.cvtColor(resized_line_edges, cv2.COLOR_GRAY2BGR)))
            cv2.imshow("Denedik :(", concatenated_image)
            cv2.waitKey(1)

        # Publish the steering angle
        steer_msg = Float64()
        steer_msg.data = degree_steering_angle
        self.steer_pub.publish(steer_msg)

        # Log information to the console
        self.get_logger().info(f"Longitudinal Distance: {longitudinal_distance}")
        self.get_logger().info(f"Steering Angle: {degree_steering_angle}")

    def new_method(self):
        self.cv_image
        return 

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Path Planning Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
