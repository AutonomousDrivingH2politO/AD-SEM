#!/usr/bin/env python3

import os
import math
import numpy as np
import cv2
from datetime import datetime

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
from shared_objects.new_utils_path import computing_lateral_distance, processing_mask
from shared_objects.ROS_utils import Topics, SHOW
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64
import tf2_geometry_msgs
from pathlib import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_plus3')

        self.debug = self.declare_parameter('debug', True).value
        self.wheelbase = self.declare_parameter('wheelbase', 1.6).value
        self.gain = self.declare_parameter('gain', 1.1).value

        qos_profile_float = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        qos_profile_image= QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.declare_parameter('debug_root', '/home/bylogix/AD-SEM/DEBUG')
        self.debug_root = Path(self.get_parameter('debug_root').get_parameter_value().string_value)
       
        self.costmap_has_obstacles = False
        self.goal_published = False

        self.bridge = CvBridge()
        self.cv_image = None
        self.counter = 0

        topics = Topics()
        self.topic_names = topics.topic_names

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        #nav2 variables to log 
        self.nav2_v = 0.0          # Stored in m/s for calculations
        self.nav2_v_kmh = 0.0      # Stored in km/h for logging
        self.nav2_omega = 0.0      # Stored in rad/s for logging
        self.nav2_steer_target_deg = 0.0

        self.create_subscription(Image, self.topic_names["segmented_image"], self.image_callback, 1)
        self.create_subscription(Image, '/zed/zed_node/rgb/color/rect/image', self.original_image_callback, 1)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 1)
        
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_log_cb, 1)
        
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.steer_pub = self.create_publisher(Float64, self.topic_names["steering"], 1)
        self.bev_pub = self.create_publisher(Image, "/birds_eye_view", 10)
        
        self._latest_goal = None
        self._goal_timer = self.create_timer(2.0, self._goal_timer_callback)

        if self.debug:
            self.logs_folder, self.output_folder, self.frames_folder = self.set_debug_folders()

        self.get_logger().info("PathPlanningPlus3 initialized (Nav2 action client)")

    #Save Nav2 data and convert to km/h ---
    def _cmd_vel_log_cb(self, msg: Twist):
        self.nav2_v = msg.linear.x
        self.nav2_omega = msg.angular.z # Store omega for logging
        
        # Convert to km/h for logging
        self.nav2_v_kmh = abs(self.nav2_v) * 3.6 
        
        # Calculate steering angle using m/s (otherwise math breaks)
        if abs(self.nav2_v) < 0.3:
            steer_deg = 0.0
        else:
            steer_deg = math.degrees(math.atan(self.wheelbase * self.nav2_omega / self.nav2_v))
        self.nav2_steer_target_deg = max(-10.0, min(10.0, steer_deg))

    def _goal_timer_callback(self):
        if self._latest_goal is None:
            return
        self.goal_pub.publish(self._latest_goal)
        self._latest_goal = None

    def costmap_callback(self, msg: OccupancyGrid):
        data = np.array(msg.data, dtype=np.int8)
        self.costmap_has_obstacles = bool(np.any(data > 0))
        
    def set_debug_folders(self):
        try:
            self.debug_root.mkdir(parents=True, exist_ok=True)
            ts_folder = self.debug_root / (datetime.now().strftime('%Y_%m_%d_%H_%M_%S') + '_plus3')
            ts_folder.mkdir()

            logs  = ts_folder / 'logs'
            out   = ts_folder / 'output'
            frames= ts_folder / 'frames'
            for p in (logs, out, frames):
                p.mkdir()

            self.get_logger().info(f'DEBUG output =>\n  logs:   {logs}\n  output: {out}\n  frames: {frames}')
            return str(logs), str(out), str(frames)

        except Exception as e:
            self.get_logger().error(f'Failed to create debug folders: {e}')
            raise

    def original_image_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")

    def image_callback(self, data):
        if self.cv_image is None:
            return

        mask = self.bridge.imgmsg_to_cv2(data, "mono8")
        line_edges = processing_mask(mask, self.cv_image, show=False)

        birds_eye_msg = self.bridge.cv2_to_imgmsg(line_edges, encoding="mono8")
        self.bev_pub.publish(birds_eye_msg)
        
        lateral_distance, longitudinal_distance, midpoints = computing_lateral_distance(
            line_edges, show=False)

        if SHOW and midpoints is not None:
            posm = midpoints[-1]
            for p in midpoints[:-1]:
                cv2.circle(line_edges, tuple(p[::-1]), 2, (200, 200, 200), 3)
            cv2.circle(line_edges, tuple(posm[::-1]), 2, (255, 255, 255), 5)

        if lateral_distance == -np.inf:
            self.get_logger().warn("Lane lost, emergency steering angle -10.0") 
            degree_steering_angle = -10.0
        elif lateral_distance == np.inf:
            self.get_logger().warn("Lane lost, emergency steering angle 10.0") 
            degree_steering_angle = 10.0
        else:
            degree_steering_angle = None

        if self.costmap_has_obstacles:
            if lateral_distance in (-np.inf, np.inf):
                self.get_logger().warn("Obstacles are present but lane lost")
                return
            
            goal_camera = PoseStamped()
            goal_camera.header.stamp = data.header.stamp  
            goal_camera.header.frame_id = "zed_camera_link"
            goal_camera.pose.position.x = float(longitudinal_distance)
            goal_camera.pose.position.y = float(-lateral_distance)
            goal_camera.pose.orientation.w = 1.0

            try:
                goal_map = self.tf_buffer.transform(goal_camera, "odom",
                                                    timeout=rclpy.duration.Duration(seconds=0.1))
            except Exception as e:
                self.get_logger().warn(f"TF camera->map failed: {e}")
                return

            self._latest_goal = goal_map
            self.goal_published = True
            
        else:
            self.goal_published = False
            self._latest_goal = None
            if degree_steering_angle is None:
                dist = (longitudinal_distance + self.gain) ** 2 + lateral_distance ** 2
                degree_steering_angle = math.degrees(math.atan2(2 * self.wheelbase * lateral_distance, dist))
            steer_msg = Float64()
            steer_msg.data = degree_steering_angle
            self.steer_pub.publish(steer_msg)

        if self.debug:
            self._save_debug(mask, line_edges, lateral_distance,
                             longitudinal_distance, midpoints, degree_steering_angle)    

    def _save_debug(self, mask, line_edges, lateral_distance,
                    longitudinal_distance, midpoints, degree_steering_angle):
        if midpoints is not None:
            posm = midpoints[-1]
            midpoints = midpoints[:-1]
            cv2.circle(line_edges, tuple(posm[::-1]), 2, (255, 255, 255), 5)
            for p in midpoints:
                cv2.circle(line_edges, tuple(p[::-1]), 2, (200, 200, 200), 3)

        resized_image = cv2.resize(self.cv_image, (540, 360))
        resized_mask = cv2.resize(mask, (540, 360))
        resized_line_edges = cv2.resize(line_edges, (540, 360))
        concatenated_image = np.hstack((
            cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB),
            cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2RGB),
            cv2.cvtColor(resized_line_edges, cv2.COLOR_GRAY2BGR)
        ))
        #put black bar and logs to directly to the images
        lat_str = f"Lat: {lateral_distance.2f}m"
        pp_str = f"RPP: {degree_steering_angle:.2f} deg" if degree_steering_angle is not None else "RPP: none"
        nav_str = f"Nav2: {self.nav2_steer_target_deg.2f} deg"
        ctrl_str = "AVTIVE: Nav2" if self.costmap_has_obstacles else "ACTIVE: Pure Pursuit"

        for txt, x, y in [(ctrl_str, 10, 35), (lat_str, 10, 75), (pp_str, 550, 35), (nav_str, 1090, 35)]:
            cv2.putText(concatenated_image, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 5)
            cv2.putText(concatenated_image, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)


        cv2.imwrite(os.path.join(self.frames_folder, f"frame_{self.counter}.png"), self.cv_image)
        cv2.imwrite(os.path.join(self.output_folder, f"output_{self.counter}.png"), concatenated_image)

        log_file = os.path.join(self.logs_folder, f"log_{self.counter}.txt")
        with open(log_file, "w") as log:
            time_str = datetime.now().isoformat()
            
            log.write(f"Frame={self.counter} | time={time_str} | lat={lateral_distance:.3f} lon={longitudinal_distance:.3f} | ")
            
            if self.costmap_has_obstacles:
                log.write(f"ACTIVE_CONTROL=Nav2_Command | nav2_v_kmh={self.nav2_v_kmh:.3f} | nav2_omega={self.nav2_omega:.3f} | nav2_steer_target={self.nav2_steer_target_deg:.3f}\n")
            else:
                log.write(f"ACTIVE_CONTROL=Pure_Pursuit | pure_pursuit_steer={degree_steering_angle:.3f}\n")

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down path_planning_plus3")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
