#!/usr/bin/env python3.10
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist
import math
import tf2_ros
import rclpy

from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float64, Header

from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration

import matplotlib.pyplot as plt
import cv2
from shared_objects.ROS_utils import Topics, SHOW



class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        topics = Topics()
        self.topic_names = topics.topic_names
        self.wheelbase = 1.6
        self.pos_x=0
        self.pos_y=0
        self.heading=0
        self.steer_pub = self.create_publisher(Float64, self.topic_names["steering"], 10) # used queue size 10 but need to check for trade-off
        #self.map_pub=self.create_publisher('myOccupancyMap',OccupancyGrid,10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        # Uncomment if you want path‐based steering:
        # self.create_subscription(
        #     Path,
        #     '/planner/move_base/TebLocalPlannerROS/local_plan',
        #     self.callback_path,
        #     10)

        # Uncomment to process projected map:
        # self.create_subscription(
        #     OccupancyGrid,
        #     '/rtabmap/grid_map',
        #     self.callback_projmap,
        #     10)

        # Always subscribe to cmd_vel
        self.create_subscription(Twist,'/planner/cmd_vel', self.callback_cmd, 10)

        self.get_logger().info('ObstacleAvoidance node started.')

    def locate_origin(self, data: OccupancyGrid, transform):
        """
        -- Locate Origin --
        
        Locate cell index nearest to car position, by doing location/resolution:
        [m]/[m/cell]
        """
        
        x_idx = round((data.info.origin.position.x + transform.transform.translation.x) / data.info.resolution) # check the 180 deg rotation
        y_idx = round((data.info.origin.position.y + transform.transform.translation.y) / data.info.resolution)
        return int(x_idx), int(y_idx)
        
    def get_direction (self,start_x, start_y, target_x, target_y, heading_angle):
        start_x = abs(start_x)
        start_y = abs(start_y)
        target_x = abs(target_x)
        target_y = abs(target_y)
        
        d_x=target_x-start_x # + because they're negative
        d_y=target_y-start_y
        self.get_logger().info(f'x distance: {d_x}')
        self.get_logger().info(f'y distance: {d_y}')
        
        direction_angle = math.degrees(math.atan2(d_y,abs(d_x)))  #angle in degrees
        steer_angle = math.degrees(heading_angle)-direction_angle

        if steer_angle < -180:
            steer_angle = 360+(steer_angle) 
            #self.get_logger().info(steer_angle)
        elif steer_angle > 180:
            steer_angle = steer_angle-360


        self.get_logger().info(f'heading angle: {math.degrees(heading_angle)}')
        self.get_logger().info(f'direction angle:{direction_angle}')
        self.get_logger().info(f'steer angle: {steer_angle}')

        return steer_angle
    
    def euler_from_quaternion(self,x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.degrees(math.atan2(t0, t1))
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.degrees(math.asin(t2))
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.degrees(math.atan2(t3, t4))
        
            return roll_x, pitch_y, yaw_z

    def callback_position(self,msg : OccupancyGrid):
    # locate car in map
        try:
            transform = self.tf_buffer.lookup_transform('map','camera_link',
                                                        Time()
                                                        ,Duration(seconds=1.0)) 
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            self.steer_pub.publish(0.0)
            return

        x=transform.transforms.rotation.x
        y=transform.transforms.rotation.y
        z=transform.transforms.rotation.z
        w=transform.transforms.rotation.w
        _,_,self.heading=self.euler_from_quaternion(x,y,z,w)
        
        self.pos_x, self.pos_y = self.locate_origin(msg, transform)

    def callback_path(self,path):
        objective_x=path[0].poses.pose.position.x/0.05
        objective_y=path[0].poses.pose.position.y/0.05
        if not path.poses:
            return

        
        steering_angle=self.get_direction(self.pos_x,self.pos_y,objective_x,objective_y,self.heading)
        msg=Float64()
        msg.data=steering_angle
        self.steer_pub.publish(msg)
        pass

    def convert_trans_rot_vel_to_steering_angle(self,v, omega, wheelbase):
        if omega == 0 or v == 0:
            return 0

        radius = v / omega
        return math.atan(wheelbase / radius)

    def callback_cmd(self,data):
        v=data.linear.x
        omega=data.angular.z
        steering_angle=self.convert_trans_rot_vel_to_steering_angle(v,omega,self.wheelbase)
        
        msg=Float64()
        msg.data=math.degrees(steering_angle)
        self.steer_pub.publish(msg)

    def fill_holes(self,map):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8,8))
        res_morph = cv2.morphologyEx(map, cv2.MORPH_CLOSE, kernel)
        return res_morph

    def callback_projmap(self,data):
        map=data.data
        info=data.info
        self.logger().info(f"{info.resolution=}\n{info.width=}\n{info.height=}\n")
        self.logger().info("---------------------------------------------------")
        new_map=self.fill_holes(map)
        #plt.imshow(new_map)
        header=Header()
        msg=OccupancyGrid()
        msg.header=header
        msg.info=info
        msg.data=new_map
        #map_pub.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Path Planning Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
