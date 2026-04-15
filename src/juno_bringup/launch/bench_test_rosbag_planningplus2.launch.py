from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Fake TF tree
    static_tf_map_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}], # Added sim time
    )
    
    static_tf_base_cam = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["1.1", "0", "1", "0", "0", "0", "base_link", "zed_camera_link"],
        parameters=[{"use_sim_time": True}], # Added sim time
    )

    # Bag
    bag_play = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "trap 'kill %1; exit' INT TERM; "
            "ros2 bag play "
            "/home/bylogix/rosbag/ros-image-extract/only_imaged_bag "
            "--remap /zed/zed_node/rgb/image_rect_color:=/zed/zed_node/rgb/color/rect/image "
            "--rate 0.5 "
            "--clock"
            "& wait"
        ],
        output="screen",
    )

    # Fake odom
    fake_odom = Node(
        package="ad_juno", executable="fake_odom_node",
        name="fake_odom_node", output="screen",
        parameters=[{"use_sim_time": True}], # Added sim time

    )

    # perception->path_planning_plus2->nav2->obs_avoidance_node pipline
    seg_node = Node(
        package="ad_juno", executable="seg_node",
        name="seg_node", output="screen",
        parameters=[{"use_sim_time": True}], # Added sim time
    )
    path_planning = Node(
        package="ad_juno", executable="path_planning_plus2",
        name="path_planning_plus2", output="screen",
        parameters=[{"debug": True}, {"use_sim_time": True}],
    )
    obstacle_avoidance = Node(
        package="ad_juno", executable="obstacle_avoidance",
        name="obstacle_avoidance", output="screen",
        parameters=[{"use_sim_time": True}], # Added sim time

    )
    steering_node = Node(
        package="ad_juno", executable="steering_node",
        name="steering_node", output="screen",
        parameters=[{"debug": True }, {"use_sim_time": True}],
    )
    can_node = Node(
        package="ad_juno", executable="can_node_resurrections",
        name="can_node", output="screen",
        parameters=[{"use_sim_time": True}], # Added sim time
    )

    return LaunchDescription([
        # TF
        static_tf_map_odom,
        static_tf_base_cam,
        # Input
        bag_play,
        fake_odom,
        # Perception → Planning
        seg_node,
        path_planning,
        # Nav2 cmd_vel → steering angle → actuator
        obstacle_avoidance,
        steering_node,
        can_node,
        # NOTE: throttle_node excluded — don't want the car driving on the bench
    ])