import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    juno_bringup_dir = get_package_share_directory("juno_bringup")
    config_dir = os.path.join(juno_bringup_dir, "launch", "gps_rl_config")
    ublox_gps_path = get_package_share_directory("ublox_gps")
    rover_node_launch = os.path.join(ublox_gps_path, "launch", "rover-launch.py")
    ekf_local_params_file = os.path.join(config_dir, "ekf_local_node.yaml")
    ekf_global_params_file = os.path.join(config_dir, "ekf_global_node.yaml")
    navsat_params_file = os.path.join(config_dir, "navsat_transform_node.yaml")

    rover_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(rover_node_launch))

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[ekf_local_params_file],
        remappings=[
            (
                "/odometry/filtered",
                "/odometry/filtered/local",
            ),  # In case you want to change filtered odom name change second;
            # not recommended because other nodes depend on it
        ],
    )

    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[navsat_params_file],
        remappings=[
            ("imu", "/zed/zed_node/imu/data"),
            (
                "gps/fix",
                "/ublox_gps_node/fix",
            ),  # change the second parameter with navsatfix
            (
                "odometry/filtered",
                "/odometry/global",
            ),  # In case of name or odom source change, change second to preferred odom
            ("odometry/gps", "/odometry/gps"),
        ],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_node",
        output="screen",
        parameters=[ekf_global_params_file],
        remappings=[
            # Global fused odom output
            ("/odometry/filtered", "/odometry/filtered/global"),
        ],
    )

    # Static TF from zed_camera_link (base) to GPS antenna,
    # should be adjusted accordingly to the position of gps antenna to the zed
    # The last line, first frame is zed_camera_link (parent) and second is gps_link (childm)
    # which can be found with ros2 topic echo /gps/fix (navsatfix message) frame_id field

    static_gps_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gps_static_tf",
        arguments=[
            "0.2",
            "0.0",
            "0.5",  # x y z
            "0.0",
            "0.0",
            "0.0",  # roll pitch yaw
            "zed_camera_link",
            "gps",
        ],
    )

    return LaunchDescription([static_gps_tf, rover_node, ekf_local, ekf_global, navsat])
