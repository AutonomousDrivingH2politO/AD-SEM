import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    juno_bringup_dir = get_package_share_directory('juno_bringup')
    config_dir = os.path.join(
        juno_bringup_dir, 
        'launch', 
        'gps_rl_config'
    )
    #ublox_dir = get_package_share_directory('ublox')
    #gps_config_path = os.path.join(
    #    ublox_dir, 
    #    'ublox_gps', 
    #    'config'
    #)
    #ublox_params_file = os.path.join(gps_config_path, 'use_existing.yaml')
    ekf_local_params_file = os.path.join(config_dir, 'ekf_local_node.yaml')
    ekf_global_params_file = os.path.join(config_dir, 'ekf_global_node.yaml')
    navsat_params_file = os.path.join(config_dir, 'navsat_transform_node.yaml')
    """
    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='both', 
        parameters=[ublox_params_file],
    )
    """ 
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[ekf_local_params_file,{"use_sim_time": True}],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered/local'), # In case you want to change filtered odom name change second; not recommended because other nodes depend on it 
        ],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_params_file,{"use_sim_time": True}],
        remappings=[
            ('imu', '/zed/zed_node/imu/data'),  #change the second parameter with imu topic if different
            ('/gps/fix', '/ublox_gps_node/fix'), # change the second parameter with your gps fix topic if different
            ('/odometry/filtered', '/odometry/filtered/local'), # In case of name or odom source change, change second to preferred odom
        ],
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_params_file,{"use_sim_time": True}],
        remappings=[
            # Global fused odom output
            ('/odometry/filtered', '/odometry/filtered/global'),
        ],
    )

    # Static TF from zed_camera_link (base) to GPS antenna, should be adjusted accordingly to the position of gps antenna to the zed
    # At the last line of argument, first frame is zed_camera_link (parent) and second is gps_link (childm) which can be found with ros2 topic echo /gps/fix (navsatfix message) frame_id field
    gps_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="gps_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "zed_camera_link", "gps"],
    )
    imu_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="zed_imu_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "zed_camera_link", "zed_imu_link"],
    )
    base_to_zed_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_link_to_zed",
        arguments=["1.1", "0", "1", "0", "0", "0", "base_link", "zed_camera_link"],
    )
    return LaunchDescription([
        gps_tf,
        imu_tf,
        base_to_zed_tf,
        ekf_local,
        ekf_global,
        navsat
    ])
