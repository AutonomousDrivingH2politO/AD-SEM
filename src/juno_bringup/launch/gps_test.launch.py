import os

import ament_index_python.packages
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Configuration Paths (Using ament_index_python as requested) ---
    # Get the share directory for the 'ublox_gps' package
    ublox_share_dir = ament_index_python.packages.get_package_share_directory('ublox_gps')
    config_dir = os.path.join(ublox_share_dir, 'config')

    # Define paths to the specific YAML files
    # 1. GPS Config (The one that works!)
    ublox_params_file = os.path.join(config_dir, 'zed_f9p.yaml')
    
    # 2. Localization Configs
    ekf_params_file = os.path.join(config_dir, 'ekf_gps_node.yaml')
    navsat_params_file = os.path.join(config_dir, 'navsat_gps.yaml')

    # --- 2. Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- 3. Node Definitions ---

    # 3.1. U-blox GPS Driver Node
    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='both',  # As requested in your snippet
        parameters=[
            ublox_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    

    # 3.2. EKF Localization Node (Robot Localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            #('odometry/filtered', 'odom')
        ]
    )

    # 3.3. Navsat Transform Node (Robot Localization)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            navsat_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('gps/fix', '/ublox_gps_node/fix'),
            ('imu', 'gps/navheading')
            #('odometry/filtered', 'odom')
        ]
    )

    # 3.4. Event Handler (Shutdown if GPS disconnects)
    gps_shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ublox_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    # --- 4. Return Launch Description ---
    return LaunchDescription([
        use_sim_time_arg,
        gps_shutdown_handler,
        
        # Nodes
        ublox_node,
        ekf_node,
        navsat_transform_node,
    ])