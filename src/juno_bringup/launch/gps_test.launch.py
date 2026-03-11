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
        'gps_test_config'
    )

    ekf_params_file = os.path.join(config_dir, 'ekf_gps_node.yaml')
    navsat_params_file = os.path.join(config_dir, 'navsat_gps.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # ---  Node Definitions ---

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
        remappings=[]
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
            ('/gps/fix', '/ublox_gps_node/fix'),
            ('/imu/data', '/gps/navheading')
        ]
    )

    # --- 4. Return Launch Description ---
    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
        navsat_transform_node,
    ])