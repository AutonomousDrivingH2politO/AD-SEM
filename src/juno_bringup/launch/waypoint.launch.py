from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ublox_dir = get_package_share_directory('ublox')
    gps_config_path = os.path.join(
        ublox_dir, 
        'ublox_gps', 
        'config', 
        'rover.yaml'
    )
    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='both',  # As requested in your snippet
        parameters=[gps_config_path], 
    )
    waypoints = Node(
        package='ad_juno',
        executable='waypoints',
        name='WaypointRecorder',
        output='screen',
    )
    return LaunchDescription([
        ublox_node,
        waypoints,
    ])

