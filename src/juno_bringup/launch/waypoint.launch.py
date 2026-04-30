from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ublox_gps"), "launch", "rover-launch.py",
            ])
        ),
    )

    waypoint_recorder = Node(
        package="ad_juno", executable="waypoint_el_node", name="waypoint_node", output="screen"
    )

    return LaunchDescription([
        gps_launch,
        waypoint_recorder,
    ])