from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
        # exz
            Node(
                package="ad_juno",
                executable="path_planning",
                name="path_planning",
                output="screen",
            ),
            Node(
                package="ad_juno",
                executable="seg_node",
                name="seg_node",
                output="screen",
            ),
          
            Node(
                package="ad_juno",
                executable="video_test",
                name="yarrak_node",
                output="screen",
            ),
        ]
    )
