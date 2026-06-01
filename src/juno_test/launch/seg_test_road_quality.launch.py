
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


USE_SIM = False


def generate_launch_description() -> LaunchDescription:

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("zed_wrapper"),
                    "launch",
                    "zed_camera.launch.py",
                ]
            )
        ),
        launch_arguments={
            "camera_model": "zed2i",
        }.items(),
    )

    # ad_juno pipeline
    seg = Node(
        package="ad_juno",
        executable="seg_node_plus1",
        name="seg_node_plus1",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    return LaunchDescription(
        [
            zed,
            seg,
        ]
    )
