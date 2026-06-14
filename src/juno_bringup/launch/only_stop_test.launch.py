from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
 
USE_SIM = False
 
 
def generate_launch_description() -> LaunchDescription:
    force_color = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")
 
    can = Node(
        package="ad_juno", executable="can_node_resurrections", name="can_node",
        output="screen", parameters=[{"use_sim_time": USE_SIM}],
    )
    throttle = Node(
        package="ad_juno", executable="throttle_node_v2", name="throttle_node_v2",
        output="screen", parameters=[{"use_sim_time": USE_SIM}],
    )
    stop = Node(
        package="ad_juno", executable="stop_node_wdepth", name="stop_node_wdepth", output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    zed_camera_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
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
        ]
    )
 
 
    return LaunchDescription([
        force_color,
        can, throttle, stop,
        zed_camera_launch
    ])
 
