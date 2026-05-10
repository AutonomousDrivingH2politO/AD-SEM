from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

USE_SIM = True
NAV2_YAML = "/home/bylogix/AD-SEM/src/juno_bringup/launch/costmap_configs/nav2_rpp_controller.yaml"


# TF chain: map->odom->zed_camera_link (REP 105)
# map->odom: provided by global EKF (GPS-fused)
# odom->zed_camera_link: provided by local EKF
def generate_launch_description() -> LaunchDescription:

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("juno_test"),  # Change to your actual package
            "launch",
            "urdf_config",  # Change to your urdf folder
            "zed_base_link.urdf.xacro",  # Change to your actual file name
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_path, " use_zed_localization:=true"]),
                    value_type=str,
                )
            }
        ],
    )
    zed_camera_launch = IncludeLaunchDescription(
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
            "publish_map_tf": "false",
            "publish_imu_tf": "true",
            "publish_tf": "true",
            "base_frame": "base_link",
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            )
        ),
        launch_arguments={
            "params_file": NAV2_YAML,
            "use_sim_time": str(USE_SIM).lower(),
        }.items(),
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            zed_camera_launch,
            nav2,
        ]
    )
