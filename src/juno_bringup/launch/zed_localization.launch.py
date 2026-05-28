from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

# TF chain: map->odom->zed_camera_link (REP 105)
def generate_launch_description() -> LaunchDescription:

    juno_bringup_dir = get_package_share_directory('juno_bringup')
    config_dir = os.path.join(
        juno_bringup_dir,
        'launch',
        'gps_rl_config'
    )
    navsat_params_file = os.path.join(config_dir, 'navsat_transform_node.yaml')

    rviz_cfg = PathJoinSubstitution(
        [
            FindPackageShare("juno_bringup"),
            "launch",
            "rviz_config",
            "ekf_check_config.rviz",
        ]
    )
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("juno_bringup"), # Change to your actual package
            "launch",
            "urdf_config",                           # Change to your urdf folder
            "zed_base_link.urdf.xacro",               # Change to your actual file name
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command(['xacro ', urdf_path, ' use_zed_localization:=true']),
                value_type=str
            )
        }],
    )
    zed_camera_launch = GroupAction(
        actions=[
            SetRemap(src='odom', dst='camera_odom'),
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
                    "publish_map_tf": "true",
                    "publish_imu_tf": "true",
                    "publish_tf": "true",
                    "enable_gnss": "true",
                    "gnss_antenna_offset": "[-0.73,0,0]"
                }.items(),
            )
        ]
    )
    gps_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ublox_gps"), "launch", "rover-launch.py",
                ])
            )
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            gps_launch,
            #imu_mag_fuse,
            zed_camera_launch,
        ]
    )


