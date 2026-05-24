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
# map->odom: provided by global EKF (GPS-fused)
# odom->zed_camera_link: provided by local EKF
# rtabmap: costmap/grid generation only (publish_tf disabled)
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
    # Disable VIO’s publication of odom->camera and instead publish nav_msgs/Odometry of the VIO’s pose solution for fusion. 
    # By default the zed_wrapper publishes this under the odom topic, but it is recommended to remap this to a non-reserved topic name (for example, camera_odom).
    # remappings=[('odom', 'camera_odom')]
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
                    "enable_gnss": "false",
                    "svo_path": "home/bylogix/Desktop/Morano_SVO_rosbag/HD720_SN39159302_11-55-11.svo2",
                    "gnss_antenna_offset": "[-0.73,0,0]"
                }.items(),
            )
        ]
    )


    return LaunchDescription(
        [
            robot_state_publisher_node,
            #navsat,
            #imu_mag_fuse,
            zed_camera_launch,
        ]
    )


