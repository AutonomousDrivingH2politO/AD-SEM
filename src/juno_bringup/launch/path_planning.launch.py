import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command

USE_SIM = False

SEG_MODE = "road_lane" # Both road and lanes, for only road make it road

def generate_launch_description() -> LaunchDescription:
    force_color = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')
    juno_share = get_package_share_directory("juno_bringup")
    nav2_costmap_yaml = os.path.join(
        juno_share, "launch", "costmap_configs", "nav2_local_costmap.yaml"
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
                    "publish_map_tf": "true",
                    "publish_imu_tf": "true",
                    "publish_tf": "true",
                    "enable_gnss": "false",
                    "gnss_antenna_offset": "[-0.73,0,0]"
                }.items(),
            )
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

    
    # The standalone executable always comes up as "costmap" in the
    # "costmap" namespace, i.e. fully-qualified node /costmap/costmap.
    costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        output="screen",
        parameters=[nav2_costmap_yaml],
    )

    # node_names must use the fully-qualified path, otherwise the manager
    # waits forever on /costmap/get_state (which does not exist).
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["costmap/costmap"],
                "bond_timeout": 0.0,
            }
        ],
    )
    # ad_juno pipeline
    seg = Node(
        package="ad_juno", executable="seg_node_plus1", name="seg_node_plus1", output="screen",
        parameters=[{"use_sim_time": USE_SIM, "segmentation_mode": SEG_MODE}],
    )
    stop = Node(
        package="ad_juno", executable="stop_node", name="stop_node", output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    base_path= Node(
        package="ad_juno", executable="path_planning", name="path_planning",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    steering = Node(
        package="ad_juno", executable="steering_node", name="steering_node",
        output="screen", parameters=[{"use_sim_time": USE_SIM}],
    )
    can = Node(
        package="ad_juno", executable="can_node_resurrections", name="can_node",
        output="screen", parameters=[{"use_sim_time": USE_SIM}],
    )
    throttle = Node(
        package="ad_juno", executable="throttle_node_v2", name="throttle_node",
        output="screen", parameters=[{"use_sim_time": USE_SIM}],
    )

    return LaunchDescription([
        force_color ,
        zed_camera_launch,
        costmap , lifecycle_manager,
        robot_state_publisher_node,
        stop,
        seg, base_path,
        steering, can, throttle,
    ])

