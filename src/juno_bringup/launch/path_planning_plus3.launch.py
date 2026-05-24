"""Live on-car launch for plus3.

plus3 runs direct-steer lane following. When /local_costmap/costmap shows any
occupied cell, plus3 publishes /goal_pose for Nav2 instead, and
old_obstacle_avoidance bridges /cmd_vel -> steering/throttle.

Localization: ZED VIO (odom) + rtabmap (occupancy grid) only. map->odom is a
static identity since there's no global anchor to align.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


USE_SIM = False


def generate_launch_description() -> LaunchDescription:
    force_color = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')
    juno_share = get_package_share_directory("juno_bringup")
    nav2_yaml = os.path.join(
        juno_share, "launch", "costmap_configs", "nav2_rpp_controller.yaml"
    )

    zed_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("juno_bringup"), "launch", "zed_localization.launch.py",
            ])
        ),
    )

    # Nav2 with RPP controller YAML
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py",
            ])
        ),
        launch_arguments={
            "params_file": nav2_yaml,
            "use_sim_time": str(USE_SIM).lower(),
        }.items(),
    )

    # ad_juno pipeline
    seg = Node(
        package="ad_juno", executable="seg_node", name="seg_node", output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    plus3 = Node(
        package="ad_juno", executable="path_planning_plus3", name="path_planning_plus3",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    masker = Node(
        package="ad_juno", executable="masker_plus2_node", name="point_cloud_masker",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    bridge = Node(
        package="ad_juno", executable="obstacle_avoidance", name="obstacle_avoidance",
        output="screen", parameters=[{"use_sim_time": USE_SIM}],
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
        #zed_localization,
        #nav2,
        seg, plus3,
        masker,
        bridge, #steering, can, throttle,
        force_color ,
    ])
