"""Live on-car launch for plus3.

plus3 runs direct-steer lane following. When /local_costmap/costmap shows any
occupied cell, plus3 publishes /goal_pose for Nav2 instead, and
old_obstacle_avoidance bridges /cmd_vel -> steering/throttle.

Localization: ZED VIO (odom) + (occupancy grid) only. map->odom is a
static identity since there's no global anchor to align.
"""

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

    # Sensors + Nav2 (zed_wrapper publishes odom TF; publishes occ grid)
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("juno_test"),
                    "launch",
                    "zed_vio.launch.py",
                ]
            )
        ),
    )

    # ad_juno pipeline
    seg = Node(
        package="ad_juno",
        executable="seg_node",
        name="seg_node",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    plus3 = Node(
        package="ad_juno",
        executable="path_planning_plus3",
        name="path_planning_plus3",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    bridge = Node( # NEW OBSTACLE AVOIDANCE
        package="ad_juno",
        executable="old_obstacle_avoidance",
        name="obstacle_avoidance",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    steering = Node(
        package="ad_juno",
        executable="steering_node",
        name="steering_node",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    can = Node(
        package="ad_juno",
        executable="can_node_resurrections",
        name="can_node",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    throttle = Node(
        package="ad_juno",
        executable="throttle_node_v2",
        name="throttle_node",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )

    return LaunchDescription(
        [
            zed,
            seg,
            plus3,
            bridge,
            steering,
            can,
            throttle,
        ]
    )
