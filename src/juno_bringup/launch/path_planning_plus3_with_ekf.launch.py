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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



USE_SIM = False


def generate_launch_description() -> LaunchDescription:
    juno_share = get_package_share_directory("juno_bringup")
    nav2_yaml = os.path.join(
        juno_share, "launch", "costmap_configs", "nav2_rpp_controller.yaml"
    )
    
    
    # ZED with urdf corresponging to ekf requirements with base_link as the parent of the zed camera frame
    zed_ekf_rtab= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("juno_bringup"), "launch", "zed_rtabmap_vio_ekf.launch.py",
            ])
        ),    
    )
    imu_mag_fuse = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': True,
            'publish_tf': False,          # let robot_localization handle TF
            'do_bias_estimation': False,  # closest to Madgwick zeta:=0.0 behavior
            'do_adaptive_gain': False,
            'gain_acc': 0.01,
            'gain_mag': 0.01,
            'use_sim_time': False,
        }],
        remappings=[
            ('imu/data_raw', '/zed/zed_node/imu/data_raw'),
            ('imu/mag',      '/zed/zed_node/imu/mag'),
            ('imu/data',     '/imu/data_fused'),
        ],
    )
    robot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("juno_bringup"), "launch", "robot_localization.launch.py",
            ])
        ),    
    )
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ublox_gps"), "launch", "rover-launch.py",
            ])
        )
    ),
    
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


    seg = Node(
        package="ad_juno", executable="seg_node", name="seg_node", output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    plus3 = Node(
        package="ad_juno", executable="path_planning_plus3", name="path_planning_plus3",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    bridge = Node(
        package="ad_juno", executable="old_obstacle_avoidance", name="obstacle_avoidance",
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
        zed_ekf_rtab,
        imu_mag_fuse,
        robot_localization,
        nav2,
        seg, plus3,
        bridge, steering, can, throttle,
        gps_launch,
    ])
