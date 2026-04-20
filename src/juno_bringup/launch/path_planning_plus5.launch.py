"""Live on-car launch for plus5.

Brings up the full stack: ZED + RTABMAP, ublox GPS, robot_localization EKF
(local + global) + navsat_transform, Nav2 with the RPP controller YAML, the
ad_juno perception + planning + actuation pipeline using throttle_node_v2.
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
NAV2_YAML = "/home/bylogix/AD-SEM/src/juno_bringup/launch/costmap_configs/nav2_rpp_controller.yaml"


def generate_launch_description() -> LaunchDescription:
    juno_share = get_package_share_directory("juno_bringup")
    gps_cfg_dir = os.path.join(juno_share, "launch", "gps_rl_config")
    ekf_local_yaml = os.path.join(gps_cfg_dir, "ekf_local_node.yaml")
    ekf_global_yaml = os.path.join(gps_cfg_dir, "ekf_global_node.yaml")
    navsat_yaml = os.path.join(gps_cfg_dir, "navsat_transform_node.yaml")

    ublox_share = get_package_share_directory("ublox")
    ublox_yaml = os.path.join(ublox_share, "ublox_gps", "config", "use_existing.yaml")

    # Sensors + SLAM (zed_wrapper + rtabmap)
    zed_rtab = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("juno_bringup"), "launch", "zed_rtabmap_vio.launch.py",
            ])
        ),
    )

    # ublox GPS driver with pre-configured receiver params
    ublox_node = Node(
        package="ublox_gps", executable="ublox_gps_node",
        name="ublox_gps_node", output="both",
        parameters=[ublox_yaml, {"use_sim_time": USE_SIM}],
    )

    # robot_localization inlined here because robot_localization.launch.py hardcodes
    # use_sim_time=True, which would hang all EKF nodes on the real car with no /clock.
    ekf_local = Node(
        package="robot_localization", executable="ekf_node", name="ekf_local_node",
        output="screen",
        parameters=[ekf_local_yaml, {"use_sim_time": USE_SIM}],
        remappings=[("/odometry/filtered", "/odometry/filtered/local")],
    )
    navsat = Node(
        package="robot_localization", executable="navsat_transform_node",
        name="navsat_transform_node", output="screen",
        parameters=[navsat_yaml, {"use_sim_time": USE_SIM}],
        remappings=[
            ("imu", "/zed/zed_node/imu/data"),
            ("/gps/fix", "/ublox_gps_node/fix"),
            ("/odometry/filtered", "/odometry/filtered/local"),
        ],
    )
    ekf_global = Node(
        package="robot_localization", executable="ekf_node", name="ekf_global_node",
        output="screen",
        parameters=[ekf_global_yaml, {"use_sim_time": USE_SIM}],
        remappings=[("/odometry/filtered", "/odometry/filtered/global")],
    )

    # Static TFs (mirrors robot_localization.launch.py + base→camera mount)
    gps_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="gps_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "zed_camera_link", "gps"],
    )
    imu_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="zed_imu_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "zed_camera_link", "zed_imu_link"],
    )
    base_to_zed_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_link_to_zed",
        arguments=["1.1", "0", "1", "0", "0", "0", "base_link", "zed_camera_link"],
    )

    # Nav2 with our RPP controller YAML
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py",
            ])
        ),
        launch_arguments={
            "params_file": NAV2_YAML,
            "use_sim_time": str(USE_SIM).lower(),
        }.items(),
    )

    # ad_juno pipeline
    seg = Node(
        package="ad_juno", executable="seg_node", name="seg_node", output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    # TEMP: navsat_transform_node publishes filtered NavSatFix on /gps/filtered; plus5
    # subscribes to gps/filtered/fix. Remap until one side is renamed.
    plus5 = Node(
        package="ad_juno", executable="path_planning_plus5", name="path_planning_plus5",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
        remappings=[("gps/filtered/fix", "/gps/filtered")],
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
        gps_tf, imu_tf, base_to_zed_tf,
        zed_rtab,
        ublox_node,
        ekf_local, navsat, ekf_global,
        nav2,
        seg, plus5,
        bridge, steering, can, throttle,
    ])
