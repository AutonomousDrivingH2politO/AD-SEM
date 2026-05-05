"""Live on-car launch for plus5.

Brings up the full stack: ZED + Occupancy Grid, ublox GPS, robot_localization EKF
(local + global) + navsat_transform, Nav2 with the RPP controller YAML, the
ad_juno perception + planning + actuation pipeline using throttle_node_v2.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


USE_SIM = False
NAV2_YAML = (
    "/home/bylogix/AD-SEM/src/juno_test/launch/costmap_configs/nav2_rpp_controller.yaml"
)


def generate_launch_description() -> LaunchDescription:
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("juno_test"),
                    "launch",
                    "zed_vio_ekf.launch.py",
                ]
            )
        ),
    )

    # ublox GPS driver with pre-configured receiver params
    gps_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ublox_gps"),
                    "launch",
                    "rover-launch.py",
                ]
            )
        ),
    )
    imu_mag_fuse = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node",
        name="imu_filter",
        output="screen",
        parameters=[
            {
                "use_mag": True,
                "publish_tf": False,  # let robot_localization handle TF
                "do_bias_estimation": False,  # closest to Madgwick zeta:=0.0 behavior
                "do_adaptive_gain": False,
                "gain_acc": 0.01,
                "gain_mag": 0.01,
                "use_sim_time": False,
            }
        ],
        remappings=[
            ("imu/data_raw", "/zed/zed_node/imu/data_raw"),
            ("imu/mag", "/zed/zed_node/imu/mag"),
            ("imu/data", "/imu/data_fused"),
        ],
    )

    robot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("juno_test"),
                    "launch",
                    "robot_localization.launch.py",
                ]
            )
        ),
    )

    # Nav2 with our RPP controller YAML
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_test"),
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

    # ad_juno pipeline
    seg = Node(
        package="ad_juno",
        executable="seg_node",
        name="seg_node",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
    # TEMP: navsat_transform_node publishes filtered NavSatFix on /gps/filtered; plus5
    # subscribes to gps/filtered/fix. Remap until one side is renamed.
    plus5 = Node(
        package="ad_juno",
        executable="path_planning_plus5",
        name="path_planning_plus5",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
        remappings=[("gps/filtered/fix", "/gps/filtered")],
    )
    bridge = Node(
        package="ad_juno",
        executable="obstacle_avoidance",
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
            gps_node,
            imu_mag_fuse,
            robot_localization,
            nav2,
            seg,
            plus5,
            bridge,
            steering,
            can,
            throttle,
        ]
    )
