"""Full-stack bench test for plus5 via rosbag.

Replays the on-car bag and runs the entire downstream pipeline on it:
robot_localization (EKF local+global + navsat), nav2 (on bag's ZED topics,
no zed_wrapper — the bag already provides them), Nav2 with the RPP controller
YAML, seg_node, plus5, the /cmd_vel bridge, steering_node, and can_node.

throttle_node is intentionally excluded — no valve actuation on the bench.
The bag has no /ecu/speed; can_node will still start but will be silent on
/speed without a real CAN bus.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


BAG_PATH = "/home/bylogix/rosbag/rosbag_toolbox_on_car1_17April"
NAV2_YAML = (
    "/home/bylogix/AD-SEM/src/juno_test/launch/costmap_configs/nav2_rpp_controller.yaml"
)
USE_SIM = True


def generate_launch_description() -> LaunchDescription:
    sim = {"use_sim_time": USE_SIM}
    """
    Need to add and change the config for rviz
    rviz_cfg = PathJoinSubstitution(
        [
            FindPackageShare("juno_test"),
            "launch",
            "rviz_config",
            "zed_rgbd.rviz",
        ]
    )
    """

    bag_play = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "trap 'kill %1; exit' INT TERM; "
            f"ros2 bag play {BAG_PATH} --loop --clock --rate 0.5 & wait",
        ],
        output="screen",
    )

    # base_link → zed_camera_link static mount (EKF + bag /tf_static cover the rest)
    base_to_zed = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_zed",
        arguments=["1.1", "0", "1", "0", "0", "0", "base_link", "zed_camera_link"],
        parameters=[sim],
    )

    # No GPS on the bench → skip ekf_global/navsat. Hold map to odom with identity so
    # Nav2 and plus5 (which transforms into odom for VISION-mode goals) have a valid
    # map → odom → base_link chain.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[sim],
    )

    ekf_local_params = PathJoinSubstitution(
        [
            FindPackageShare("juno_test"),
            "launch",
            "gps_rl_config",
            "ekf_local_node.yaml",
        ]
    )
    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[ekf_local_params, sim],
        remappings=[("/odometry/filtered", "/odometry/filtered/local")],
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

    seg = Node(
        package="ad_juno",
        executable="seg_node",
        name="seg_node",
        output="screen",
        parameters=[sim],
    )
    # TEMP: plus5 subscribes to gps/filtered/fix; navsat_transform publishes /gps/filtered.
    plus5 = Node(
        package="ad_juno",
        executable="path_planning_plus5",
        name="path_planning_plus5",
        output="screen",
        parameters=[{"debug": True}, sim],
        remappings=[("gps/filtered/fix", "/gps/filtered")],
    )
    bridge = Node(
        package="ad_juno",
        executable="obstacle_avoidance",
        name="obstacle_avoidance",
        output="screen",
        parameters=[sim],
    )
    steering = Node(
        package="ad_juno",
        executable="steering_node",
        name="steering_node",
        output="screen",
        parameters=[sim],
    )
    can = Node(
        package="ad_juno",
        executable="can_node_resurrections",
        name="can_node",
        output="screen",
        parameters=[sim],
    )

    return LaunchDescription(
        [
            base_to_zed,
            map_to_odom,
            ekf_local,
            bag_play,
            nav2,
            seg,
            plus5,
            bridge,
            steering,
            can,
        ]
    )
