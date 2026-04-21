"""Full-stack bench test for plus5 via rosbag.

Replays the on-car bag and runs the entire downstream pipeline on it:
robot_localization (EKF local+global + navsat), rtabmap (on bag's ZED topics,
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
NAV2_YAML = "/home/bylogix/AD-SEM/src/juno_bringup/launch/costmap_configs/nav2_rpp_controller.yaml"
USE_SIM = True


def generate_launch_description() -> LaunchDescription:
    sim = {"use_sim_time": USE_SIM}
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("juno_bringup"), "launch", "rviz_config", "zed_rtabmap_rgbd.rviz",
    ])

    bag_play = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "trap 'kill %1; exit' INT TERM; "
            f"ros2 bag play {BAG_PATH} --clock --rate 0.5 & wait",
        ],
        output="screen",
    )

    # base_link → zed_camera_link static mount (EKF + bag /tf_static cover the rest)
    base_to_zed = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_to_zed",
        arguments=["1.1", "0", "1", "0", "0", "0", "base_link", "zed_camera_link"],
        parameters=[sim],
    )

    # robot_localization (the existing launch hardcodes use_sim_time=True, which is
    # correct here — we replay with --clock).
    robot_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("juno_bringup"), "launch", "robot_localization.launch.py",
            ])
        ),
    )

    # rtabmap on the bag's ZED topics (no zed_wrapper — bag provides depth/rgb/imu).
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py",
            ])
        ),
        launch_arguments={
            "namespace": "rtabmap",
            "rviz_cfg": rviz_cfg,
            "args": "--delete_db_on_start",
            "frame_id": "zed_camera_link",
            "odom_topic": "/zed/zed_node/odom",
            "visual_odometry": "false",
            "publish_tf": "false",
            "rgb_topic": "/zed/zed_node/rgb/color/rect/image",
            "depth_topic": "/zed/zed_node/depth/depth_registered",
            "camera_info_topic": "/zed/zed_node/rgb/color/rect/image/camera_info",
            "wait_imu_to_init": "true",
            "imu_topic": "/zed/zed_node/imu/data",
            "rviz": "true",
            "rtabmap_viz": "false",
            "approx_sync": "false",
            "rgbd_sync": "true",
            "approx_rgbd_sync": "false",
            "topic_queue_size": "2",
            "use_sim_time": "true",
        }.items(),
    )

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

    seg = Node(
        package="ad_juno", executable="seg_node", name="seg_node", output="screen",
        parameters=[sim],
    )
    # TEMP: plus5 subscribes to gps/filtered/fix; navsat_transform publishes /gps/filtered.
    plus5 = Node(
        package="ad_juno", executable="path_planning_plus5", name="path_planning_plus5",
        output="screen", parameters=[{"debug": True}, sim],
        remappings=[("gps/filtered/fix", "/gps/filtered")],
    )
    bridge = Node(
        package="ad_juno", executable="obstacle_avoidance", name="obstacle_avoidance",
        output="screen", parameters=[sim],
    )
    steering = Node(
        package="ad_juno", executable="steering_node", name="steering_node", output="screen",
        parameters=[sim],
    )
    can = Node(
        package="ad_juno", executable="can_node_resurrections", name="can_node",
        output="screen", parameters=[sim],
    )

    return LaunchDescription([
        base_to_zed,
        bag_play,
        robot_loc,
        rtabmap,
        #nav2,
        seg, plus5,
        bridge, steering, can,
    ])
