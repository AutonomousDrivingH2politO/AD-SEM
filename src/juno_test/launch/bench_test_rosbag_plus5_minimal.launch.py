"""Minimal bench test for plus5 mode-switching logic.

Plays the rosbag and runs only seg_node + path_planning_plus5. Static TFs stand
in for robot_localization and the ZED mount. No Nav2, no bridge, no actuators
— the goal is to verify /nav_mode transitions and /goal_pose content against
the real GPS/ZED stream in the bag.

Since no navsat_transform is running, plus5's gps/filtered/fix subscription is
remapped directly to the bag's raw /ublox_gps_node/fix. With that bag's
status=0 / stddev ~7 m it should enter VISION ~0.5 s after the first GPS msg.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


BAG_PATH = "/home/bylogix/rosbag/rosbag_toolbox_on_car1_17April"
USE_SIM = True


def generate_launch_description() -> LaunchDescription:
    sim = {"use_sim_time": USE_SIM}

    # Fake TF chain: map → odom → base_link → zed_camera_link (+ gps)
    map_to_odom = Node(
        package="tf2_ros", executable="static_transform_publisher", name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[sim],
    )
    odom_to_base = Node(
        package="tf2_ros", executable="static_transform_publisher", name="odom_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        parameters=[sim],
    )
    base_to_zed = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_to_zed",
        arguments=["1.1", "0", "1", "0", "0", "0", "base_link", "zed_camera_link"],
        parameters=[sim],
    )
    zed_to_gps = Node(
        package="tf2_ros", executable="static_transform_publisher", name="zed_to_gps",
        arguments=["0", "0", "0", "0", "0", "0", "zed_camera_link", "gps"],
        parameters=[sim],
    )

    bag_play = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "trap 'kill %1; exit' INT TERM; "
            f"ros2 bag play {BAG_PATH} --clock --rate 0.5 & wait",
        ],
        output="screen",
    )

    seg = Node(
        package="ad_juno", executable="seg_node", name="seg_node", output="screen",
        parameters=[sim],
    )
    # No navsat in this launch — bind plus5's filtered sub directly to raw ublox.
    plus5 = Node(
        package="ad_juno", executable="path_planning_plus5", name="path_planning_plus5",
        output="screen",
        parameters=[{"debug": True}, sim],
        remappings=[("gps/filtered/fix", "/ublox_gps_node/fix")],
    )

    return LaunchDescription([
        map_to_odom, odom_to_base, base_to_zed, zed_to_gps,
        bag_play,
        seg,
        plus5,
    ])
