from launch import LaunchDescription
from launch_ros.actions import Node

# Offline test launch — no hardware required.
# Pipeline: video file → seg_node_plus2 → path_planning_lane
#
# Before running, set the video path in:
#   src/ad_juno/ad_juno/webcam_node.py  →  self.path = "/path/to/Silesia rosbag.mp4"

USE_SIM = True
SEG_MODE = "road_lane"


def generate_launch_description():
    video = Node(
        package="ad_juno",
        executable="video_test",
        name="video_publisher",
        output="screen",
    )

    seg = Node(
        package="ad_juno",
        executable="seg_node_plus2",
        name="seg_node_plus2",
        output="screen",
        parameters=[{
            "use_sim_time": USE_SIM,
            "segmentation_mode": SEG_MODE,
            "debug": True,
        }],
    )

    path_planning = Node(
        package="ad_juno",
        executable="path_planning_lane",
        name="path_planning",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )

    return LaunchDescription([
        video,
        seg,
        path_planning,
    ])
