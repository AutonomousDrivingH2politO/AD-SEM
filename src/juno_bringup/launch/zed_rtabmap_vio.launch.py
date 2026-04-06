from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap
# TF chain: map->odom->zed_camera_link (REP 105)
# map->odom: provided by global EKF (GPS-fused)
# odom->zed_camera_link: provided by local EKF
# rtabmap: costmap/grid generation only (publish_tf disabled)
def generate_launch_description() -> LaunchDescription:
    rviz_cfg = PathJoinSubstitution(
        [
            FindPackageShare("juno_bringup"),
            "launch",
            "rviz_config",
            "zed_rtabmap_rgbd.rviz",
        ]
    )

    zed_camera_launch = IncludeLaunchDescription(
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
            "publish_map_tf": "false",
            "publish_imu_tf": "true",
            "publish_tf": "true",
            "svo_path": "/home/bylogix/Downloads/HD720_SN39159302_11-55-11.svo2",
        }.items(),
    )

    rtabmap_launch = GroupAction([
        SetRemap("goal", "/rtabmap/goal_internal"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("rtabmap_launch"),
                        "launch",
                        "rtabmap.launch.py",
                    ]
                )
            ),
            launch_arguments={
            # Run rtabmap_slam in "rtabmap" namespace so mapping topics
            # appear under /rtabmap (e.g., /rtabmap/map).
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
        }.items(),
        ),
    ])

    return LaunchDescription(
        [
            zed_camera_launch,
            rtabmap_launch,
        ]
    )
