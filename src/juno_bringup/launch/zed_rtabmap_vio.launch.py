from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# This launch file uses ZED VIO to position the frame of the vechile therefore the tf looks like map->odom->camera_link 
# only important thing to preserve is map->odom as specified by REP 105 ROS convention
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
            "publish_tf": "true"
        }.items(),
    )

    rtabmap_launch = IncludeLaunchDescription(
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
    )

    return LaunchDescription(
        [
            zed_camera_launch,
            rtabmap_launch,
        ]
    )
