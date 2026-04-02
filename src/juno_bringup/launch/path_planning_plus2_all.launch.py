from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("juno_bringup"), "launch", "rviz_config", "zed_rtabmap_rgbd.rviz",
    ])

    # ZED camera 
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py",
            ])
        ),
        launch_arguments={
            "camera_model": "zed2i",
            "publish_map_tf": "false",
            "publish_imu_tf": "true",
            "publish_tf": "true",
        }.items(),
    )

    # RTABMAP (mapping / costmap only, no TF)
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
        }.items(),
    )

    # Nav2 controller 
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py",
            ])
        ),
        launch_arguments={
            "params_file": "/home/bylogix/AD-SEM/src/juno_bringup/launch/costmap_configs/nav2_mppi_controller.yaml",
            "use_sim_time": "false",
        }.items(),
    )

    ad_juno_nodes = [
        Node(package="ad_juno", executable=exe, name=name, output="screen")
        for exe, name in [
            ("path_planning_plus2", "path_planning_plus2"),
            ("obstacle_avoidance", "obstacle_avoidance"),
            ("steering_node", "steering_node"),
            ("seg_node", "seg_node"),
            ("can_node_resurrections", "can_node"),
            ("throttle_node", "throttle_node"),
        ]
    ]

    return LaunchDescription([
        zed_camera,
        rtabmap,
        nav2,
        *ad_juno_nodes,
    ])