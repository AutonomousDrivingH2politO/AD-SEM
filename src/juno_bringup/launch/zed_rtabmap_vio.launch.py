from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

# TF chain: map->odom->zed_camera_link (REP 105)
# map->odom: provided by global EKF (GPS-fused)
# odom->zed_camera_link: provided by local EKF
# rtabmap: costmap/grid generation only (publish_tf disabled)
def generate_launch_description() -> LaunchDescription:

    juno_bringup_dir = get_package_share_directory('juno_bringup')
    config_dir = os.path.join(
        juno_bringup_dir,
        'launch',
        'gps_rl_config'
    )
    ekf_local_params_file = os.path.join(config_dir, 'ekf_local_node.yaml')

    rviz_cfg = PathJoinSubstitution(
        [
            FindPackageShare("juno_bringup"),
            "launch",
            "rviz_config",
            "zed_rtabmap_rgbd.rviz",
        ]
    )
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("juno_bringup"), # Change to your actual package
            "launch",
            "urdf_config",                           # Change to your urdf folder
            "zed_base_link.urdf.xacro",               # Change to your actual file name
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command(['xacro ', urdf_path, ' use_zed_localization:=false']),
                value_type=str
            )
        }],
    )
    # Disable VIO’s publication of odom->camera and instead publish nav_msgs/Odometry of the VIO’s pose solution for fusion. 
    # By default the zed_wrapper publishes this under the odom topic, but it is recommended to remap this to a non-reserved topic name (for example, camera_odom).
    # remappings=[('odom', 'camera_odom')]
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
            "publish_tf": "false",
            "base_frame": "base_link",
        }.items(),
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
            "args": "--delete_db_on_start --Rtabmap/DetectionRate 5.00",
            "frame_id": "base_link",
            "odom_topic": "/zed/zed_node/odom",
            "rgb_topic": "/zed/zed_node/rgb/color/rect/image",
            "visual_odometry": "false",
            "publish_tf": "true",
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
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[ekf_local_params_file,{"use_sim_time": False}],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered/local'), # In case you want to change filtered odom name change second; not recommended because other nodes depend on it
        ],
    )



    return LaunchDescription(
        [
            robot_state_publisher_node,
            zed_camera_launch,
            imu_mag_fuse,
            ekf_local,
            #rtabmap_launch,
        ]
    )


