from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=['src/ad_juno/config/ekf_local_node.yaml'],
        remappings=[
            ('odometry/filtered', '/odometry/local'), # In case you want to change filtered odom name change second; not recommended because other nodes depend on it
        ],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=['src/ad_juno/config/navsat_transform_node.yaml'],
        remappings=[
            ('imu', '/zed/zed_node/imu/data'),
            ('gps/fix', '/gps/fix'),                     # change the second parameter with navsatfix
            ('odometry/filtered', '/odometry/local'), # In case of name or odom source change, change second to preferred odom
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=['src/ad_juno/config/ekf_global_node.yaml'],
        remappings=[
            # Global fused odom output
            ('odometry/filtered', '/odometry/global'),
        ],
    )

    # Static TF from zed_camera_link (base) to GPS antenna, should be adjusted accordingly to the position of gps antenna to the zed
    # At the last line of argument, first frame is zed_camera_link (parent) and second is gps_link (childm) which can be found with ros2 topic echo /gps/fix (navsatfix message) frame_id field
    static_gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_static_tf',
        arguments=[
            '0.2', '0.0', '0.5',    # x y z
            '0.0', '0.0', '0.0',    # roll pitch yaw
            'zed_camera_link', 'gps_link'
        ],
    )

    return LaunchDescription([
        ekf_local,
        navsat,
        ekf_global,
        static_gps_tf,
    ])
