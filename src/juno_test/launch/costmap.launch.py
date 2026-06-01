# local_costmap_only.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

PARAMS = '/home/bylogix/AD-SEM/src/juno_bringup/launch/costmap_configs/nav2_local_costmap.yaml'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',          
            output='screen',
            parameters=[PARAMS],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['local_costmap'],
                'bond_timeout': 0.0,
            }],
        ),
    ])