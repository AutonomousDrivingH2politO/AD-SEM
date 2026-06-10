"""Local costmap bring-up.
Brings up a standalone nav2_costmap_2d node (local costmap), its lifecycle
manager with autostart, the ZED camera wrapper, and robot_state_publisher.
"""

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PARAMS = (
    "/home/bafe/AD-SEM/src/juno_bringup/launch/costmap_configs/nav2_local_costmap.yaml"
)



def generate_launch_description() -> LaunchDescription:
    # URDF for robot_state_publisher (base_link <-> ZED camera transforms)
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("juno_bringup"),
            "launch",
            "urdf_config",
            "zed_base_link.urdf.xacro",
        ]
    )

    # The standalone executable always comes up as "costmap" in the
    # "costmap" namespace, i.e. fully-qualified node /costmap/costmap.
    costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        output="screen",
        parameters=[PARAMS],
    )

    # node_names must use the fully-qualified path, otherwise the manager
    # waits forever on /costmap/get_state (which does not exist).
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["costmap/costmap"],
                "bond_timeout": 0.0,
            }
        ],
    )

    # base_frame=base_link so positional tracking is referenced to the robot
    # base on the ground, not to the camera ~1 m up.
    zed = GroupAction(
        actions=[
            IncludeLaunchDescription(
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
                    #"base_frame": "base_link",
                    "publish_map_tf": "true",
                    "publish_imu_tf": "true",
                    "publish_tf": "true",
                    "enable_gnss": "false",
                    "gnss_antenna_offset": "[-0.73,0,0]",
                }.items(),
            ),
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_path, " use_zed_localization:=true"]),
                    value_type=str,
                )
            }
        ],
    )

    return LaunchDescription(
        [
            costmap,
            lifecycle_manager,
            zed,
            robot_state_publisher,
        ]
    )
