from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
 
 
USE_SIM = False
 
 
def generate_launch_description() -> LaunchDescription:
    force_color = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")
 
    can = Node(
        package="ad_juno", executable="can_node_resurrections", name="can_node",
        output="screen", parameters=[{"use_sim_time": USE_SIM}],
    )
    throttle = Node(
        package="ad_juno", executable="throttle_node", name="throttle_node",
        output="screen", parameters=[{"use_sim_time": USE_SIM}],
    )
    stop = Node(
        package="ad_juno", executable="stop_node", name="stop_node", output="screen",
        parameters=[{"use_sim_time": USE_SIM}],
    )
 
    return LaunchDescription([
        force_color,
        can, throttle, stop,
    ])
 