"""
Launch file for the Unitree Go2 charging system.

This launch file starts the battery monitor which automatically triggers
navigation to the charging station when battery is low.

Usage:
    ros2 launch go2_sdk go2_charge.launch.py
    ros2 launch go2_sdk go2_charge.launch.py use_sim:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Use simulation mode (skips camera publisher, uses sim charger)",
    )

    # Battery monitor node
    go2_battery_monitor_node = Node(
        package="go2_auto_dock",
        executable="go2_battery_monitor",
        name="go2_battery_monitor",
        output="screen",
        parameters=[{"use_sim": LaunchConfiguration("use_sim")}],
    )

    return LaunchDescription(
        [
            use_sim_arg,
            go2_battery_monitor_node,
        ]
    )
