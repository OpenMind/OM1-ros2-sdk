from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    go2_battery_monitor_node = Node(
        package="go2_auto_dock",
        executable="go2_battery_monitor",
        name="go2_battery_monitor",
        output="screen",
    )
    return LaunchDescription(
        [
            go2_battery_monitor_node,
        ]
    )
