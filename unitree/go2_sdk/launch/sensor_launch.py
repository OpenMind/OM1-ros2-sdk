from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    EnvironmentVariable,
    LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import Node

from om_common.launch.sensor_launch import get_sensor_launch


def generate_launch_description():
    """
    Generate the launch description for Go2 robot sensors.
    """
    # Re-define LaunchConfigurations to use in conditions
    use_sim = LaunchConfiguration(
        "use_sim",
        default=EnvironmentVariable("USE_SIM", default_value="false"),
    )
    go2_camera_stream_enable = LaunchConfiguration(
        "go2_camera_stream_enable",
        default=EnvironmentVariable("GO2_CAMERA_STREAM_ENABLE", default_value="true"),
    )

    entities = get_sensor_launch()

    entities = [
        e
        for e in entities
        if not (
            isinstance(e, Node)
            and hasattr(e, "_Node__node_name")
            and e._Node__node_name == "static_transform_publisher_camera"
        )
    ]

    entities.append(
        DeclareLaunchArgument(
            "go2_camera_stream_enable",
            default_value=go2_camera_stream_enable,
            description="Enable or disable the go2_camera_stream node (can be set via GO2_CAMERA_STREAM_ENABLE environment variable)",
        )
    )

    entities.extend(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher_camera",
                arguments=[
                    "--x",
                    "0.2",
                    "--y",
                    "0",
                    "--z",
                    "0.10",
                    "--roll",
                    "0",
                    "--pitch",
                    "0.436332313",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "camera_link",
                ],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="odom_relay",
                arguments=["/utlidar/robot_odom", "/odom"],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="go2_sdk",
                executable="go2_camera_stream",
                name="go2_camera_stream",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=IfCondition(
                    AndSubstitution(go2_camera_stream_enable, NotSubstitution(use_sim))
                ),
            ),
        ]
    )

    return LaunchDescription(entities)
