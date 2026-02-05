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
    d435_camera_ahead = LaunchConfiguration(
        "d435_camera_ahead",
        default=EnvironmentVariable("D435_CAMERA_AHEAD", default_value="0"),
    )
    d435_camera_height = LaunchConfiguration(
        "d435_camera_height",
        default=EnvironmentVariable("D435_CAMERA_HEIGHT", default_value="1.05"),
    )
    d435_tilt_angle = LaunchConfiguration(
        "d435_tilt_angle",
        default=EnvironmentVariable("D435_TILT_ANGLE", default_value="35.0"),
    )
    d435_obstacle_threshold = LaunchConfiguration(
        "d435_obstacle_threshold",
        default=EnvironmentVariable("D435_OBSTACLE_THRESHOLD", default_value="0.10"),
    )
    use_sim = LaunchConfiguration(
        "use_sim",
        default=EnvironmentVariable("USE_SIM", default_value="false"),
    )

    entities = get_sensor_launch()

    entities = [
        e
        for e in entities
        if not (
            isinstance(e, Node)
            and hasattr(e, "_Node__node_name")
            and e._Node__node_name in ["static_transform_publisher_camera", "rplidar_node", "d435_obstacle_dector"]
        )
    ]

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
                    "0.610865",
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
                package="om_common",
                executable="d435_obstacle_dector",
                name="d435_obstacle_dector",
                parameters=[
                    {
                        "camera_ahead": d435_camera_ahead,
                        "camera_height": d435_camera_height,
                        "tilt_angle": d435_tilt_angle,
                        "obstacle_threshold": d435_obstacle_threshold,
                    }
                ],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan_node',
                remappings=[('cloud_in', '/utlidar/cloud_livox_mid360')],
                parameters=[{
                    'target_frame': 'livox_frame',
                    'transform_tolerance': 0.01,
                    'min_height': 0.05,
                    'max_height': 0.8,
                    'angle_min': -3.14159,
                    'angle_max': 3.14159,
                    'angle_increment': 0.00436,
                    'scan_time': 0.05,
                    'range_min': 0.15,
                    'range_max': 30.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0,
                }],
            ),
        ]
    )

    return LaunchDescription(entities)
