from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from om_common.launch.sensor_launch import get_sensor_launch


def generate_launch_description():
    """
    Generate the launch description for TRON robot sensors.
    """
    d435_camera_ahead = LaunchConfiguration(
        "d435_camera_ahead",
        default=EnvironmentVariable("D435_CAMERA_AHEAD", default_value="0"),
    )
    d435_camera_height = LaunchConfiguration(
        "d435_camera_height",
        default=EnvironmentVariable("D435_CAMERA_HEIGHT", default_value="0.85"),
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
        if not (hasattr(e, "node_name") and e.node_name == "d435_obstacle_dector")
    ]

    entities.extend(
        [
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
                package="tron_sdk",
                executable="tron_odom",
                name="tron_odom",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="tron_sdk",
                executable="cmd_vel_to_tron",
                name="cmd_vel_to_tron",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=UnlessCondition(use_sim),
            ),
        ]
    )

    return LaunchDescription(entities)
