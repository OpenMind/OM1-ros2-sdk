from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from om_common.launch.sensor_launch import get_sensor_launch


def generate_launch_description():
    """
    Generate the launch description for TRON robot sensors.
    """
    use_sim = LaunchConfiguration(
        "use_sim",
        default=EnvironmentVariable("USE_SIM", default_value="false"),
    )

    entities = get_sensor_launch()

    entities.extend(
        [
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
