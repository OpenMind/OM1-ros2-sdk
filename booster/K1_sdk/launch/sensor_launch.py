from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for K1 robot sensors.
    """
    use_sim = LaunchConfiguration(
        "use_sim",
        default=EnvironmentVariable("USE_SIM", default_value="false"),
    )

    entities = []

    entities.extend(
        [
            Node(
                package="K1_sdk",
                executable="K1_odom",
                name="K1_odom",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="K1_sdk",
                executable="cmd_vel_to_K1",
                name="cmd_vel_to_K1",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="K1_sdk",
                executable="insta360_stream",
                name="insta360_stream",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=UnlessCondition(use_sim),
            ),
        ]
    )

    return LaunchDescription(entities)
