from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for the orchestrator nodes.
    """
    use_sim = LaunchConfiguration("use_sim", default="false")

    return LaunchDescription(
        [
            Node(
                package="orchestrator",
                executable="orchestrator_api",
                name="orchestrator_api",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[{"use_sim": use_sim}],
            ),
            Node(
                package="orchestrator",
                executable="orchestrator_cloud",
                name="orchestrator_cloud",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[{"use_sim": use_sim}],
            ),
        ]
    )
