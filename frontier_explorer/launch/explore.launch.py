import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for the frontier explorer node.
    """
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("frontier_explorer"), "config", "params.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the explore node",
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="frontier_explorer",
        name="explore_node",
        namespace=namespace,
        executable="explore",
        parameters=[config, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remappings,
    )
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(node)
    return ld
