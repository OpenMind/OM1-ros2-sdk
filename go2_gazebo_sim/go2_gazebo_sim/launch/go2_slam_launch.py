import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    go2_gazebo_sim = FindPackageShare("go2_gazebo_sim")
    go2_sdk = FindPackageShare("go2_sdk")

    slam_config = PathJoinSubstitution([go2_sdk, "config", "slam.yaml"])
    rviz_config = PathJoinSubstitution([go2_sdk, "config", "rviz.rviz"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    go2_description = FindPackageShare("go2_description")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="maze_world.sdf",
        description="World file name (e.g., walled_world.sdf, maze_world.sdf)",
    )

    world_path = PathJoinSubstitution([go2_description, "worlds", world])

    # Include the simulation launch file
    # Disable rviz in sim launch, we launch it here
    # Disable publish_map_tf because SLAM handles it
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([go2_gazebo_sim, "launch", "unitree_go2_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz": "false",
            "publish_map_tf": "false",
            "world": world_path,
        }.items(),
    )

    # SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_config,
        }.items(),
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        sim_launch,
        slam_toolbox,
        rviz_node,
    ])
