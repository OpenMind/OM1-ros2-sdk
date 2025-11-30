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

    nav2_config = PathJoinSubstitution([go2_sdk, "config", "nav2_params.yaml"])
    rviz_config = PathJoinSubstitution([go2_sdk, "config", "rviz.rviz"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    world = LaunchConfiguration("world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([go2_gazebo_sim, "map", "map.yaml"]), # Default map path, might need to be created
        description="Full path to map yaml file to load",
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
    # Disable publish_map_tf because AMCL handles it
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

    nav2_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/cmd_vel", "/cmd_vel_nav"),
                ("/utlidar/cloud_deskewed", "/unitree_lidar/points"),
            ],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/cmd_vel", "/cmd_vel_nav"),
                ("/cmd_vel_smoothed", "/cmd_vel"),
            ],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}, {"yaml_filename": map_file}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
        ),
    ]

    # Lifecycle manager
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": [
                "map_server",
                "amcl",
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ]},
        ],
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
        declare_map,
        declare_world,
        sim_launch,
        *nav2_nodes,
        nav2_lifecycle_manager,
        rviz_node,
    ])
