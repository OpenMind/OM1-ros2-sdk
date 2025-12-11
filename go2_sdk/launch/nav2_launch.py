import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# Simulation-specific parameter overrides as (node_name, param_path, value) tuples
# These are applied via ros2 param set when use_sim:=true
SIM_PARAM_OVERRIDES = [
    # Controller server - MPPI controller params
    ("/controller_server", "FollowPath.vx_std", "0.2"),
    ("/controller_server", "FollowPath.wz_std", "0.4"),
    ("/controller_server", "FollowPath.vx_max", "0.5"),
    ("/controller_server", "FollowPath.vx_min", "-0.35"),
    ("/controller_server", "FollowPath.wz_max", "1.2"),
    ("/controller_server", "FollowPath.ObstaclesCritic.repulsion_weight", "10.0"),
    (
        "/controller_server",
        "FollowPath.ObstaclesCritic.collision_margin_distance",
        "0.05",
    ),
    ("/controller_server", "FollowPath.ObstaclesCritic.near_goal_distance", "0.4"),
    ("/controller_server", "FollowPath.GoalAngleCritic.cost_power", "4"),
    ("/controller_server", "FollowPath.PreferForwardCritic.cost_weight", "16.0"),
    # Local costmap inflation layer
    ("/local_costmap/local_costmap", "inflation_layer.cost_scaling_factor", "5.0"),
    ("/local_costmap/local_costmap", "inflation_layer.inflation_radius", "0.30"),
    # Velocity smoother
    ("/velocity_smoother", "max_velocity", "[0.5, 0.0, 1.2]"),
    ("/velocity_smoother", "min_velocity", "[-0.35, 0.0, -1.2]"),
    ("/velocity_smoother", "max_accel", "[2.0, 0.0, 2.5]"),
    ("/velocity_smoother", "max_decel", "[-2.0, 0.0, -2.5]"),
]


def configure_nav2_nodes(context, *args, **kwargs):
    """Configure Nav2 nodes with optional simulation parameter overrides."""
    pkg_dir = get_package_share_directory("go2_sdk")

    urdf_file = os.path.join(pkg_dir, "urdf", "go2.urdf")
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    nav2_config_file = os.path.join(pkg_dir, "config", "nav2_params.yaml")

    # Resolve use_sim at launch time
    use_sim_str = LaunchConfiguration("use_sim").perform(context)
    is_sim = use_sim_str.lower() == "true"
    use_sim = LaunchConfiguration("use_sim")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    global_localization_particles = LaunchConfiguration("global_localization_particles")

    # Build ros2 param set commands for simulation overrides
    sim_param_commands = []
    if is_sim:
        for node_name, param_path, value in SIM_PARAM_OVERRIDES:
            sim_param_commands.append(
                ExecuteProcess(
                    cmd=["ros2", "param", "set", node_name, param_path, value],
                    output="screen",
                )
            )

    # Wrap sim param commands in a TimerAction to wait for nodes to be ready
    sim_param_timer = (
        TimerAction(
            period=10.0,  # Wait for Nav2 nodes to be fully active
            actions=sim_param_commands,
        )
        if sim_param_commands
        else None
    )

    # Standard params for all nodes
    standard_params = [nav2_config_file, {"use_sim_time": use_sim}]

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}],
            condition=UnlessCondition(use_sim),
        ),
        Node(
            package="go2_sdk",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(use_sim),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_laser",
            arguments=[
                "--x",
                "0.2",
                "--y",
                "0",
                "--z",
                "0.05",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "3.14159",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "laser",
            ],
            output="screen",
            condition=UnlessCondition(use_sim),
        ),
        Node(
            package="go2_sdk",
            executable="pose_to_tf",
            output="screen",
            condition=UnlessCondition(use_sim),
        ),
        Node(
            package="go2_sdk",
            executable="cmd_vel_to_go2",
            name="cmd_vel_to_go2",
            output="screen",
            condition=UnlessCondition(use_sim),
        ),
        Node(
            package="go2_sdk",
            executable="go2_sport_action",
            name="go2_sport_action",
            output="screen",
            condition=UnlessCondition(use_sim),
        ),
        Node(
            package="go2_sdk",
            executable="waypoint_manager",
            name="waypoint_manager",
            output="screen",
        ),
        Node(
            package="go2_sdk",
            executable="go2_nav2_api",
            name="go2_nav2_api_node",
            output="screen",
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            condition=UnlessCondition(use_sim),
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            output="screen",
            parameters=[
                {
                    "axis_linear.x": 1,
                    "axis_linear.y": 0,
                    "axis_angular.z": 3,
                    "enable_button": 10,
                    "scale_linear.x": 0.5,
                    "scale_angular.z": 0.5,
                    "enable_turbo_button": 9,
                    "scale_turbo_linear.x": 1.5,
                    "scale_turbo_angular.z": 2.0,
                }
            ],
            condition=UnlessCondition(use_sim),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {
                    "node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                    ]
                },
                {"use_sim_time": use_sim},
            ],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=standard_params,
            remappings=[("/cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=standard_params,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=standard_params,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=standard_params,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=standard_params,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=standard_params,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=standard_params,
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
            parameters=[{"use_sim_time": use_sim, "yaml_filename": map_yaml_file}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim},
                {"autostart": True},
                {"node_names": ["map_server"]},
            ],
        ),
        Node(
            package="go2_sdk",
            executable="go2_lidar_localization",
            name="go2_lidar_localization",
            output="screen",
            parameters=[
                {
                    "base_frame": "base_link",
                    "odom_frame": "odom",
                    "laser_frame": "laser",
                    "laser_topic": "scan",
                    "global_localization_particles": global_localization_particles,
                    "use_sim_time": use_sim,
                }
            ],
        ),
    ]

    # Add the sim param timer if we're in simulation mode
    if sim_param_timer:
        nodes.append(sim_param_timer)

    return nodes


def generate_launch_description():
    channel_type = LaunchConfiguration(
        "channel_type",
        default=EnvironmentVariable("LIDAR_CHANNEL_TYPE", default_value="serial"),
    )
    serial_port = LaunchConfiguration(
        "serial_port",
        default=EnvironmentVariable("LIDAR_SERIAL_PORT", default_value="/dev/ttyUSB0"),
    )
    serial_baudrate = LaunchConfiguration(
        "serial_baudrate",
        default=EnvironmentVariable("LIDAR_SERIAL_BAUDRATE", default_value="115200"),
    )
    frame_id = LaunchConfiguration(
        "frame_id",
        default=EnvironmentVariable("LIDAR_FRAME_ID", default_value="laser"),
    )
    inverted = LaunchConfiguration(
        "inverted",
        default=EnvironmentVariable("LIDAR_INVERTED", default_value="false"),
    )
    angle_compensate = LaunchConfiguration(
        "angle_compensate",
        default=EnvironmentVariable("LIDAR_ANGLE_COMPENSATE", default_value="true"),
    )
    scan_mode = LaunchConfiguration(
        "scan_mode",
        default=EnvironmentVariable("LIDAR_SCAN_MODE", default_value="Sensitivity"),
    )
    global_localization_particles = LaunchConfiguration(
        "global_localization_particles",
        default=EnvironmentVariable(
            "GLOBAL_LOCALIZATION_PARTICLES", default_value="5000"
        ),
    )
    use_sim = LaunchConfiguration("use_sim", default="false")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "channel_type",
                default_value=channel_type,
                description="Specifying channel type of lidar",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value=serial_port,
                description="Specifying usb port to connected lidar",
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value=serial_baudrate,
                description="Specifying usb port baudrate to connected lidar",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value=frame_id,
                description="Specifying frame_id of lidar",
            ),
            DeclareLaunchArgument(
                "inverted",
                default_value=inverted,
                description="Specifying whether or not to invert scan data",
            ),
            DeclareLaunchArgument(
                "angle_compensate",
                default_value=angle_compensate,
                description="Specifying whether or not to enable angle_compensate of scan data",
            ),
            DeclareLaunchArgument(
                "scan_mode",
                default_value=scan_mode,
                description="Specifying scan mode of lidar",
            ),
            DeclareLaunchArgument(
                "map_yaml_file",
                description="Full path to map yaml file (leave empty for SLAM mode)",
            ),
            DeclareLaunchArgument(
                "global_localization_particles",
                default_value=global_localization_particles,
                description="Number of particles for global localization",
            ),
            DeclareLaunchArgument(
                "use_sim",
                default_value=use_sim,
                description="Whether to use simulation",
            ),
            OpaqueFunction(function=configure_nav2_nodes),
        ]
    )
