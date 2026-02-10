import os
from typing import List

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_robot_group(
    context,
    robot_id: int,
    spawn_x: float,
    spawn_y: float,
    spawn_z: float,
    use_sim_time,
    go2_gazebo_sim: str,
    go2_description: str,
    joints_config: str,
    links_config: str,
    gait_config: str,
) -> List:
    """
    Generate all nodes for a single robot instance.

    Args:
        robot_id: Robot number (1, 2, 3, ...)
        spawn_x, spawn_y, spawn_z: Spawn position
        use_sim_time: Use simulation time
        go2_gazebo_sim: Package share directory for go2_gazebo_sim
        go2_description: Package share directory for go2_description
        joints_config, links_config, gait_config: Config file paths

    Returns
    -------
        List of nodes for this robot
    """
    robot_name = f"robot{robot_id}"
    ns = robot_name

    # Generate robot description with namespace
    xacro_path = os.path.join(go2_description, "urdf/unitree_go2_robot.xacro")
    robot_description_content = Command(
        [f"xacro {xacro_path} robot_namespace:={robot_name}"]
    )

    nodes = []

    # Robot State Publisher (publishes robot_description and TF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
            {"frame_prefix": f"{robot_name}/"},
        ],
    )
    nodes.append(robot_state_publisher)

    # Spawn robot in Gazebo
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=ns,
        name="spawn_robot",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            f"/{ns}/robot_description",
            "-x",
            str(spawn_x),
            "-y",
            str(spawn_y),
            "-z",
            str(spawn_z),
            "-Y",
            "0.0",
        ],
    )
    nodes.append(gazebo_spawn_robot)

    # Gazebo Bridge - bridge Gazebo topics to ROS with namespace
    # Topics use the robot namespace prefix from URDF
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=ns,
        name="gazebo_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            # Clock (global, only needed once but harmless to repeat)
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Namespaced topics from Gazebo (using robot_name prefix from URDF)
            f"/{robot_name}/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
            f"/{robot_name}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            f"/{robot_name}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            f"/{robot_name}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            f"/{robot_name}/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            f"/{robot_name}/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image",
            f"/{robot_name}/camera/realsense2_camera_node/depth/image_rect_gazebo_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            f"/{robot_name}/camera/realsense2_camera_node/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            f"/{robot_name}/camera/realsense2_camera_node/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            f"/{robot_name}/camera/realsense2_camera_node/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            # ROS to Gazebo (cmd_vel needs model-specific topic)
            f"/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # Joint trajectory for ros2_control
            f"/model/{robot_name}/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
            f"/model/{robot_name}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
        ],
        remappings=[
            # Remap model-specific topics to namespaced topics
            (f"/model/{robot_name}/joint_states", "joint_states"),
            (
                f"/model/{robot_name}/joint_trajectory",
                "joint_group_effort_controller/joint_trajectory",
            ),
        ],
    )
    nodes.append(gazebo_bridge)

    # TF Relay - forward namespaced TF (odom -> base_link) to global /tf
    # The Gazebo odometry plugin publishes to /robotX/tf, but TF2 listens to /tf
    tf_relay = Node(
        package="topic_tools",
        executable="relay",
        namespace=ns,
        name="tf_relay",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[f"/{robot_name}/tf", "/tf"],
    )
    nodes.append(tf_relay)

    # CHAMP Quadruped Controller
    quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        namespace=ns,
        name="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {
                "joint_controller_topic": "joint_group_effort_controller/joint_trajectory"
            },
            {"urdf": robot_description_content},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"close_loop_odom": True},
        ],
        remappings=[
            ("cmd_vel/smooth", "cmd_vel"),  # Both relative for namespace compatibility
            ("odom/raw", "odom"),
        ],
    )
    nodes.append(quadruped_controller)

    # State Estimator
    state_estimator = Node(
        package="champ_base",
        executable="state_estimation_node",
        namespace=ns,
        name="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": True},
            {"urdf": robot_description_content},
            joints_config,
            links_config,
            gait_config,
        ],
    )
    nodes.append(state_estimator)

    # Static TF: map -> robot_namespace/odom
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=ns,
        name="map_to_odom_tf",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            f"{robot_name}/odom",
        ],
    )
    nodes.append(map_to_odom_tf)

    # Static TF: robot_namespace/base_footprint -> robot_namespace/base_link
    base_footprint_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=ns,
        name="base_footprint_to_base_link_tf",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            f"{robot_name}/base_link",
            "--child-frame-id",
            f"{robot_name}/base_footprint",
        ],
    )
    nodes.append(base_footprint_to_base_link_tf)

    # Go2 sport node (simulated API)
    go2_sport = Node(
        package="go2_gazebo_sim",
        executable="go2_sport_node",
        namespace=ns,
        name="go2_sport_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    nodes.append(go2_sport)

    # Go2 lowstate node
    go2_lowstate = Node(
        package="go2_gazebo_sim",
        executable="go2_lowstate_node",
        namespace=ns,
        name="go2_lowstate_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    nodes.append(go2_lowstate)

    # Go2 remapping node
    go2_remapping = Node(
        package="go2_gazebo_sim",
        executable="go2_remapping_node",
        namespace=ns,
        name="go2_remapping_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    nodes.append(go2_remapping)

    # Intel D435 depth remapping
    intel435_depth = Node(
        package="go2_gazebo_sim",
        executable="intel435_depth_node",
        namespace=ns,
        name="intel435_depth_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    nodes.append(intel435_depth)

    # Controller spawners - spawn and activate ros2_control controllers
    # These need to be delayed to give Gazebo time to initialize
    ros_control_config = os.path.join(
        go2_gazebo_sim, "config/ros_control/ros_control.yaml"
    )

    controller_spawner_js = TimerAction(
        period=20.0 + (robot_id - 1) * 5.0,  # Stagger spawns for multiple robots
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=ns,
                name=f"spawner_joint_states_{robot_name}",
                output="screen",
                arguments=[
                    "--controller-manager-timeout",
                    "120",
                    "-c",
                    f"/{ns}/controller_manager",
                    "--param-file",
                    ros_control_config,
                    "joint_states_controller",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )
    nodes.append(controller_spawner_js)

    controller_spawner_effort = TimerAction(
        period=30.0 + (robot_id - 1) * 5.0,  # Stagger spawns for multiple robots
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=ns,
                name=f"spawner_effort_{robot_name}",
                output="screen",
                arguments=[
                    "--controller-manager-timeout",
                    "120",
                    "-c",
                    f"/{ns}/controller_manager",
                    "--param-file",
                    ros_control_config,
                    "joint_group_effort_controller",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )
    nodes.append(controller_spawner_effort)

    return nodes


def launch_setup(context, *args, **kwargs):
    """
    OpaqueFunction callback to dynamically generate robot nodes based on robot count.
    """
    # Get launch configurations
    num_robots = int(LaunchConfiguration("robots").perform(context))
    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )
    robot_spacing = float(LaunchConfiguration("robot_spacing").perform(context))

    # Package paths
    go2_gazebo_sim = launch_ros.substitutions.FindPackageShare(
        package="go2_gazebo_sim"
    ).find("go2_gazebo_sim")
    go2_description = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")

    # Config files
    joints_config = os.path.join(go2_gazebo_sim, "config/joints/joints.yaml")
    links_config = os.path.join(go2_gazebo_sim, "config/links/links.yaml")
    gait_config = os.path.join(go2_gazebo_sim, "config/gait/gait.yaml")

    all_nodes = []

    # Generate nodes for each robot
    for i in range(1, num_robots + 1):
        # Spawn robots in a line along Y-axis
        spawn_x = 0.0
        spawn_y = (i - 1) * robot_spacing
        spawn_z = 0.375

        # Core robot nodes
        robot_nodes = generate_robot_group(
            context,
            robot_id=i,
            spawn_x=spawn_x,
            spawn_y=spawn_y,
            spawn_z=spawn_z,
            use_sim_time=use_sim_time,
            go2_gazebo_sim=go2_gazebo_sim,
            go2_description=go2_description,
            joints_config=joints_config,
            links_config=links_config,
            gait_config=gait_config,
        )
        all_nodes.extend(robot_nodes)

    # Joystick control nodes (global, not per-robot)
    # Joy node reads from physical joystick
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"device_id": 0},
            {"deadzone": 0.1},
            {"autorepeat_rate": 20.0},
        ],
    )
    all_nodes.append(joy_node)

    # Joy robot switcher routes joystick to selected robot
    # LB (button 4) = control robot1, RB (button 5) = control robot2
    joy_robot_switcher = Node(
        package="go2_gazebo_sim",
        executable="joy_robot_switcher_node",
        name="joy_robot_switcher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"num_robots": num_robots},
            {"robot1_button": 4},  # LB = robot1
            {"robot2_button": 5},  # RB = robot2
            {"linear_axis": 1},  # Left stick Y
            {"angular_axis": 0},  # Left stick X
            {"linear_scale": 0.5},
            {"angular_scale": 1.0},
        ],
    )
    all_nodes.append(joy_robot_switcher)

    return all_nodes


def generate_launch_description():
    """
    Generate the launch description for multi-Go2 Gazebo simulation.
    """
    go2_gazebo_sim = launch_ros.substitutions.FindPackageShare(
        package="go2_gazebo_sim"
    ).find("go2_gazebo_sim")
    go2_description = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")

    default_world_path = os.path.join(go2_description, "worlds/home_world.sdf")
    go2_description_models = os.path.join(go2_description, "models")

    # Ensure GZ_SIM_RESOURCE_PATH includes the models directory
    current_gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if go2_description_models not in current_gz_resource_path:
        if current_gz_resource_path:
            new_gz_resource_path = (
                current_gz_resource_path + ":" + go2_description_models
            )
        else:
            new_gz_resource_path = go2_description_models
    else:
        new_gz_resource_path = current_gz_resource_path

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=new_gz_resource_path
    )

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_robots = DeclareLaunchArgument(
        "robots",
        default_value="1",
        description="Number of robots to spawn",
    )

    declare_robot_spacing = DeclareLaunchArgument(
        "robot_spacing",
        default_value="2.0",
        description="Spacing between robots in meters (along Y-axis)",
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Gazebo world file",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz",
    )

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [LaunchConfiguration("world"), " -r"]}.items(),
    )

    # RViz (single instance for all robots)
    rviz_config = os.path.join(go2_gazebo_sim, "rviz/rviz_multi.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Robot nodes generated dynamically
    robot_nodes = OpaqueFunction(function=launch_setup)

    return LaunchDescription(
        [
            # Environment setup
            set_gz_resource_path,
            # Launch arguments
            declare_use_sim_time,
            declare_robots,
            declare_robot_spacing,
            declare_world,
            declare_rviz,
            # Gazebo
            gz_sim,
            # RViz
            rviz2,
            # Dynamic robot nodes
            robot_nodes,
        ]
    )
