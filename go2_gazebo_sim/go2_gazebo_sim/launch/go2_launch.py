import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for Go2 robot Gazebo simulation.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")

    go2_gazebo_sim = launch_ros.substitutions.FindPackageShare(
        package="go2_gazebo_sim"
    ).find("go2_gazebo_sim")
    go2_description = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")

    joints_config = os.path.join(go2_gazebo_sim, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        go2_gazebo_sim, "config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(go2_gazebo_sim, "config/gait/gait.yaml")
    links_config = os.path.join(go2_gazebo_sim, "config/links/links.yaml")
    default_model_path = os.path.join(go2_description, "urdf/unitree_go2_robot.xacro")
    default_world_path = os.path.join(go2_description, "worlds/home_world.sdf")
    aruco_model_path = os.path.join(go2_description, "models/aruco_marker/model.sdf")
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

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.375")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    declare_description_path = DeclareLaunchArgument(
        "go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )

    declare_publish_map_tf = DeclareLaunchArgument(
        "publish_map_tf",
        default_value="true",
        description="Publish static map to odom transform",
    )

    # Description nodes and parameters
    robot_description = {
        "robot_description": Command(
            ["xacro ", LaunchConfiguration("go2_description_path")]
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # CHAMP controller nodes
    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
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
            {"urdf": Command(["xacro ", LaunchConfiguration("go2_description_path")])},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"publish_foot_contacts": False},
            {"close_loop_odom": True},
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": True},
            {"urdf": Command(["xacro ", LaunchConfiguration("go2_description_path")])},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    # base_to_footprint_ekf = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="base_to_footprint_ekf",
    #     output="screen",
    #     parameters=[
    #         {"base_link_frame": base_frame},
    #         {"use_sim_time": use_sim_time},
    #         os.path.join(
    #             get_package_share_directory("champ_base"),
    #             "config",
    #             "ekf",
    #             "base_to_footprint.yaml",
    #         ),
    #     ],
    #     remappings=[("odometry/filtered", "odom/local")],
    # )

    # footprint_to_odom_ekf = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="footprint_to_odom_ekf",
    #     output="screen",
    #     parameters=[
    #         {"use_sim_time": use_sim_time},
    #         {"base_link_frame": "base_footprint"},
    #         {"odom_frame": "odom"},
    #         {"world_frame": "odom"},
    #         {"publish_tf": True},
    #         {"frequency": 50.0},
    #         {"two_d_mode": True},
    #         {"odom0": "odom/raw"},
    #         {"odom0_config": [False, False, False, False, False, False, True, True, False, False, False, True, False, False, False]},
    #         {"imu0": "imu/data"},
    #         {"imu0_config": [False, False, False, False, False, True, False, False, False, False, False, True, False, False, False]},
    #     ],
    #     remappings=[("odometry/filtered", "odom")],
    # )

    # Go2 static frame connection (map -> odom)
    map_to_odom_tf_node = Node(
        package="tf2_ros",
        name="map_to_odom_tf_node",
        executable="static_transform_publisher",
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
            "odom",
        ],
        condition=IfCondition(LaunchConfiguration("publish_map_tf")),
    )

    # Go2 URDF connection (base_footprint -> base_link)
    base_footprint_to_base_link_tf_node = Node(
        package="tf2_ros",
        name="base_footprint_to_base_link_tf_node",
        executable="static_transform_publisher",
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
            "base_footprint",
            "--child-frame-id",
            "base_link",
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(go2_gazebo_sim, "rviz/rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        # parameters=[{"use_sim_time": use_sim_time}]
    )

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                LaunchConfiguration("world"),
                " -r",
            ]  # Add -r flag to start unpaused
        }.items(),
    )

    gazebo_spawn_aruco = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_aruco",
        output="screen",
        arguments=[
            "-name",
            "aruco_marker",
            "-file",
            aruco_model_path,
            "-x",
            "-1.91",
            "-y",
            "-1.99",
            "-z",
            "0.1",
            "-Y",
            "1.57",
        ],
    )

    # Spawn robot in Gazebo Sim
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            LaunchConfiguration("robot_name"),
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("world_init_x"),
            "-y",
            LaunchConfiguration("world_init_y"),
            "-z",
            LaunchConfiguration("world_init_z"),
            "-Y",
            LaunchConfiguration("world_init_heading"),
        ],
    )

    # Bridge ROS 2 topics to Gazebo Sim
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            # Gazebo to ROS
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            # '/velodyne_points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            "/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/realsense2_camera_node/depth/image_rect_gazebo_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/realsense2_camera_node/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/realsense2_camera_node/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/realsense2_camera_node/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            # ROS to Gazebo
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
        ],
    )

    # Use spawner nodes directly to handle the configuration step. (load → configure → activate)
    controller_spawner_js = TimerAction(
        period=20.0,  # Wait for Gazebo to fully initialize
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout",
                    "120",  # Longer timeout
                    "joint_states_controller",  # No --inactive flag to ensure full activation
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    controller_spawner_effort = TimerAction(
        period=30.0,  # Wait 5 seconds after joint_states_controller
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout",
                    "120",  # Longer timeout
                    "joint_group_effort_controller",  # No --inactive flag to ensure full activation
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    # Shell script to manually check controller status
    controller_status_check = TimerAction(
        period=25.0,  # Check status after controllers should be loaded
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    "echo 'Checking controller status:' && ros2 control list_controllers",
                ],
                output="screen",
            )
        ],
    )

    # Go2 remapping node
    go2_remapping_node = Node(
        package="go2_gazebo_sim",
        executable="go2_remapping_node",
        name="go2_remapping_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Go2 sport node
    go2_sport_node = Node(
        package="go2_gazebo_sim",
        executable="go2_sport_node",
        name="go2_sport_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Go2 lowstate node
    go2_lowstate_node = Node(
        package="go2_gazebo_sim",
        executable="go2_lowstate_node",
        name="go2_lowstate_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # depth remapping node for Intel D435
    intel435_depth_node = Node(
        package="go2_gazebo_sim",
        executable="intel435_depth_node",
        name="intel435_depth_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    point_cloud_xyz_sim = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="point_cloud_xyz_sim",
        remappings=[
            (
                "image_rect",
                "/camera/realsense2_camera_node/depth/image_rect_gazebo_raw",
            ),
            ("camera_info", "/camera/realsense2_camera_node/depth/camera_info"),
            ("points", "/camera/depth/points"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(use_sim_time),
    )

    # Camera topic relay for auto-docking (sim only)
    camera_relay_node = Node(
        package="topic_tools",
        executable="relay",
        name="camera_relay",
        output="screen",
        arguments=[
            "/camera/realsense2_camera_node/color/image_raw",
            "/camera/image_raw",
        ],
        condition=IfCondition(use_sim_time),
    )

    # Nodes for teleoperation
    joy = Node(package="joy", executable="joy_node", name="joy_node", output="screen")
    # RB is the enable button
    # The left joystick controls linear movement
    # The right joystick controls angular movement
    joy_teleops_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[
            {
                "axis_linear.x": 1,
                "axis_linear.y": 0,
                "axis_angular.z": 3,
                "enable_button": 7,
                "scale_linear.x": 0.5,
                "scale_angular.z": 1.0,
                "enable_turbo_button": 4,
                "scale_turbo_linear.x": 1.5,
                "scale_turbo_angular.z": 2.0,
            }
        ],
    )

    return LaunchDescription(
        [
            # Launch arguments
            set_gz_resource_path,
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path,
            declare_publish_map_tf,
            # Gazebo and robot nodes first
            gz_sim,
            robot_state_publisher_node,
            gazebo_spawn_robot,
            gazebo_spawn_aruco,
            gazebo_bridge,
            # CHAMP controller nodes
            quadruped_controller_node,
            state_estimator_node,
            # EKF nodes for localization
            # base_to_footprint_ekf,
            # footprint_to_odom_ekf,
            # TF publishers for frame connections
            map_to_odom_tf_node,
            base_footprint_to_base_link_tf_node,
            # Controller spawners that handle the complete lifecycle
            controller_spawner_js,
            controller_spawner_effort,
            controller_status_check,
            # Visualization (only if rviz flag is set)
            rviz2,
            # Go2 sport node
            go2_sport_node,
            # Go2 lowstate node
            go2_lowstate_node,
            # Go2 remapping node
            go2_remapping_node,
            # Intel D435 depth remapping node
            intel435_depth_node,
            # pointcloud2 remapping node
            point_cloud_xyz_sim,
            # Camera relay (sim)
            camera_relay_node,
            # Teleoperation nodes
            joy,
            joy_teleops_node,
        ]
    )
