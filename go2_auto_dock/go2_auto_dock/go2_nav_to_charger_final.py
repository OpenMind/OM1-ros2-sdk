#!/usr/bin/env python3

import math
import os
import signal
import subprocess
import threading
import time
from typing import List, Optional

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_from_quaternion(x, y, z, w):
    """Return yaw (degrees) from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def normalize_angle_deg(angle_deg):
    """Normalize angle to [-180, 180] degrees."""
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg <= -180.0:
        angle_deg += 360.0
    return angle_deg


class ProcessManager:
    """
    ProcessManager handles starting and stopping of ROS2 nodes as subprocesses.
    It manages process groups to ensure clean termination of ROS2 CLI wrappers and child nodes.
    """

    def __init__(self, name: str, pkill_pattern: Optional[str] = None):
        """
        Initialize the ProcessManager.

        Parameters:
        name : str
            Human-readable name for logging purposes
        pkill_pattern : str, optional
            Pattern to use with 'pkill -f' for forceful cleanup (e.g. node name)
        """
        self.process: Optional[subprocess.Popen] = None
        self.name = name
        self.pkill_pattern = pkill_pattern

    def run(self, command: List[str]) -> bool:
        """
        Start a command as a subprocess.

        Parameters:
        ----------
        command : List[str]
            Command and arguments to execute

        Returns:
        -------
        bool
            True if the process was started successfully, False if a process is already running.
        """
        if self.process is None or self.process.poll() is not None:
            # Use os.setsid to create a new process group for clean termination
            self.process = subprocess.Popen(command, preexec_fn=os.setsid)
            return True
        return False

    def stop(self) -> bool:
        """
        Stop the currently running subprocess.
        Tries graceful SIGTERM first, then force kill SIGKILL if needed.
        Also runs pkill if a pattern was provided to ensure cleanup.

        Returns:
        -------
        bool
            True if the process was stopped (or wasn't running), False on failure.
        """
        # Always try pkill if pattern exists to catch zombie windows or detached processes
        if self.pkill_pattern:
            try:
                subprocess.run(
                    ["pkill", "-9", "-f", self.pkill_pattern],
                    check=False,
                    capture_output=True,
                )
            except Exception:
                pass  # pkill failure is non-critical

        if self.process and self.process.poll() is None:
            try:
                # Send SIGTERM to the process group
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Force kill if timeout expired
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    self.process.wait(timeout=1)

                self.process = None
                return True
            except (ProcessLookupError, OSError):
                # Process might have already died
                self.process = None
                return True
        return False

    def is_running(self) -> bool:
        """Check if the process is currently running."""
        return self.process is not None and self.process.poll() is None

    def wait(self) -> int:
        """Wait for the process to complete and return its exit code."""
        if self.process:
            return self.process.wait()
        return 0


class SimpleGoalSender(Node):
    """
    A ROS2 node that navigates to the charging station and launches docking nodes.

    Supports both real robot and simulation modes via use_sim parameter.
    """

    def __init__(self):
        super().__init__("simple_goal_sender")

        # Declare use_sim parameter (also check environment variable)
        self.declare_parameter("use_sim", False)
        use_sim_param = self.get_parameter("use_sim").value

        # Also check USE_SIM_TIME environment variable as fallback
        use_sim_env = os.environ.get("USE_SIM_TIME", "false").lower() == "true"
        self.use_sim = use_sim_param or use_sim_env

        # Create action client for NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Define goal position and orientation (bearing angle)
        self.goal_position = {"x": -0.9578, "y": -1.952, "z": 0.0}
        self.goal_orientation = {"x": 0.0, "y": 0.0, "z": 0.99, "w": -0.067}

        # Precompute goal yaw (deg)
        self.goal_yaw_deg = yaw_from_quaternion(
            self.goal_orientation["x"],
            self.goal_orientation["y"],
            self.goal_orientation["z"],
            self.goal_orientation["w"],
        )

        # Initialize process managers with pkill patterns for safety
        self.camera_manager = ProcessManager("Camera/Relay", "topic_tools relay")
        self.detector_manager = ProcessManager(
            "AprilTag Detector", "go2_apriltag_detector"
        )
        self.charger_manager = ProcessManager("Charger Routine", "go2_sim_charger")

        # Flag to signal shutdown
        self.should_shutdown = False

        mode_str = "SIMULATION" if self.use_sim else "REAL ROBOT"
        self.get_logger().info(f"Simple Goal Sender initialized (Mode: {mode_str})")
        self.get_logger().info("Waiting for navigate_to_pose action server...")

    def send_goal(self):
        """Send the navigation goal to the robot."""
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose action server not available!")
            return False

        self.get_logger().info("Action server found! Sending goal...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"

        # Position
        goal_msg.pose.pose.position.x = self.goal_position["x"]
        goal_msg.pose.pose.position.y = self.goal_position["y"]
        goal_msg.pose.pose.position.z = self.goal_position["z"]

        # Orientation
        goal_msg.pose.pose.orientation.x = self.goal_orientation["x"]
        goal_msg.pose.pose.orientation.y = self.goal_orientation["y"]
        goal_msg.pose.pose.orientation.z = self.goal_orientation["z"]
        goal_msg.pose.pose.orientation.w = self.goal_orientation["w"]

        self.get_logger().info("Sending goal to position:")
        self.get_logger().info(f"   X: {goal_msg.pose.pose.position.x:.3f}")
        self.get_logger().info(f"   Y: {goal_msg.pose.pose.position.y:.3f}")
        self.get_logger().info(f"   Z: {goal_msg.pose.pose.position.z:.3f}")

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Called when the goal is accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was REJECTED by the action server!")
            return

        self.get_logger().info("Goal ACCEPTED by action server!")
        self.get_logger().info("Robot is now navigating to the goal...")
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Called periodically during navigation."""
        current_pose = feedback_msg.feedback.current_pose.pose

        dx = self.goal_position["x"] - current_pose.position.x
        dy = self.goal_position["y"] - current_pose.position.y
        distance_to_goal = math.hypot(dx, dy)

        q = current_pose.orientation
        current_yaw_deg = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        heading_error_deg = normalize_angle_deg(self.goal_yaw_deg - current_yaw_deg)

        self.get_logger().info(
            f"Pos: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f}) | "
            f"Dist: {distance_to_goal:.3f} m | "
            f"Yaw: {current_yaw_deg:.1f}° | Goal Yaw: {self.goal_yaw_deg:.1f}° | "
            f"Heading error: {heading_error_deg:.1f}°"
        )

    def monitor_charging_process(self):
        """Monitor the charging process and signal shutdown when complete."""
        if self.charger_manager.is_running():
            self.get_logger().info("Monitoring charging process...")
            returncode = self.charger_manager.wait()  # Block until process completes
            self.get_logger().info(
                f"Charging process completed with return code: {returncode}"
            )

            # Clean up other processes
            self.cleanup_processes()

            # Signal that we should shutdown
            self.should_shutdown = True

    def cleanup_processes(self):
        """Terminate all spawned processes gracefully."""
        self.get_logger().info("Cleaning up processes...")

        for manager in [
            self.detector_manager,
            self.camera_manager,
            self.charger_manager,
        ]:
            if manager.is_running():
                self.get_logger().info(f"Stopping {manager.name}...")
                manager.stop()
                self.get_logger().info(f"{manager.name} stopped successfully")

    def start_docking_nodes_real(self):
        """Start docking nodes for real robot."""
        self.get_logger().info("[REAL] Starting camera publisher...")
        self.camera_manager.run(
            ["ros2", "run", "go2_auto_dock", "go2_camera_publisher"]
        )
        time.sleep(4)

        self.get_logger().info("[REAL] Starting AprilTag detector...")
        self.detector_manager.run(
            ["ros2", "run", "go2_auto_dock", "go2_apriltag_detector"]
        )
        time.sleep(4)

        self.get_logger().info("[REAL] Starting charging routine...")
        self.charger_manager.run(["ros2", "run", "go2_auto_dock", "go2_tag_charger"])

    def start_docking_nodes_sim(self):
        """Start docking nodes for simulation."""
        # In simulation, Gazebo already publishes camera
        # Use topic_tools relay to remap Gazebo camera to expected topic
        self.get_logger().info("[SIM] Starting camera topic relay...")
        self.camera_manager.run(
            [
                "ros2",
                "run",
                "topic_tools",
                "relay",
                "/camera/realsense2_camera_node/color/image_raw",
                "/camera/image_raw",
            ]
        )
        time.sleep(2)

        self.get_logger().info("[SIM] Starting AprilTag detector...")
        self.detector_manager.run(
            ["ros2", "run", "go2_auto_dock", "go2_apriltag_detector"]
        )
        time.sleep(4)

        self.get_logger().info("[SIM] Starting simulation charging routine...")
        self.charger_manager.run(["ros2", "run", "go2_auto_dock", "go2_sim_charger"])

    def get_result_callback(self, future):
        """Called when navigation is complete."""
        result = future.result().result
        if result:
            self.get_logger().info("SUCCESS! Robot has ARRIVED at the goal position!")
            self.get_logger().info("Navigation completed successfully!")

            # Start post-navigation tasks based on mode
            if self.use_sim:
                self.start_docking_nodes_sim()
            else:
                self.start_docking_nodes_real()

            # Start monitoring thread
            monitor_thread = threading.Thread(
                target=self.monitor_charging_process, daemon=True
            )
            monitor_thread.start()

        else:
            self.get_logger().error("FAILED! Robot could not reach the goal position.")
            self.get_logger().error("Navigation was unsuccessful.")
            self.should_shutdown = True


def main():
    """Main entry point for the navigation to charger node."""
    rclpy.init()
    node = SimpleGoalSender()
    try:
        if node.send_goal():
            # Spin until shutdown is signaled
            while rclpy.ok() and not node.should_shutdown:
                rclpy.spin_once(node, timeout_sec=0.5)

            node.get_logger().info("Shutting down gracefully...")
        else:
            node.get_logger().error("Failed to send goal. Make sure Nav2 is running!")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        node.cleanup_processes()
    except Exception as e:
        node.get_logger().error(f"Error occurred: {str(e)}")
        node.cleanup_processes()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
