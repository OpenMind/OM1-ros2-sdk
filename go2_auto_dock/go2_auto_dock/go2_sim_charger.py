#!/usr/bin/env python3
import subprocess
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from unitree_go.msg import LowState


class SimChargerNode(Node):
    """
    Simulation charger node that handles docking without physical sit command.

    Monitors ArUco marker detection and simulates charging by setting
    the is_charging parameter on the go2_lowstate_node.
    """

    def __init__(self):
        """Initialize the simulation charger node."""
        super().__init__("go2_sim_charger")

        # State tracking
        self.zone_counter = 0
        self.charging_started = False
        self.battery_full = False
        self.latest_tag = None
        self.latest_battery_soc = 0.0

        # Subscribe to AprilTag detection
        self.tag_sub = self.create_subscription(
            Float32MultiArray, "/apriltag_relative", self.tag_callback, 10
        )

        # Subscribe to battery state
        self.battery_sub = self.create_subscription(
            LowState, "/lf/lowstate", self.battery_callback, 10
        )

        # Timer for charging logic (1 Hz)
        self.timer = self.create_timer(1.0, self.charging_loop)

        self.get_logger().info("Simulation Charger Node initialized")
        self.get_logger().info("Waiting for ArUco marker detection...")

    def tag_callback(self, msg: Float32MultiArray):
        """
        Process AprilTag detection data.

        Parameters:
        -----------
        msg : Float32MultiArray
            AprilTag relative position data
        """
        self.latest_tag = msg.data

    def battery_callback(self, msg: LowState):
        """
        Process battery state updates.

        Parameters:
        -----------
        msg : LowState
            Low state message containing battery information
        """
        self.latest_battery_soc = float(msg.bms_state.soc)

    def is_in_charging_zone(self) -> bool:
        """
        Check if robot can see the ArUco marker (simplified for simulation).

        In simulation, we just need to detect the AprilTag marker.
        Position-based docking will be fine-tuned later.

        Returns:
        --------
        bool
            True if ArUco marker is detected
        """
        if self.latest_tag is None or len(self.latest_tag) < 6:
            return False

        see_target = int(self.latest_tag[5])
        return see_target == 1  # ArUco marker detected

    def start_charging(self):
        """Start the charging process by setting is_charging parameter."""
        self.get_logger().info("=" * 50)
        self.get_logger().info("DOCKING SUCCESSFUL! Starting charging simulation...")
        self.get_logger().info("=" * 50)

        try:
            # Set is_charging parameter on go2_lowstate_node
            subprocess.run(
                [
                    "ros2",
                    "param",
                    "set",
                    "/go2_lowstate_node",
                    "is_charging",
                    "true",
                ],
                check=True,
                capture_output=True,
            )
            self.charging_started = True
            self.get_logger().info("Charging started! Monitoring battery level...")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to set charging parameter: {e}")
            self.get_logger().error("Make sure go2_lowstate_node is running")

    def stop_charging(self):
        """Stop the charging process."""
        try:
            subprocess.run(
                [
                    "ros2",
                    "param",
                    "set",
                    "/go2_lowstate_node",
                    "is_charging",
                    "false",
                ],
                check=True,
                capture_output=True,
            )
            self.get_logger().info("Charging stopped")
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f"Failed to stop charging: {e}")

    def charging_loop(self):
        """Main charging logic loop (runs at 1 Hz)."""
        # Already done charging
        if self.battery_full:
            return

        # Not yet charging - check if in position
        if not self.charging_started:
            if self.is_in_charging_zone():
                self.zone_counter += 1
                self.get_logger().info(
                    f"In charging zone ({self.zone_counter}/3 confirmations)"
                )
                if self.zone_counter >= 3:
                    self.start_charging()
            else:
                if self.zone_counter > 0:
                    self.get_logger().debug("Left charging zone, resetting counter")
                self.zone_counter = 0

                # Log tag status
                if self.latest_tag is not None and len(self.latest_tag) >= 6:
                    x_right = self.latest_tag[0]
                    z_forward = self.latest_tag[2]
                    see_target = int(self.latest_tag[5])
                    if see_target == 1:
                        self.get_logger().info(
                            f"ArUco detected: x_right={x_right:.3f}, "
                            f"z_forward={z_forward:.3f} (not in zone yet)"
                        )
                    else:
                        self.get_logger().info("Searching for ArUco marker...")
                else:
                    self.get_logger().info("No ArUco data received yet...")
            return

        # Currently charging - monitor battery
        self.get_logger().info(f"Charging... Battery: {self.latest_battery_soc:.1f}%")

        if self.latest_battery_soc >= 100.0:
            self.get_logger().info("=" * 50)
            self.get_logger().info("BATTERY FULL! Charging complete!")
            self.get_logger().info("=" * 50)
            self.stop_charging()
            self.battery_full = True

            # Exit to trigger cleanup in parent process
            self.get_logger().info("Simulation charger exiting...")
            sys.exit(0)


def main(args=None):
    """Main entry point for the simulation charger node."""
    rclpy.init(args=args)
    node = SimChargerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Charger node interrupted")
    finally:
        # Make sure charging is stopped
        if node.charging_started and not node.battery_full:
            node.stop_charging()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        node.get_logger().info("Charger node exiting...")


if __name__ == "__main__":
    main()
