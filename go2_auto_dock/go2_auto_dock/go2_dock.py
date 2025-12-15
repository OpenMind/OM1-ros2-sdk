#!/usr/bin/env python3
"""
Battery monitor node for autonomous charging.

This node monitors the robot's battery level and triggers navigation to the
charging station when battery drops below the threshold.

Usage:
    ros2 run go2_auto_dock go2_battery_monitor
    ros2 run go2_auto_dock go2_battery_monitor --ros-args -p use_sim:=true
"""

import os
import subprocess

import rclpy
from rclpy.node import Node

from unitree_go.msg import LowState


class Go2AutoChargeMonitor(Node):
    """
    Battery monitor node that triggers automatic charging when battery is low.

    Supports both real robot and simulation modes via use_sim parameter.
    """

    def __init__(self):
        """Initialize the battery monitor node."""
        super().__init__("go2_auto_charge_monitor")

        # Declare use_sim parameter (also check environment variable)
        self.declare_parameter("use_sim", False)
        use_sim_param = self.get_parameter("use_sim").value

        # Also check USE_SIM_TIME environment variable as fallback
        use_sim_env = os.environ.get("USE_SIM_TIME", "false").lower() == "true"
        self.use_sim = use_sim_param or use_sim_env

        mode_str = "SIMULATION" if self.use_sim else "REAL ROBOT"
        self.get_logger().info(f"Go2 Auto Charge Monitor started (Mode: {mode_str})")

        self.subscription = self.create_subscription(
            LowState, "/lf/lowstate", self.listener_callback, 10
        )

        # State tracking
        self.low_battery_threshold = 53.0  # Percentage
        self.charging_navigation_triggered = False
        self.is_charging = False
        self.last_soc = None

        # Timer to periodically check battery status (every 10 seconds)
        self.timer = self.create_timer(10.0, self.periodic_check)

    def listener_callback(self, msg):
        """
        Process incoming battery data.

        Parameters:
        -----------
        msg : LowState
            Low state message containing battery information
        """
        soc = msg.bms_state.soc  # State of charge (battery percentage)
        current = msg.bms_state.current  # Current in mA (positive = charging)

        # Update charging status
        self.is_charging = current > 0
        self.last_soc = soc

        # Only log battery status if navigation hasn't been triggered
        if not self.charging_navigation_triggered:
            charging_status = "CHARGING" if self.is_charging else "DISCHARGING"
            self.get_logger().info(
                f"Battery: {soc:.1f}% | Current: {current} mA | Status: {charging_status}"
            )

        # Check if we need to navigate to charger
        if (
            soc < self.low_battery_threshold
            and not self.is_charging
            and not self.charging_navigation_triggered
        ):
            self.get_logger().warn(
                f"Battery low ({soc:.1f}% < {self.low_battery_threshold}%) and not charging!"
            )
            self.trigger_charging_navigation()

        # Reset trigger flag if battery is above threshold or charging
        if soc >= self.low_battery_threshold or self.is_charging:
            if self.charging_navigation_triggered:
                self.get_logger().info(
                    "Battery recovered or charging detected. Resetting trigger."
                )
            self.charging_navigation_triggered = False

    def periodic_check(self):
        """Periodic status check."""
        # Only print periodic updates if navigation hasn't been triggered
        if self.last_soc is not None and not self.charging_navigation_triggered:
            status = "CHARGING" if self.is_charging else "DISCHARGING"
            self.get_logger().info(
                f"[Periodic Check] Battery: {self.last_soc:.1f}% | Status: {status}"
            )

    def trigger_charging_navigation(self):
        """Launch the navigation to charger script."""
        self.get_logger().warn("=" * 60)
        self.get_logger().warn("INITIATING AUTOMATIC CHARGING SEQUENCE!")
        self.get_logger().warn("=" * 60)

        try:
            # Build command with use_sim parameter if needed
            cmd = ["ros2", "run", "go2_auto_dock", "go2_nav_to_charger"]
            if self.use_sim:
                cmd.extend(["--ros-args", "-p", "use_sim:=true"])

            subprocess.Popen(cmd)
            self.charging_navigation_triggered = True
            self.get_logger().info(
                "Navigation to charger script launched successfully."
            )

        except Exception as e:
            self.get_logger().error(f"Failed to launch charging navigation: {str(e)}")
            self.charging_navigation_triggered = False


def main(args=None):
    """Main entry point for the battery monitor node."""
    rclpy.init(args=args)
    monitor = Go2AutoChargeMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("Keyboard interrupt detected, shutting down...")
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
