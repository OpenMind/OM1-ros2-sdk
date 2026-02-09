#!/usr/bin/env python3

import json
import os
import subprocess
from enum import IntEnum

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from unitree_api.msg import Request, RequestHeader, RequestIdentity
from unitree_go.msg import LowState

from om_api.msg import OMChargingStatus


class ChargingStatus(IntEnum):
    """Charging status codes matching OMChargingStatus message."""

    DISCHARGING = 0
    CHARGING = 1
    ENROUTE_CHARGING = 2
    FULLY_CHARGED = 3


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
        self.sport_pub = self.create_publisher(Request, "/api/sport/request", 10)

        # Publisher for charging status
        self.status_pub = self.create_publisher(
            OMChargingStatus, "/om/go2/charging_status", 10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped, "/utlidar/robot_pose", self.pose_callback, 10
        )
        self.current_z_height = 0.0
        self.waiting_for_stand = False

        # State tracking
        self.low_battery_threshold = 15.0  # Percentage
        self.full_battery_threshold = 95.0  # Percentage
        self.charging_navigation_triggered = False
        self.fully_charged_stand_triggered = False
        self.is_charging = False
        self.last_soc = None
        self.current_status = (
            ChargingStatus.DISCHARGING
        )  # Track current status to avoid redundant publishing

        # Timer to periodically check battery status (every 10 seconds)
        self.timer = self.create_timer(10.0, self.periodic_check)

    def listener_callback(self, msg):
        """
        Process incoming battery data.

        Parameters
        ----------
        msg : LowState
            Low state message containing battery information
        """
        soc = msg.bms_state.soc  # State of charge (battery percentage)
        current = msg.bms_state.current  # Current in mA (positive = charging)

        # Update charging status
        self.is_charging = current > 0
        self.last_soc = soc

        # Determine and publish current status
        new_status = self._determine_status(soc)
        if new_status != self.current_status:
            self.current_status = new_status
            self._publish_status(new_status)

        # Only log battery status if navigation hasn't been triggered
        if not self.charging_navigation_triggered:
            charging_status = "CHARGING" if self.is_charging else "DISCHARGING"
            self.get_logger().info(
                f"Battery: {soc:.1f}% | Current: {current} mA | Status: {charging_status}"
            )

        # Check for full battery stand-up
        if (
            soc >= self.full_battery_threshold
            and self.is_charging
            and not self.fully_charged_stand_triggered
        ):
            self.get_logger().info(
                f"Battery is full ({soc:.1f}% >= {self.full_battery_threshold}%). Standing up!"
            )
            self.send_balance_stand_command()
            self.fully_charged_stand_triggered = True

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

        # Always publish current status periodically
        if self.last_soc is not None:
            current_status = self._determine_status(self.last_soc)
            self._publish_status(current_status)

    def _determine_status(self, soc):
        """
        Determine the current charging status based on state.

        Parameters
        ----------
        soc : float
            State of charge (battery percentage)

        Returns
        -------
        int
            Status code: 0=DISCHARGING, 1=CHARGING, 2=ENROUTE_CHARGING, 3=FULLY_CHARGED
        """
        if soc >= self.full_battery_threshold and self.is_charging:
            return ChargingStatus.FULLY_CHARGED
        elif self.charging_navigation_triggered:
            return ChargingStatus.ENROUTE_CHARGING
        elif self.is_charging:
            return ChargingStatus.CHARGING
        else:
            return ChargingStatus.DISCHARGING

    def _publish_status(self, status):
        """
        Publish the charging status.

        Parameters
        ----------
        status : int
            The status code to publish (0-3)

        """
        try:
            # Ensure status is an Enum member
            status_enum = ChargingStatus(status)
        except ValueError:
            # Fallback for unknown status values
            status_enum = None

        msg = OMChargingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.code = status

        if status_enum is not None:
            msg.status = status_enum.name
        else:
            msg.status = "UNKNOWN"

        self.status_pub.publish(msg)
        self.get_logger().info(f"Published charging status: {msg.status} ({status})")

    def trigger_charging_navigation(self):
        """Initiate the charging sequence by standing up and waiting for height confirmation."""
        self.get_logger().warn("INITAITING AUTOMATIC CHARGING SEQUENCE!")

        # Only command stand if not already standing
        if self.current_z_height < 0.25:
            self.send_balance_stand_command()

        self.waiting_for_stand = True
        self.charging_navigation_triggered = True
        self.get_logger().info("Waiting for stand confirmation...")

    def pose_callback(self, msg):
        """Track robot height and trigger navigation when standing."""
        self.current_z_height = msg.pose.position.z

        if self.waiting_for_stand:
            # Check for standing height (threshold 0.25m)
            if self.current_z_height > 0.25:
                self.get_logger().info(
                    f"Robot standing (Z={self.current_z_height:.2f}m). Launching."
                )
                self.waiting_for_stand = False
                self.launch_navigation_script()

    def launch_navigation_script(self):
        """Launch the navigation to charger script."""
        try:
            # Build command with use_sim parameter if needed
            cmd = ["ros2", "run", "go2_auto_dock", "go2_nav_to_charger"]
            if self.use_sim:
                cmd.extend(["--ros-args", "-p", "use_sim:=true"])

            subprocess.Popen(cmd)
            self.fully_charged_stand_triggered = False  # Reset for next cycle
            self.get_logger().info(
                "Navigation to charger script launched successfully."
            )

        except Exception as e:
            self.get_logger().error(f"Failed to launch charging navigation: {str(e)}")
            # If launch fails, we might want to allow retry, but for now we leave triggered=True
            # to prevent spamming. User can restart node if needed.

    def _publish_request(self, api_id: int, params_dict=None):
        """
        Low-level helper to publish a Unitree Request with JSON 'parameter'.

        Args:
            api_id (int): The API ID for the request.
            params_dict (dict, optional): Dictionary of parameters to be serialized to JSON. Defaults to None.
        """
        req = Request()
        req.header = RequestHeader()
        req.header.identity = RequestIdentity()
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(params_dict) if params_dict is not None else ""
        self.sport_pub.publish(req)

    def send_balance_stand_command(self):
        """Prepare robot for movement by standing up."""
        ROBOT_SPORT_API_ID_BALANCESTAND = 1006
        self._publish_request(ROBOT_SPORT_API_ID_BALANCESTAND)
        self.get_logger().info("BALANCE_STAND command sent (ensuring robot is mobile)")


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
