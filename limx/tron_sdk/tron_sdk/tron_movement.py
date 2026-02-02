#!/usr/bin/env python3
"""ROS2 to Tron1 WebSocket Bridge.

Subscribes to /cmd_vel (geometry_msgs/Twist) and forwards to Tron1 via WebSocket.
Sends commands at 30Hz as required by Tron1 SDK.

Usage:
    ros2 run tron_sdk cmd_vel_to_tron

    or directly:
    python3 tron_movement.py

Then publish cmd_vel:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import json
import threading
import time
from uuid import uuid4

import rclpy
import websocket
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Tron1Bridge(Node):
    """Bridge ROS2 cmd_vel messages to Tron1 WebSocket API."""

    def __init__(self):
        super().__init__("tron1_cmd_vel_bridge")

        # Parameters
        self.declare_parameter("host", "ws://10.192.1.2:5000")
        self.declare_parameter("accid", "WF_TRON1A_299")
        self.declare_parameter("max_linear", 1.0)
        self.declare_parameter("max_angular", 1.0)

        self.host = self.get_parameter("host").value
        self.accid = self.get_parameter("accid").value
        self.max_linear = self.get_parameter("max_linear").value
        self.max_angular = self.get_parameter("max_angular").value

        # Movement timeout (stop if no cmd_vel received)
        self.movement_timeout = 1.0  # seconds
        self.last_cmd_time = time.time()

        # Velocity state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.running = True

        # WebSocket connection (retry on boot to wait for Tron1)
        self.ws = None
        self.connect_ws(retry_on_fail=True)

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Start 30Hz send loop
        self.send_thread = threading.Thread(target=self.send_loop, daemon=True)
        self.send_thread.start()

        self.get_logger().info(
            f"Tron1 Bridge started - Host: {self.host}, ACCID: {self.accid}"
        )
        self.get_logger().info("Subscribing to /cmd_vel")

    def connect_ws(self, retry_on_fail: bool = False, max_retries: int = 10):
        """Connect to Tron1 WebSocket with optional retry logic.

        Args:
            retry_on_fail: If True, retry connection on failure (for boot-time connection)
            max_retries: Maximum number of retry attempts when retry_on_fail is True
        """
        retries = 0
        while True:
            try:
                self.ws = websocket.create_connection(self.host, timeout=5)
                self.get_logger().info("WebSocket connected!")
                return True
            except Exception as e:
                self.get_logger().error(f"WebSocket connection failed: {e}")
                self.ws = None

                if not retry_on_fail:
                    return False

                retries += 1
                if retries >= max_retries:
                    self.get_logger().error(
                        f"Failed to connect after {max_retries} attempts, "
                        "will continue retrying in send loop"
                    )
                    return False

                self.get_logger().info(
                    f"Retrying connection in 3 seconds... ({retries}/{max_retries})"
                )
                time.sleep(3)

    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to Tron1 ratio format (-1 to 1)."""
        # Update last command time
        self.last_cmd_time = time.time()

        # Normalize to -1 to 1 range
        self.x = max(-1.0, min(1.0, msg.linear.x / self.max_linear))
        self.y = max(-1.0, min(1.0, msg.linear.y / self.max_linear))
        self.z = max(-1.0, min(1.0, msg.angular.z / self.max_angular))

    def send_loop(self):
        """Send twist commands at 30Hz."""
        while self.running:
            # Check for movement timeout - stop if no cmd_vel received
            if (time.time() - self.last_cmd_time) > self.movement_timeout:
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

            if self.ws:
                try:
                    msg = {
                        "accid": self.accid,
                        "title": "request_twist",
                        "timestamp": int(time.time() * 1000),
                        "guid": uuid4().hex,
                        "data": {"x": self.x, "y": self.y, "z": self.z},
                    }
                    self.ws.send(json.dumps(msg))
                except Exception as e:
                    self.get_logger().warn(f"Send failed: {e}, reconnecting...")
                    self.connect_ws()
            time.sleep(1 / 30)

    def destroy_node(self):
        """Cleanup on shutdown."""
        self.running = False
        if self.ws:
            # Send stop command
            try:
                msg = {
                    "accid": self.accid,
                    "title": "request_twist",
                    "timestamp": int(time.time() * 1000),
                    "guid": uuid4().hex,
                    "data": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
                self.ws.send(json.dumps(msg))
                self.ws.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    """Run the Tron1 bridge node."""
    rclpy.init(args=args)
    node = Tron1Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
