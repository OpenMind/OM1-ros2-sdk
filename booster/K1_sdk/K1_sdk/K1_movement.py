#!/usr/bin/env python3
"""ROS2 to K1 Bridge.

Subscribes to /cmd_vel (geometry_msgs/Twist) and forwards to K1
via /remote_controller_state (booster_interface/RemoteControllerState).
"""

import threading
import time

import rclpy
from booster_interface.msg import RemoteControllerState
from geometry_msgs.msg import Twist
from rclpy.node import Node


class K1Bridge(Node):
    """Bridge ROS2 cmd_vel messages to K1 RemoteControllerState."""

    def __init__(self):
        super().__init__("K1_cmd_vel_bridge")

        # Parameters
        self.declare_parameter("max_linear", 1.0)
        self.declare_parameter("max_angular", 1.0)

        self.max_linear = self.get_parameter("max_linear").value
        self.max_angular = self.get_parameter("max_angular").value

        # Movement timeout (stop if no cmd_vel received)
        self.movement_timeout = 1.0  # seconds
        self.last_cmd_time = time.time()

        # Velocity state
        self.lx = 0.0  # Strafe (linear.y)
        self.ly = 0.0  # Forward/Backward (linear.x)
        self.rx = 0.0  # Yaw (angular.z)
        self.running = True

        # Publisher for RemoteControllerState
        self.remote_pub = self.create_publisher(
            RemoteControllerState, "remote_controller_state", 10
        )

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Start 30Hz send loop
        self.send_thread = threading.Thread(target=self.send_loop, daemon=True)
        self.send_thread.start()

        self.get_logger().info(
            "K1 Bridge started - Publishing to /remote_controller_state"
        )

    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to K1 remote controller state."""
        # Update last command time
        self.last_cmd_time = time.time()

        # Normalize to -1 to 1 range
        # ly: left stick vertical (push front to -1, push back to 1) -> Inverted logic from ROS?
        # Standard ROS: +x is forward. RemoteControllerState: push front to -1.
        # So ly = -linear.x
        self.ly = -max(-1.0, min(1.0, msg.linear.x / self.max_linear))

        # lx: left stick horizontal (push left to -1, push right to 1)
        # Standard ROS: +y is left. RemoteControllerState: push left to -1.
        # So lx = -linear.y
        self.lx = -max(-1.0, min(1.0, msg.linear.y / self.max_linear))

        # rx: right stick horizontal (push left to -1, push right to 1)
        # Standard ROS: +z is left turn (CCW). RemoteControllerState: push left to -1.
        # So rx = -angular.z
        self.rx = -max(-1.0, min(1.0, msg.angular.z / self.max_angular))

    def send_loop(self):
        """Send remote controller state at 30Hz."""
        while self.running:
            # Check for movement timeout - stop if no cmd_vel received
            if (time.time() - self.last_cmd_time) > self.movement_timeout:
                self.lx = 0.0
                self.ly = 0.0
                self.rx = 0.0

            msg = RemoteControllerState()
            msg.ly = float(self.ly)
            msg.lx = float(self.lx)
            msg.rx = float(self.rx)
            # ry is unused for basic movement

            self.remote_pub.publish(msg)
            time.sleep(1 / 30)

    def destroy_node(self):
        """Cleanup on shutdown."""
        self.running = False
        # Send stop command
        msg = RemoteControllerState()
        self.remote_pub.publish(msg)
        super().destroy_node()


def main(args=None):
    """Run the K1 bridge node."""
    rclpy.init(args=args)
    node = K1Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
