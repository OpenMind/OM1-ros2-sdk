#!/usr/bin/env python3
"""
Tron1 Odometry WebSocket Bridge.

Connects to Tron1 via WebSocket, requests odometry data,
and publishes to ROS2 /odom topic.

Usage:
    ros2 run <package> tron1_odom_bridge.py

    or directly:
    python3 tron1_odom_bridge.py
"""

import json
import threading
import time
from uuid import uuid4

import rclpy
import websocket
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Header


class Tron1OdomBridge(Node):
    """ROS2 node that bridges Tron1 WebSocket odometry to ROS2 /odom topic."""

    def __init__(self):
        super().__init__("tron1_odom_bridge")

        # Parameters
        self.declare_parameter("host", "ws://10.192.1.2:5000")
        self.declare_parameter("accid", "WF_TRON1A_299")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        self.host = self.get_parameter("host").value
        self.accid = self.get_parameter("accid").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        # WebSocket
        self.ws = None
        self.connected = False
        self.running = True

        # Start WebSocket thread
        self.ws_thread = threading.Thread(target=self.websocket_loop, daemon=True)
        self.ws_thread.start()

        self.get_logger().info(
            f"Tron1 Odom Bridge started - Host: {self.host}, ACCID: {self.accid}"
        )

    def connect(self):
        """Connect to Tron1 WebSocket."""
        try:
            self.ws = websocket.WebSocketApp(
                self.host,
                on_open=self.on_open,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close,
            )
            return True
        except Exception as e:
            self.get_logger().error(f"WebSocket connection failed: {e}")
            return False

    def on_open(self, ws):
        """Called when WebSocket connection is established."""
        self.connected = True
        self.get_logger().info("WebSocket connected!")

        # Send request to enable odometry
        self.send_enable_odom(True)

    def on_message(self, ws, message):
        """Called when a message is received."""
        try:
            data = json.loads(message)

            if data.get("title") == "notify_odom":
                self.handle_odom(data)
            elif data.get("title") == "response_enable_odom":
                result = data.get("data", {}).get("result", "unknown")
                self.get_logger().info(f"Enable odom response: {result}")

        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse message: {e}")

    def on_error(self, ws, error):
        """Called when an error occurs."""
        self.get_logger().error(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        """Called when connection is closed."""
        self.connected = False
        self.get_logger().warn(f"WebSocket closed: {close_status_code} - {close_msg}")

    def send_enable_odom(self, enable: bool):
        """Send request to enable/disable odometry streaming."""
        if not self.connected or not self.ws:
            return

        msg = {
            "accid": self.accid,
            "title": "request_enable_odom",
            "timestamp": int(time.time() * 1000),
            "guid": uuid4().hex,
            "data": {"enable": enable},
        }

        try:
            self.ws.send(json.dumps(msg))
            self.get_logger().info(f"Sent enable_odom request: enable={enable}")
        except Exception as e:
            self.get_logger().error(f"Failed to send enable_odom: {e}")

    def handle_odom(self, data: dict):
        """Convert WebSocket odom data to ROS2 Odometry message and publish."""
        try:
            odom_data = data.get("data", {})

            # Extract data
            pose_position = odom_data.get("pose_position", [0.0, 0.0, 0.0])
            pose_orientation = odom_data.get("pose_orientation", [0.0, 0.0, 0.0, 1.0])
            twist_linear = odom_data.get("twist_linear", [0.0, 0.0, 0.0])
            twist_angular = odom_data.get("twist_angular", [0.0, 0.0, 0.0])

            # Create Odometry message
            odom_msg = Odometry()

            # Header
            odom_msg.header = Header()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.frame_id
            odom_msg.child_frame_id = self.child_frame_id

            # Pose
            odom_msg.pose = PoseWithCovariance()
            odom_msg.pose.pose = Pose()
            odom_msg.pose.pose.position = Point(
                x=float(pose_position[0]),
                y=float(pose_position[1]),
                z=float(pose_position[2]),
            )
            odom_msg.pose.pose.orientation = Quaternion(
                x=float(pose_orientation[0]),
                y=float(pose_orientation[1]),
                z=float(pose_orientation[2]),
                w=float(pose_orientation[3]),
            )

            # Twist
            odom_msg.twist = TwistWithCovariance()
            odom_msg.twist.twist = Twist()
            odom_msg.twist.twist.linear = Vector3(
                x=float(twist_linear[0]),
                y=float(twist_linear[1]),
                z=float(twist_linear[2]),
            )
            odom_msg.twist.twist.angular = Vector3(
                x=float(twist_angular[0]),
                y=float(twist_angular[1]),
                z=float(twist_angular[2]),
            )

            # Publish
            self.odom_pub.publish(odom_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to handle odom: {e}")

    def websocket_loop(self):
        """Main WebSocket loop with reconnection."""
        while self.running:
            try:
                self.get_logger().info(f"Connecting to {self.host}...")
                self.connect()
                self.ws.run_forever()
            except Exception as e:
                self.get_logger().error(f"WebSocket error: {e}")

            if self.running:
                self.get_logger().info("Reconnecting in 3 seconds...")
                time.sleep(3)

    def destroy_node(self):
        """Cleanup on shutdown."""
        self.running = False

        # Disable odom streaming
        if self.connected:
            self.send_enable_odom(False)
            time.sleep(0.1)

        if self.ws:
            self.ws.close()

        super().destroy_node()


def main(args=None):
    """Run the Tron1 Odom Bridge node."""
    rclpy.init(args=args)
    node = Tron1OdomBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
