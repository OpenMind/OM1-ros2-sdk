#!/usr/bin/env python3
"""
K1 Odometry Bridge.

Subscribes to /odometer_state (booster_interface/Odometer)
and publishes to ROS2 /odom topic.

Usage:
    ros2 run <package> k1_odom.py

    or directly:
    python3 k1_odom.py
"""

import math

import rclpy
from booster_interface.msg import Odometer
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


class K1OdomBridge(Node):
    """ROS2 node that bridges K1 Odometer msg to ROS2 /odom topic."""

    def __init__(self):
        super().__init__("k1_odom_bridge")

        # Parameters
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        # Subscriber
        self.subscription = self.create_subscription(
            Odometer, "odometer_state", self.odom_callback, 10
        )

        self.get_logger().info(
            "K1 Odom Bridge started - Subscribing to /odometer_state"
        )

    def odom_callback(self, msg: Odometer):
        """Convert Odometer msg to ROS2 Odometry message and publish."""
        try:
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
                x=float(msg.x),
                y=float(msg.y),
                z=0.0,
            )

            # Convert theta to quaternion
            # Yaw (theta) rotation around Z axis
            cy = math.cos(msg.theta * 0.5)
            sy = math.sin(msg.theta * 0.5)
            # cp = math.cos(0 * 0.5) = 1
            # sp = math.sin(0 * 0.5) = 0
            # cr = math.cos(0 * 0.5) = 1
            # sr = math.sin(0 * 0.5) = 0

            # w = cr * cp * cy + sr * sp * sy = 1 * 1 * cy = cy
            # x = sr * cp * cy - cr * sp * sy = 0
            # y = cr * sp * cy + sr * cp * sy = 0
            # z = cr * cp * sy - sr * sp * cy = 1 * 1 * sy = sy

            odom_msg.pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=float(sy),
                w=float(cy),
            )

            # Twist - Not provided in Odometer msg, usually required for odom
            # We leave it as zero or could compute from difference if needed.
            odom_msg.twist = TwistWithCovariance()
            odom_msg.twist.twist = Twist()
            odom_msg.twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
            odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

            # Publish
            self.odom_pub.publish(odom_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to handle odom: {e}")


def main(args=None):
    """Run the K1 Odom Bridge node."""
    rclpy.init(args=args)
    node = K1OdomBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
