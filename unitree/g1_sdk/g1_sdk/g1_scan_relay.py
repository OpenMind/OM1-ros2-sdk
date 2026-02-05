#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    """
    ROS2 node that subscribes to /scan_raw, reverses the LaserScan data, and republishes to /scan.
    """

    def __init__(self):
        super().__init__("scan_relay")

        qos_sub = QoSProfile(depth=10)
        qos_sub.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_sub.durability = DurabilityPolicy.VOLATILE

        qos_pub = QoSProfile(depth=10)
        qos_pub.reliability = ReliabilityPolicy.RELIABLE
        qos_pub.durability = DurabilityPolicy.VOLATILE

        self.sub = self.create_subscription(
            LaserScan, "/scan_raw", self.callback, qos_sub
        )
        self.pub = self.create_publisher(LaserScan, "/scan", qos_pub)

    def callback(self, msg: LaserScan):
        """
        Callback function that reverses the LaserScan data and republishes it.

        Parameters
        ----------
        msg : LaserScan
            The incoming LaserScan message from /scan_raw.
        """
        msg.ranges = list(reversed(msg.ranges))
        msg.intensities = list(reversed(msg.intensities))

        msg.angle_min = msg.angle_min + 3.14159265359
        msg.angle_max = msg.angle_max + 3.14159265359

        self.pub.publish(msg)


def main():
    """
    ROS2 node that subscribes to /scan_raw, reverses the LaserScan data, and republishes to /scan.
    """
    rclpy.init()
    node = ScanRelay()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
