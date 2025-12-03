#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node


class Go2RemappingNode(Node):
    """
    A ROS2 node that handles remapping of topics for the Go2 robot in Gazebo simulation.
    """

    def __init__(self):
        super().__init__("go2_remapping_node")

        self.get_logger().info("Go2 Remapping Node initialized")

        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.robot_pose_publisher = self.create_publisher(
            PoseStamped, "/utlidar/robot_pose", 10
        )

        self.lidar_subscription = self.create_subscription(
            PointCloud2, "/unitree_lidar/points", self.lidar_callback, 10
        )

        self.lidar_publisher = self.create_publisher(
            PointCloud2, "/utlidar/cloud_deskewed", 10
        )

    def odom_callback(self, msg: Odometry):
        """
        Callback function for odometry messages.
        Remaps odometry data to robot pose with frame_id set to 'odom'.

        Parameters:
        -----------
        msg : nav_msgs.msg.Odometry
            The incoming odometry message containing position and orientation.
        """
        robot_pose = PoseStamped()

        robot_pose.header.stamp = self.get_clock().now().to_msg()
        robot_pose.header.frame_id = "odom"

        robot_pose.pose.position = msg.pose.pose.position
        robot_pose.pose.orientation = msg.pose.pose.orientation

        self.robot_pose_publisher.publish(robot_pose)
        self.get_logger().debug("Published remapped robot pose with frame_id: odom")

    def lidar_callback(self, msg: PointCloud2):
        """
        Callback function for LiDAR point cloud messages.
        Remaps LiDAR data from /unitree_lidar/points to /utlidar/cloud_deskewed.

        Parameters:
        -----------
        msg : sensor_msgs.msg.PointCloud2
            The incoming LiDAR point cloud message.
        """
        self.lidar_publisher.publish(msg)
        self.get_logger().debug("Published remapped LiDAR point cloud")


def main(args=None):
    """
    Main entry point for the go2_remapping_node.
    """
    rclpy.init(args=args)

    node = None
    try:
        node = Go2RemappingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
