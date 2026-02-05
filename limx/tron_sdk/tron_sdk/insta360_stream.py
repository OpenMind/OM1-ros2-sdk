#!/usr/bin/env python3
"""
OpenMind BrainPack Insta360 Stream RTSP Bridge.

Connects to Insta360 Stream via RTSP, receives video frames,and publishes to ROS2 /insta360/camera/image_raw topic.

Usage:
    ros2 run <package> insta360_stream.py

    or directly:
    python3 insta360_stream.py
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Insta360Stream(Node):
    """ROS2 node that bridges Insta360 Stream RTSP to ROS2 /insta360/camera/image_raw topic."""

    def __init__(self):
        super().__init__("insta360_stream")

        self.declare_parameter("rtsp_url", "rtsp://localhost:8554/top_camera")
        self.declare_parameter("decode_format", "H264")
        self.declare_parameter("fps", 30)

        self.rtsp_url = self.get_parameter("rtsp_url").value
        self.fps = self.get_parameter("fps").value

        self.bridge = CvBridge()

        self.image_publisher = self.create_publisher(
            Image, "/insta360/camera/image_raw", 10
        )

        self.cap = None
        self.initialize_capture()

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture_and_publish)

        self.get_logger().info(f"Insta360 Stream Bridge started")
        self.get_logger().info(f"RTSP URL: {self.rtsp_url}")
        self.get_logger().info(f"Publishing to: /insta360/camera/image_raw")

    def initialize_capture(self):
        """Initialize video capture from RTSP stream."""
        try:
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open RTSP stream: {self.rtsp_url}")
                return False

            self.get_logger().info("Successfully connected to RTSP stream")
            return True
        except Exception as e:
            self.get_logger().error(f"Error initializing capture: {e}")
            return False

    def capture_and_publish(self):
        """Capture frame from RTSP stream and publish to ROS2 topic."""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn("Attempting to reconnect to RTSP stream...")
            self.initialize_capture()
            return

        try:
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().warn("Failed to capture frame, attempting reconnection...")
                self.cap.release()
                self.initialize_capture()
                return

            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "insta360_camera"

            self.image_publisher.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f"Error capturing/publishing frame: {e}")

    def destroy_node(self):
        """Clean up resources."""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)

    try:
        node = Insta360Stream()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

