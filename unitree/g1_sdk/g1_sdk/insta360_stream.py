#!/usr/bin/env python3
"""
OpenMind BrainPack Insta360 Stream RTSP Bridge.

Connects to Insta360 Stream via RTSP, receives video frames,and publishes to ROS2 /camera/insta360/image_raw topic.

Usage:
    ros2 run <package> insta360_stream.py

    or directly:
    python3 insta360_stream.py
"""

import queue
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class Insta360Stream(Node):
    """ROS2 node that bridges Insta360 Stream RTSP to ROS2 /camera/insta360/image_rawtopic."""

    def __init__(self):
        super().__init__("insta360_stream")

        self.declare_parameter("rtsp_url", "rtsp://localhost:8554/top_camera_raw")
        self.declare_parameter("decode_format", "H264")
        self.declare_parameter("fps", 30)

        self.rtsp_url = self.get_parameter("rtsp_url").value
        self.fps = self.get_parameter("fps").value
        self.decode_format = self.get_parameter("decode_format").value

        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_publisher = self.create_publisher(
            Image, "/camera/insta360/image_raw2", qos_profile
        )

        self.cap = None
        self.running = True
        self.initialize_capture()

        self.publish_queue = queue.Queue(maxsize=2)

        self.frame_count = 0
        self.last_fps_time = time.time()
        self.create_timer(2.0, self.report_fps)

        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.publish_thread.start()

        self.get_logger().info("Insta360 Stream Bridge started")
        self.get_logger().info(f"RTSP URL: {self.rtsp_url}")
        self.get_logger().info("Publishing to: /camera/insta360/image_raw")

    def initialize_capture(self):
        """Initialize video capture from RTSP stream."""
        try:
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open RTSP stream: {self.rtsp_url}")
                return False

            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*self.decode_format),  # type: ignore
            )
            self.cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 1000)
            self.cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 100)

            self.get_logger().info("Successfully connected to RTSP stream")
            return True
        except Exception as e:
            self.get_logger().error(f"Error initializing capture: {e}")
            return False

    def _capture_loop(self):
        """Dedicated thread for reading frames from RTSP stream."""
        while self.running and rclpy.ok():
            if self.cap is None or not self.cap.isOpened():
                self.get_logger().warn("Attempting to reconnect to RTSP stream...")
                self.initialize_capture()
                time.sleep(1)
                continue

            try:
                ret, frame = self.cap.read()

                if not ret:
                    self.get_logger().warn(
                        "Failed to capture frame, attempting reconnection..."
                    )
                    self.cap.release()
                    self.initialize_capture()
                    time.sleep(1)
                    continue

                try:
                    self.publish_queue.put_nowait((frame, self.get_clock().now()))
                except queue.Full:
                    pass

                self.frame_count += 1

            except Exception as e:
                self.get_logger().error(f"Error capturing frame: {e}")
                time.sleep(0.1)

    def _publish_loop(self):
        """Dedicated thread for publishing frames to ROS2 topic."""
        while self.running and rclpy.ok():
            try:
                frame, timestamp = self.publish_queue.get(timeout=1.0)

                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                image_msg.header.stamp = timestamp.to_msg()
                image_msg.header.frame_id = "insta360_camera"

                self.image_publisher.publish(image_msg)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error publishing frame: {e}")

    def report_fps(self):
        """Report actual capture FPS."""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        self.get_logger().info(f"Capturing at {fps:.2f} FPS")
        self.frame_count = 0
        self.last_fps_time = current_time

    def destroy_node(self):
        """Clean up resources."""
        self.running = False
        if self.capture_thread is not None and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)

        if self.publish_thread is not None and self.publish_thread.is_alive():
            self.publish_thread.join(timeout=2.0)

        if self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)
    node = None

    try:
        node = Insta360Stream()
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
