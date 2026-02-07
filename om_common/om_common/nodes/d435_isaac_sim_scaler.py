#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class D435IsaacSimScaler(Node):
    """
    A ROS2 node that subscribes to raw depth and color image topics from Isaac Sim, scales the pixel values by 1000, and republishes them on standard raw topics.
    This is necessary because Isaac Sim may publish depth images in meters (float32) while some ROS2 nodes expect depth in millimeters (uint16).
    """
    def __init__(self):
        super().__init__('d435_isaac_sim_scaler')

        self.bridge = CvBridge()

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/realsense2_camera_node/depth/image_rect_isaac_sim_raw',
            self.depth_callback,
            10
        )

        self.color_sub = self.create_subscription(
            Image,
            '/camera/realsense2_camera_node/color/image_isaac_sim_raw',
            self.color_callback,
            10
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/camera/realsense2_camera_node/depth/image_rect_raw',
            10
        )

        self.color_pub = self.create_publisher(
            Image,
            '/camera/realsense2_camera_node/color/image_raw',
            10
        )

        self.get_logger().info('D435 Isaac Sim Scaler node initialized')
        self.get_logger().info('Subscribing to Isaac Sim raw topics and scaling by 1000')

    def depth_callback(self, msg: Image):
        """
        Scale depth image data by 1000 and republish

        Parameters
        ----------
        msg : sensor_msgs.msg.Image
            The incoming depth image message from Isaac Sim.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            scaled_image = (cv_image * 1000).astype(cv_image.dtype)
            scaled_msg = self.bridge.cv2_to_imgmsg(scaled_image, encoding=msg.encoding)
            scaled_msg.header = msg.header

            self.depth_pub.publish(scaled_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def color_callback(self, msg: Image):
        """
        Scale color image data by 1000 and republish

        Parameters
        ----------
        msg : sensor_msgs.msg.Image
            The incoming color image message from Isaac Sim.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            scaled_image = (cv_image * 1000).astype(cv_image.dtype)
            scaled_msg = self.bridge.cv2_to_imgmsg(scaled_image, encoding=msg.encoding)
            scaled_msg.header = msg.header

            self.color_pub.publish(scaled_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing color image: {str(e)}')


def main(args=None):
    """Main function to run the D435IsaacSimScaler node."""
    rclpy.init(args=args)
    node = D435IsaacSimScaler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
