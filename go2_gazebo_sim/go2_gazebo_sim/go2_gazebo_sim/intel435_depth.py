#!/usr/bin/env python3

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Intel435DepthNode(Node):
    """
    A ROS2 node that handles remapping of topics for depth camera in Intel RealSense D435.
    Multiplies depth values by 1000 to convert from meters to millimeters.
    """

    def __init__(self):
        super().__init__("intel435_depth_node")

        self.bridge = CvBridge()

        self.get_logger().info("Intel435 Depth Node initialized")

        # Use relative topic names for namespace compatibility
        self.depth_image_subscription = self.create_subscription(
            Image,
            "camera/realsense2_camera_node/depth/image_rect_gazebo_raw",
            self.depth_callback,
            10,
        )

        self.depth_image_publisher = self.create_publisher(
            Image, "camera/realsense2_camera_node/depth/image_rect_raw", 10
        )

    def depth_callback(self, msg: Image):
        """
        Callback function for depth image messages.
        Remaps depth image data from /camera/realsense2_camera_node/depth/image_rect_gazebo_raw
        to /camera/realsense2_camera_node/depth/image_rect_raw

        Multiplies depth values by 1000 to convert from meters to millimeters.

        Parameters:
        -----------
        msg : sensor_msgs.msg.Image
            The incoming depth image message.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            if cv_image.dtype == np.float32:
                processed_image = cv_image * 1000.0
            elif cv_image.dtype == np.uint16:
                processed_image = (cv_image.astype(np.float32) * 1000.0).astype(
                    np.uint16
                )
            else:
                processed_image = cv_image * 1000

            output_msg = self.bridge.cv2_to_imgmsg(
                processed_image, encoding=msg.encoding
            )
            output_msg.header = msg.header
            self.depth_image_publisher.publish(output_msg)

            self.get_logger().debug(
                f"Published remapped depth image with shape: {cv_image.shape}"
            )

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")
            self.depth_image_publisher.publish(msg)


def main(args=None):
    """
    Main entry point for the intel435 depth node.
    """
    rclpy.init(args=args)

    node = None
    try:
        node = Intel435DepthNode()
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
