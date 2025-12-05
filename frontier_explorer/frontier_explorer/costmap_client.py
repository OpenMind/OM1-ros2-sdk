import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.node import Node
from tf2_ros import Buffer


class Costmap2DClient:
    """Client for accessing and managing costmap data from Nav2.

    Subscribes to costmap topics, handles updates, and provides thread-safe
    access to costmap data and robot pose information.
    """

    def __init__(self, node: Node, tf_buffer: Buffer):
        """Initialize the Costmap2DClient with ROS2 node and TF buffer.

        Sets up subscriptions to costmap and costmap updates topics, initializes
        parameters, and waits for initial costmap data to be available.

        Parameters:
        ----------
        node: Node
            The ROS2 node used for subscriptions and logging.
        tf_buffer: Buffer
            The TF2 buffer for looking up transforms.
        """
        self.node = node
        self.tf_buffer = tf_buffer
        self.costmap = None
        self.costmap_info = None
        self.global_frame = None
        self.robot_base_frame = None
        self.transform_tolerance = 0.3
        self._lock = threading.Lock()

        # Parameters
        self.node.declare_parameter("robot_base_frame", "base_link")
        self.node.declare_parameter("costmap_topic", "costmap")
        self.node.declare_parameter("costmap_updates_topic", "costmap_updates")

        self.robot_base_frame = self.node.get_parameter("robot_base_frame").value
        costmap_topic = self.node.get_parameter("costmap_topic").value
        costmap_updates_topic = self.node.get_parameter("costmap_updates_topic").value

        # Subscriptions
        self.costmap_sub = self.node.create_subscription(
            OccupancyGrid, costmap_topic, self.costmap_callback, 10
        )
        self.costmap_updates_sub = self.node.create_subscription(
            OccupancyGridUpdate,
            costmap_updates_topic,
            self.costmap_updates_callback,
            10,
        )

        self.node.get_logger().info("Waiting for costmap to be ready...")
        while self.costmap is None and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().info("Costmap ready.")

    def costmap_callback(self, msg: OccupancyGrid):
        """Handle incoming full costmap messages.

        Updates the entire costmap data, metadata, and global frame information.
        Thread-safe operation using a lock.

        Parameters:
        ----------
        msg: OccupancyGrid
            Full costmap message containing the entire costmap data.
        """
        with self._lock:
            self.costmap_info = msg.info
            self.costmap = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width
            )
            self.global_frame = msg.header.frame_id

    def costmap_updates_callback(self, msg: OccupancyGridUpdate):
        """Handle incremental costmap update messages.

        Applies partial updates to the existing costmap for efficiency.
        Thread-safe operation using a lock.

        Parameters:
        ----------
        msg: OccupancyGridUpdate
            Incremental costmap update message containing updated regions.
        """
        with self._lock:
            if self.costmap is None:
                return

            # Update the costmap
            data = np.array(msg.data, dtype=np.int8).reshape(msg.height, msg.width)
            self.costmap[msg.y : msg.y + msg.height, msg.x : msg.x + msg.width] = data

    def get_robot_pose(self) -> Pose:
        """Get the current pose of the robot in the global frame.

        Uses TF2 to look up the transform from the global frame to the robot base frame.

        Returns:
        -------
        Pose
            The current pose of the robot, or a default Pose if unavailable.
        """
        try:
            if self.global_frame is None:
                self.node.get_logger().warn("Global frame not set yet")
                return Pose()

            trans = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, rclpy.time.Time()
            )

            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            self.node.get_logger().warn(f"Could not get robot pose: {e}")
            return Pose()

    def get_costmap(self) -> np.ndarray:
        """Get the current costmap data.

        Thread-safe accessor for the costmap numpy array.

        Returns:
        -------
        np.ndarray
            2D array representing the costmap, or None if not yet available.
        """
        with self._lock:
            return self.costmap

    def get_costmap_info(self) -> MapMetaData:
        """Get the costmap metadata information.

        Thread-safe accessor for costmap metadata including resolution, dimensions,
        and origin.

        Returns:
        -------
        MapMetaDatas
            Metadata of the costmap, or None if not yet available.
        """
        with self._lock:
            return self.costmap_info

    def get_global_frame_id(self) -> str:
        """Get the global reference frame ID for the costmap.

        Returns:
        -------
        str
            The global frame ID, or None if not yet available.
        """
        return self.global_frame
