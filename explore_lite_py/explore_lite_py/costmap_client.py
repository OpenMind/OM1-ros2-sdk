import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
import numpy as np
import threading

class Costmap2DClient:
    def __init__(self, node: Node, tf_buffer: Buffer):
        self.node = node
        self.tf_buffer = tf_buffer
        self.costmap = None
        self.costmap_info = None
        self.global_frame = None
        self.robot_base_frame = None
        self.transform_tolerance = 0.3
        self._lock = threading.Lock()

        # Parameters
        self.node.declare_parameter('robot_base_frame', 'base_link')
        self.node.declare_parameter('costmap_topic', 'costmap')
        self.node.declare_parameter('costmap_updates_topic', 'costmap_updates')

        self.robot_base_frame = self.node.get_parameter('robot_base_frame').value
        costmap_topic = self.node.get_parameter('costmap_topic').value
        costmap_updates_topic = self.node.get_parameter('costmap_updates_topic').value

        # Subscriptions
        self.costmap_sub = self.node.create_subscription(
            OccupancyGrid,
            costmap_topic,
            self.costmap_callback,
            10
        )
        self.costmap_updates_sub = self.node.create_subscription(
            OccupancyGridUpdate,
            costmap_updates_topic,
            self.costmap_updates_callback,
            10
        )

        self.node.get_logger().info("Waiting for costmap to be ready...")
        while self.costmap is None and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().info("Costmap ready.")

    def costmap_callback(self, msg: OccupancyGrid):
        with self._lock:
            self.costmap_info = msg.info
            self.costmap = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            self.global_frame = msg.header.frame_id

    def costmap_updates_callback(self, msg: OccupancyGridUpdate):
        with self._lock:
            if self.costmap is None:
                return

            # Update the costmap
            data = np.array(msg.data, dtype=np.int8).reshape(msg.height, msg.width)
            self.costmap[msg.y:msg.y+msg.height, msg.x:msg.x+msg.width] = data

    def get_robot_pose(self) -> Pose:
        try:
            # Get the transform from global frame to robot base frame
            # We need the position of the robot in the global frame
            # So we look up transform from global_frame to robot_base_frame?
            # No, we want robot pose IN global frame. So we transform (0,0,0) of base_link to global_frame.
            # Or simply lookup transform from base_link to global_frame?
            # lookup_transform(target_frame, source_frame) -> transform source to target
            # We want base_link expressed in global_frame.
            # So target=global_frame, source=base_link

            if self.global_frame is None:
                self.node.get_logger().warn("Global frame not set yet")
                return Pose()

            trans = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time()
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

    def get_costmap(self):
        with self._lock:
            return self.costmap

    def get_costmap_info(self):
        with self._lock:
            return self.costmap_info

    def get_global_frame_id(self):
        return self.global_frame
