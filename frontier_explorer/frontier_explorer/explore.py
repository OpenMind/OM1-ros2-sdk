import json
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from frontier_explorer.costmap_client import Costmap2DClient
from frontier_explorer.frontier_search import FrontierSearch


class Explore(Node):
    """
    A ROS2 node that performs autonomous exploration using frontier-based methods.
    """

    def __init__(self):
        """
        Initialize the Explore node with ROS2 parameters, clients, and timers.

        Sets up the navigation action client, costmap client, frontier search,
        publishers, subscriptions, and begins the exploration process.
        """
        super().__init__("explore_node")

        self.logger = self.get_logger()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.costmap_client = Costmap2DClient(self, self.tf_buffer)

        self.prev_distance = 0
        self.last_markers_count = 0
        self.exploration_complete = False
        self.frontier_blacklist = {}
        self.last_progress = self.get_clock().now()
        self.resuming = False
        self.initial_pose = None
        self.prev_goal = None
        self.current_goal_handle = None

        self.last_robot_pose = None
        self.last_pose_time = None

        # Parameters
        self.declare_parameter("planner_frequency", 1.0)
        self.declare_parameter("progress_timeout", 30.0)
        self.declare_parameter("min_velocity_threshold", 0.01)
        self.declare_parameter("blacklist_attempts", 3)
        self.declare_parameter("blacklist_duration", 300.0)
        self.declare_parameter("visualize", False)
        self.declare_parameter("potential_scale", 1e-3)
        self.declare_parameter("orientation_scale", 0.0)
        self.declare_parameter("gain_scale", 1.0)
        self.declare_parameter("min_frontier_size", 0.5)
        self.declare_parameter("return_to_init", False)

        self.planner_frequency = self.get_parameter("planner_frequency").value
        self.progress_timeout = self.get_parameter("progress_timeout").value
        self.min_velocity_threshold = self.get_parameter("min_velocity_threshold").value
        self.blacklist_attempts = self.get_parameter("blacklist_attempts").value
        self.blacklist_duration = self.get_parameter("blacklist_duration").value
        self.visualize = self.get_parameter("visualize").value
        self.potential_scale = self.get_parameter("potential_scale").value
        self.orientation_scale = self.get_parameter("orientation_scale").value
        self.gain_scale = self.get_parameter("gain_scale").value
        self.min_frontier_size = self.get_parameter("min_frontier_size").value
        self.return_to_init = self.get_parameter("return_to_init").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value

        self.move_base_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.search = FrontierSearch(
            self.costmap_client,
            self.potential_scale,
            self.gain_scale,
            self.min_frontier_size,
        )

        if self.visualize:
            self.marker_array_publisher = self.create_publisher(
                MarkerArray, "explore/frontiers", 10
            )

        self.exploration_status_publisher = self.create_publisher(
            String, "explore/status", 10
        )

        self.resume_subscription = self.create_subscription(
            Bool, "explore/resume", self.resume_callback, 10
        )

        self.logger.info("Waiting to connect to move_base nav2 server")
        self.move_base_client.wait_for_server()
        self.logger.info("Connected to move_base nav2 server")

        if self.return_to_init:
            self.logger.info(
                "Will attempt to get initial pose for return-to-start feature"
            )

        self.logger.info("Starting exploration...")
        self.exploring_timer = self.create_timer(
            1.0 / self.planner_frequency, self.make_plan
        )

        self.status_reporter_timer = self.create_timer(5.0, self.report_status)

        self.publish_exploration_status(False, "Exploration initialized")

    def resume_callback(self, msg: Bool):
        """
        Handle resume/stop commands from the explore/resume topic.

        Parameters
        ----------
        msg: Bool
            Message indicating whether to resume (True) or stop (False) exploration.
        """
        if msg.data:
            self.resume()
        else:
            self.stop()

    def visualize_frontiers(self, frontiers: list):
        """
        Publish visualization markers for frontiers in RViz.

        Creates point markers for frontier boundaries and sphere markers for centroids.
        Blacklisted frontiers are shown in red, others in blue, with centroids in green.

        Parameters
        ----------
        frontiers: list
            List of Frontier objects to visualize.
        """
        blue = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        markers_msg = MarkerArray()
        current_time = self.get_clock().now().to_msg()

        min_cost = frontiers[0].cost if frontiers else 0.0

        id = 0
        for frontier in frontiers:
            # Points marker
            m = Marker()
            m.header.frame_id = self.costmap_client.get_global_frame_id()
            m.header.stamp = current_time
            m.ns = "frontiers"
            m.id = id
            id += 1
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.points = frontier.points
            if self.goal_on_blacklist(frontier.centroid):
                m.color = red
            else:
                m.color = blue
            m.lifetime = Duration(seconds=0).to_msg()
            m.frame_locked = True
            markers_msg.markers.append(m)

            # Sphere marker (centroid)
            m_sphere = Marker()
            m_sphere.header.frame_id = self.costmap_client.get_global_frame_id()
            m_sphere.header.stamp = current_time
            m_sphere.ns = "frontiers"
            m_sphere.id = id
            id += 1
            m_sphere.type = Marker.SPHERE
            m_sphere.action = Marker.ADD
            m_sphere.pose.position = frontier.initial
            m_sphere.pose.orientation.w = 1.0
            scale = (
                min(abs(min_cost * 0.4 / frontier.cost), 0.5)
                if frontier.cost != 0
                else 0.1
            )
            m_sphere.scale.x = scale
            m_sphere.scale.y = scale
            m_sphere.scale.z = scale
            m_sphere.color = green
            m_sphere.lifetime = Duration(seconds=0).to_msg()
            m_sphere.frame_locked = True
            markers_msg.markers.append(m_sphere)

        # Delete unused markers
        for i in range(id, self.last_markers_count):
            m = Marker()
            m.ns = "frontiers"
            m.id = i
            m.action = Marker.DELETE
            markers_msg.markers.append(m)

        self.last_markers_count = id
        self.marker_array_publisher.publish(markers_msg)

    def check_robot_progress(self, current_pose: Point) -> bool:
        """
        Check if the robot is making progress using velocity-based detection.

        Progress is defined as the robot moving at a velocity above the threshold,
        regardless of whether it's getting closer to the goal (handles obstacle avoidance).

        Parameters
        ----------
        current_pose: Point
            The robot's current position.

        Returns
        -------
        bool
            True if robot is making progress (moving), False if stuck.
        """
        current_time = self.get_clock().now()

        if self.last_robot_pose is None or self.last_pose_time is None:
            self.last_robot_pose = current_pose
            self.last_pose_time = current_time
            return True

        time_delta = (current_time - self.last_pose_time).nanoseconds / 1e9

        if time_delta < 0.1:
            return True

        dx = current_pose.x - self.last_robot_pose.x
        dy = current_pose.y - self.last_robot_pose.y
        distance = math.sqrt(dx * dx + dy * dy)

        velocity = distance / time_delta

        self.last_robot_pose = current_pose
        self.last_pose_time = current_time

        is_moving = velocity >= self.min_velocity_threshold

        if not is_moving:
            self.logger.debug(
                f"Robot velocity: {velocity:.3f} m/s (threshold: {self.min_velocity_threshold:.3f} m/s)"
            )

        return is_moving

    def make_plan(self):
        """
        Main planning loop that searches for frontiers and sends navigation goals.

        Finds available frontiers, filters out blacklisted ones, selects the best
        frontier based on cost, and sends a navigation goal. Handles timeout detection
        and exploration completion. Called periodically by the planner timer.
        """
        if self.return_to_init and self.initial_pose is None:
            self.capture_initial_pose_if_needed()

        pose = self.costmap_client.get_robot_pose()
        frontiers = self.search.search_from(pose.position)

        self.logger.debug(f"found {len(frontiers)} frontiers")

        if not frontiers:
            self.logger.warn("No frontiers found, stopping.")
            self.exploration_complete = True
            self.publish_exploration_status(
                True, "No frontiers found - exploration complete"
            )
            self.stop(True)
            return

        if self.visualize:
            self.visualize_frontiers(frontiers)

        # Find non-blacklisted frontier
        frontier = None
        target_position = None

        for f in frontiers:
            if self.goal_on_blacklist(f.centroid):
                continue

            candidate = f.centroid

            # If centroid is too close to the robot, use the closest point on the frontier instead
            dist_to_centroid = math.sqrt(
                (candidate.x - pose.position.x) ** 2
                + (candidate.y - pose.position.y) ** 2
            )
            if dist_to_centroid < 0.5:
                candidate = f.middle
                if self.goal_on_blacklist(candidate):
                    continue
                self.logger.info(
                    f"Centroid is too close ({dist_to_centroid:.2f}m), using closest frontier point instead"
                )

            frontier = f
            target_position = candidate
            break

        if frontier is None:
            self.logger.warn("All frontiers traversed/tried out, stopping.")
            self.exploration_complete = True
            self.publish_exploration_status(
                True, "All frontiers explored - exploration complete"
            )
            self.stop(True)
            return

        if not self.exploration_complete:
            info = f"Exploring - {len(frontiers)} frontiers available"
            self.publish_exploration_status(False, info)

        if self.prev_goal is not None:
            same_goal = self.same_point(self.prev_goal, target_position)
        else:
            same_goal = False

        self.prev_goal = target_position

        is_making_progress = self.check_robot_progress(pose.position)

        getting_closer = self.prev_distance > frontier.min_distance

        # Reset progress timer if:
        # 1. Goal changed (new frontier selected)
        # 2. Robot is moving (velocity-based)
        # 3. Robot is getting closer to goal (distance-based backup)
        if not same_goal or is_making_progress or getting_closer:
            self.last_progress = self.get_clock().now()
            self.prev_distance = frontier.min_distance

        # Timeout check - only trigger if robot is truly stuck
        time_since_progress = (
            self.get_clock().now() - self.last_progress
        ).nanoseconds / 1e9

        if time_since_progress > self.progress_timeout and not self.resuming:
            self.add_to_blacklist(target_position)
            self.logger.warn(
                f"Robot stuck for {time_since_progress:.1f}s (no movement detected). Adding goal to blacklist."
            )
            self.last_robot_pose = None
            self.last_pose_time = None
            self.make_plan()
            return

        if self.resuming:
            self.resuming = False

        if same_goal:
            return

        distance_to_goal = frontier.min_distance
        self.logger.info(
            f"Sending goal to move base nav2: ({target_position.x:.2f}, {target_position.y:.2f}) at {distance_to_goal:.1f}m distance"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position = target_position
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.header.frame_id = self.costmap_client.get_global_frame_id()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        future = self.move_base_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self.goal_response_callback(f, target_position)
        )

    def goal_response_callback(self, future: object, target_position: Point):
        """
        Handle the response from the navigation action server.

        Parameters
        ----------
        future: object
            The future object containing the goal response.
        target_position: Point
            The target position of the navigation goal.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info("Goal rejected :(")
            return

        self.logger.info("Goal accepted")
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.reached_goal(f, target_position, goal_handle)
        )

    def reached_goal(self, future: object, frontier_goal: Point, goal_handle):
        """
        Process the result of a navigation goal attempt.

        Handles successful completion, aborted goals (adds to blacklist), and
        canceled goals. Triggers the next planning cycle on success.

        Parameters
        ----------
        future: object
            The future object containing the result.
        frontier_goal: Point
            The target position of the navigation goal.
        goal_handle: goal_handle
            The goal handle for the navigation action.
        """
        if self.current_goal_handle is goal_handle:
            self.current_goal_handle = None

        result = future.result()
        status = result.status

        from action_msgs.msg import GoalStatus

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.debug("Goal was successful")
            self.last_robot_pose = None
            self.last_pose_time = None
        elif status == GoalStatus.STATUS_ABORTED:
            self.logger.warn("Goal was aborted")
            self.add_to_blacklist(frontier_goal)
            self.logger.warn("Adding current goal to blacklist")
            self.last_robot_pose = None
            self.last_pose_time = None
            return
        elif status == GoalStatus.STATUS_CANCELED:
            self.logger.debug("Goal was canceled")
            self.last_robot_pose = None
            self.last_pose_time = None
            return
        else:
            self.logger.warn("Unknown result code from move base nav2")

        self.make_plan()

    def start(self):
        """
        Start the exploration process.

        Note: Exploration actually starts automatically in __init__.
        This method mainly serves as a status indicator.
        """
        self.logger.info("Exploration started.")

    def stop(self, finished_exploring=False):
        """
        Stop the exploration process and cancel any active navigation goals.

        Parameters
        ----------
        finished_exploring: bool
            Indicates if exploration has completed naturally.
        """
        self.logger.info("Exploration stopped.")
        self.exploring_timer.cancel()

        if self.current_goal_handle:
            self.logger.info("Cancelling current nav2 goal")
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

        if self.return_to_init and finished_exploring:
            self.return_to_initial_pose()

    def resume(self):
        """
        Resume exploration after it has been stopped.

        Resets the exploration complete flag, restarts the planning timer,
        and immediately triggers a new planning cycle.
        """
        self.resuming = True
        self.logger.info("Exploration resuming.")
        self.exploration_complete = False
        # Reset velocity tracking
        self.last_robot_pose = None
        self.last_pose_time = None
        self.publish_exploration_status(False, "Exploration resumed")
        self.exploring_timer.reset()
        self.make_plan()

    def return_to_initial_pose(self):
        """
        Send a navigation goal to return to the initial pose.

        Only executes if an initial pose was captured. Sends the robot back
        to where exploration started.
        """
        if self.initial_pose is None:
            return

        self.logger.info("Returning to initial pose.")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position = self.initial_pose.position
        goal_msg.pose.pose.orientation = self.initial_pose.orientation
        goal_msg.pose.header.frame_id = self.costmap_client.get_global_frame_id()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.move_base_client.send_goal_async(goal_msg)

    def capture_initial_pose_if_needed(self):
        """
        Capture the robot's current pose as the initial pose for return-to-start.

        Uses TF2 to get the robot's position in the global frame. Only captures
        if return_to_init parameter is enabled and pose hasn't been captured yet.
        """
        if not self.return_to_init:
            return

        try:
            map_frame = self.costmap_client.get_global_frame_id()
            if map_frame is None:
                return

            # Try to get current transform
            t = self.tf_buffer.lookup_transform(
                map_frame, self.robot_base_frame, Time(), timeout=Duration(seconds=0.1)
            )

            self.initial_pose = PoseStamped()
            self.initial_pose.pose.position.x = t.transform.translation.x
            self.initial_pose.pose.position.y = t.transform.translation.y
            self.initial_pose.pose.position.z = t.transform.translation.z
            self.initial_pose.pose.orientation = t.transform.rotation

            self.logger.info(
                f"Initial pose captured: ({self.initial_pose.pose.position.x:.2f}, {self.initial_pose.pose.position.y:.2f})"
            )

        except TransformException as ex:
            self.logger.debug(
                f"Still waiting for transform to capture initial pose: {ex}"
            )

    def publish_exploration_status(self, complete: bool, info: str):
        """
        Publish the current exploration status to ROS topic.

        Parameters
        ----------
        complete: bool
            True if exploration is complete, False otherwise.
        info: str
            Additional information about the exploration status.
        """
        status_msg = String()
        status_data = {"complete": complete, "info": info if info else ""}
        status_msg.data = json.dumps(status_data)
        self.exploration_status_publisher.publish(status_msg)

        self.logger.info(
            f"Exploration status: {'COMPLETE' if complete else 'ACTIVE'} - {info}"
        )

    def goal_on_blacklist(self, goal: Point):
        """
        Check if a frontier should be skipped based on attempts and time.
        Give up after 3 attempts or wait 5 minutes before retry.

        Parameters
        ----------
        goal: Point
            The goal point to check against the blacklist.
        """
        key = (round(goal.x, 1), round(goal.y, 1))
        if key in self.frontier_blacklist:
            data = self.frontier_blacklist[key]

            # Give up after max attempts
            if data["attempts"] >= self.blacklist_attempts:
                self.get_logger().info(
                    f"Goal ({goal.x:.2f}, {goal.y:.2f}) blacklisted due to too many attempts"
                )
                return True

            # Retry after blacklist duration
            time_since_last = (
                self.get_clock().now() - data["last_attempt"]
            ).nanoseconds / 1e9
            if time_since_last < self.blacklist_duration:
                self.get_logger().info(
                    f"Goal ({goal.x:.2f}, {goal.y:.2f}) still blacklisted, last attempt {time_since_last:.1f}s ago"
                )
                return True
            else:
                self.get_logger().info(
                    f"Goal ({goal.x:.2f}, {goal.y:.2f}) removed from blacklist after timeout"
                )
                del self.frontier_blacklist[key]

        return False

    def add_to_blacklist(self, goal: Point):
        """
        Add or update a goal in the blacklist with attempt tracking.

        Parameters
        ----------
        goal: Point
            The goal point to add to the blacklist.
        """
        key = (round(goal.x, 1), round(goal.y, 1))
        if key in self.frontier_blacklist:
            self.frontier_blacklist[key]["attempts"] += 1
            self.frontier_blacklist[key]["last_attempt"] = self.get_clock().now()
        else:
            self.frontier_blacklist[key] = {
                "attempts": 1,
                "last_attempt": self.get_clock().now(),
            }
        self.get_logger().info(
            f"Added/updated goal ({goal.x:.2f}, {goal.y:.2f}) in blacklist with {self.frontier_blacklist[key]['attempts']} attempts"
        )

    def same_point(self, p1: Point, p2: Point):
        """Check if two points are essentially the same location.

        Parameters
        ----------
        p1: Point
            First point to compare.
        p2: Point
            Second point to compare.

        Returns
        -------
        bool
            True if points are within 1 cm of each other, False otherwise.
        """
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dist = math.sqrt(dx * dx + dy * dy)
        return dist < 0.01

    def report_status(self):
        """
        Periodic status reporter called by timer.

        Publishes exploration status messages at regular intervals to keep
        monitoring systems updated.
        """
        if self.exploration_complete:
            self.publish_exploration_status(True, "Exploration complete")
        else:
            self.publish_exploration_status(False, "Exploring")


def main(args=None):
    """
    Main entry point for the explore node.

    Initializes ROS2, creates the Explore node, and spins until shutdown.

    Parameters
    ----------
    args: list, optional
        Command line arguments for ROS2 initialization.
    """
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
