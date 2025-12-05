import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from std_msgs.msg import Bool, String, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException
import math
import threading
from explore_lite_py.costmap_client import Costmap2DClient
from explore_lite_py.frontier_search import FrontierSearch

class Explore(Node):
    def __init__(self):
        super().__init__('explore_node')

        self.logger = self.get_logger()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.costmap_client = Costmap2DClient(self, self.tf_buffer)

        self.prev_distance = 0
        self.last_markers_count = 0
        self.exploration_complete = False
        self.frontier_blacklist = []
        # self.prev_goal = Point() # Removed
        self.last_progress = self.get_clock().now()
        self.resuming = False
        self.initial_pose = None
        self.prev_goal = None

        # Parameters
        self.declare_parameter('planner_frequency', 1.0)
        self.declare_parameter('progress_timeout', 30.0)
        self.declare_parameter('visualize', False)
        self.declare_parameter('potential_scale', 1e-3)
        self.declare_parameter('orientation_scale', 0.0)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('min_frontier_size', 0.5)
        self.declare_parameter('return_to_init', False)

        self.planner_frequency = self.get_parameter('planner_frequency').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.visualize = self.get_parameter('visualize').value
        self.potential_scale = self.get_parameter('potential_scale').value
        self.orientation_scale = self.get_parameter('orientation_scale').value
        self.gain_scale = self.get_parameter('gain_scale').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.return_to_init = self.get_parameter('return_to_init').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value

        self.move_base_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.search = FrontierSearch(self.costmap_client, self.potential_scale, self.gain_scale, self.min_frontier_size)

        if self.visualize:
            self.marker_array_publisher = self.create_publisher(MarkerArray, 'explore/frontiers', 10)

        self.exploration_status_publisher = self.create_publisher(Bool, 'explore/status', 10)
        self.exploration_info_publisher = self.create_publisher(String, 'explore/info', 10)

        self.resume_subscription = self.create_subscription(Bool, 'explore/resume', self.resume_callback, 10)

        self.logger.info("Waiting to connect to move_base nav2 server")
        self.move_base_client.wait_for_server()
        self.logger.info("Connected to move_base nav2 server")

        if self.return_to_init:
            self.logger.info("Will attempt to get initial pose for return-to-start feature")
            # We will try to capture it in make_plan if not yet captured

        self.logger.info("Starting exploration...")
        self.exploring_timer = self.create_timer(1.0 / self.planner_frequency, self.make_plan)

        self.status_reporter_timer = self.create_timer(5.0, self.report_status)

        # Initial status
        self.publish_exploration_status(False, "Exploration initialized")

    def resume_callback(self, msg):
        if msg.data:
            self.resume()
        else:
            self.stop()

    def visualize_frontiers(self, frontiers):
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
            scale = min(abs(min_cost * 0.4 / frontier.cost), 0.5) if frontier.cost != 0 else 0.1
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

    def make_plan(self):
        if self.return_to_init and self.initial_pose is None:
            self.capture_initial_pose_if_needed()

        pose = self.costmap_client.get_robot_pose()
        frontiers = self.search.search_from(pose.position)

        self.logger.debug(f"found {len(frontiers)} frontiers")

        if not frontiers:
            self.logger.warn("No frontiers found, stopping.")
            self.exploration_complete = True
            self.publish_exploration_status(True, "No frontiers found - exploration complete")
            self.stop(True)
            return

        if self.visualize:
            self.visualize_frontiers(frontiers)

        # Find non-blacklisted frontier
        frontier = None
        for f in frontiers:
            if not self.goal_on_blacklist(f.centroid):
                frontier = f
                break

        if frontier is None:
            self.logger.warn("All frontiers traversed/tried out, stopping.")
            self.exploration_complete = True
            self.publish_exploration_status(True, "All frontiers explored - exploration complete")
            self.stop(True)
            return

        target_position = frontier.centroid

        if not self.exploration_complete:
            info = f"Exploring - {len(frontiers)} frontiers available"
            self.publish_exploration_status(False, info)

        # Timeout check
        if self.prev_goal is not None:
            same_goal = self.same_point(self.prev_goal, target_position)
        else:
            same_goal = False

        self.prev_goal = target_position

        if not same_goal or self.prev_distance > frontier.min_distance:
            self.last_progress = self.get_clock().now()
            self.prev_distance = frontier.min_distance

        if (self.get_clock().now() - self.last_progress).nanoseconds / 1e9 > self.progress_timeout and not self.resuming:
            self.frontier_blacklist.append(target_position)
            self.logger.debug("Adding current goal to black list")
            # Recursively call make_plan? Or just return and let timer call it again?
            # C++ calls makePlan().
            # But we are in a timer callback. Calling make_plan recursively might be bad if depth is high.
            # But here we just blacklisted one, so next call will pick another one.
            # Let's just return, timer will trigger again soon.
            # Actually, if we return, we wait for next timer tick.
            # If we want immediate replan, we can call make_plan again.
            # Let's call it.
            self.make_plan()
            return

        if self.resuming:
            self.resuming = False

        if same_goal:
            return

        self.logger.info(f"Sending goal to move base nav2: ({target_position.x:.2f}, {target_position.y:.2f})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position = target_position
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.header.frame_id = self.costmap_client.get_global_frame_id()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        future = self.move_base_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, target_position))

    def goal_response_callback(self, future, target_position):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info('Goal rejected :(')
            return

        self.logger.info('Goal accepted :)')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.reached_goal(f, target_position))

    def reached_goal(self, future, frontier_goal):
        result = future.result()
        status = result.status
        # status 4 is SUCCEEDED, 5 is CANCELED, 6 is ABORTED in rclpy action client result?
        # Actually result.status is from action_msgs/msg/GoalStatus
        # STATUS_SUCCEEDED = 4
        # STATUS_ABORTED = 6
        # STATUS_CANCELED = 5

        from action_msgs.msg import GoalStatus

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.debug("Goal was successful")
        elif status == GoalStatus.STATUS_ABORTED:
            self.logger.debug("Goal was aborted")
            self.frontier_blacklist.append(frontier_goal)
            self.logger.debug("Adding current goal to black list")
            return
        elif status == GoalStatus.STATUS_CANCELED:
            self.logger.debug("Goal was canceled")
            return
        else:
            self.logger.warn("Unknown result code from move base nav2")

        self.make_plan()

    def start(self):
        self.logger.info("Exploration started.")

    def stop(self, finished_exploring=False):
        self.logger.info("Exploration stopped.")
        # Cancel all goals?
        # There is no async_cancel_all_goals in rclpy ActionClient directly exposed like that?
        # We can't easily cancel all goals without tracking handles.
        # But if we are stopping, we probably don't want to send new goals.
        self.exploring_timer.cancel()

        if self.return_to_init and finished_exploring:
            self.return_to_initial_pose()

    def resume(self):
        self.resuming = True
        self.logger.info("Exploration resuming.")
        self.exploration_complete = False
        self.publish_exploration_status(False, "Exploration resumed")
        self.exploring_timer.reset()
        self.make_plan()

    def return_to_initial_pose(self):
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
        if not self.return_to_init:
            return

        try:
            map_frame = self.costmap_client.get_global_frame_id()
            if map_frame is None:
                return

            # Try to get current transform
            t = self.tf_buffer.lookup_transform(
                map_frame,
                self.robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.1)
            )

            self.initial_pose = PoseStamped()
            self.initial_pose.pose.position.x = t.transform.translation.x
            self.initial_pose.pose.position.y = t.transform.translation.y
            self.initial_pose.pose.position.z = t.transform.translation.z
            self.initial_pose.pose.orientation = t.transform.rotation

            self.logger.info(f"Initial pose captured: ({self.initial_pose.pose.position.x:.2f}, {self.initial_pose.pose.position.y:.2f})")

        except TransformException as ex:
            self.logger.debug(f"Still waiting for transform to capture initial pose: {ex}")

    def publish_exploration_status(self, complete, info):
        status_msg = Bool()
        status_msg.data = complete
        self.exploration_status_publisher.publish(status_msg)

        if info:
            info_msg = String()
            info_msg.data = info
            self.exploration_info_publisher.publish(info_msg)
            self.logger.info(f"Exploration status: {'COMPLETE' if complete else 'ACTIVE'} - {info}")

    def goal_on_blacklist(self, goal: Point):
        tolerance = 5
        info = self.costmap_client.get_costmap_info()
        if info is None:
            return False

        resolution = info.resolution

        for frontier_goal in self.frontier_blacklist:
            x_diff = abs(goal.x - frontier_goal.x)
            y_diff = abs(goal.y - frontier_goal.y)

            if x_diff < tolerance * resolution and y_diff < tolerance * resolution:
                return True
        return False

    def same_point(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dist = math.sqrt(dx*dx + dy*dy)
        return dist < 0.01

    def report_status(self):
        if self.exploration_complete:
            self.publish_exploration_status(True, "Exploration complete")
        else:
            self.publish_exploration_status(False, "Exploring")

def main(args=None):
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
