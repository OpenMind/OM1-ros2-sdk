import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray

from om_api.msg import Paths


def create_straight_line_path_from_angle(angle_degrees: int, length=2.0, num_points=20):
    """
    Create a straight line path from origin at specified angle and length.

    Parameters
    ----------
    angle_degrees : int
        The angle in degrees for the path direction (+X forward, +Y left).
    length : float, optional
        The length of the path in meters, by default 2.0.
    num_points : int, optional
        The number of points to generate along the path, by default 20.
    """
    angle_rad = math.radians(angle_degrees)
    end_x = length * math.cos(angle_rad)  # +X forward
    end_y = length * math.sin(angle_rad)  # +Y left

    x_vals = np.linspace(0.0, end_x, num_points)
    y_vals = np.linspace(0.0, end_y, num_points)
    return np.array([x_vals, y_vals])


# Define 9 straight line paths separated by 15 degrees
# Center path is 0° (straight forward), then ±15°, ±30°, ±45°, ±60°
path_angles = [60, 45, 30, 15, 0, -15, -30, -45, -60, 180]  # degrees
path_length = 2.0  # meters
path_ranges = [50, 100, 200]  # cm

paths = [create_straight_line_path_from_angle(a, path_length) for a in path_angles]


class OMPath(Node):
    """
    Fuse LaserScan, depth obstacles, and hazard PC2 to choose feasible paths.

    Notes
    -----
    * All computations in robot frame: +X forward, +Y left.
    * Hazards (stairs/holes/edges) only block forward rays (indices 0..8).
    * Reverse ray (index 9) ignores hazards and only checks points behind robot (x<0).
    """

    def __init__(self):
        super().__init__("om_path")

        # Declare and use the 'use_sim' parameter to determine if running in simulation
        self.declare_parameter("use_sim", False)
        self.use_sim = self.get_parameter("use_sim").value

        self.half_width_robot = 0.20
        self.sensor_mounting_angle = 180.0
        self.relevant_distance_min = 0.20
        self.obstacle_threshold = 0.50  # 50 data points
        self.robot_frame = "base_link"

        self.scan = None
        self.obstacle: PointCloud | None = None
        self.hazard_xy_robot = np.zeros((0, 2), dtype=np.float32)

        self.scan_info = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.obstacle_info = self.create_subscription(
            PointCloud,
            "/camera/realsense2_camera_node/depth/obstacle_point",
            self.obstacle_callback,
            10,
        )
        self.hazard_info = self.create_subscription(
            PointCloud2, "/traversability/hazard_points2", self.hazard_callback_pc2, 10
        )

        self.paths_pubs = {}
        self.markers_pubs = {}

        for range_key in path_ranges:
            self.paths_pubs[range_key] = self.create_publisher(
                Paths, f"/om/paths/r{range_key}", 10
            )
            self.markers_pubs[range_key] = self.create_publisher(
                MarkerArray, f"/om/paths_markers/r{range_key}", 10
            )

        self.paths_pub_default = self.create_publisher(Paths, "/om/paths", 10)
        self.markers_pub_default = self.create_publisher(
            MarkerArray, "/om/paths_markers", 10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"OMPath node started with detection ranges: {path_ranges}"
        )

    def scan_callback(self, msg: LaserScan):
        """
        Process incoming LaserScan message to determine feasible paths.

        Parameters
        ----------
        msg : sensor_msgs.msg.LaserScan
            The incoming LaserScan message.
        """
        self.scan = msg

        angles = list(
            map(
                lambda x: 360.0 * (x + math.pi) / (2 * math.pi) - 180.0,
                np.arange(
                    self.scan.angle_min,
                    self.scan.angle_max,
                    self.scan.angle_increment,
                ),
            )
        )
        angles_final = angles
        # angles now run from 360.0 to 0 degrees
        data = list(zip(angles_final, self.scan.ranges))

        complexes = []

        for angle, distance in data:
            d_m = distance

            if not math.isfinite(d_m) or d_m > 3.0 or d_m < self.relevant_distance_min:
                continue

            # Only rotate sensor data if not in simulation
            if not self.use_sim:
                # first, correctly orient the sensor zero to the robot zero
                angle = angle + self.sensor_mounting_angle
                if angle >= 360.0:
                    angle = angle - 360.0
                elif angle < 0.0:
                    angle = 360.0 + angle

            # then, convert to radians
            a_rad = angle * math.pi / 180.0

            x = d_m * math.cos(a_rad)
            y = d_m * math.sin(a_rad)

            angle = angle - 180.0

            # the final data ready to use for path planning
            complexes.append([x, y, angle, d_m])

        if self.obstacle and len(self.obstacle.points) > self.obstacle_threshold:
            for p in self.obstacle.points:
                x = float(p.x)
                y = float(p.y)

                angle, distance = self.calculate_angle_and_distance(x, y)
                complexes.append([x, y, angle, distance])

        if not complexes:
            return

        array = np.array(complexes)
        sorted_indices = array[:, 2].argsort()
        array = array[sorted_indices]

        X = array[:, 0]
        Y = array[:, 1]
        D = array[:, 3]

        possible_paths = {r: np.arange(10) for r in path_ranges}
        blocked_by_obstacle = {r: set() for r in path_ranges}
        blocked_by_hazard = {r: set() for r in path_ranges}

        # Check obstacles
        for x, y, d in list(zip(X, Y, D)):
            for apath in range(10):
                if all(apath not in possible_paths[r] for r in path_ranges):
                    continue

                # For going back, only consider obstacles that are:
                # 1. Behind the robot (negative x in robot frame)
                # 2. Within a certain distance threshold

                # Assuming robot faces positive x direction
                # Only check obstacles behind the robot
                if apath == 9 and x >= 0.0:
                    continue

                path_points = paths[apath]
                start_x, start_y = path_points[0][0], path_points[1][0]
                end_x, end_y = path_points[0][-1], path_points[1][-1]

                dist_to_line = self.distance_point_to_line_segment(
                    x, y, start_x, start_y, end_x, end_y
                )

                if dist_to_line < self.half_width_robot:
                    for range_key in path_ranges:
                        if (
                            d <= range_key / 100.0
                            and apath in possible_paths[range_key]
                        ):
                            blocked_by_obstacle[range_key].add(int(apath))
                            possible_paths[range_key] = np.setdiff1d(
                                possible_paths[range_key], np.array([apath])
                            )
                    break

        # Check hazards
        if self.hazard_xy_robot.size > 0:
            for hx, hy in self.hazard_xy_robot:
                hazard_dist = math.sqrt(hx**2 + hy**2)

                for apath in range(10):
                    if all(apath not in possible_paths[r] for r in path_ranges):
                        continue

                    # For going back, only consider obstacles that are:
                    # 1. Behind the robot (negative x in robot frame)
                    # 2. Within a certain distance threshold

                    # Assuming robot faces positive x direction
                    # Only check obstacles behind the robot
                    if apath == 9:
                        continue

                    path_points = paths[apath]
                    start_x, start_y = path_points[0][0], path_points[1][0]
                    end_x, end_y = path_points[0][-1], path_points[1][-1]

                    dist_to_line = self.distance_point_to_line_segment(
                        hx, hy, start_x, start_y, end_x, end_y
                    )

                    if dist_to_line < self.half_width_robot:
                        for range_key in path_ranges:
                            if (
                                hazard_dist <= range_key / 100.0
                                and apath in possible_paths[range_key]
                            ):
                                blocked_by_hazard[range_key].add(int(apath))
                                possible_paths[range_key] = np.setdiff1d(
                                    possible_paths[range_key], np.array([apath])
                                )
                        break

        timestamp = self.get_clock().now().to_msg()

        for range_key in path_ranges:
            self._publish_paths(
                str(range_key),
                timestamp,
                possible_paths[range_key],
                blocked_by_obstacle[range_key],
                blocked_by_hazard[range_key],
                X,
                Y,
                D,
                range_key / 100.0,
            )

        self._publish_paths(
            "default",
            timestamp,
            possible_paths[100],
            blocked_by_obstacle[100],
            blocked_by_hazard[100],
            X,
            Y,
            D,
            100 / 100.0,
        )

    def _publish_paths(
        self,
        range_key: list,
        timestamp: str,
        possible_paths: list,
        blocked_by_obstacle: list,
        blocked_by_hazard: list,
        X: list,
        Y: list,
        D: list,
        max_distance: float,
    ):
        """
        Publish feasible paths and visualization markers.

        Parameters
        ----------
        range_key : list
            The key for the path range (e.g., "50", "100", "200").
        timestamp : str
            The timestamp for the message header.
        possible_paths : list
            List of path indices that are possible (not blocked).
        blocked_by_obstacle : list
            List of path indices that are blocked by obstacles.
        blocked_by_hazard : list
            List of path indices that are blocked by hazards.
        X : list
            List of x coordinates of detected points in robot frame.
        Y : list
            List of y coordinates of detected points in robot frame.
        D : list
            List of distances of detected points from the robot.
        max_distance : float
            The maximum distance to visualize paths (in meters).
        """
        paths_msg = Paths()
        paths_msg.header.stamp = timestamp
        paths_msg.header.frame_id = self.robot_frame
        paths_msg.paths = possible_paths.tolist()
        paths_msg.blocked_by_obstacle_idx = sorted(blocked_by_obstacle)
        paths_msg.blocked_by_hazard_idx = sorted(blocked_by_hazard)

        if range_key == "default":
            self.paths_pub_default.publish(paths_msg)
        else:
            self.paths_pubs[int(range_key)].publish(paths_msg)
        bad_union = blocked_by_obstacle | blocked_by_hazard

        self._publish_markers(
            range_key=range_key,
            max_distance=max_distance,
            possible_paths=set(paths_msg.paths),
            bad_paths=bad_union,
            obstacles_xy=list(zip(X.tolist(), Y.tolist())),
            hazards_xy=self.hazard_xy_robot.tolist(),
            frame_id=self.robot_frame,
        )

    def obstacle_callback(self, obstacle_cloud_msg: PointCloud):
        """
        Store obstacle point cloud message.

        Parameters
        ----------
        obstacle_cloud_msg : sensor_msgs.msg.PointCloud
            The incoming obstacle point cloud message.
        """
        self.obstacle = obstacle_cloud_msg

    def _rot_array_deg(self, arr_xy: np.ndarray, yaw_deg: float) -> np.ndarray:
        """
        Rotate Nx2 [x,y] points by yaw_deg.

        Parameters
        ----------
        arr_xy : np.ndarray
            An (N, 2) array of points to rotate.
        yaw_deg : float
            The rotation angle in degrees.

        Returns
        -------
        np.ndarray
            The rotated (N, 2) array of points.
        """
        if arr_xy.size == 0:
            return arr_xy
        r = math.radians(yaw_deg)
        c, s = math.cos(r), math.sin(r)
        x = arr_xy[:, 0]
        y = arr_xy[:, 1]
        xr = c * x - s * y
        yr = s * x + c * y
        return np.stack([xr, yr], axis=1)

    def hazard_callback_pc2(self, msg: PointCloud2):
        """
        Read hazard points already expressed in robot frame (base_link),
        store as an (N, 2) array [x, y] in self.hazard_xy_robot.

        Parameters
        ----------
        msg : sensor_msgs.msg.PointCloud2
            The incoming hazard point cloud message.
        """
        try:
            arr = pc2.read_points_numpy(msg, field_names=("x", "y"), skip_nans=True)
            arr = np.asarray(arr, dtype=np.float32).reshape(-1, 2)
            self.hazard_xy_robot = arr
        except Exception as e:
            self.get_logger().warn(f"Failed to parse hazard_points2: {e}")

    def calculate_angle_and_distance(
        self, world_x: int, world_y: int
    ) -> tuple[float, float]:
        """
        Return (angle_degrees, distance) from x,y in robot frame.

        Parameters
        ----------
        world_x : int
            The x coordinate of the point in robot frame.
        world_y : int
            The y coordinate of the point in robot frame.

        Returns
        -------
        tuple[float, float]
            A tuple containing the angle in degrees and the distance from the robot.
        """
        distance = math.sqrt(world_x**2 + world_y**2)
        angle_rad = math.atan2(world_y, world_x)
        angle_degrees = math.degrees(angle_rad)
        return angle_degrees, distance

    def distance_point_to_line_segment(
        self, px: int, py: int, x1: int, y1: int, x2: int, y2: int
    ) -> float:
        """
        Distance from (px,py) to segment (x1,y1)-(x2,y2).

        Parameters
        ----------
        px, py : int
            The point coordinates.
        x1, y1 : int
            The start point of the line segment.
        x2, y2 : int
            The end point of the line segment.
        """
        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        t = max(0, min(1, t))

        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)

    def _publish_markers(
        self,
        range_key: str,
        max_distance: float,
        possible_paths: set,
        bad_paths: set,
        obstacles_xy: list,
        hazards_xy: list,
        frame_id: str,
    ):
        """
        Publish RViz markers for candidate paths, obstacles, and hazards.
        Truncate path visualization to max_distance.

        Parameters
        ----------
        range_key : str
            The key for the path range (e.g., "50", "100", "200").
        max_distance : float
            The maximum distance to visualize paths (in meters).
        possible_paths : set
            Set of path indices that are possible (not blocked).
        bad_paths : set
            Set of path indices that are blocked.
        obstacles_xy : list
            List of (x, y) tuples for obstacle points in robot frame.
        hazards_xy : list
            List of (x, y) tuples for hazard points in robot frame.
        frame_id : str
            The frame ID to use for the markers (e.g., "base_link").
        """
        ma = MarkerArray()
        msg_time_stamp = Time().to_msg()

        wipe = Marker()
        wipe.header.frame_id = frame_id
        wipe.header.stamp = msg_time_stamp
        wipe.action = Marker.DELETEALL
        ma.markers.append(wipe)

        for idx, path_arr in enumerate(paths):
            line = Marker()
            line.header.frame_id = frame_id
            line.header.stamp = msg_time_stamp
            line.ns = f"candidate_paths_{range_key}"
            line.id = idx
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.02

            if idx in possible_paths:
                line.color.r, line.color.g, line.color.b, line.color.a = (
                    0.0,
                    1.0,
                    0.0,
                    1.0,
                )
            elif idx in bad_paths:
                line.color.r, line.color.g, line.color.b, line.color.a = (
                    1.0,
                    0.0,
                    0.0,
                    1.0,
                )
            else:
                line.color.r, line.color.g, line.color.b, line.color.a = (
                    0.6,
                    0.6,
                    0.6,
                    1.0,
                )

            for x, y in zip(path_arr[0], path_arr[1]):
                dist = math.sqrt(x**2 + y**2)
                if dist <= max_distance:
                    line.points.append(Point(x=float(x), y=float(y), z=0.0))
                else:
                    if len(line.points) > 0:
                        angle = math.atan2(y, x)
                        final_x = max_distance * math.cos(angle)
                        final_y = max_distance * math.sin(angle)
                        line.points.append(Point(x=final_x, y=final_y, z=0.0))
                    break

            ma.markers.append(line)

            if line.points:
                t = Marker()
                t.header.frame_id = frame_id
                t.header.stamp = msg_time_stamp
                t.ns = f"path_labels_{range_key}"
                t.id = 1000 + idx
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.scale.z = 0.08
                t.color.r = t.color.g = t.color.b = 1.0
                t.color.a = 0.9
                last_pt = line.points[-1]
                t.pose.position.x = last_pt.x
                t.pose.position.y = last_pt.y
                t.pose.position.z = 0.06
                t.text = f"{idx} ({int(path_angles[idx])}°)"
                ma.markers.append(t)

        if obstacles_xy:
            pts = Marker()
            pts.header.frame_id = frame_id
            pts.header.stamp = msg_time_stamp
            pts.ns = f"obstacles_{range_key}"
            pts.id = 2000
            pts.type = Marker.POINTS
            pts.action = Marker.ADD
            pts.scale.x = pts.scale.y = 0.03
            pts.color.r, pts.color.g, pts.color.b, pts.color.a = 1.0, 0.5, 0.0, 0.9
            for x, y in obstacles_xy:
                pts.points.append(Point(x=float(x), y=float(y), z=0.0))
            ma.markers.append(pts)

        if hazards_xy:
            h = Marker()
            h.header.frame_id = frame_id
            h.header.stamp = msg_time_stamp
            h.ns = f"hazards_{range_key}"
            h.id = 3000
            h.type = Marker.POINTS
            h.action = Marker.ADD
            h.scale.x = h.scale.y = 0.035
            h.color.r, h.color.g, h.color.b, h.color.a = 0.2, 0.5, 0.9, 0.9
            for x, y in hazards_xy:
                h.points.append(Point(x=float(x), y=float(y), z=0.0))
            ma.markers.append(h)

        if range_key == "default":
            self.markers_pub_default.publish(ma)
        else:
            self.markers_pubs[int(range_key)].publish(ma)


def main(args=None):
    """
    Main function to run the OMPath node.
    """
    rclpy.init(args=args)
    node = OMPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
