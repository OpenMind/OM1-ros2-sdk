import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import MapMetaData

from frontier_explorer.costmap_client import Costmap2DClient

# Constants
NO_INFORMATION = -1
FREE_SPACE = 0
LETHAL_OBSTACLE = 100


class Frontier:
    """
    Represents a frontier region in the occupancy grid.

    A frontier is a boundary between explored free space and unexplored territory.
    Contains information about the frontier's size, location, cost, and constituent points.
    """

    def __init__(self):
        """
        Initialize a Frontier with default values.
        """
        self.size = 0
        self.min_distance = float("inf")
        self.cost = 0.0
        self.initial = Point()
        self.centroid = Point()
        self.middle = Point()
        self.points = []


class FrontierSearch:
    """
    Searches for and identifies frontier regions in an occupancy grid costmap.

    Uses breadth-first search (BFS) to find boundaries between known free space
    and unknown areas. Computes costs for each frontier to prioritize exploration targets.
    """

    def __init__(
        self,
        costmap_client: Costmap2DClient,
        potential_scale: float,
        gain_scale: float,
        min_frontier_size: float,
    ):
        """Initialize the FrontierSearch with costmap client and search parameters.

        Parameters
        ----------
        costmap_client: Costmap2DClient
            Client to access costmap data and metadata.
        potential_scale: float
            Scaling factor for distance cost in frontier evaluation.
        gain_scale: float
            Scaling factor for size gain in frontier evaluation.
        min_frontier_size: float
            Minimum size (in meters) for a frontier to be considered valid.
        """
        self.costmap_client = costmap_client
        self.potential_scale = potential_scale
        self.gain_scale = gain_scale
        self.min_frontier_size = min_frontier_size
        self.logger = rclpy.logging.get_logger("frontier_search")

    def search_from(self, position: Point) -> list:
        """
        Search for frontiers starting from the robot's current position.

        Uses BFS to explore free space and identify frontier cells (boundaries between
        free and unknown space). Groups adjacent frontier cells into frontier regions,
        computes their properties, filters by minimum size, and sorts by cost.

        Parameters
        ----------
        position: Point
            The robot's current position in the costmap frame.

        Returns
        -------
        list
            A list of identified Frontier objects, sorted by cost (lowest cost first).
        """
        frontier_list = []

        costmap = self.costmap_client.get_costmap()
        info = self.costmap_client.get_costmap_info()

        if costmap is None or info is None:
            self.logger.warn("Costmap not ready")
            return frontier_list

        # Get robot position in map coordinates
        mx = int((position.x - info.origin.position.x) / info.resolution)
        my = int((position.y - info.origin.position.y) / info.resolution)

        if not (0 <= mx < info.width and 0 <= my < info.height):
            self.logger.error("Robot out of costmap bounds")
            return frontier_list

        size_x = info.width
        size_y = info.height

        # Initialize flags
        # We use a set for visited to save memory if map is huge, or a boolean array
        # Boolean array is faster.
        visited_flag = np.zeros((size_y, size_x), dtype=bool)
        frontier_flag = np.zeros((size_y, size_x), dtype=bool)

        bfs = deque()

        start_idx = (my, mx)
        if costmap[my, mx] == FREE_SPACE:
            bfs.append(start_idx)
        else:
            # Search for nearest free cell
            # Simple spiral or BFS to find free cell
            found_free = False
            q = deque([start_idx])
            visited_start = {start_idx}
            while q:
                cy, cx = q.popleft()
                if costmap[cy, cx] == FREE_SPACE:
                    bfs.append((cy, cx))
                    found_free = True
                    break

                for ny, nx in self.nhood4(cy, cx, size_x, size_y):
                    if (ny, nx) not in visited_start:
                        visited_start.add((ny, nx))
                        q.append((ny, nx))

                if len(visited_start) > 100:  # Limit search
                    break

            if not found_free:
                bfs.append(start_idx)
                self.logger.warn("Could not find nearby clear cell to start search")

        visited_flag[bfs[0][0], bfs[0][1]] = True

        while bfs:
            idx = bfs.popleft()
            cy, cx = idx

            for ny, nx in self.nhood4(cy, cx, size_x, size_y):
                if not visited_flag[ny, nx]:
                    if (
                        costmap[ny, nx] == costmap[cy, cx]
                        and costmap[ny, nx] == FREE_SPACE
                    ):
                        # If neighbor is free and unvisited, add to BFS
                        visited_flag[ny, nx] = True
                        bfs.append((ny, nx))
                    elif self.is_new_frontier_cell(
                        ny, nx, frontier_flag, costmap, size_x, size_y
                    ):
                        frontier_flag[ny, nx] = True
                        new_frontier = self.build_new_frontier(
                            ny, nx, position, frontier_flag, costmap, info
                        )
                        if (
                            new_frontier.size * info.resolution
                            >= self.min_frontier_size
                        ):
                            frontier_list.append(new_frontier)

        if frontier_list:
            frontier_positions = np.array(
                [[f.centroid.x, f.centroid.y] for f in frontier_list]
            )
            robot_pos = np.array([position.x, position.y])

            distances_to_centroid = np.linalg.norm(
                frontier_positions - robot_pos, axis=1
            )

            sizes = np.array([f.size for f in frontier_list])

            costs = (self.potential_scale * distances_to_centroid) - (
                self.gain_scale * sizes * info.resolution
            )

            for i, frontier in enumerate(frontier_list):
                frontier.cost = costs[i]

        frontier_list.sort(key=lambda f: f.cost)

        return frontier_list

    def build_new_frontier(
        self,
        initial_cy: int,
        initial_cx: int,
        reference_pos: Point,
        frontier_flag: np.ndarray,
        costmap: np.ndarray,
        info: MapMetaData,
    ) -> Frontier:
        """
        Build a complete frontier region starting from an initial frontier cell.

        Uses BFS with 8-connected neighborhood to grow the frontier region, tracking
        all constituent points, computing centroid, and finding the closest point to
        the reference position.

        Parameters
        ----------
        initial_cy: int
            Initial cell y-coordinate of the frontier.
        initial_cx: int
            Initial cell x-coordinate of the frontier.
        reference_pos: Point
            The robot's current position for distance calculations.
        frontier_flag: numpy.ndarray
            Boolean array of already marked frontier cells.
        costmap: numpy.ndarray
            The occupancy grid costmap data.
        info: MapMetaData
            Metadata of the costmap.

        Returns
        -------
        Frontier
            The constructed Frontier object with all properties set.
        """
        output = Frontier()
        output.centroid.x = 0.0
        output.centroid.y = 0.0
        output.size = 1
        output.min_distance = float("inf")

        ix = info.origin.position.x + (initial_cx + 0.5) * info.resolution
        iy = info.origin.position.y + (initial_cy + 0.5) * info.resolution
        output.initial.x = ix
        output.initial.y = iy

        bfs = deque([(initial_cy, initial_cx)])

        size_x = info.width
        size_y = info.height

        while bfs:
            cy, cx = bfs.popleft()

            # Try adding cells in 8-connected neighborhood
            for ny, nx in self.nhood8(cy, cx, size_x, size_y):
                if self.is_new_frontier_cell(
                    ny, nx, frontier_flag, costmap, size_x, size_y
                ):
                    frontier_flag[ny, nx] = True

                    wx = info.origin.position.x + (nx + 0.5) * info.resolution
                    wy = info.origin.position.y + (ny + 0.5) * info.resolution

                    point = Point()
                    point.x = wx
                    point.y = wy
                    output.points.append(point)

                    output.size += 1
                    output.centroid.x += wx
                    output.centroid.y += wy

                    dist = math.sqrt(
                        (reference_pos.x - wx) ** 2 + (reference_pos.y - wy) ** 2
                    )
                    if dist < output.min_distance:
                        output.min_distance = dist
                        output.middle.x = wx
                        output.middle.y = wy

                    bfs.append((ny, nx))

        if output.size > 0:
            output.centroid.x /= output.size
            output.centroid.y /= output.size

        return output

    def is_new_frontier_cell(
        self,
        cy: int,
        cx: int,
        frontier_flag: np.ndarray,
        costmap: np.ndarray,
        size_x: int,
        size_y: int,
    ) -> bool:
        """Check if a cell qualifies as a new frontier cell.

        A cell is a frontier if it's unknown (not yet explored) and has at least
        one free space neighbor in the 4-connected neighborhood.

        Parameters
        ----------
        cy: int
            Cell y-coordinate.
        cx: int
            Cell x-coordinate.
        frontier_flag: numpy.ndarray
            Boolean array of already marked frontier cells.
        costmap: numpy.ndarray
            The occupancy grid costmap data.
        size_x: int
            Width of the costmap.
        size_y: int
            Height of the costmap.

        Returns
        -------
        bool
            True if the cell is a new frontier cell, False otherwise.
        """
        # Check that cell is unknown and not already marked
        if costmap[cy, cx] != NO_INFORMATION or frontier_flag[cy, cx]:
            return False

        # Frontier cells should have at least one cell in 4-connected neighbourhood that is free
        for ny, nx in self.nhood4(cy, cx, size_x, size_y):
            if costmap[ny, nx] == FREE_SPACE:
                return True

        return False

    def nhood4(self, cy: int, cx: int, size_x: int, size_y: int) -> list:
        """Get 4-connected neighbors (up, down, left, right) of a cell.

        Only returns neighbors that are within the costmap bounds.

        Parameters
        ----------
        cy: int
            Cell y-coordinate.
        cx: int
            Cell x-coordinate.
        size_x: int
            Width of the costmap.
        size_y: int
            Height of the costmap.

        Returns
        -------
        list
            List of (y, x) tuples for valid neighbors.
        """
        neighbors = []
        if cx > 0:
            neighbors.append((cy, cx - 1))
        if cx < size_x - 1:
            neighbors.append((cy, cx + 1))
        if cy > 0:
            neighbors.append((cy - 1, cx))
        if cy < size_y - 1:
            neighbors.append((cy + 1, cx))
        return neighbors

    def nhood8(self, cy: int, cx: int, size_x: int, size_y: int) -> list:
        """Get 8-connected neighbors (including diagonals) of a cell.

        Returns all orthogonal and diagonal neighbors within costmap bounds.

        Parameters
        ----------
        cy: int
            Cell y-coordinate.
        cx: int
            Cell x-coordinate.
        size_x: int
            Width of the costmap.
        size_y: int
            Height of the costmap.

        Returns
        -------
        list
            List of (y, x) tuples for valid neighbors.
        """
        neighbors = self.nhood4(cy, cx, size_x, size_y)
        # Diagonals
        if cx > 0 and cy > 0:
            neighbors.append((cy - 1, cx - 1))
        if cx < size_x - 1 and cy > 0:
            neighbors.append((cy - 1, cx + 1))
        if cx > 0 and cy < size_y - 1:
            neighbors.append((cy + 1, cx - 1))
        if cx < size_x - 1 and cy < size_y - 1:
            neighbors.append((cy + 1, cx + 1))
        return neighbors
