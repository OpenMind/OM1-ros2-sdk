import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
from collections import deque
import math

# Constants
NO_INFORMATION = -1
FREE_SPACE = 0
LETHAL_OBSTACLE = 100
# In OccupancyGrid, values are -1, 0..100.
# We will use these values directly.

class Frontier:
    def __init__(self):
        self.size = 0
        self.min_distance = float('inf')
        self.cost = 0.0
        self.initial = Point()
        self.centroid = Point()
        self.middle = Point()
        self.points = []

class FrontierSearch:
    def __init__(self, costmap_client, potential_scale, gain_scale, min_frontier_size):
        self.costmap_client = costmap_client
        self.potential_scale = potential_scale
        self.gain_scale = gain_scale
        self.min_frontier_size = min_frontier_size
        self.logger = rclpy.logging.get_logger("frontier_search")

    def search_from(self, position: Point):
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

        # Find closest clear cell to start search
        # In C++, it searches for nearest FREE_SPACE.
        # Here we just check if current is free, if not search neighborhood.
        # For simplicity, let's assume robot is in free space or close to it.
        # If robot is in lethal, we might have issues.

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

                if len(visited_start) > 100: # Limit search
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
                    if costmap[ny, nx] == costmap[cy, cx] and costmap[ny, nx] == FREE_SPACE:
                        # If neighbor is free and unvisited, add to BFS
                        visited_flag[ny, nx] = True
                        bfs.append((ny, nx))
                    elif self.is_new_frontier_cell(ny, nx, frontier_flag, costmap, size_x, size_y):
                        frontier_flag[ny, nx] = True
                        new_frontier = self.build_new_frontier(ny, nx, position, frontier_flag, costmap, info)
                        if new_frontier.size * info.resolution >= self.min_frontier_size:
                            frontier_list.append(new_frontier)

        # Set costs
        for frontier in frontier_list:
            frontier.cost = self.frontier_cost(frontier, info.resolution)

        frontier_list.sort(key=lambda f: f.cost)

        return frontier_list

    def build_new_frontier(self, initial_cy, initial_cx, reference_pos, frontier_flag, costmap, info):
        output = Frontier()
        output.centroid.x = 0.0
        output.centroid.y = 0.0
        output.size = 1
        output.min_distance = float('inf')

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
                if self.is_new_frontier_cell(ny, nx, frontier_flag, costmap, size_x, size_y):
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

                    dist = math.sqrt((reference_pos.x - wx)**2 + (reference_pos.y - wy)**2)
                    if dist < output.min_distance:
                        output.min_distance = dist
                        output.middle.x = wx
                        output.middle.y = wy

                    bfs.append((ny, nx))

        if output.size > 0:
            output.centroid.x /= output.size
            output.centroid.y /= output.size

        return output

    def is_new_frontier_cell(self, cy, cx, frontier_flag, costmap, size_x, size_y):
        # Check that cell is unknown and not already marked
        if costmap[cy, cx] != NO_INFORMATION or frontier_flag[cy, cx]:
            return False

        # Frontier cells should have at least one cell in 4-connected neighbourhood that is free
        for ny, nx in self.nhood4(cy, cx, size_x, size_y):
            if costmap[ny, nx] == FREE_SPACE:
                return True

        return False

    def frontier_cost(self, frontier, resolution):
        return (self.potential_scale * frontier.min_distance * resolution) - \
               (self.gain_scale * frontier.size * resolution)

    def nhood4(self, cy, cx, size_x, size_y):
        neighbors = []
        if cx > 0: neighbors.append((cy, cx - 1))
        if cx < size_x - 1: neighbors.append((cy, cx + 1))
        if cy > 0: neighbors.append((cy - 1, cx))
        if cy < size_y - 1: neighbors.append((cy + 1, cx))
        return neighbors

    def nhood8(self, cy, cx, size_x, size_y):
        neighbors = self.nhood4(cy, cx, size_x, size_y)
        # Diagonals
        if cx > 0 and cy > 0: neighbors.append((cy - 1, cx - 1))
        if cx < size_x - 1 and cy > 0: neighbors.append((cy - 1, cx + 1))
        if cx > 0 and cy < size_y - 1: neighbors.append((cy + 1, cx - 1))
        if cx < size_x - 1 and cy < size_y - 1: neighbors.append((cy + 1, cx + 1))
        return neighbors
