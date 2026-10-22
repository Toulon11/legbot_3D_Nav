"""Sparse 3-D ray evidence and same-floor frontier selection.

No ROS dependencies: the same geometry can be exercised with recorded points.
Unknown voxels have no entry. A positive value is occupied; a nonpositive value
is observed free. Path search permits unknown XY cells with a configurable cost.
"""
from collections import deque
from dataclasses import dataclass
import heapq
import math


@dataclass
class Settings:
    resolution: float = 0.25
    max_ray_range: float = 9.0
    max_voxels: int = 900000
    obstacle_min_relative_z: float = -0.15
    obstacle_max_relative_z: float = 0.85
    free_min_relative_z: float = -0.45
    robot_radius: float = 0.35
    local_search_radius: float = 10.0
    min_goal_distance: float = 1.0
    max_goal_distance: float = 8.0
    min_frontier_cells: int = 3
    frontier_gain_weight: float = 1.0
    path_cost_weight: float = 1.0
    unknown_path_multiplier: float = 1.5


class VoxelMap:
    def __init__(self, settings):
        self.cfg = settings
        self.voxels = {}
        self.version = 0

    def key(self, point):
        return tuple(math.floor(value / self.cfg.resolution) for value in point)

    def cell_center(self, cell):
        return tuple((value + 0.5) * self.cfg.resolution for value in cell)

    def _mark(self, key, hit):
        previous = self.voxels.get(key)
        if previous is None and len(self.voxels) >= self.cfg.max_voxels:
            return
        # Endpoint evidence is stronger than a single later clearing ray.
        self.voxels[key] = max(-3, min(6, (previous or 0) + (3 if hit else -1)))

    def integrate(self, origin, points, base_z):
        if not math.isfinite(base_z):
            return 0
        inserted = 0
        for point in points:
            if not all(math.isfinite(v) for v in point):
                continue
            dx, dy, dz = (point[i] - origin[i] for i in range(3))
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance < self.cfg.resolution or distance > self.cfg.max_ray_range:
                continue
            steps = max(1, math.ceil(distance / (self.cfg.resolution * 0.5)))
            last = None
            for step in range(steps):
                fraction = step / steps
                cell = self.key((origin[0] + dx * fraction,
                                 origin[1] + dy * fraction,
                                 origin[2] - base_z + dz * fraction))
                if cell != last:
                    self._mark(cell, False)
                    last = cell
            endpoint = self.key((point[0], point[1], point[2] - base_z))
            obstacle = (base_z + self.cfg.obstacle_min_relative_z <= point[2]
                        <= base_z + self.cfg.obstacle_max_relative_z)
            self._mark(endpoint, obstacle)
            inserted += 1
        if inserted:
            self.version += 1
        return inserted

    def project(self, center_xy):
        """Return occupied/free XY cells in the base-relative height slice."""
        xy = {}
        r = self.cfg.local_search_radius + self.cfg.resolution * 2
        occupied_low = self.cfg.obstacle_min_relative_z
        free_low = self.cfg.free_min_relative_z
        high = self.cfg.obstacle_max_relative_z
        for (x, y, z), value in self.voxels.items():
            wx, wy, wz = self.cell_center((x, y, z))
            if not free_low <= wz <= high or abs(wx - center_xy[0]) > r or abs(wy - center_xy[1]) > r:
                continue
            cell = (x, y)
            if value > 0 and wz >= occupied_low:
                xy[cell] = 1
            elif value <= 0 and cell not in xy:
                xy[cell] = 0
        return xy


NEIGHBORS4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
NEIGHBORS8 = NEIGHBORS4 + ((1, 1), (1, -1), (-1, 1), (-1, -1))


def select_frontier(voxels, robot_xyz, settings, suppressed=(), external_obstacles=()):
    """Return (goal XY, score, frontier count), or (None, None, count)."""
    center = voxels.key(robot_xyz)[:2]
    grid = voxels.project(robot_xyz[:2])
    radius = math.ceil(settings.local_search_radius / settings.resolution)
    occupied = {cell for cell, value in grid.items() if value == 1}
    inflation = math.ceil(settings.robot_radius / settings.resolution)
    blocked = set()
    for x, y in occupied:
        for dx in range(-inflation, inflation + 1):
            for dy in range(-inflation, inflation + 1):
                if math.hypot(dx, dy) * settings.resolution <= settings.robot_radius:
                    blocked.add((x + dx, y + dy))
    for x, y in external_obstacles:
        if math.isfinite(x) and math.isfinite(y):
            cx, cy = math.floor(x / settings.resolution), math.floor(y / settings.resolution)
            for dx in range(-inflation, inflation + 1):
                for dy in range(-inflation, inflation + 1):
                    if math.hypot(dx, dy) * settings.resolution <= settings.robot_radius:
                        blocked.add((cx + dx, cy + dy))

    def inside(cell):
        return abs(cell[0] - center[0]) <= radius and abs(cell[1] - center[1]) <= radius

    # Exploration frontier: genuinely unknown cell adjacent to observed free.
    frontier = set()
    for x, y in grid:
        if grid[(x, y)] != 0:
            continue
        for dx, dy in NEIGHBORS4:
            cell = (x + dx, y + dy)
            if inside(cell) and cell not in grid and cell not in blocked:
                frontier.add(cell)

    clusters = []
    while frontier:
        seed = frontier.pop()
        cluster = [seed]
        queue = deque([seed])
        while queue:
            x, y = queue.popleft()
            for dx, dy in NEIGHBORS8:
                cell = (x + dx, y + dy)
                if cell in frontier:
                    frontier.remove(cell)
                    cluster.append(cell)
                    queue.append(cell)
        if len(cluster) >= settings.min_frontier_cells:
            clusters.append(cluster)

    # One Dijkstra search serves all clusters. Unknown is allowed by policy.
    costs = {center: 0.0}
    heap = [(0.0, center)]
    while heap:
        cost, cell = heapq.heappop(heap)
        if cost > costs[cell]:
            continue
        for dx, dy in NEIGHBORS8:
            nxt = (cell[0] + dx, cell[1] + dy)
            if not inside(nxt) or nxt in blocked:
                continue
            if dx and dy and ((cell[0] + dx, cell[1]) in blocked or
                              (cell[0], cell[1] + dy) in blocked):
                continue
            step = settings.resolution * math.hypot(dx, dy)
            if nxt not in grid:
                step *= settings.unknown_path_multiplier
            new_cost = cost + step
            if new_cost < costs.get(nxt, math.inf):
                costs[nxt] = new_cost
                heapq.heappush(heap, (new_cost, nxt))

    best = None
    best_score = -math.inf
    for cluster in clusters:
        mx = sum(c[0] for c in cluster) / len(cluster)
        my = sum(c[1] for c in cluster) / len(cluster)
        # The geometric centroid can lie inside an obstacle. Use a real cell.
        ordered = sorted(cluster, key=lambda c: (c[0] - mx) ** 2 + (c[1] - my) ** 2)
        for cell in ordered:
            if cell not in costs:
                continue
            x, y = voxels.cell_center(cell)
            distance = math.hypot(x - robot_xyz[0], y - robot_xyz[1])
            if not settings.min_goal_distance <= distance <= settings.max_goal_distance:
                continue
            if any(math.hypot(x - sx, y - sy) < settings.robot_radius * 2
                   for sx, sy in suppressed):
                continue
            score = (settings.frontier_gain_weight * math.log1p(len(cluster))
                     - settings.path_cost_weight * costs[cell])
            if score > best_score:
                best = (x, y)
                best_score = score
            break
    return best, (best_score if best is not None else None), len(clusters)
