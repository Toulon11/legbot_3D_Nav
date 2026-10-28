"""稀疏三维射线证据地图与同层前沿选点。

本文件不依赖 ROS，便于直接用小规模点云测试几何算法。
未保存的体素表示未知；正证据表示障碍，零或负证据表示已观察空闲。
路径搜索允许经过未知的 XY 网格，但会按参数提高路径代价。
"""
# 导入 collections 提供的类型或函数。
from collections import deque
# 从 dataclasses 导入装饰器，让 Settings 自动获得构造函数。
from dataclasses import dataclass
# 导入 实现按路径代价排序的最小堆。
import heapq
# 导入 数学函数，如开方、向下取整和距离计算。
import math


# 让 Python 自动生成参数类的初始化等基础方法。
@dataclass
# 定义一个把相关状态和方法放在一起的类。
class Settings:
    # 参数 resolution：每格的实际米数；等号右边是默认值。
    resolution: float = 0.25
    # 参数 max_ray_range：雷达点的最大有效距离；等号右边是默认值。
    max_ray_range: float = 9.0
    # 参数 max_voxels：地图最大格数；等号右边是默认值。
    max_voxels: int = 900000
    # 参数 obstacle_min_relative_z：障碍高度下界；等号右边是默认值。
    obstacle_min_relative_z: float = -0.15
    # 参数 obstacle_max_relative_z：障碍高度上界；等号右边是默认值。
    obstacle_max_relative_z: float = 0.85
    # 参数 free_min_relative_z：空闲投影高度下界；等号右边是默认值。
    free_min_relative_z: float = -0.45
    # 参数 robot_radius：机器人避障半径；等号右边是默认值。
    robot_radius: float = 0.35
    # 参数 local_search_radius：局部探索半径；等号右边是默认值。
    local_search_radius: float = 10.0
    # 参数 global_search：先在矩形内选点，再检查完整累计地图；等号右边是默认值。
    global_search: bool = False
    # 参数 exploration_min_x：全图边界的 X 最小值；等号右边是默认值。
    exploration_min_x: float = -31.5
    # 参数 exploration_max_x：全图边界的 X 最大值；等号右边是默认值。
    exploration_max_x: float = -0.5
    # 参数 exploration_min_y：全图边界的 Y 最小值；等号右边是默认值。
    exploration_min_y: float = -15.5
    # 参数 exploration_max_y：全图边界的 Y 最大值；等号右边是默认值。
    exploration_max_y: float = 15.5
    # 参数 min_goal_distance：目标最小距离；等号右边是默认值。
    min_goal_distance: float = 1.0
    # 参数 max_goal_distance：目标最大距离；等号右边是默认值。
    max_goal_distance: float = 8.0
    # 参数 min_frontier_cells：有效前沿簇至少包含的格数；等号右边是默认值。
    min_frontier_cells: int = 3
    # 参数 frontier_gain_weight：前沿大小的评分权重；等号右边是默认值。
    frontier_gain_weight: float = 1.0
    # 参数 path_cost_weight：路径成本的评分权重；等号右边是默认值。
    path_cost_weight: float = 1.0
    # 参数 unknown_path_multiplier：未知区域路径成本倍率；等号右边是默认值。
    unknown_path_multiplier: float = 1.5


# 限制没有新地图收益时的反复回退。
class RecoveryGate:
    """没有有效前沿或地图无增长时，防止机器人盲目反复回退。"""

    # 创建对象并为后续运行准备数据。
    def __init__(self, min_new_cells):
        # 更新对象字段 min_new_cells，供后续方法读取。
        self.min_new_cells = min_new_cells
        # 更新对象字段 started_at，供后续方法读取。
        self.started_at = None
        # 更新对象字段 blocked_at，供后续方法读取。
        self.blocked_at = None

    # 判断地图是否增长到允许再次回退的门槛。
    def waiting_for_new_cells(self, observed_cells):
        # 返回是否仍须等待地图新增格。
        return (self.blocked_at is not None and
                observed_cells - min(observed_cells, self.blocked_at) < self.min_new_cells)

    # 判断当前是否满足沿旧轨迹回退的条件。
    def eligible(self, clusters, reachable_frontier_cells, candidate_goals, observed_cells):
        # 返回是否允许沿旧轨迹回退。
        return (clusters > 0 and reachable_frontier_cells == 0 and candidate_goals == 0
                and not self.waiting_for_new_cells(observed_cells))

    # 记录回退开始时已观察的网格数。
    def start(self, observed_cells):
        # 更新对象字段 started_at，供后续方法读取。
        self.started_at = observed_cells

    # 记录一次回退或目标结束后的地图增长。
    def finish(self, observed_cells):
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if self.started_at is None:
            # 结束当前函数，不再执行后面的语句。
            return
        # 条件检查：新增观察格数是否达到收益门槛；成立才进入下面的分支。
        if observed_cells - min(observed_cells, self.started_at) < self.min_new_cells:
            # 更新对象字段 blocked_at，供后续方法读取。
            self.blocked_at = observed_cells
        # 上述条件不成立时，改走这个分支。
        else:
            # 更新对象字段 blocked_at，供后续方法读取。
            self.blocked_at = None
        # 更新对象字段 started_at，供后续方法读取。
        self.started_at = None


# 阻止已到达但没有获得新地图的目标被反复选中。
class GoalProgressGate:
    """到达目标却没有增加已观察格时，暂时不再选择该目标。"""

    # 创建对象并为后续运行准备数据。
    def __init__(self, min_new_cells):
        # 更新对象字段 min_new_cells，供后续方法读取。
        self.min_new_cells = min_new_cells
        # 更新对象字段 entries，供后续方法读取。
        self.entries = []

    # 记录一次回退或目标结束后的地图增长。
    def finish(self, goal, before, after):
        # 条件检查：新增观察格数是否达到收益门槛；成立才进入下面的分支。
        if after - min(after, before) < self.min_new_cells:
            # 把本次结果放入容器，供下一轮处理。
            self.entries.append((goal, after + self.min_new_cells))

    # 返回尚未解除屏蔽的目标。
    def blocked(self, observed_cells):
        # 更新对象字段 entries，供后续方法读取。
        self.entries = [(goal, unlock_at) for goal, unlock_at in self.entries
                        if observed_cells < unlock_at]
        # 返回当前仍需屏蔽的目标列表。
        return [goal for goal, _ in self.entries]


# 维护稀疏三维体素证据，未保存的格子就是未知。
class VoxelMap:
    # 创建对象并为后续运行准备数据。
    def __init__(self, settings):
        # 更新对象字段 cfg，供后续方法读取。
        self.cfg = settings
        # 更新对象字段 voxels，供后续方法读取。
        self.voxels = {}
        # 更新对象字段 version，供后续方法读取。
        self.version = 0

    # 把米制坐标转换为离散网格编号。
    def key(self, point):
        # 返回点所在体素的整数 X/Y/Z 格号。
        return tuple(math.floor(value / self.cfg.resolution) for value in point)

    # 把网格号换成格子中心的米制坐标。
    def cell_center(self, cell):
        # 返回目标格中心的米制坐标。
        return tuple((value + 0.5) * self.cfg.resolution for value in cell)

    # 给射线穿过的格减证据，给障碍端点加证据。
    def _mark(self, key, hit):
        # 将计算结果保存为 previous（这个体素更新之前的证据值）。
        previous = self.voxels.get(key)
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if previous is None and len(self.voxels) >= self.cfg.max_voxels:
            # 结束当前函数，不再执行后面的语句。
            return
        # 射线终点的障碍证据 +3，强于单条后续空闲射线的 -1。
        self.voxels[key] = max(-3, min(6, (previous or 0) + (3 if hit else -1)))

    # 沿雷达射线更新三维体素地图。
    def integrate(self, origin, points, base_z):
        # 机体高度若为 NaN 或无穷大，不能用来建立相对高度地图。
        if not math.isfinite(base_z):
            # 返回零，表示本次没有有效数据。
            return 0
        # 将计算结果保存为 inserted（本帧纳入地图的点数）。
        inserted = 0
        # 依次处理一帧点云点集。
        for point in points:
            # 某个点坐标无效就跳过这个点，不影响同帧其他点。
            if not all(math.isfinite(v) for v in point):
                # 跳过本次循环，检查下一个元素。
                continue
            # 把雷达终点减去起点，得到射线在 X/Y/Z 三个方向的位移。
            dx, dy, dz = (point[i] - origin[i] for i in range(3))
            # 将计算结果保存为 distance（机器人到目标的水平距离）。
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            # 条件检查：射线超出允许的雷达距离；成立才进入下面的分支。
            if distance < self.cfg.resolution or distance > self.cfg.max_ray_range:
                # 跳过本次循环，检查下一个元素。
                continue
            # 将计算结果保存为 steps（射线采样步数）。
            steps = max(1, math.ceil(distance / (self.cfg.resolution * 0.5)))
            # 记住上次射线采样所在体素，避免同一体素重复扣证据。
            last = None
            # 依次处理一条射线的离散采样次数。
            for step in range(steps):
                # 将计算结果保存为 fraction（当前采样点在射线上的比例）。
                fraction = step / steps
                # 把当前射线采样点换算成机体相对高度下的体素编号。
                cell = self.key((origin[0] + dx * fraction,
                                 origin[1] + dy * fraction,
                                 origin[2] - base_z + dz * fraction))
                # 只有进入新体素时才更新一次空闲证据。
                if cell != last:
                    # 射线途经体素标为观察空闲，证据减少一。
                    self._mark(cell, False)
                    # 更新上次体素编号，供下一次采样去重。
                    last = cell
            # 将计算结果保存为 endpoint（射线末端所在的体素）。
            endpoint = self.key((point[0], point[1], point[2] - base_z))
            # 将计算结果保存为 obstacle（射线末端是否在障碍高度范围）。
            obstacle = (base_z + self.cfg.obstacle_min_relative_z <= point[2]
                        <= base_z + self.cfg.obstacle_max_relative_z)
            # 射线末端按高度判定为障碍或空闲后更新证据。
            self._mark(endpoint, obstacle)
            # 本帧有效点计数加一。
            inserted += 1
        # 只有本帧有有效点时才更新地图版本。
        if inserted:
            # 地图版本加一，表示已有新点云并入。
            self.version += 1
        # 返回更新后的体素证据。
        return inserted

    # 把三维体素投影到机体附近的二维平面。
    def project(self, center_xy, all_observed=False):
        """把机体相对高度范围内的三维证据投影为二维占用/空闲格。"""
        # 将计算结果保存为 xy（地图坐标系下的目标水平坐标）。
        xy = {}
        # 投影区域比搜索半径稍大，给边缘邻格留出计算空间。
        r = self.cfg.local_search_radius + self.cfg.resolution * 2
        # 将计算结果保存为 occupied_low（障碍判定高度下界）。
        occupied_low = self.cfg.obstacle_min_relative_z
        # 将计算结果保存为 free_low（投影参与的最低高度）。
        free_low = self.cfg.free_min_relative_z
        # 将计算结果保存为 high（投影参与的最高高度）。
        high = self.cfg.obstacle_max_relative_z
        # 依次处理三维体素证据表。
        for (x, y, z), value in self.voxels.items():
            # 把整数体素编号换算成米制中心坐标。
            wx, wy, wz = self.cell_center((x, y, z))
            # 高度不在当前同层切片内时跳过这个体素。
            if not free_low <= wz <= high:
                # 跳过本次循环，检查下一个元素。
                continue
            # 条件检查：当前启用全图搜索；成立才进入下面的分支。
            if not all_observed and self.cfg.global_search:
                # 全图模式也只接纳配置矩形边界内的体素。
                if not (self.cfg.exploration_min_x <= wx <= self.cfg.exploration_max_x
                        and self.cfg.exploration_min_y <= wy <= self.cfg.exploration_max_y):
                    # 跳过本次循环，检查下一个元素。
                    continue
            # 上一条件不满足时，再检查这个分支。
            elif (not all_observed and not self.cfg.global_search and
                  (abs(wx - center_xy[0]) > r or abs(wy - center_xy[1]) > r)):
                # 跳过本次循环，检查下一个元素。
                continue
            # 丢掉高度编号，得到水平二维格号。
            cell = (x, y)
            # 障碍证据为正且高度足够时，二维格标记为障碍。
            if value > 0 and wz >= occupied_low:
                # 将计算结果保存为 xy（地图坐标系下的目标水平坐标）。
                xy[cell] = 1
            # 上一条件不满足时，再检查这个分支。
            elif value <= 0 and cell not in xy:
                # 将计算结果保存为 xy（地图坐标系下的目标水平坐标）。
                xy[cell] = 0
        # 返回本函数计算得到的结果。
        return xy


# 只含上下左右四个方向，用于定义未知边界。
NEIGHBORS4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
# 在四邻域基础上增加四个对角方向，用于聚类和路径搜索。
NEIGHBORS8 = NEIGHBORS4 + ((1, 1), (1, -1), (-1, 1), (-1, -1))


def assess_coverage(grid, settings):
    """逐格统计全局矩形；无效边界返回 None，不能据此宣布完成。"""
    edges = (settings.exploration_min_x, settings.exploration_max_x,
             settings.exploration_min_y, settings.exploration_max_y)
    r = settings.resolution
    if (not settings.global_search or not math.isfinite(r) or r <= 0 or
            not all(map(math.isfinite, edges)) or edges[0] >= edges[1] or
            edges[2] >= edges[3] or
            (edges[1] - edges[0]) * (edges[3] - edges[2]) / r**2 > 2_000_000):
        return None
    x0, x1 = math.ceil(edges[0] / r - .5), math.floor(edges[1] / r - .5)
    y0, y1 = math.ceil(edges[2] / r - .5), math.floor(edges[3] / r - .5)
    if x1 < x0 or y1 < y0:
        return None
    free = occupied = unknown = 0
    for x in range(x0, x1 + 1):
        for y in range(y0, y1 + 1):
            state = grid.get((x, y))
            if state is None:
                unknown += 1
            elif state == 0:
                free += 1
            else:
                occupied += 1
    total = free + occupied + unknown
    return {'total_cells': total, 'known_free_cells': free,
            'known_occupied_cells': occupied, 'unknown_cells': unknown,
            'coverage_ratio': (total - unknown) / total}


def assess_reachable_coverage(grid, settings, robot_xy):
    """用已知障碍膨胀后的连通区域验收；未知格仍按潜在通路处理。"""
    from collections import deque

    result = assess_coverage(grid, settings)
    if result is None:
        return None
    result.update(reachability_valid=False, reachable_cells=0,
                  reachable_unknown_cells=0, excluded_unknown_cells=0,
                  reachable_coverage_ratio=0.0)
    r = settings.resolution
    radius = settings.robot_radius
    if not all(map(math.isfinite, robot_xy)) or not math.isfinite(radius) or radius < 0:
        return result
    x0 = math.ceil(settings.exploration_min_x / r - .5)
    x1 = math.floor(settings.exploration_max_x / r - .5)
    y0 = math.ceil(settings.exploration_min_y / r - .5)
    y1 = math.floor(settings.exploration_max_y / r - .5)

    def inside(cell):
        return x0 <= cell[0] <= x1 and y0 <= cell[1] <= y1

    # 只膨胀累计点云的障碍；SCAN 临时障碍不能缩小完成验收范围。
    blocked = set()
    inflation = math.ceil(radius / r)
    for (x, y), state in grid.items():
        if state != 1:
            continue
        for dx in range(-inflation, inflation + 1):
            for dy in range(-inflation, inflation + 1):
                cell = (x + dx, y + dy)
                if inside(cell) and math.hypot(dx, dy) * r <= radius:
                    blocked.add(cell)
    robot = (math.floor(robot_xy[0] / r), math.floor(robot_xy[1] / r))
    if not inside(robot):
        return result
    search = math.ceil((radius + r) / r)
    candidates = ((math.hypot((robot[0] + dx + .5) * r - robot_xy[0],
                              (robot[1] + dy + .5) * r - robot_xy[1]),
                   (robot[0] + dx, robot[1] + dy))
                  for dx in range(-search, search + 1)
                  for dy in range(-search, search + 1))
    seed = min(((distance, cell) for distance, cell in candidates
                if distance <= radius + r and inside(cell) and cell not in blocked
                and grid.get(cell) == 0), default=None)
    if seed is None:
        return result
    result['reachability_valid'] = True
    queue = deque([seed[1]])
    seen = {seed[1]}
    reachable_unknown = 0
    while queue:
        cell = queue.popleft()
        reachable_unknown += cell not in grid
        for dx, dy in NEIGHBORS8:
            if dx and dy and ((cell[0] + dx, cell[1]) in blocked and
                              (cell[0], cell[1] + dy) in blocked):
                continue
            next_cell = (cell[0] + dx, cell[1] + dy)
            if inside(next_cell) and next_cell not in blocked and next_cell not in seen:
                seen.add(next_cell)
                queue.append(next_cell)
    result['reachable_cells'] = len(seen)
    result['reachable_unknown_cells'] = reachable_unknown
    result['excluded_unknown_cells'] = result['unknown_cells'] - reachable_unknown
    result['reachable_coverage_ratio'] = (len(seen) - reachable_unknown) / len(seen)
    return result


def assess_global_frontiers(grid, settings, robot_xy):
    """只从机器人所在的已知空闲连通区域寻找未经选点过滤的原始前沿。"""
    invalid = {'valid': False, 'reachable_free_cells': 0, 'frontier_cells': 0}
    r, radius = settings.resolution, settings.robot_radius
    if (not grid or not math.isfinite(r) or r <= 0 or
            not math.isfinite(radius) or radius < 0 or
            not all(map(math.isfinite, robot_xy))):
        return invalid

    # 验收只使用累计障碍，不能让瞬时 SCAN 障碍抹掉待探索边界。
    blocked = set()
    inflation = math.ceil(radius / r)
    for (x, y), state in grid.items():
        if state != 1:
            continue
        for dx in range(-inflation, inflation + 1):
            for dy in range(-inflation, inflation + 1):
                if math.hypot(dx, dy) * r <= radius:
                    blocked.add((x + dx, y + dy))

    robot = (math.floor(robot_xy[0] / r), math.floor(robot_xy[1] / r))
    search = math.ceil((radius + r) / r)
    candidates = ((math.hypot((robot[0] + dx + .5) * r - robot_xy[0],
                              (robot[1] + dy + .5) * r - robot_xy[1]),
                   (robot[0] + dx, robot[1] + dy))
                  for dx in range(-search, search + 1)
                  for dy in range(-search, search + 1))
    seed = min(((distance, cell) for distance, cell in candidates
                if distance <= radius + r and cell not in blocked and
                grid.get(cell) == 0), default=None)
    if seed is None:
        return invalid

    queue = deque([seed[1]])
    seen = {seed[1]}
    frontiers = set()
    while queue:
        x, y = queue.popleft()
        for dx, dy in NEIGHBORS4:
            neighbor = (x + dx, y + dy)
            if neighbor in blocked:
                continue
            state = grid.get(neighbor)
            if state is None:
                frontiers.add(neighbor)
            elif state == 0 and neighbor not in seen:
                seen.add(neighbor)
                queue.append(neighbor)
    return {'valid': True, 'reachable_free_cells': len(seen),
            'frontier_cells': len(frontiers), 'cells': frontiers}


def select_observation_goal(voxels, robot_xyz, settings, grid, suppressed=(),
                            external_obstacles=()):
    """无普通前沿时，从已知空闲连通区找可观测未知格的位置。"""
    if assess_coverage(grid, settings) is None:
        return None
    r = settings.resolution
    inflation = math.ceil(settings.robot_radius / r)
    blocked = set()
    centers = [cell for cell, value in grid.items() if value == 1]
    centers.extend((math.floor(x / r), math.floor(y / r))
                   for x, y in external_obstacles if math.isfinite(x) and math.isfinite(y))
    for x, y in centers:
        for dx in range(-inflation, inflation + 1):
            for dy in range(-inflation, inflation + 1):
                if math.hypot(dx, dy) * r <= settings.robot_radius:
                    blocked.add((x + dx, y + dy))

    def free(cell):
        return grid.get(cell) == 0 and cell not in blocked

    origin = voxels.key(robot_xyz)[:2]
    queue, reached = deque([origin]), {origin}
    best, best_score = None, -math.inf
    stride = max(1, round(1.0 / r))
    ray_cells = math.ceil(settings.max_ray_range / r)
    while queue:
        cell = queue.popleft()
        xy = voxels.cell_center(cell)
        travel = math.dist(xy, robot_xyz[:2])
        if (free(cell) and settings.min_goal_distance <= travel <= settings.max_goal_distance
                and cell[0] % stride == 0 and cell[1] % stride == 0 and
                all(math.dist(xy, old) >= settings.robot_radius * 2 for old in suppressed)):
            visible = 0
            for dx in range(-ray_cells, ray_cells + 1):
                if visible >= 16:
                    break
                for dy in range(-ray_cells, ray_cells + 1):
                    if visible >= 16:
                        break
                    if dx == dy == 0 or math.hypot(dx, dy) * r > settings.max_ray_range:
                        continue
                    target = (cell[0] + dx, cell[1] + dy)
                    tx, ty = voxels.cell_center(target)
                    if (target in grid or not settings.exploration_min_x <= tx <= settings.exploration_max_x
                            or not settings.exploration_min_y <= ty <= settings.exploration_max_y):
                        continue
                    steps = max(abs(dx), abs(dy)) * 2
                    occluded = False
                    for step in range(1, steps):
                        crossed = (math.floor(cell[0] + .5 + dx * step / steps),
                                   math.floor(cell[1] + .5 + dy * step / steps))
                        if grid.get(crossed) == 1:
                            occluded = True
                            break
                    if not occluded:
                        visible += 1
            score = visible - .25 * travel
            if visible and score > best_score:
                best, best_score = xy, score
        for dx, dy in NEIGHBORS4:
            nxt = (cell[0] + dx, cell[1] + dy)
            if nxt not in reached and free(nxt):
                reached.add(nxt)
                queue.append(nxt)
    return best


# 聚类未知边界、计算路径代价并选出目标。
def select_frontier(voxels, robot_xyz, settings, suppressed=(), external_obstacles=(),
                    all_observed=False, allowed_frontiers=None):
    """返回目标、评分、簇数、原始前沿格数及诊断计数。"""
    # 流程与 C++ 版相同：三维地图投影 → 障碍膨胀 → 未知边界聚类 → Dijkstra → 评分。
    # 将计算结果保存为 center（机器人所在的网格或待查询区域中心）。
    center = voxels.key(robot_xyz)[:2]
    # 将计算结果保存为 grid（高度切片后的二维占用表）。
    grid = voxels.project(robot_xyz[:2], all_observed=all_observed)
    min_x = min((cell[0] for cell in grid), default=center[0])
    max_x = max((cell[0] for cell in grid), default=center[0])
    min_y = min((cell[1] for cell in grid), default=center[1])
    max_y = max((cell[1] for cell in grid), default=center[1])
    min_x, max_x = min(min_x, center[0]), max(max_x, center[0])
    min_y, max_y = min(min_y, center[1]), max(max_y, center[1])
    search_limited = (all_observed and
                      (max_x - min_x + 3) * (max_y - min_y + 3) > 2_000_000)
    # 将计算结果保存为 radius（搜索半径或膨胀格数）。
    radius = math.ceil(settings.local_search_radius / settings.resolution)
    # 将计算结果保存为 occupied（二维图中已观察到的障碍格）。
    # 第一步：找出已知障碍；按机器人半径把它们扩张为禁行格。
    occupied = {cell for cell, value in grid.items() if value == 1}
    # 将计算结果保存为 inflation（机器人半径折算的格数）。
    inflation = math.ceil(settings.robot_radius / settings.resolution)
    # 将计算结果保存为 blocked（考虑机器人半径后的禁行格）。
    blocked = set()
    # 依次处理已知障碍格。
    for x, y in occupied:
        # 依次处理按机器人半径向外扩张的格数。
        for dx in range(-inflation, inflation + 1):
            # 依次处理按机器人半径向外扩张的格数。
            for dy in range(-inflation, inflation + 1):
                # 仅扩张到机器人半径以内，不把方形角落也算成障碍。
                if math.hypot(dx, dy) * settings.resolution <= settings.robot_radius:
                    # 把已知障碍周围不可通行的格子加入禁行集合。
                    blocked.add((x + dx, y + dy))
    # 依次处理SCAN 提供的额外障碍点。
    for x, y in external_obstacles:
        # 条件检查：数据必须是有效有限数；成立才进入下面的分支。
        if math.isfinite(x) and math.isfinite(y):
            # 把 SCAN 障碍点的米制 XY 换算成二维格号。
            cx, cy = math.floor(x / settings.resolution), math.floor(y / settings.resolution)
            # 依次处理按机器人半径向外扩张的格数。
            for dx in range(-inflation, inflation + 1):
                # 依次处理按机器人半径向外扩张的格数。
                for dy in range(-inflation, inflation + 1):
                    # 仅扩张到机器人半径以内，不把方形角落也算成障碍。
                    if math.hypot(dx, dy) * settings.resolution <= settings.robot_radius:
                        # 把 SCAN 障碍周围的膨胀格也加入禁行集合。
                        blocked.add((cx + dx, cy + dy))

    # 判断一个格子是否处在局部窗口或全局矩形范围内。
    def inside(cell):
        if all_observed:
            return (min_x - 1 <= cell[0] <= max_x + 1 and
                    min_y - 1 <= cell[1] <= max_y + 1)
        # 条件检查：当前启用全图搜索；成立才进入下面的分支。
        if settings.global_search:
            # 把候选前沿格转成可以发布的米制目标坐标。
            x, y = voxels.cell_center(cell)
            # 返回目标格中心的米制坐标。
            return (settings.exploration_min_x <= x <= settings.exploration_max_x
                    and settings.exploration_min_y <= y <= settings.exploration_max_y)
        # 返回目标格中心的米制坐标。
        return abs(cell[0] - center[0]) <= radius and abs(cell[1] - center[1]) <= radius

    # 第二步：前沿是已观察空闲格旁边的真正未知格；字典中没出现过才算未知。
    # 将计算结果保存为 frontier（已知空闲格旁边的未知格集合）。
    frontier = set()
    # 依次处理二维投影地图。
    for x, y in grid:
        # 只有已经观察为空闲的格子能作为搜索未知邻格的起点。
        if grid[(x, y)] != 0:
            # 跳过本次循环，检查下一个元素。
            continue
        # 依次处理集合中的每一项。
        for dx, dy in NEIGHBORS4:
            # 沿当前方向找到旁边的一个候选格。
            cell = (x + dx, y + dy)
            # 条件检查：候选格位于允许的搜索范围内；成立才进入下面的分支。
            if (inside(cell) and cell not in grid and cell not in blocked and
                    (allowed_frontiers is None or cell in allowed_frontiers)):
                # 这个邻格尚未被观察且不在障碍膨胀区，因此加入前沿。
                frontier.add(cell)

    # 将计算结果保存为 frontier_cells（聚类前的原始前沿格数）。
    frontier_cells = len(frontier)
    # 将计算结果保存为 clusters（达到最小规模的前沿簇集合）。
    # 第三步：八邻接聚类。deque 中先进入的格先处理，直到当前簇搜索完。
    clusters = []
    # 反复处理已知空闲格与未知格的交界，直到循环条件不再成立。
    while frontier:
        # 从容器中取走已处理或失效的数据。
        seed = frontier.pop()
        # 将计算结果保存为 cluster（当前相连的前沿格集合）。
        cluster = [seed]
        # 将计算结果保存为 queue（广度搜索或最短路待处理队列）。
        queue = deque([seed])
        # 反复处理待处理的搜索队列，直到循环条件不再成立。
        while queue:
            # 取出当前簇里下一个待搜索的前沿格。
            x, y = queue.popleft()
            # 依次处理集合中的每一项。
            for dx, dy in NEIGHBORS8:
                # 沿当前方向找到旁边的一个候选格。
                cell = (x + dx, y + dy)
                # 尚未归属其他簇的邻格才加入当前簇。
                if cell in frontier:
                    # 从容器中取走已处理或失效的数据。
                    frontier.remove(cell)
                    # 把本次结果放入容器，供下一轮处理。
                    cluster.append(cell)
                    # 把本次结果放入容器，供下一轮处理。
                    queue.append(cell)
        # 条件检查：前沿簇达到最小规模门槛；成立才进入下面的分支。
        if len(cluster) >= settings.min_frontier_cells:
            # 把本次结果放入容器，供下一轮处理。
            clusters.append(cluster)

    # 初始化各类计数，最终发布到状态话题帮助诊断。
    diagnostics = {'reachable_frontier_cells': 0, 'candidate_goals': 0,
                   'suppressed_goals': 0, 'open_start_neighbors': 0}
    # 条件检查：候选格超出允许的搜索范围；成立才进入下面的分支。
    if not clusters or not inside(center) or search_limited:
        # 返回本函数计算得到的结果。
        return None, None, len(clusters), frontier_cells, diagnostics
    # 统计机器人起点周围未被膨胀障碍堵住的方向数。
    diagnostics['open_start_neighbors'] = sum(
        inside((center[0] + dx, center[1] + dy)) and
        (center[0] + dx, center[1] + dy) not in blocked for dx, dy in NEIGHBORS8)

    # 第四步：从机器人格运行一次 Dijkstra；未知格可经过，但路径代价额外乘倍率。
    # 将计算结果保存为 costs（起点到各个格子的最小路径代价）。
    costs = {center: 0.0}
    # 将计算结果保存为 parents（最短路径中每格的前驱格）。
    parents = {}
    # 将计算结果保存为 heap（按路径代价升序处理的格子队列）。
    heap = [(0.0, center)]
    # 反复处理按代价排序的搜索队列，直到循环条件不再成立。
    while heap:
        # 取出当前已知最低路径代价的格子继续扩展。
        cost, cell = heapq.heappop(heap)
        # 条件检查：此格已有更短的到达路径；成立才进入下面的分支。
        if cost > costs[cell]:
            # 跳过本次循环，检查下一个元素。
            continue
        # 依次处理集合中的每一项。
        for dx, dy in NEIGHBORS8:
            # 求出沿当前方向迈一步后的新格号。
            nxt = (cell[0] + dx, cell[1] + dy)
            # 条件检查：候选格超出允许的搜索范围；成立才进入下面的分支。
            if not inside(nxt) or nxt in blocked:
                # 跳过本次循环，检查下一个元素。
                continue
            # 条件检查：候选格已被障碍膨胀区占据；成立才进入下面的分支。
            if dx and dy and ((cell[0] + dx, cell[1]) in blocked or
                              (cell[0], cell[1] + dy) in blocked):
                # 跳过本次循环，检查下一个元素。
                continue
            # 将计算结果保存为 step（移动到相邻格的代价或射线采样序号）。
            step = settings.resolution * math.hypot(dx, dy)
            # 条件检查：候选格还没有被雷达观测；成立才进入下面的分支。
            if nxt not in grid:
                # 未知格可经过，但把这一步的路径成本按倍率增加。
                step *= settings.unknown_path_multiplier
            # 将计算结果保存为 new_cost（经过当前路径到下一格的新代价）。
            new_cost = cost + step
            # 条件检查：此格已有更短的到达路径；成立才进入下面的分支。
            if new_cost < costs.get(nxt, math.inf):
                # 将计算结果保存为 costs（起点到各个格子的最小路径代价）。
                costs[nxt] = new_cost
                # 将计算结果保存为 parents（最短路径中每格的前驱格）。
                parents[nxt] = cell
                # 将更短路径对应的新格放回最小堆，稍后继续扩展。
                heapq.heappush(heap, (new_cost, nxt))

    # 将计算结果保存为 best（当前评分最高的候选目标）。
    # 第五步：按簇大小和路径代价评分；远处簇先转成导航范围内的分段落点。
    best = None
    # 将计算结果保存为 best_score（目前的最高目标评分）。
    best_score = -math.inf
    # 依次处理有效前沿簇列表。
    for cluster in clusters:
        # 统计这个簇里确实存在搜索路径的前沿格数。
        diagnostics['reachable_frontier_cells'] += sum(cell in costs for cell in cluster)
        # 将计算结果保存为 mx（簇内各格 X 坐标的平均值）。
        mx = sum(c[0] for c in cluster) / len(cluster)
        # 将计算结果保存为 my（簇内各格 Y 坐标的平均值）。
        my = sum(c[1] for c in cluster) / len(cluster)
        # 几何中心可能在障碍内，因此选距中心最近的真实前沿格。
        ordered = sorted(cluster, key=lambda c: (c[0] - mx) ** 2 + (c[1] - my) ** 2)
        # 依次处理当前二维网格。
        for cell in ordered:
            # 条件检查：此格已有更短的到达路径；成立才进入下面的分支。
            if cell not in costs:
                # 跳过本次循环，检查下一个元素。
                continue
            # 把候选前沿格转成可以发布的米制目标坐标。
            x, y = voxels.cell_center(cell)
            # 将计算结果保存为 distance（机器人到目标的水平距离）。
            distance = math.hypot(x - robot_xyz[0], y - robot_xyz[1])
            # 条件检查：目标是否离机器人太近；成立才进入下面的分支。
            if distance < settings.min_goal_distance:
                # 跳过本次循环，检查下一个元素。
                continue
            # 条件检查：目标是否超出单次导航范围；成立才进入下面的分支。
            if distance > settings.max_goal_distance:
                # 远处前沿先在局部规划范围内找一个已观察空闲格作分段目标。
                # 将计算结果保存为 stage（远处前沿路径上的分段目标格）。
                stage = cell
                # 将计算结果保存为 stage_goal（路径上满足距离范围的已知空闲落点）。
                stage_goal = None
                # 反复处理最短路径上的前一格，直到循环条件不再成立。
                while stage != center and stage in parents:
                    # 条件检查：候选格已被障碍膨胀区占据；成立才进入下面的分支。
                    if grid.get(stage) == 0 and stage not in blocked:
                        # 将计算结果保存为 candidate（本次检查的候选落点）。
                        candidate = voxels.cell_center(stage)
                        # 计算分段落点到机器人的直线距离。
                        stage_distance = math.dist(candidate, robot_xyz[:2])
                        # 条件检查：目标是否离机器人太近；成立才进入下面的分支。
                        if settings.min_goal_distance <= stage_distance <= settings.max_goal_distance:
                            # 将计算结果保存为 stage_goal（路径上满足距离范围的已知空闲落点）。
                            stage_goal = candidate
                            # 找到满足条件的结果，退出当前循环。
                            break
                    # 将计算结果保存为 stage（远处前沿路径上的分段目标格）。
                    stage = parents[stage]
                # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
                if stage_goal is None:
                    # 跳过本次循环，检查下一个元素。
                    continue
                # 把远处前沿目标改成导航距离范围内的分段落点。
                x, y = stage_goal
            # 记录一个通过距离范围检查的候选目标。
            diagnostics['candidate_goals'] += 1
            # 若目标落在失败冷却区附近，本轮不要再发它。
            if any(math.hypot(x - sx, y - sy) < settings.robot_radius * 2
                   # 依次处理暂时禁止重选的目标。
                   for sx, sy in suppressed):
                # 因冷却而过滤掉的候选数加一。
                diagnostics['suppressed_goals'] += 1
                # 跳过本次循环，检查下一个元素。
                continue
            # 将计算结果保存为 score（目标收益减去路径代价后的评分）。
            score = (settings.frontier_gain_weight * math.log1p(len(cluster))
                     - settings.path_cost_weight * costs[cell])
            # 条件检查：当前候选的评分是否超过旧目标；成立才进入下面的分支。
            if score > best_score:
                # 将计算结果保存为 best（当前评分最高的候选目标）。
                best = (x, y)
                # 将计算结果保存为 best_score（目前的最高目标评分）。
                best_score = score
            # 找到满足条件的结果，退出当前循环。
            break
    # 返回本函数计算得到的结果。
    return best, (best_score if best is not None else None), len(clusters), frontier_cells, diagnostics
