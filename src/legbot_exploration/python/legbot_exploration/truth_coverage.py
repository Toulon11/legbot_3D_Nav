"""离线解析单层 Gazebo SDF，独立核验探索网格；不参与在线选点。"""

from collections import deque
import math
import xml.etree.ElementTree as ET


def _pose(element):
    text = element.findtext('pose')
    return tuple(map(float, text.split())) if text else (0., 0., 0., 0., 0., 0.)


def scene_geometry(sdf_path, robot_height=1.1):
    """读取地面平面与和机器人高度重叠的静态碰撞体。"""
    root = ET.parse(sdf_path).getroot()
    ground = []
    obstacles = []
    for model in root.findall('.//world/model'):
        mx, my, mz, _, _, _ = _pose(model)
        if model.findtext('static') != 'true':
            continue
        for link in model.findall('link'):
            lx, ly, lz, _, _, yaw = _pose(link)
            x, y, z = mx + lx, my + ly, mz + lz
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry is None:
                    continue
                plane = geometry.find('plane')
                box = geometry.find('box')
                cylinder = geometry.find('cylinder')
                if plane is not None:
                    sx, sy = map(float, plane.findtext('size').split())
                    ground.append((x - sx / 2, x + sx / 2,
                                   y - sy / 2, y + sy / 2))
                elif box is not None:
                    sx, sy, sz = map(float, box.findtext('size').split())
                    if z - sz / 2 <= robot_height and z + sz / 2 >= 0:
                        obstacles.append(('box', x, y, sx, sy, yaw))
                elif cylinder is not None:
                    radius = float(cylinder.findtext('radius'))
                    length = float(cylinder.findtext('length'))
                    if z - length / 2 <= robot_height and z + length / 2 >= 0:
                        obstacles.append(('cylinder', x, y, radius))
    if not ground:
        raise ValueError('SDF 中没有可验收的地面 plane')
    return ground, obstacles


def reachable_ground_cells(sdf_path, origin_x, origin_y, width, height, resolution,
                           spawn_xy=(-27., 6.), robot_radius=.5):
    """根据真实静态几何，计算从出生点可达的平地网格索引。"""
    ground, obstacles = scene_geometry(sdf_path)
    traversable = set()
    for iy in range(height):
        y = origin_y + (iy + .5) * resolution
        for ix in range(width):
            x = origin_x + (ix + .5) * resolution
            if not any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1 in ground):
                continue
            blocked = False
            for obstacle in obstacles:
                if obstacle[0] == 'cylinder':
                    _, cx, cy, radius = obstacle
                    blocked = math.hypot(x - cx, y - cy) <= radius + robot_radius
                else:
                    _, cx, cy, sx, sy, yaw = obstacle
                    dx, dy = x - cx, y - cy
                    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
                    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
                    near_x = max(abs(local_x) - sx / 2, 0.)
                    near_y = max(abs(local_y) - sy / 2, 0.)
                    blocked = math.hypot(near_x, near_y) <= robot_radius
                if blocked:
                    break
            if not blocked:
                traversable.add((ix, iy))
    start = (math.floor((spawn_xy[0] - origin_x) / resolution),
             math.floor((spawn_xy[1] - origin_y) / resolution))
    if start not in traversable:
        raise ValueError('出生点不在可通行平地内；请检查场景、坐标或机器人半径')
    reached, queue = {start}, deque([start])
    while queue:
        x, y = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (x + dx, y + dy)
            if neighbor in traversable and neighbor not in reached:
                reached.add(neighbor)
                queue.append(neighbor)
    return {y * width + x for x, y in reached}


def verify_grid(sdf_path, origin_x, origin_y, width, height, resolution, data,
                spawn_xy=(-27., 6.), robot_radius=.5):
    if len(data) != width * height:
        raise ValueError('OccupancyGrid data 与 width/height 不一致')
    truth = reachable_ground_cells(sdf_path, origin_x, origin_y, width, height,
                                   resolution, spawn_xy, robot_radius)
    missing = sum(data[index] == -1 for index in truth)
    return {'reachable_ground_cells': len(truth), 'unobserved_reachable_cells': missing,
            'reachable_ground_coverage': 1 - missing / len(truth)}
