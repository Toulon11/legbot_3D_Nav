"""Native PCT A* backed by a borrowed sparse-map accessor, plus native RDP."""
from types import SimpleNamespace
from collections import OrderedDict
from functools import partial
import time
import math
import weakref
import numpy as np
from lib import a_star
from .map_evidence import same_ground_surface


class BlockPlanner:
    def __init__(self, mapping, wrapper_type):
        self.map = mapping
        self.wrapper_type = wrapper_type
        self.cfg = SimpleNamespace(planner=SimpleNamespace(route_simplify_epsilon=.08))
        self.planner = a_star.Astar()
        if not hasattr(self.planner, 'init_surface_accessor'):
            raise RuntimeError('PCT surface accessor extension missing; rebuild pct_planner')
        self._intervals=hasattr(mapping,'_interval_nodes')
        if self._intervals and not hasattr(self.planner,'init_interval_accessor'):
            raise RuntimeError('PCT interval accessor extension missing; rebuild pct_planner')
        self.search_nodes = 0
        self.search_diagnostics = {}
        self._failures=OrderedDict()
        self._read_blocks=set()
        # A pybind std::function must not own a bound method that owns this
        # same native planner, which would form an uncollectable cycle.
        self._reader = lambda layer, row, col, floor=None, ref=weakref.ref(self): (
            ref()._read_interval(layer,row,col,floor) if ref()._intervals else ref()._read(layer,row,col,floor))

    def _read_interval(self, layer, row, column, floor=None):
        m=self.map
        key=(layer+m.first_layer,row+int(m.lower[0]),column+int(m.lower[1]))
        b=m.settings.block_size
        self._read_blocks.add((key[0],key[1]//b,key[2]//b))
        observed=m.observed_cell(key)
        owner=layer
        if observed is not None and np.isfinite(observed['G']):
            physical=m._interval_nodes.get(key[1:],{})
            source=next((k for k,node in physical.items()
                         if node.ground_id==int(observed['g_surface_id'])),key[0])
            owner=source-m.first_layer
        result=self._read(owner,row,column,floor)
        if result[0]>=50:return (*result,math.nan,math.nan,owner)
        return (*result,float(observed['C']),float(observed['node_z']),owner)

    def _read(self, layer, row, column, floor=None):
        m = self.map
        key = (layer+m.first_layer, row+int(m.lower[0]), column+int(m.lower[1]))
        b=m.settings.block_size
        self._read_blocks.add((key[0],key[1]//b,key[2]//b))
        if self._intervals:
            s=m.settings
            radius=max(s.inflation_kernel_size//2,
                       math.ceil((s.robot_radius+s.safety_margin)/s.resolution)+1)
            bands=math.ceil(2*s.step_max/s.slice_dh)+1
            self._read_blocks.update((k,bx,by)
                for k in range(key[0]-bands,key[0]+bands+1)
                for bx in range((key[1]-radius)//b,(key[1]+radius)//b+1)
                for by in range((key[2]-radius)//b,(key[2]+radius)//b+1))
        cell = m.observed_cell(key)
        if cell is not None and cell['solid'] and m.obstacle_at(key) is not None:
            # Known blocked interval, not a support state. Native search may
            # still reach a separately measured adjacent-cut tread within
            # the step limit; this cost never reports missing evidence.
            return (50.,-100.,0.,0)
        if (cell is None or not np.isfinite(cell['G']) or
                (floor is not None and abs(cell['G']-floor[0])>floor[1]+1e-7)):
            return (1e6, -100., 0., 0)
        cost = m.planning_cost(key)
        if (cell['passage_cost']>=50 and m.tread_above_riser(*key,cell,float(cell['G'])) is not None):
            # This cut ends at the riser. Let native A* check the higher
            # measured tread as a separate neighboring state.
            k,x,y=key
            self._read_blocks.add((k+1,x//b,y//b))
            return (1e6, -100., 0., 0)
        if (cell['unknown_hard'] or cell['solid_hard']) and m.seam_candidate(key[0],float(cell['G'])):
            reach = math.ceil((m.settings.robot_radius+m.settings.safety_margin)/m.settings.resolution+.5)
            k,x,y = key
            self._read_blocks.update((cut,bx,by) for cut in (k-1,k,k+1)
                                     for bx in range((x-reach)//b,(x+reach)//b+1)
                                     for by in range((y-reach)//b,(y+reach)//b+1))
        ground, surface_id = float(cell['G']), int(cell['g_surface_id'])
        gateway = 0.
        if cost > 8 and not cell['solid'] and not cell['unknown'] and cell['inflated_terrain_cost'] >= 0:
            for sign in (-1, 1):
                other_key = (key[0]+sign, key[1], key[2])
                self._read_blocks.add((other_key[0],key[1]//b,key[2]//b))
                other = m.cell(other_key)
                if other is None or other['solid'] or other['unknown'] or not np.isfinite(other['G']):
                    continue
                other_ground = float(other['G'])
                if not same_ground_surface(surface_id, ground, int(other['g_surface_id']), other_ground):
                    continue
                if floor is not None and abs(other_ground-floor[0])>floor[1]+1e-7:
                    continue
                other_cost=m.planning_cost(other_key)
                if other_cost <= 20 and cost-other_cost > 8:
                    gateway = 2.*sign
        return (float(cost), ground, gateway, surface_id)

    def _unknown_blockers(self, blocked, goal_height):
        m = self.map
        for (k, x, y), parent_height in blocked.items():
            cell = m.cell((k, x, y))
            height = min(parent_height+m.settings.step_max,
                         max(parent_height-m.settings.step_max, goal_height))
            if cell is None or cell['unknown'] or not np.isfinite(cell['G']) or cell['solid']:
                yield from m.unknown_blockers(k,[(x,y)],height)
            elif cell['unknown_hard'] or cell['solid_hard']:
                unknown,solid,_=m.footprint_conflicts(m.position((k,x,y)),k,separate=True)
                yield from m.unknown_blockers(k,unknown | solid,height)

    def plan(self, start, target, optimize=False, *, start_check=None, goal_check=None, floor=None,
             collect_blockers=False, dense=False, deadline=None):
        if optimize:
            raise ValueError('online PCT uses native A* and RDP; GPMP is disabled')
        m = self.map
        # Adapter calls pass the checks made under this same map read lock.
        # Direct callers use ground coordinates (reference height zero).
        if start_check is None:
            start_check = m.point_safe_check(start, 0., floor=floor)
        if goal_check is None:
            goal_check = m.point_safe_check(target, 0.,floor=floor)
        starts = list(start_check.get('safe_layers', [start_check['layer']] if start_check['ready'] else []))
        goals = list(goal_check.get('safe_layers', [goal_check['layer']] if goal_check['ready'] else []))
        self.search_nodes = 0
        self.search_diagnostics = dict(admitted_start_layer=start_check['layer'],
            admitted_goal_layer=goal_check['layer'], start_layers=starts, goal_layers=goals,
            attempts=[], selected_start_layer=None, selected_goal_layer=None)
        if not starts or not goals:
            return None
        shape = m.upper-m.lower
        if max(shape) > np.iinfo(np.int32).max:
            raise ValueError('pct_native_coordinate_limit')
        if m.last_layer-m.first_layer+1 > m.settings.max_cells//m.settings.block_size**2:
            raise ValueError('pct_native_layer_limit')
        a = m.indices(start)-m.lower
        b = m.indices(target)-m.lower
        if not collect_blockers and not m.connectivity.possible(m.indices(start),m.indices(target)):
            self.search_diagnostics['prefilter']='disconnected_xy_projection'
            return None
        cache_key=(tuple(a),tuple(b),tuple(starts),tuple(goals),tuple(m.lower),tuple(m.upper),
                   m.first_layer,m.last_layer,floor,collect_blockers)
        versions=m.planning_block_versions
        cached=self._failures.get(cache_key)
        if cached is not None:
            dependencies, blocker_records, revision = cached
            if (not collect_blockers or m.revision==revision) and all(
                    versions.get(key,0)==version for key,version in dependencies.items()):
                self.search_diagnostics['cached_disconnection']=True
                if collect_blockers:
                    self.search_diagnostics['unknown_blockers']=blocker_records
                return None
            del self._failures[cache_key]
        budget=m.settings.planning_budget_seconds
        limit=time.monotonic()+budget
        deadline=limit if deadline is None else min(deadline,limit)
        self._read_blocks.clear()
        # Try the lowest-cost admitted surfaces first, preferring their common
        # upper cuts. Each native search still sees every cut.
        pairs = [(first, last) for first in starts for last in goals]
        primary = (start_check['layer'], goal_check['layer'])
        higher_ground = max(start_check['ground_z'], goal_check['ground_z'])
        start_equivalent = set(start_check.get('equivalent_safe_layers',starts))
        goal_equivalent = set(goal_check.get('equivalent_safe_layers',goals))
        upper_common = {(layer, layer) for layer in set(starts) & set(goals)
                        if m.settings.origin_z+layer*m.settings.slice_dh > higher_ground+1e-7}
        pairs.sort(key=lambda pair: (pair[0] not in start_equivalent or pair[1] not in goal_equivalent,
                                    pair not in upper_common, pair[0] if pair in upper_common else 0,
                                    pair != primary, pair[0] != pair[1], abs(pair[0]-pair[1]),
                                    starts.index(pair[0]), goals.index(pair[1])))
        reader=self._reader if floor is None else partial(self._reader,floor=floor)
        blockers = {}
        radius = m.settings.robot_radius+m.settings.safety_margin
        self.planner.set_blocked_edge_radius(int(np.ceil(3*radius/m.settings.resolution)) if collect_blockers else 0)
        self.planner.set_surface_step_limit(m.settings.step_max)
        for first, last in pairs:
            began = time.monotonic()
            remaining=deadline-began
            if remaining <= 0:
                self.search_diagnostics['timed_out']=True
                return None
            initialize=self.planner.init_interval_accessor if self._intervals else self.planner.init_surface_accessor
            initialize(20., m.last_layer-m.first_layer+1,
                int(shape[0]), int(shape[1]), m.settings.resolution, 1., reader)
            if self._intervals:self.planner.set_passage_clearance(m.settings.minimum_clearance)
            remaining=deadline-time.monotonic()
            if remaining <= 0:
                self.search_diagnostics['timed_out']=True
                return None
            self.planner.set_time_limit(remaining)
            same = np.array_equal(a, b) and first == last
            # Native A* accepts [layer,column,row], and exports [layer,row,column].
            found = same or self.planner.search(
                np.asarray([first-m.first_layer, a[1], a[0]], dtype=np.int32),
                np.asarray([last-m.first_layer, b[1], b[0]], dtype=np.int32))
            nodes = 0 if same else self.planner.get_search_node_count()
            self.search_nodes += nodes
            self.search_diagnostics['attempts'].append(dict(start_layer=first, goal_layer=last,
                found=bool(found), search_nodes=nodes, timed_out=bool(self.planner.timed_out()),
                planning_ms=1000*(time.monotonic()-began)))
            if self.planner.timed_out():
                self.search_diagnostics['timed_out']=True
                return None
            if found:
                self.search_diagnostics.update(selected_start_layer=first, selected_goal_layer=last)
                break
            if collect_blockers:
                for k, x, y, parent_k, parent_x, parent_y in self.planner.get_blocked_edges().values():
                    key = (k+m.first_layer, x+int(m.lower[0]), y+int(m.lower[1]))
                    parent = m.observed_cell((parent_k+m.first_layer, parent_x+int(m.lower[0]), parent_y+int(m.lower[1])))
                    if parent is not None and np.isfinite(parent['G']):
                        blockers.setdefault(key, float(parent['G']))
        else:
            if collect_blockers:
                heading = np.asarray(target[:2])-start[:2]
                length = np.linalg.norm(heading)
                if length > 0.:
                    heading /= length
                side = np.array([-heading[1], heading[0]])
                radius = m.settings.robot_radius+m.settings.safety_margin
                self.search_diagnostics['unknown_blockers'] = m.merge_unknown_blockers(
                    (p for p in self._unknown_blockers(blockers,target[2])
                     if abs(side@(np.asarray(p[:2])-start[:2])) <= radius+m.settings.resolution), start)
            self._failures[cache_key]=({key:versions.get(key,0) for key in self._read_blocks},
                                       self.search_diagnostics.get('unknown_blockers', []),m.revision)
            self._failures.move_to_end(cache_key)
            while len(self._failures)>128:self._failures.popitem(last=False)
            return None
        if same:
            key = (last, *map(int, m.indices(target)))
            return np.asarray([m.position(key)+[m.observed_cell(key)['G']+.5]])
        cells = np.rint(self.planner.get_result_matrix()).astype(np.int64)
        points = []
        for k, x, y in cells:
            key = (int(k+m.first_layer), int(x+m.lower[0]), int(y+m.lower[1]))
            points.append(m.position(key)+[float(m.observed_cell(key)['G'])+.5])
        if not points:
            return None
        points = np.asarray(points)
        return points if dense else self.wrapper_type._simplify_route(points, self.cfg.planner.route_simplify_epsilon)
