"""Dependency-directed local geometry and PCT costs."""
import math
import numpy as np
from scipy import ndimage
from .fixed_layer_map import cost_changed, different

CARDINAL = ((1, 0), (-1, 0), (0, 1), (0, -1))


def expand_layers(m, dirty):
    """Add only local slice blocks required by neighboring surface computation.

    A column with all hits below a cut still has a known G at that cut. Keeping
    only its own min/max slice span would create artificial cracks on slopes.
    Persist local demands across batches: a column observed later must also
    inherit cuts requested earlier by its neighbours. Only cuts from real
    local height ranges seed a halo; inherited cuts never seed another halo.
    """
    s = m.settings
    b = s.block_size
    halo = s.traversability_kernel_size//2+1+max(s.inflation_kernel_size//2,
                                               math.ceil((s.robot_radius+s.safety_margin)/s.resolution+.5))
    seeds = {}
    work = set(dirty)
    for x, y in dirty:
        column = m.columns.get((x, y), {})
        if not column:
            continue
        low, high = min(e[0] for e in column.values()), max(e[1] for e in column.values())
        first = math.ceil((low-s.origin_z)/s.slice_dh-1e-7)-1
        last = math.floor((high-s.origin_z)/s.slice_dh+1e-7)+1
        if (last-first+1)*b*b > s.max_cells:
            raise ValueError('pct_map_cell_limit')
        retained = m.column_layers.setdefault((x, y), set())
        for k in range(first, last+1):
            native_key = (k, x//b, y//b)
            native = m._native_slice_masks.get(native_key)
            if native is None:
                native = m._native_slice_masks[native_key] = np.zeros((b, b), dtype=bool)
            if native[x % b, y % b]:
                continue
            native[x % b, y % b] = True
            mask = seeds.setdefault((k, x//b, y//b), np.zeros((b, b), dtype=bool))
            mask[x % b, y % b] = True
        retained.update(range(first, last+1))
        m.first_layer = first if m.first_layer is None else min(first, m.first_layer)
        m.last_layer = last if m.last_layer is None else max(last, m.last_layer)
        m.lower = np.array([x, y]) if m.lower is None else np.minimum(m.lower, [x, y])
        m.upper = np.array([x+1, y+1]) if m.upper is None else np.maximum(m.upper, [x+1, y+1])
    span = math.ceil(halo/b)
    targets = {(k, bx+dx, by+dy) for k, bx, by in seeds
               for dx in range(-span, span+1) for dy in range(-span, span+1)}
    for k, bx, by in targets:
        origin = np.array([bx*b-halo, by*b-halo])
        end = origin+b+2*halo
        mask = np.zeros((b+2*halo, b+2*halo), dtype=bool)
        for sx in range(bx-span, bx+span+1):
            for sy in range(by-span, by+span+1):
                source = seeds.get((k, sx, sy))
                if source is None:
                    continue
                start = np.array([sx*b, sy*b])
                lo, hi = np.maximum(start, origin), np.minimum(start+b, end)
                if (lo >= hi).any():
                    continue
                mask[lo[0]-origin[0]:hi[0]-origin[0], lo[1]-origin[1]:hi[1]-origin[1]] |= \
                    source[lo[0]-start[0]:hi[0]-start[0], lo[1]-start[1]:hi[1]-start[1]]
        grown = ndimage.maximum_filter(mask, size=2*halo+1, mode='constant')[halo:halo+b, halo:halo+b]
        if not grown.any():
            continue
        requirements = m._required_slice_masks.setdefault((bx, by), {})
        previous = requirements.get(k)
        added = grown if previous is None else grown & ~previous
        if previous is None:
            requirements[k] = grown.copy()
        else:
            previous |= grown
        for i, j in np.argwhere(added):
            key = (int(bx*b+i), int(by*b+j))
            if key in m.columns and k not in m.column_layers.setdefault(key, set()):
                m.column_layers[key].add(k)
                work.add(key)
    # The existing halo may have been created before a dirty XY column was
    # observed. Read only that column's local demand tile, never the full map
    # or the global layer range. G/C are still taken from its own evidence.
    for x, y in dirty:
        if (x, y) not in m.columns:
            continue
        retained = m.column_layers.setdefault((x, y), set())
        for k, required in m._required_slice_masks.get((x//b, y//b), {}).items():
            if required[x % b, y % b]:
                retained.add(k)
    return work


def neighborhood(keys, radius):
    return {(k, x+dx, y+dy) for k, x, y in keys
            for dx in range(-radius, radius+1) for dy in range(-radius, radius+1)}


def gather(m, block_key, halo, field, default):
    """Small temporary block halo, never a map-sized array."""
    k, bx, by = block_key
    b = m.settings.block_size
    origin = np.array([bx*b-halo, by*b-halo])
    end = origin+b+2*halo
    result = np.full((b+2*halo, b+2*halo), default,
                     dtype=bool if isinstance(default, bool) else float)
    span = math.ceil(halo/b)
    for sx in range(bx-span, bx+span+1):
        for sy in range(by-span, by+span+1):
            source = m.blocks.get((k, sx, sy))
            if source is None:
                continue
            start = np.array([sx*b, sy*b])
            lo, hi = np.maximum(start, origin), np.minimum(start+b, end)
            if (lo >= hi).any():
                continue
            result[lo[0]-origin[0]:hi[0]-origin[0], lo[1]-origin[1]:hi[1]-origin[1]] = \
                source[field][lo[0]-start[0]:hi[0]-start[0], lo[1]-start[1]:hi[1]-start[1]]
    return result


def target_masks(m, keys, radius, include_unknown=False):
    b = m.settings.block_size
    seeds = {}
    for k, x, y in keys:
        seeds.setdefault((k, x//b, y//b), np.zeros((b, b), dtype=bool))[x % b, y % b] = True
    span = math.ceil(radius/b)
    targets = {(k, bx+dx, by+dy) for k, bx, by in seeds
               for dx in range(-span, span+1) for dy in range(-span, span+1)}
    for key in targets:
        block = m.blocks.get(key)
        if block is None:
            continue
        k, bx, by = key
        mask = np.zeros((b+2*radius, b+2*radius), dtype=bool)
        origin = np.array([bx*b-radius, by*b-radius])
        end = origin+b+2*radius
        for sx in range(bx-span, bx+span+1):
            for sy in range(by-span, by+span+1):
                source = seeds.get((k, sx, sy))
                if source is None:
                    continue
                start = np.array([sx*b, sy*b])
                lo, hi = np.maximum(start, origin), np.minimum(start+b, end)
                if (lo >= hi).any():
                    continue
                mask[lo[0]-origin[0]:hi[0]-origin[0], lo[1]-origin[1]:hi[1]-origin[1]] |= \
                    source[lo[0]-start[0]:hi[0]-start[0], lo[1]-start[1]:hi[1]-start[1]]
        grown = ndimage.maximum_filter(mask, size=2*radius+1, mode='constant')[radius:radius+b, radius:radius+b]
        if not include_unknown:
            grown &= block['measured']
        if grown.any():
            yield key, block, grown


def changes_mask(old, new, threshold):
    category = lambda v: np.where(v < 0, -1, np.where(v >= 50, 2, np.where(v > 20, 1, 0)))
    return (old != new) & ((category(old) != category(new)) | (abs(old-new) > threshold))


def inflation_boundary_changed(m, key, old, new):
    if old == new or old < 0 or new < 0:
        return False
    s = m.settings;half = s.inflation_kernel_size//2
    for dx in range(-half, half+1):
        for dy in range(-half, half+1):
            row = m.cell((key[0], key[1]+dx, key[2]+dy))
            if row is None or row['inflated_terrain_cost'] < 0:
                continue
            weight = float(np.clip(1.-(math.hypot(dx,dy)*s.resolution-s.robot_radius)/(s.safety_margin+s.resolution),0.,1.))
            before, after = old*weight, new*weight
            current = row['inflated_terrain_cost']
            for boundary in (20.,50.):
                if current <= boundary < after and before <= boundary:
                    return True
                if current > boundary >= after and before >= current-1e-8:
                    return True
    return False


def update_costs(m, base, changed_g):
    s = m.settings
    feature_keys = set(changed_g)
    feature_keys.update((k, x+dx, y+dy) for k, x, y in changed_g for dx, dy in CARDINAL)
    relief_keys = set(base)
    changed_flat = set()
    changed = set()
    stand_sq = (1.2*s.resolution*s.slope_max)**2
    for key in feature_keys:
        row = m.cell(key)
        if row is None:
            continue
        old = (bool(row['feature_valid']), bool(row['flat']), float(row['q']), float(row['m']))
        values = []
        if np.isfinite(row['G']):
            for axis in (0, 1):
                differences = []
                for sign in (-1, 1):
                    neighbor = (key[0], key[1]+(sign if axis == 0 else 0), key[2]+(sign if axis == 1 else 0))
                    other = m.cell(neighbor)
                    if other is not None and np.isfinite(other['G']):
                        differences.append((other['G']-row['G'])*sign)
                values.append(max(differences, key=abs) if differences else np.nan)
        else:
            values = [np.nan, np.nan]
        row['feature_valid'] = np.isfinite(values).all()
        row['gx'], row['gy'] = np.asarray(values)/s.resolution
        row['q'] = sum(v*v for v in values)
        row['m'] = max((v*v for v in values), default=np.nan)
        row['flat'] = row['feature_valid'] and row['q'] <= stand_sq
        new = (bool(row['feature_valid']), bool(row['flat']), float(row['q']), float(row['m']))
        if old[:2] != new[:2] or any(different(a, b) for a, b in zip(old[2:], new[2:])):
            relief_keys.add(key)
            changed.add(key)
        if old[1] != new[1] or old[0] != new[0]:
            changed_flat.add(key)
    # Standable-neighbor counts only change when a flat/valid flag changes.
    relief_keys.update(neighborhood(changed_flat, s.traversability_kernel_size//2))
    terrain_changed, state_changed = set(), set()
    relief_work = 0
    for key in relief_keys:
        row = m.cell(key)
        if row is None or not row['measured']:
            continue
        relief_work += 1
        old_state = (bool(row['solid']), bool(row['unknown']), row['passage_cost']<0)
        passage = -1.
        has_ground = np.isfinite(row['G'])
        if has_ground:
            ceiling = row['C']
            if np.isfinite(ceiling):
                clearance = ceiling-row['G']
                passage = 50. if clearance < s.minimum_clearance else max(0., 20.*(s.desired_clearance-clearance))
            else:
                passage = 0.
            if passage<50 and row['pending_ceiling_z']-row['G']<s.minimum_clearance:
                passage=-1.
        relief = -1.
        if row['feature_valid']:
            if row['flat']:
                relief = 15.*row['q']/stand_sq
            elif row['m'] <= s.step_max**2 and s.step_max > 0:
                count = 0
                half = s.traversability_kernel_size//2
                for dx in range(-half, half+1):
                    for dy in range(-half, half+1):
                        other = m.cell((key[0], key[1]+dx, key[2]+dy))
                        count += other is not None and other['flat'] and other['q'] < stand_sq
                required = int(s.standable_ratio*s.traversability_kernel_size**2)-1
                relief = 20.*row['m']/s.step_max**2 if count >= required else 50.
            else:
                relief = 50.
        elif passage >= 50:
            relief = 0.  # A known blocked interval needs no invented slope.
        if cost_changed(row['relief_cost'], relief, s.relief_update_threshold):
            row['relief_cost'] = relief
            changed.add(key)
        if row['passage_cost'] != passage:
            row['passage_cost'] = passage
            changed.add(key)
        terrain = passage+row['relief_cost'] if passage >= 0 and row['relief_cost'] >= 0 else -1.
        if row['terrain_cost'] != terrain:
            row['terrain_cost'] = terrain
            changed.add(key)
        # Preserve T=passage+relief exactly. The deadband governs propagation,
        # compared with the last value propagated to inflation, not last frame.
        if cost_changed(row['terrain_inflation_reference'], terrain, s.terrain_update_threshold) or \
                inflation_boundary_changed(m,key,float(row['terrain_inflation_reference']),terrain):
            row['terrain_inflation_reference'] = terrain
            terrain_changed.add(key)
        row['solid'] = row['terrain_cost'] >= 50
        row['unknown'] = not has_ground
        if old_state != (bool(row['solid']), bool(row['unknown']), passage<0):
            state_changed.add(key)
    half = s.inflation_kernel_size//2
    weights = [(dx, dy, float(np.clip(1.-(math.hypot(dx, dy)*s.resolution-s.robot_radius)/(s.safety_margin+s.resolution), 0., 1.)))
               for dx in range(-half, half+1) for dy in range(-half, half+1)]
    weights = [(dx, dy, w) for dx, dy, w in weights if w > 0]
    inflation_work = 0
    b = s.block_size
    for block_key, block, mask in target_masks(m, terrain_changed, half):
        inflation_work += int(mask.sum())
        source = gather(m, block_key, half, 'terrain_cost', -1.)
        value = np.full((b, b), -1.)
        for dx, dy, weight in weights:
            part = source[half+dx:half+dx+b, half+dy:half+dy+b]
            np.maximum(value, np.where(part >= 0, part*weight, -1.), out=value)
        value[block['terrain_cost'] < 0] = -1.
        update = mask & changes_mask(block['inflated_terrain_cost'], value, s.inflation_update_threshold)
        block['inflated_terrain_cost'][update] = value[update]
        k, bx, by = block_key
        changed.update((k, bx*b+int(x), by*b+int(y)) for x, y in np.argwhere(update))
    radius = s.robot_radius+s.safety_margin
    disk_half = math.ceil(radius/s.resolution+.5)
    disk = [(dx, dy) for dx in range(-disk_half, disk_half+1) for dy in range(-disk_half, disk_half+1)
            if math.hypot(max(abs(dx)*s.resolution-s.resolution*.5, 0),
                          max(abs(dy)*s.resolution-s.resolution*.5, 0)) <= radius+1e-8]
    structure = np.zeros((2*disk_half+1, 2*disk_half+1), dtype=bool)
    for dx, dy in disk:
        structure[dx+disk_half, dy+disk_half] = True
    hard_work = 0
    for block_key, block, mask in target_masks(m, state_changed, disk_half):
        hard_work += int(mask.sum())
        unseen=gather(m,block_key,disk_half,'unknown',True) | (
            gather(m,block_key,disk_half,'passage_cost',-1.)<0)
        unknown=ndimage.binary_dilation(unseen,structure=structure)
        solid = ndimage.binary_dilation(gather(m, block_key, disk_half, 'solid', False), structure=structure)
        unknown = unknown[disk_half:disk_half+b, disk_half:disk_half+b]
        solid = solid[disk_half:disk_half+b, disk_half:disk_half+b]
        update = mask & ((unknown != block['unknown_hard']) | (solid != block['solid_hard']))
        block['unknown_hard'][update], block['solid_hard'][update] = unknown[update], solid[update]
        block['hard'][update] = (unknown | solid)[update]
        k, bx, by = block_key
        changed.update((k, bx*b+int(x), by*b+int(y)) for x, y in np.argwhere(update))
    m.frontier_state_changed=state_changed
    m.last_work = dict(base_cells=len(base), feature_cells=len(feature_keys), terrain_cells=relief_work,
                       inflation_cells=inflation_work, hard_cells=hard_work)
    # Counts are cached per touched block; repeated frames do not scan all tiles.
    if not hasattr(m, '_block_counts'):
        m._block_counts, m.counts = {}, {}
    b = s.block_size
    touched = {(k, x//b, y//b) for k, x, y in base | changed}
    for block_key in touched:
        block = m.blocks[block_key]
        surface = np.isfinite(block['G'])
        safe = surface & ~block['unknown'] & ~block['solid'] & ~block['hard'] & (block['inflated_terrain_cost'] >= 0) & (block['inflated_terrain_cost'] <= 20)
        current = dict(measured_surface_cells=int(surface.sum()),
            solid_cells=int(block['solid'].sum()), unknown_inflated_cells=int((surface & block['hard']).sum()),
            safe_cells=int(safe.sum()), unassessed_gradient_cells=int((surface & ~block['feature_valid']).sum()),
            low_clearance_cells=int((surface & (block['C']-block['G'] < s.minimum_clearance)).sum()))
        previous = m._block_counts.get(block_key, {})
        for name, count in current.items():
            m.counts[name] = m.counts.get(name, 0)+count-previous.get(name, 0)
        m._block_counts[block_key] = current
    return changed
