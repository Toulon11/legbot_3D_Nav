"""Shared terminal/missing classification for navigation recovery targets."""
import math


def footprint_status(m, layer, xy, expected_ground):
    """The same physical support predicate used by parking and recovery."""
    row=m.footprint_support(layer,*xy,expected_ground)
    if (row is None or row['unknown'] or not row['measured'] or
            not m.surface_observation.is_ground_source(xy,int(row['g_surface_id'])) or
            not m.surface_observation.ground_observed(xy,float(row['G']),layer)):
        return 'missing',row
    if row['solid'] or abs(float(row['G'])-expected_ground)>m.settings.step_max+1e-5:
        return 'unsafe',row
    return 'supported',row


def target_status(m, point):
    s=m.settings;xy=tuple(map(int,m.indices(point)))
    tolerance=s.surface_noise_tolerance+1e-7
    # An expected tread height may be revised by at most one real step.
    # Do not borrow an unrelated floor just because its C covers the cut.
    for g,_,_,lo,_ in m._column_known_intervals.get(xy,()):
        if abs(g-point[2])>min(.16,s.step_max)+1e-7:continue
        state,row=footprint_status(m,lo,xy,point[2])
        if state=='missing' or abs(float(row['G'])-g)>tolerance:continue
        return dict(resolved=True,resolution_reason='known_obstacle' if state=='unsafe' else
                    'support_interval_observed',actual_ground=g)
    # A thin tread return at the next discrete cut is not a body-band wall.
    if m.frontier_index.observation_model.support_obstacle(xy,point[2]):
        return dict(resolved=True,resolution_reason='known_obstacle')
    pending=list(getattr(m,'_pending_ground',{}).get(xy,()))
    top=getattr(m,'_pending_open',{}).get(xy)
    if top is not None:pending.append((top[0],math.nan,top[1]))
    choices=[r for r in pending if abs(r[0]-point[2])<=min(.16,s.step_max)+tolerance]
    if choices:
        g,c,clear=min(choices,key=lambda r:abs(r[0]-point[2]))
        column=m.columns[xy]
        source=min(column.values(),key=lambda r:abs(r.high-g))
        direct=(source.support_hit_z is not None and abs(source.support_hit_z-g)<=tolerance)
        previous=getattr(m,'_confirmed_grounds',{}).get(xy,{}).get(source.id,math.nan)
        confirmed=math.isfinite(previous) and abs(previous-g)<=tolerance
        reason='missing_support_hit' if not (direct or confirmed) else 'missing_continuous_clearance'
        return dict(resolved=False,resolution_reason=reason,candidate_ground=g,
                    measured_ceiling=c if math.isfinite(c) else None,
                    clearance_lower_bound=max(0.,clear-g) if math.isfinite(clear) else 0.)
    return dict(resolved=False,resolution_reason='missing_ground_or_headroom')
