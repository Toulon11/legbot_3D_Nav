"""Resolve missing navigation evidence; raw returns alone do not settle gaps."""
import math


class BlockerReview:
    def __init__(self, mapping):
        self.map = mapping
        self.attempted = {}
        self.active = None

    def _target_status(self, point):
        m=self.map;s=m.settings;xy=tuple(map(int,m.indices(point)))
        tolerance=s.surface_noise_tolerance+1e-7
        # An expected tread height may be revised by at most one real step.
        # Do not borrow an unrelated floor just because its C covers the cut.
        for g,_,_,lo,_ in m._column_known_intervals.get(xy,()):
            if abs(g-point[2])>min(.16,s.step_max)+1e-7:continue
            row=m.observed_cell((lo,*xy))
            if row is None or row['unknown'] or abs(float(row['G'])-g)>tolerance:continue
            return dict(resolved=True,resolution_reason='known_obstacle' if row['solid'] else
                        'support_interval_observed',actual_ground=g)
        cut=m.surface_observation.first_ground_slice(point[2])
        if (m.obstacle_at((cut,*xy)) is not None or
                m.frontier_index.observation_model.support_obstacle(xy,point[2])):
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

    def available(self, point):
        xy = tuple(map(int, self.map.indices(point)))
        tolerance = self.map.settings.surface_noise_tolerance + 1e-7
        return not self.map.surface_observation.refuted(xy,point[2]) and not any(low-tolerance <= point[2] <= high+tolerance
                       for low,high in self.attempted.get(xy, ()))

    def begin(self, targets):
        unique=[]
        tolerance=self.map.settings.surface_noise_tolerance+1e-7
        for row in targets:
            point=tuple(map(float,row[:3]))
            xy=tuple(map(int,self.map.indices(point)))
            if self.available(point) and not any(
                    tuple(map(int,self.map.indices(other)))==xy and abs(point[2]-other[2])<=tolerance
                    for other in unique):
                unique.append(point)
        self.active = dict(targets=unique, serial=None, received=set(), evidence={},baseline={}) if unique else None
        return dict(ready=bool(unique), reason='blocker_review_ready' if unique else 'blocker_already_reviewed')

    def arm(self):
        if self.active is None:
            return dict(ready=False, reason='blocker_review_not_started')
        if self.active['serial'] is None:
            self.active['serial'] = self.map.integrated_serial
            for point in self.active['targets']:
                xy = tuple(map(int, self.map.indices(point)))
                noise=self.map.settings.surface_noise_tolerance+1e-7
                kind='geometry' if any(
                    math.isfinite(row['C']) and row['passage_cost']>=50 and
                    abs(float(row['C'])-point[2])<=noise
                    for _,row in self.map.layers_at(*xy)) else 'ground'
                evidence=self.map.surface_observation
                search=[evidence.records[i] for i in evidence.by_column.get(xy,())
                        if evidence.records[i].kind=='search' and abs(evidence.records[i].height-point[2])<=noise]
                record=min(search,key=lambda r:abs(r.height-point[2])) if search else evidence.expect(
                    point,review=True,kind=kind,sources=evidence.local_sources(point,kind))
                if record.state=='pending':
                    evidence.arm_review(record,self.active['serial'])
                self.active['evidence'][point]=record
                self.active['baseline'][point]=self._target_status(point)['resolved']
        return dict(ready=True, reason='blocker_review_armed')

    def progress(self):
        task = self.active
        if task is None or task['serial'] is None:
            return dict(ready=False, fresh=False, gain=0, reason='blocker_review_not_armed')
        tolerance=self.map.settings.surface_noise_tolerance+1e-7
        states=[];resolved=set()
        for point in task['targets']:
            record=task['evidence'][point]
            check=self.map.surface_observation.query(point,after=task['serial'],kind=record.kind,expectation=record)
            status=self._target_status(point)
            if check['state']=='refuted':status=dict(resolved=True,resolution_reason='surface_refuted')
            states.append(dict(point=list(point),**check,**status))
            if check['ready'] or check['new_heights']:
                task['received'].add(point)
            # A free-only update can activate an already hit open ground.
            # Conversely, matching a raw height cannot settle a missing node.
            if status['resolved'] and (point in task['received'] or not task['baseline'][point]):
                resolved.add(point)
        task['resolved']=resolved
        count=len(resolved);received=len(task['received'])
        ready=bool(task['targets']) and count==len(task['targets'])
        matched=sum(check['state']=='matched' for check in states)
        refuted=sum(check['state']=='refuted' for check in states)
        return dict(ready=ready, fresh=bool(received or count), gain=count,
                    reason='blocker_navigation_evidence_resolved' if ready else
                           'blocker_returns_received_but_unresolved' if received else 'waiting_blocker_target_returns',
                    target_count=len(task['targets']), observed_count=received,resolved_count=count,
                    matched_count=matched,refuted_count=refuted,
                    unresolved_count=len(states)-count,surface_evidence=states,
                    targets=[list(point) for point in task['targets']], serial=task['serial'],
                    return_height_tolerance=tolerance)

    def finish(self):
        result = self.progress()
        if self.active is not None:
            tolerance=self.map.settings.surface_noise_tolerance+1e-7
            # Retire only resolved physical targets. Inconclusive scans keep
            # their gaps; existing C++ stance, move and region budgets bound
            # alternative views and prevent repeating an exhausted viewpoint.
            for point in self.active.get('resolved',()):
                xy=tuple(map(int,self.map.indices(point)))
                status=self._target_status(point)
                heights={point[2]}
                if 'actual_ground' in status:heights.add(status['actual_ground'])
                for height in heights:
                    if not any(low-tolerance<=height<=high+tolerance
                               for low,high in self.attempted.get(xy,())):
                        self.attempted.setdefault(xy,[]).append([height,height])
        self.active = None
        return result
