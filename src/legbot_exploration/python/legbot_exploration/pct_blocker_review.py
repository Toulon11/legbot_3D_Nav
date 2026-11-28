"""Resolve missing navigation evidence; raw returns alone do not settle gaps."""
import math
from .navigation_evidence import target_status


class BlockerReview:
    def __init__(self, mapping):
        self.map = mapping
        self.attempted = {}
        self.active = None

    def _target_status(self, point):
        return target_status(self.map,point)

    def available(self, point):
        xy = tuple(map(int, self.map.indices(point)))
        tolerance = self.map.settings.surface_noise_tolerance + 1e-7
        return not self._target_status(point)['resolved'] and not self.map.surface_observation.refuted(xy,point[2]) and not any(low-tolerance <= point[2] <= high+tolerance
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
        if task is not None and task['serial'] is None:
            states=[dict(point=list(p),**self._target_status(p)) for p in task['targets']]
            if states and all(s['resolved'] for s in states):
                task['resolved']=set(task['targets'])
                return dict(ready=True,fresh=False,gain=0,reason='blocker_evidence_already_settled',
                    target_count=len(states),resolved_count=len(states),observed_count=0,unresolved_count=0,
                    known_obstacle_count=sum(s['resolution_reason']=='known_obstacle' for s in states),
                    surface_evidence=states,targets=[list(p) for p in task['targets']],serial=None)
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
            # Known support/obstacles settle evidence without requiring a new
            # return at the old hypothesis height. Navigation safety is separate.
            if status['resolved']:
                resolved.add(point)
        task['resolved']=resolved
        count=len(resolved);received=len(task['received'])
        gained=sum(not task['baseline'].get(point,False) for point in resolved)
        obstacles=sum(s.get('resolution_reason')=='known_obstacle' for s in states)
        ready=bool(task['targets']) and count==len(task['targets'])
        matched=sum(check['state']=='matched' for check in states)
        refuted=sum(check['state']=='refuted' for check in states)
        return dict(ready=ready, fresh=bool(received or gained), gain=gained,known_obstacle_count=obstacles,
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
