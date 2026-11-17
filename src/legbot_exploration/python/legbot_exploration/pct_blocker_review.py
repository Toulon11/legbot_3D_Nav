"""One real observation attempt per XY and expected physical surface height."""
import math


class BlockerReview:
    def __init__(self, mapping):
        self.map = mapping
        self.attempted = {}
        self.active = None

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
        self.active = dict(targets=unique, serial=None, received=set(), evidence={}) if unique else None
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
                    (math.isfinite(row['C']) and row['passage_cost']>=50 and
                     abs(float(row['C'])-point[2])<=noise) or
                    (self.map.clearance_needs_observation(row) and
                     abs(float(row['pending_ceiling_z'])-point[2])<=noise)
                    for _,row in self.map.layers_at(*xy)) else 'ground'
                evidence=self.map.surface_observation
                search=[evidence.records[i] for i in evidence.by_column.get(xy,())
                        if evidence.records[i].kind=='search' and abs(evidence.records[i].height-point[2])<=noise]
                record=min(search,key=lambda r:abs(r.height-point[2])) if search else evidence.expect(
                    point,review=True,kind=kind,sources=evidence.local_sources(point,kind))
                if record.state=='pending':
                    evidence.arm_review(record,self.active['serial'])
                self.active['evidence'][point]=record
        return dict(ready=True, reason='blocker_review_armed')

    def progress(self):
        task = self.active
        if task is None or task['serial'] is None:
            return dict(ready=False, fresh=False, gain=0, reason='blocker_review_not_armed')
        tolerance = self.map.settings.surface_noise_tolerance + 1e-7
        states=[]
        for point in task['targets']:
            record=task['evidence'][point]
            check=self.map.surface_observation.query(point,after=task['serial'],kind=record.kind,expectation=record)
            states.append(dict(point=list(point),**check))
            # Completion authorizes a fresh-map replan, never a fabricated high
            # surface. A different real return is reported separately from a
            # matched/refuted hypothesis, which the shared query retains.
            if check['ready'] or check['new_heights']:
                task['received'].add(point)
                xy=tuple(map(int,self.map.indices(point)))
                # A turn without any target evidence must not consume the
                # physical observation. Pose/region scan budgets still cap retries.
                for height in (point[2],*check['new_heights']):
                    if not any(low-tolerance<=height<=high+tolerance for low,high in self.attempted.get(xy,())):
                        self.attempted.setdefault(xy,[]).append([height,height])
        count = len(task['received'])
        ready = count == len(task['targets'])
        matched=sum(check['state']=='matched' for check in states)
        refuted=sum(check['state']=='refuted' for check in states)
        return dict(ready=ready, fresh=bool(count), gain=count,
                    reason=('blocker_target_observed' if matched+refuted==len(states) else
                            'blocker_actual_surface_observed') if ready else 'waiting_blocker_target_returns',
                    target_count=len(task['targets']), observed_count=count,
                    matched_count=matched,refuted_count=refuted,
                    unresolved_count=len(states)-matched-refuted,surface_evidence=states,
                    targets=[list(point) for point in task['targets']], serial=task['serial'],
                    return_height_tolerance=tolerance)

    def finish(self):
        result = self.progress()
        self.active = None
        return result
