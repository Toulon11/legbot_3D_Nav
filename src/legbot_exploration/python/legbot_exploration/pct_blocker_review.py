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
        return not any(low-tolerance <= point[2] <= high+tolerance
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
        self.active = dict(targets=unique, serial=None, received=set(), bands={}) if unique else None
        return dict(ready=bool(unique), reason='blocker_review_ready' if unique else 'blocker_already_reviewed')

    def arm(self):
        if self.active is None:
            return dict(ready=False, reason='blocker_review_not_started')
        if self.active['serial'] is None:
            self.active['serial'] = self.map.integrated_serial
            for point in self.active['targets']:
                xy = tuple(map(int, self.map.indices(point)))
                band=[point[2],point[2]]
                self.attempted.setdefault(xy, []).append(band)
                self.active['bands'][point]=band
        return dict(ready=True, reason='blocker_review_armed')

    def progress(self):
        task = self.active
        if task is None or task['serial'] is None:
            return dict(ready=False, fresh=False, gain=0, reason='blocker_review_not_armed')
        tolerance = max(self.map.settings.surface_noise_tolerance,
                        min(.16, self.map.settings.step_max)) + 1e-7
        for point in task['targets']:
            column = self.map.columns.get(tuple(map(int, self.map.indices(point))))
            matches=[record.height for record in column.values() if record.last_seen > task['serial'] and
                     math.isfinite(record.height) and abs(record.height-point[2]) <= tolerance] if column else []
            if matches:
                task['received'].add(point)
                band=task['bands'][point]
                band[:]=[min(*band,*matches),max(*band,*matches)]
        count = len(task['received'])
        ready = count == len(task['targets'])
        return dict(ready=ready, fresh=bool(count), gain=count,
                    reason='blocker_target_observed' if ready else 'waiting_blocker_target_returns',
                    target_count=len(task['targets']), observed_count=count,
                    targets=[list(point) for point in task['targets']], serial=task['serial'],
                    return_height_tolerance=tolerance)

    def finish(self):
        result = self.progress()
        self.active = None
        return result
