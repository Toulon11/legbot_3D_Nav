#!/usr/bin/env python3
"""Compare two ROS exploration traces at equal time after GO2 readiness."""

import json
import sys


def load(path):
    with open(path, encoding='utf-8') as stream:
        rows = [json.loads(line) for line in stream]
    ready = next(row for row in rows if row['ready'] and row['sim_s'] is not None)
    grids = [row for row in rows if row['event'] == 'grid' and
             'reachable_ground_coverage' in row['truth'] and row['ready']]
    for row in grids:
        row['after_ready_wall_s'] = row['wall_s'] - ready['wall_s']
        row['after_ready_sim_s'] = row['sim_s'] - ready['sim_s']
    return ready, grids


def sample(grids, axis, time_s):
    items = [row for row in grids if row[axis] <= time_s]
    if not items:
        return None
    row = items[-1]
    return {'actual_s': round(row[axis], 3),
            'coverage': round(row['truth']['reachable_ground_coverage'], 6),
            'unobserved_cells': row['truth']['unobserved_reachable_cells'],
            'path_m': round(row['path_m'], 3),
            'state': row['status'].get('state')}


def results(path):
    ready, grids = load(path)
    last = grids[-1]
    output = {'path': path,
              'ready_wall_s': ready['wall_s'],
              'ready_sim_s': ready['sim_s'],
              'reachable_ground_cells': last['truth']['reachable_ground_cells'],
              'last': sample(grids, 'after_ready_wall_s', 1e9),
              'wall_budget': {}, 'sim_budget': {}, 'thresholds': {}}
    for budget in (60, 120, 180, 240, 300):
        output['wall_budget'][str(budget)] = sample(grids, 'after_ready_wall_s', budget)
        output['sim_budget'][str(budget)] = sample(grids, 'after_ready_sim_s', budget)
    for threshold in (0.25, 0.5, 0.7, 0.8, 0.9, 0.95):
        hit = next((r for r in grids if
                    r['truth']['reachable_ground_coverage'] >= threshold), None)
        output['thresholds'][str(threshold)] = (
            {'wall_s': round(hit['after_ready_wall_s'], 3),
             'sim_s': round(hit['after_ready_sim_s'], 3),
             'path_m': round(hit['path_m'], 3)} if hit else None)
    return output


if __name__ == '__main__':
    print(json.dumps([results(path) for path in sys.argv[1:]],
                     ensure_ascii=False, indent=2))
