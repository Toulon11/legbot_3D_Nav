#!/usr/bin/env python3
"""Plot independent SDF-ground coverage against time after GO2 readiness."""

import json
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def series(path):
    with open(path) as stream:
        rows = [json.loads(line) for line in stream]
    first = next(r for r in rows if r['ready'] and r['sim_s'] is not None)
    grids = [r for r in rows if r['event'] == 'grid' and r['ready'] and
             'reachable_ground_coverage' in r['truth']]
    return ([r['wall_s'] - first['wall_s'] for r in grids],
            [100 * r['truth']['reachable_ground_coverage'] for r in grids])


def main():
    current, baseline, output = sys.argv[1:]
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False})
    fig, ax = plt.subplots(figsize=(7.4, 4.4))
    for path, label, color in ((current, 'Full method', '#1565c0'),
                               (baseline, 'Local-frontier ablation', '#ef6c00')):
        x, y = series(path)
        ax.plot(x, y, label=label, color=color, linewidth=2)
    ax.set(xlabel='Wall time after GO2 ready (s)',
           ylabel='Observed reachable ground (%)',
           title='GO2 flat-scene exploration pilot')
    ax.grid(alpha=.25)
    ax.legend(loc='lower right')
    ax.set_ylim(0, 100)
    fig.tight_layout()
    fig.savefig(output, dpi=180)


if __name__ == '__main__':
    main()
