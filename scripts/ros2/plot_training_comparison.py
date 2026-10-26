#!/usr/bin/python3
"""Render the measured flat-ground evaluation results; no ROS required."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('baseline', type=Path)
p.add_argument('candidate', type=Path)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
reports = [json.loads(path.read_text()) for path in (a.baseline, a.candidate)]
names = [case['case'] for case in reports[0]['cases']]
assert names == [case['case'] for case in reports[1]['cases']]
fig, axes = plt.subplots(1, 2, figsize=(14, 7), layout='constrained')
y = np.arange(len(names))
for ax, key, title, limit in zip(axes,
        ['mean_velocity_error', 'mean_yaw_error'],
        ['Planar velocity error (m/s)', 'Yaw rate error (rad/s)'], [.06, .15]):
    for offset, report, label, color in zip([-.18, .18], reports,
            ['Initial: 300 iterations', 'Continued: 300 additional iterations'], ['#6183ba', '#df9038']):
        ax.barh(y + offset, [case[key] for case in report['cases']],
                height=.34, label=label, color=color)
    ax.axvline(limit, color='red', linestyle='--', label='Acceptance limit')
    ax.set(yticks=y, yticklabels=names, xlabel=title)
    ax.invert_yaxis()
    ax.grid(axis='x', alpha=.25)
    ax.legend(fontsize=8)
fig.suptitle('Go2W flat-ground policy comparison | 8 environments, seed 42\n'
             'Lower error is better; first 1 s of each command excluded')
a.output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(a.output, dpi=160)
print(a.output)
