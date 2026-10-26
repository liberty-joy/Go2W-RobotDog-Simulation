#!/usr/bin/python3
"""Summarize saved robustness reports without starting the simulator."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parents[1]
summary=[]
for name in ['flat','friction','push','slope_up','slope_down']:
    report=json.loads((root/'outputs/training'/f'wheel_verified_{name}'/'suite.json').read_text())
    cases=report['cases']
    summary.append({'scenario':name,'passed':sum(case['passed'] for case in cases),'total':len(cases),
        'velocity_error':sum(case['mean_velocity_error'] for case in cases)/len(cases),
        'yaw_error':sum(case['mean_yaw_error'] for case in cases)/len(cases),
        'min_height':min(case['min_height'] for case in cases),
        'early_resets':sum(case['early_resets'] for case in cases),
        'timeout_resets':sum(case['timeout_resets'] for case in cases),
        'push_events':report['push_events']})
out=root/'outputs/training'
(out/'robustness_summary.json').write_text(json.dumps(summary,indent=2))
fig,axes=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
for ax,key,title,limit in zip(axes,['velocity_error','yaw_error','min_height'],
        ['Mean planar error (m/s)','Mean yaw-rate error (rad/s)','Minimum base height above terrain (m)'],[.06,.15,.30]):
    bars=ax.bar([r['scenario'] for r in summary],[r[key] for r in summary],color='#318576')
    ax.bar_label(bars,labels=[f"{r['passed']}/{r['total']} passed" for r in summary],fontsize=8,padding=3)
    ax.axhline(limit,color='#b93f3f',ls='--',lw=1)
    ax.set_title(title,fontsize=10)
    ax.tick_params(axis='x',rotation=25)
    ax.grid(axis='y',alpha=.2)
fig.suptitle('Go2W wheel controller robustness | 8 environments, seed 42\n'
             '10 continuous commands per scenario; includes command-dependent stop gate')
fig.savefig(out/'go2w_robustness.png',dpi=160)
print(json.dumps(summary,indent=2))
