#!/usr/bin/python3
"""Plot measured wheel-hub lift and body speed from a walking evaluation."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser()
p.add_argument('report_directory',type=Path)
a=p.parse_args()
report=json.loads((a.report_directory/'suite.json').read_text())
rows=[row for row in csv.DictReader((a.report_directory/'trace.csv').open())
      if row['case']=='forward' and 30<=int(row['step'])<120]
t=[int(row['step'])*report['control_dt'] for row in rows]
fig,axes=plt.subplots(2,1,figsize=(10,6),layout='constrained',sharex=True)
for leg,color,style in [('FL','#297aa6','-'),('RR','#70b7df','--'),('FR','#b47727','-'),('RL','#e1b56e','--')]:
    axes[0].plot(t,[float(row[leg+'_height']) for row in rows],label=leg,color=color,ls=style)
axes[0].axhline(.086,color='gray',ls=':',label='Approximate wheel radius')
axes[0].axhline(.106,color='red',ls=':',label='Lift test threshold')
axes[0].set(ylabel='Mean wheel-hub height (m)')
axes[0].legend(ncol=3,fontsize=8)
axes[1].plot(t,[float(row['mean_speed']) for row in rows],label='Mean body speed',color='#327d63')
command=next(case['command_vx'] for case in report['cases'] if case['case']=='forward')
axes[1].axhline(command,color='gray',ls='--',label='Command')
axes[1].set(xlabel='Time within forward command (s)',ylabel='Planar speed (m/s)')
axes[1].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.2)
fig.suptitle('Go2W walking evaluation | wheel joints in position hold\n'
             'Foot heights averaged across 8 robots; kinematic lift test, no force-contact measurement')
dest=a.report_directory/'gait.png'
fig.savefig(dest,dpi=160)
print(dest)
