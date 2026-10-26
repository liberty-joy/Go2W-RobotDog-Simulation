"""Recompute rolling replay metrics from saved 30 Hz joint traces; no simulator."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True);metrics={}
for label,name in [('before','wheel_before.csv'),('after','wheel_after_stance.csv')]:
    data=np.genfromtxt(a.run/name,delimiter=',',names=True);steady=(data['time']>=3)&(data['time']<=7.5);stop=data['time']>10
    x=-.213*np.sin(data['q_FR_thigh_joint'])-.2264*np.sin(data['q_FR_thigh_joint']+data['q_FR_calf_joint'])
    metrics[label]={'steady_window_s':[3,7.5],'FR_hub_fore_aft_p2p_mm':float(np.ptp(x[steady])*1000),
        'FR_thigh_target_p2p_rad':float(np.ptp(data['target_FR_thigh_joint'][steady])),
        'FR_wheel_velocity_p2p_rad_s':float(np.ptp(data['vel_FR'][steady])),
        'FR_wheel_velocity_min_rad_s':float(np.min(data['vel_FR'][steady])),
        'base_forward_mean_m_s':float(np.mean(data['base_vx'][steady])),
        'base_height_min_m':float(np.min(data['base_z'])),
        'stop_max_abs_vx_m_s':float(np.max(np.abs(data['base_vx'][stop])))}
    axes[0].plot(data['time'],x*1000,label=label)
    axes[1].plot(data['time'],data['vel_FR'],label=label)
    axes[2].plot(data['time'],data['base_vx'],label=label)
axes[0].set_ylabel('FR hub x / mm');axes[1].set_ylabel('FR wheel / rad s-1');axes[2].set_ylabel('Base vx / m s-1');axes[2].set_xlabel('Simulation time / s')
for ax in axes:ax.grid(alpha=.25);ax.legend();ax.axvspan(3,7.5,color='grey',alpha=.08)
fig.suptitle('Held-W replay: nominal 0.15 m/s, forward 2-8 s, stop 8-11 s')
fig.tight_layout();fig.savefig(a.run/'wheel_fix.png',dpi=160)
metrics['scope']='Single flat-scene headless replay; kinematic hub x relative to hip from measured thigh/calf angles. Not wheel world displacement or manual GUI validation.'
metrics['reduction_fraction']=1-metrics['after']['FR_hub_fore_aft_p2p_mm']/metrics['before']['FR_hub_fore_aft_p2p_mm']
(a.run/'wheel_metrics.json').write_text(json.dumps(metrics,indent=2))
print(json.dumps(metrics,indent=2))
