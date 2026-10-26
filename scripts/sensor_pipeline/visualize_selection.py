"""Visualize saved selection smoke samples; no matching or fusion."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--sample',type=Path,required=True);a=p.parse_args()
s=json.loads((a.sample/'smoke.json').read_text());xyz=np.load(a.sample/'lidar_xyz.npy');times=np.load(a.sample/'timestamp.npy')
fig,axes=plt.subplots(2,2,figsize=(10,7))
for ax,name in zip(axes[0],['left','right']):ax.imshow(Image.open(a.sample/(name+'.png')));ax.set_title(name+' RGB (not matched depth)');ax.axis('off')
c=axes[1,0].scatter(xyz[:,0],xyz[:,1],c=xyz[:,2],s=.4,cmap='viridis');axes[1,0].set_aspect('equal');axes[1,0].set_title('RTX returns, sensor XY');axes[1,0].set_xlabel('x / m');axes[1,0].set_ylabel('y / m');fig.colorbar(c,ax=axes[1,0],label='z / m')
axes[1,1].hist((times-times.min())/1e6,bins=32);axes[1,1].set_title('Returned point time span');axes[1,1].set_xlabel('Offset from first returned point / ms');axes[1,1].set_ylabel('Returns')
fig.suptitle('Stage 1 parameterized sensors: measurement smoke only');fig.tight_layout();fig.savefig(a.sample/'sensor_preview.png',dpi=160)
report={'finite_xyz':bool(np.isfinite(xyz).all()),'point_count':len(xyz),'timestamp_span_ns':int(times.max()-times.min()),'timestamp_unit':'nanoseconds per annotator API; cross-sensor timing not calibrated','left_render_time':s['left_rendering_time'],'right_render_time':s['right_rendering_time'],'imu_time':s['imu']['time'],'sync_status':'NOT_ACCEPTED: one sample only, latest-image delay exists','GUI_manual':'NOT_TESTED'}
(a.sample/'sample_checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
