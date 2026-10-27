"""Recorded transport checks and figures; never changes frozen inference parameters."""
import argparse,csv,json,re,hashlib,shutil
from pathlib import Path
import numpy as np,cv2
from stage6_offline import readbag,dump,rows
from stage6_bag import stamp

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run=a.run.resolve();out=run/'offline';record=json.loads((run/'record/metrics.json').read_text());pub=json.loads((run/'publisher/publisher_summary.json').read_text());stats={};imu=[];imagechecks=[];infochecks=[];depth=[];schema={}
 for t,m,receipt in readbag(run):
  if hasattr(m,'header'):stats.setdefault(t,[]).append(stamp(m))
  if t.endswith('/image_raw'):
   isdepth='/depth/' in t;imagechecks.append(m.width==640 and m.height==480 and m.step==(2560 if isdepth else 1920) and m.encoding==('32FC1' if isdepth else 'rgb8'))
   schema[t]={'type':'sensor_msgs/msg/Image','encoding':m.encoding,'frame_id':m.header.frame_id,'size':[m.width,m.height],'unit':'m optical Z' if isdepth else '8bit RGB','invalid':'NaN unknown' if isdepth else 'not applicable','qos':'reliable keep_last20 (publisher); reliable keep_last100 (recorder)'}
   if isdepth:
    z=np.frombuffer(bytes(m.data),'<f4');depth.append({'stamp_ns':stamp(m),'finite_fraction':float(np.isfinite(z).mean()),'nan_fraction':float(np.isnan(z).mean()),'zero_fraction':float((z==0).mean())})
  elif t.endswith('/camera_info'):
   infochecks.append(m.distortion_model=='plumb_bob' and len(m.d)==5 and not any(m.d) and m.width==640 and m.height==480 and abs(m.k[0]-381.361)<.01 and (abs(m.p[3]+m.k[0]*.1)<1e-5 if '/right/' in t else m.p[3]==0))
  elif t=='/go2w/imu/data':imu.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.orientation_covariance[0]])
 rates=[]
 for t,ts in stats.items():
  delta=np.diff(ts)*1e-9;positive=delta[delta>0];nominal=.1 if 'camera/' in t or '/stereo/' in t or '/lidar/' in t else None
  rates.append({'topic':t,'count':len(ts),'stamp_hz':float((len(ts)-1)/((ts[-1]-ts[0])*1e-9)) if len(ts)>1 else None,'wall_hz':record['topics'][t]['wall_hz'],'gap_p50_ms':float(np.median(positive)*1000) if len(positive) else None,'gap_p95_ms':float(np.percentile(positive,95)*1000) if len(positive) else None,'gaps_over_1p5_nominal':int((delta>nominal*1.5).sum()) if nominal else None,'backward_stamps':int((delta<0).sum()),'publisher_sent':pub['counts'].get(t),'dds_missing_vs_sent':pub['counts'].get(t,len(ts))-len(ts)})
 rows(out/'transport_rates.csv',rates);rows(out/'depth_validity.csv',depth);dump(out/'topic_contract.json',schema)
 mem=[];gpu=[]
 for line in (run/'resources.log').read_text().splitlines():
  if re.match(r'^\s*\d+\s+\d+\s+[\d.]+\s*$',line):mem.append(int(line.split()[1])/1024)
  elif re.match(r'^\d+, \d+$',line):gpu.append(int(line.split(',')[1]))
 serialized=sum(v['serialized_bytes'] for v in record['topics'].values());bag_bytes=sum(p.stat().st_size for p in (run/'record/bag').rglob('*') if p.is_file());ev=np.asarray(imu)
 received={t:record['topics'].get(t,{}).get('count',0)==n for t,n in pub['counts'].items()}
 validation=json.loads((out/'validation.json').read_text());fair=json.loads((out/'fairness_checks.json').read_text());replay=json.loads((run/'replay/verification.json').read_text())
 result={'run':str(run),'wall_recording_s':record['duration_wall_s'],'bag_bytes':bag_bytes,'serialized_bytes':serialized,'serialized_MiB_per_s':serialized/record['duration_wall_s']/1024**2,'bag_MiB_per_s':bag_bytes/record['duration_wall_s']/1024**2,'estimated_bag_GiB_per_hour':bag_bytes/record['duration_wall_s']*3600/1024**3,'simulator_peak_rss_MiB':max(mem),'simulator_peak_vram_MiB':max(gpu),'current_free_bytes':shutil.disk_usage(run).free,'all_publisher_counts_received':all(received.values()),'publisher_count_checks':received,'image_encoding_size_pass':all(imagechecks),'camera_info_pass':all(infochecks),'imu_messages':len(imu),'imu_orientation_marked_unavailable':bool((ev[:,6]==-1).all()),'imu_accel_median_m_s2':np.median(ev[:,:3],axis=0).tolist(),'imu_gyro_abs_p95_rad_s':np.percentile(abs(ev[:,3:6]),95,axis=0).tolist(),'replay':replay,'exact_image_info_bundles':validation['exact_image_info_bundles'],'image_bundles':validation['image_bundles'],'fairness':fair,'GUI':'not run','clock':'one recorded clock source on replay; no --clock generation','dynamic_fusion':'not evaluated; robot stationary, no ground truth motion compensation','depth_mean_finite_fraction':float(np.mean([d['finite_fraction'] for d in depth]))}
 dump(run/'acceptance.json',result)
 # Recomputable figures from recorded inputs; no synthetic images.
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 folders=sorted((out/'inputs').iterdir());fig,axs=plt.subplots(4,4,figsize=(14,10))
 for row,folder in enumerate(folders[::6]):
  for col,name in enumerate(['public','left','right']):axs[row,col].imshow(cv2.cvtColor(cv2.imread(str(folder/(name+'.png'))),cv2.COLOR_BGR2RGB));axs[row,col].axis('off');axs[row,col].set_title(folder.name+' '+name)
  z=np.load(folder/'depth.npy');im=axs[row,3].imshow(np.ma.masked_invalid(z),vmin=.5,vmax=6,cmap='turbo');axs[row,3].axis('off');axs[row,3].set_title('SGBM Z (m), invalid blank')
 fig.colorbar(im,ax=axs[:,3],fraction=.02);fig.subplots_adjust(wspace=.03,hspace=.18);fig.savefig(out/'recorded_sensor_views.png',dpi=140);plt.close(fig)
 with (out/'pairing.csv').open() as f:ps=list(csv.DictReader(f))
 fig,ax=plt.subplots(figsize=(8,4));ax.hist([float(x['lidar_minus_image_ms']) for x in ps],bins=30);ax.set(xlabel='LiDAR end - image exposure (ms)',ylabel='Bundles',title='Recorded software time pairing, not hardware synchronization');fig.tight_layout();fig.savefig(out/'sync_distribution.png',dpi=140);plt.close(fig)
 # Plot compute-only latency separately from capture/readout and offline waiting.
 with (out/'per_sample_metrics.csv').open() as f:rs=list(csv.DictReader(f))
 fig,axs=plt.subplots(1,2,figsize=(10,4))
 for i,mode in enumerate(['A','B']):
  vals=[float(r['total_compute_ms']) for r in rs if r['mode']==mode];axs[0].bar(i,np.percentile(vals,50),yerr=[[0],[np.percentile(vals,95)-np.percentile(vals,50)]],capsize=6,label=mode)
 axs[0].set(xticks=[0,1],xticklabels=['A','B'],ylabel='ms',title='CPU compute P50 / P95 (not end-to-end)')
 metrics=[json.loads((out/m/'metrics.json').read_text()) for m in ['A','B']];axs[1].bar(['A','B'],[m['peak_rss_mib'] for m in metrics]);axs[1].set(ylabel='MiB',title='Inference process peak RSS; CUDA unused');fig.tight_layout();fig.savefig(out/'latency_resources.png',dpi=150);plt.close(fig)
 # Freeze the exact executable sources without copying environments or weights.
 import subprocess
 snapshot=run/'source_snapshot';paths=[Path('scripts/go2w_teleop.py'),Path('scripts/go2w_ros2.py'),Path('scripts/run_go2w_ros2.sh'),Path('scripts/run_stage6_record.sh'),Path('scripts/ros2/env.sh')]
 paths+=sorted(Path('scripts/sensor_pipeline').glob('*.py'));paths+=sorted(Path('configs/sensors').glob('*.json'))
 manifest=[]
 for src in paths:
  target=snapshot/src;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,target);manifest.append({'path':str(src),'sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'bytes':src.stat().st_size})
 dump(run/'source_manifest.json',{'files':manifest,'git_parent':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'dirty_diff':'working_tree.patch','hash_scope':'exact sources used for final report; capture provenance recorded separately'})
 (run/'working_tree.patch').write_text(subprocess.check_output(['git','diff','--binary'],text=True))
 (run/'commands.txt').write_text('scripts/run_stage6_record.sh '+str(run)+'\nsource /opt/ros/jazzy/setup.bash\n/usr/bin/python3 scripts/sensor_pipeline/stage6_offline.py --run '+str(run)+'\n/usr/bin/python3 scripts/sensor_pipeline/stage6_report.py --run '+str(run)+'\n')
 print(json.dumps({k:result[k] for k in ['wall_recording_s','bag_bytes','serialized_MiB_per_s','simulator_peak_rss_MiB','simulator_peak_vram_MiB','all_publisher_counts_received','image_encoding_size_pass','camera_info_pass']},indent=2))
if __name__=='__main__':main()
