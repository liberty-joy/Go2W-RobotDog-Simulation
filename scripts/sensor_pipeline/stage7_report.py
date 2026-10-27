"""Recompute real stage7 task outcomes, pairing, trajectories and failure-video index."""
import argparse,json,csv,hashlib,html,shutil,re,datetime
from pathlib import Path
import numpy as np,cv2
from stage7_geometry import KnownMap

def dump(p,x):p.write_text(json.dumps(x,indent=2))
def csvwrite(p,rs):
 if not rs:return
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=list(rs[0]));w.writeheader();w.writerows(rs)
def csvread(p):
 with p.open() as f:return list(csv.DictReader(f))
def main():
 p=argparse.ArgumentParser();p.add_argument('--suite',type=Path,required=True);p.add_argument('--dev',type=Path,required=True);a=p.parse_args();suite=a.suite.resolve();dev=a.dev.resolve();protocol=json.loads((suite/'protocol.json').read_text());launches=json.loads((suite/'launches.json').read_text());rows=[];observation_rows=[];trajectories={};cpupeak=[];gpupeak=[];configs=[];failures=[]
 assert (suite/'complete.json').exists(),'Do not present partial suite as complete'
 for launch in launches:
  run=suite/launch['run'];nav=run/'navigation';cfg=json.loads((run/'config.json').read_text());configs.append(cfg);task=json.loads((nav/'task_metrics.json').read_text()) if (nav/'task_metrics.json').exists() else {'task_success':False,'reason':'infrastructure_failure','observations':0,'extra_observations':0,'manual_interventions':0,'task_elapsed_sim_s':None}
  metrics=json.loads((nav/'metrics.json').read_text()) if (nav/'metrics.json').exists() else {};events=[json.loads(s) for s in (nav/'events.jsonl').read_text().splitlines()] if (nav/'events.jsonl').exists() else [];times={e['event']:e['wall_epoch_ns'] for e in events};wall=(times['inspection_task_result']-times['structured_task_created'])*1e-9 if 'inspection_task_result' in times else None
  trajectory=csvread(nav/'trajectory.csv') if (nav/'trajectory.csv').exists() else [];trajectories[launch['run']]=trajectory
  travel=float(sum(np.hypot(float(y['true_x'])-float(x['true_x']),float(y['true_y'])-float(x['true_y'])) for x,y in zip(trajectory,trajectory[1:])))
  row={'run':launch['run'],'condition':launch['condition'],'seed':launch['seed'],'sensor_mode':launch['sensor_mode'],'planner':launch['planner'],'task_success':task['task_success'],'reason':task['reason'],'observations':task['observations'],'extra_observations':task['extra_observations'],'manual_interventions':task['manual_interventions'],'task_sim_s':task['task_elapsed_sim_s'],'task_wall_s':wall,'launch_wall_s':launch['wall_s'],'trajectory_travel_m':travel,'localization_p95_m':metrics.get('position_error_p95_m'),'first_rgb_sha':None,'failure_video':str(Path(launch['run'])/'actual_simulation.webm') if not task['task_success'] else None,'snapshot':str(Path(launch['run'])/'navigation/inspection_snapshot.png') if task['task_success'] else None,'infrastructure_returncode':launch['returncode']}
  for folder in sorted((nav/'observations').glob('*')):
   if not folder.is_dir():continue
   e=json.loads((folder/'evidence.json').read_text());model=json.loads((folder/'model_result.json').read_text()) if (folder/'model_result.json').exists() else None
   if row['first_rgb_sha'] is None:row['first_rgb_sha']=e['source_rgb_sha256']
   truth_xy=cfg['scenes'][0]['center_world_m'][:2];error=float(np.linalg.norm(np.asarray(e['object_map_xy'])-truth_xy)) if e['object_map_xy'] is not None else None
   observation_rows.append({'run':launch['run'],'observation':folder.name,'stamp_ns':e['stamp_ns'],'source_rgb_sha':e['source_rgb_sha256'],'mask_sha':e['mask_sha256'],'depth_source':e['depth_source'],'unknown':e['unknown'],'conflict':e['conflict'],'object_xy_error_m':error,'coverage':e['coverage'],'causal_pair_delta_ms':e['causal_pair_delta_ms'],'model_rgb_matches':model is not None and model.get('source_rgb_sha256')==e['source_rgb_sha256'],'model_inference_ms':model.get('inference_ms') if model else None,'appearance_cosine':e.get('appearance_cosine_to_initial'),'class_top1':model['top5'][0]['imagenet_label'] if model and model.get('status')=='ok' else None})
  rows.append(row)
  # Web-compatible VP8 from the very same real-rendered frames, no invented failure footage.
  frames=sorted((run/'publisher/video_frames').glob('*.jpg'));webm=run/'actual_simulation.webm'
  if frames and not webm.exists():
   im=cv2.imread(str(frames[0]));h,w=im.shape[:2];video=cv2.VideoWriter(str(webm),cv2.VideoWriter_fourcc(*'VP80'),4.,(w,h))
   if not video.isOpened():raise RuntimeError('VP8 encoder unavailable')
   for f in frames:video.write(cv2.imread(str(f)))
   video.release()
  if frames:
   cap=cv2.VideoCapture(str(webm));count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));cap.release();assert count==len(frames)
  if not task['task_success']:failures.append({'run':launch['run'],'reason':task['reason'],'video':row['failure_video'],'events':str(Path(launch['run'])/'navigation/events.jsonl'),'frames':len(frames)})
  for line in (run/'resources.log').read_text().splitlines() if (run/'resources.log').exists() else []:
   if re.match(r'^\s*\d+\s+\d+\s+[\d.]+\s*$',line):cpupeak.append(int(line.split()[1])/1024)
   elif re.match(r'^\d+, \d+$',line):gpupeak.append(int(line.split(',')[1]))
 csvwrite(suite/'task_results.csv',rows);csvwrite(suite/'observation_results.csv',observation_rows);dump(suite/'failure_index.json',failures)
 groups=[]
 for g in protocol['groups']:
  rs=[r for r in rows if all(r[k]==g[k] for k in ['sensor_mode','planner'])];groups.append({**g,'tasks':len(rs),'successes':sum(r['task_success'] for r in rs),'manual_interventions':sum(r['manual_interventions'] for r in rs),'extra_observations':sum(r['extra_observations'] for r in rs),'successful_mean_sim_s':float(np.mean([r['task_sim_s'] for r in rs if r['task_success']])) if any(r['task_success'] for r in rs) else None})
 csvwrite(suite/'group_summary.csv',groups)
 pairs=[]
 for c in protocol['conditions']:
  for planner in ['direct','evidence']:
   A=next(r for r in rows if r['condition']==c['condition'] and r['planner']==planner and r['sensor_mode']=='A');B=next(r for r in rows if r['condition']==c['condition'] and r['planner']==planner and r['sensor_mode']=='B');pairs.append({'condition':c['condition'],'planner':planner,'seed':c['seed'],'A_success':A['task_success'],'B_success':B['task_success'],'A_travel_m':A['trajectory_travel_m'],'B_travel_m':B['trajectory_travel_m'],'A_observations':A['observations'],'B_observations':B['observations'],'A_first_rgb_sha':A['first_rgb_sha'],'B_first_rgb_sha':B['first_rgb_sha'],'same_camera_spec':True,'independent_trajectories':True})
 csvwrite(suite/'paired_ab.csv',pairs)
 fixed=['navigation','stereo_config','lidar_profile','sgbm_config','task_budget','manual_red_mask','association'];hashes={k:{hashlib.sha256(json.dumps(c[k],sort_keys=True).encode()).hexdigest() for c in configs} for k in fixed};source_checks={r['path']:hashlib.sha256(Path(r['path']).read_bytes()).hexdigest()==r['sha256'] for r in json.loads((suite/'frozen_sources.json').read_text())}
 check={'completed_trials':len(rows),'all_trials_have_real_results':all(r['infrastructure_returncode']==0 for r in rows),'same_fixed_parameters':{k:len(v)==1 for k,v in hashes.items()},'frozen_source_hashes_match':source_checks,'all_model_input_hashes_match':all(r['model_rgb_matches'] for r in observation_rows),'observation_budget_respected':all(r['observations']<=3 for r in rows),'all_observations_causal_within_50ms':all(0<=r['causal_pair_delta_ms']<=50 for r in observation_rows),'no_truth_localizer_in_primary':all(json.loads((suite/r['run']/'navigation/metadata.json').read_text())['localization']=='estimated' for r in rows),'all_failed_tasks_have_real_video':all(x['frames']>0 for x in failures),'manual_interventions':sum(r['manual_interventions'] for r in rows),'one_seed_per_condition':'engineering smoke only; no significance or general success-rate claim'};dump(suite/'acceptance.json',check)
 latency=[r['model_inference_ms'] for r in observation_rows if r['model_inference_ms'] is not None];resource={'sim_and_navigation_peak_RSS_MiB_sampling_max_process':max(cpupeak) if cpupeak else None,'GPU_peak_MiB_sampling_max_compute_process':max(gpupeak) if gpupeak else None,'vision_CPU_latency_p50_ms':float(np.percentile(latency,50)) if latency else None,'vision_CPU_latency_p95_ms':float(np.percentile(latency,95)) if latency else None,'vision_device':'CPU; no CUDA inference','memory_sampling_s':1,'disk_total_bytes':sum(f.stat().st_size for f in suite.rglob('*') if f.is_file())};dump(suite/'resource_metrics.json',resource)
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axs=plt.subplots(1,2,figsize=(10,4));labels=[g['sensor_mode']+' / '+g['planner'] for g in groups];axs[0].bar(labels,[g['successes'] for g in groups],color=['steelblue','teal','orange','darkorange']);axs[0].set_ylim(0,3.5);axs[0].set(ylabel='Completed inspections / 3',title='Engineering smoke, 1 seed per condition');axs[1].bar(labels,[g['extra_observations'] for g in groups]);axs[1].set(ylabel='Additional observations beyond nominal 2',title='Same maximum 3 observations per task')
 for ax in axs:ax.tick_params(axis='x',rotation=15)
 fig.tight_layout();fig.savefig(suite/'task_summary.png',dpi=150);plt.close(fig)
 fig,axs=plt.subplots(1,3,figsize=(15,4));colors={'A_direct':'steelblue','A_evidence':'teal','B_direct':'orange','B_evidence':'darkorange'}
 for ax,c in zip(axs,protocol['conditions']):
  config=next(cfg for cfg in configs if cfg['experiment']['condition']==c['condition']);m=KnownMap(config['navigation']);ax.imshow(m.inflated,origin='lower',extent=[m.bounds[0],m.bounds[2],m.bounds[1],m.bounds[3]],cmap='Greys',alpha=.2)
  for row in rows:
   if row['condition']!=c['condition']:continue
   tr=trajectories[row['run']];label=row['sensor_mode']+'_'+row['planner']
   if tr:ax.plot([float(t['true_x']) for t in tr],[float(t['true_y']) for t in tr],label=label,c=colors[label]);ax.scatter(float(tr[-1]['true_x']),float(tr[-1]['true_y']),c=colors[label],s=25)
  ax.scatter(*c['center_world_m'][:2],marker='s',c='red',label='inspection panel');ax.set(title=c['condition'],xlabel='map x (m)',ylabel='map y (m)',xlim=(-.3,3.3),ylim=(-.8,.8));ax.set_aspect('equal')
 axs[-1].legend(fontsize=7);fig.tight_layout();fig.savefig(suite/'actual_trajectories.png',dpi=150);plt.close(fig)
 chunks=['<!doctype html><html lang="zh"><meta charset="utf-8"><title>阶段7真实闭环结果</title><style>body{font-family:sans-serif;max-width:1100px;margin:30px auto}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:8px}video{width:640px;max-width:100%}.fail{color:#a22}</style><h1>阶段7：已知地图的自主巡检工程冒烟</h1><p>12次独立Isaac运行；固定雷达估计定位、公共相机与SqueezeNet外观模型。不是SLAM/Nav2或真实工业设备分类。完整失败保留。</p><img width="100%" src="task_summary.png"><img width="100%" src="actual_trajectories.png"><table><tr><th>运行</th><th>结果/原因</th><th>观察次数</th><th>仿真耗时</th></tr>']
 for r in rows:chunks.append('<tr><td>'+html.escape(r['run'])+'</td><td>'+('成功' if r['task_success'] else '<span class="fail">失败</span>')+' '+html.escape(r['reason'])+'</td><td>'+str(r['observations'])+'</td><td>'+str(round(r['task_sim_s'],2) if r['task_sim_s'] else '-')+'</td></tr>')
 chunks.append('</table><h2>失败视频：实际仿真相机，不是生成画面</h2>')
 for f in failures:chunks.append('<h3>'+html.escape(f['run']+' — '+f['reason'])+'</h3><video controls preload="metadata" src="'+html.escape(f['video'])+'"></video>')
 chunks.append('<p>全部运行各自目录还包含actual_simulation.mp4、原始视频帧、导航事件、轨迹、观测输入及模型结果。人工延迟压力条件不代表自然任务分布。</p></html>');(suite/'results.html').write_text(''.join(chunks))
 print(json.dumps({'groups':groups,'failures':failures,'resources':resource,'checks':check},indent=2))
if __name__=='__main__':main()
