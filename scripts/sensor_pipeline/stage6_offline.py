"""Extract frozen samples, infer without evaluation access, then score separately."""
import argparse,csv,hashlib,json,time,resource,subprocess,sys
from pathlib import Path
import numpy as np,cv2,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from stage6_bag import TOPICS,stamp
from fusion_geometry import red_component_mask,project_optical,decide_hybrid
from analyze_stage5_fusion import lidar_object

def dump(p,x):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,indent=2,allow_nan=False))
def rows(p,rs):
 if not rs:return
 with p.open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(rs[0]));w.writeheader();w.writerows(rs)
def readbag(run):
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(run/'record/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'));types={t:get_message(typ) for t,typ in TOPICS.items()}
 while r.has_next():
  t,b,receipt=r.read_next()
  if t in types:yield t,deserialize_message(b,types[t]),receipt

def matrix(tf):
 t=tf.transform.translation;q=tf.transform.rotation;T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();T[:3,3]=[t.x,t.y,t.z];return T

def extract(run):
 cfg=json.loads((run/'config.json').read_text());scenes=[];poses=[];mounts={};sets={t:set() for t in TOPICS};scan_stamps=[];scan_spans=[];clock_resets=0;lastclock=-1;schema={}
 for t,m,receipt in readbag(run):
  s=stamp(m);sets[t].add(s)
  if t=='/go2w/sensors/scene':scenes.append(json.loads(m.data))
  elif t=='/evaluation/go2w/poses':poses.append(json.loads(m.data))
  elif t=='/tf_static':
   for tf in m.transforms:mounts[tf.child_frame_id]={'parent':tf.header.frame_id,'T_base_sensor':matrix(tf).tolist()}
  elif t=='/go2w/lidar/points':
   fields=[{'name':f.name,'offset':f.offset,'datatype':f.datatype} for f in m.fields];schema={'fields':fields,'point_step':m.point_step,'frame_id':m.header.frame_id}
   dtype=np.dtype({'names':['x','y','z','offset','emitter','sec','nsec'],'formats':['<f4','<f4','<f4','<f8','<u4','<u4','<u4'],'offsets':[0,4,8,12,20,24,28],'itemsize':32})
   arr=np.frombuffer(bytes(m.data),dtype=dtype);pt=arr['sec'].astype(np.uint64)*1000000000+arr['nsec']
   scan_stamps.append(s);scan_spans.append({'stamp_ns':s,'points':len(arr),'span_ms':float((pt.max()-pt.min())*1e-6),'header_end_delta_ns':int(s)-int(pt.max())})
  elif t=='/clock':
   if s<lastclock:clock_resets+=1
   lastclock=s
 image_topics=[t for t in TOPICS if t.endswith('/image_raw') or t.endswith('/camera_info')]
 selected=[];pairs=[]
 for meta in scenes:
  s=meta['stamp_ns'];delta=min(scan_stamps,key=lambda x:abs(x-s))-s if scan_stamps else None
  pairs.append({'scene_id':meta['scene_id'],'stamp_ns':s,'all_images_info_exact':all(s in sets[t] for t in image_topics),'lidar_minus_image_ms':delta*1e-6 if delta is not None else None,'within_50ms':delta is not None and abs(delta)<=cfg['pair_tolerance_ms']*1e6})
 for scene in cfg['scenes']:
  candidates=[m for m in scenes if m['scene_id']==scene['scene_id'] and m['stable']]
  for i,offset in enumerate(cfg['sample_offsets_s']):
   candidate=next((m for m in candidates if m['scene_elapsed_s']>=offset),None)
   if candidate is None:raise RuntimeError('Frozen sample absent; do not retry or replace')
   m=dict(candidate);m['sample_id']=scene['scene_id']+f'_{i:02}';m['requested_offset_s']=offset
   close=min(scan_stamps,key=lambda x:abs(x-m['stamp_ns']));m['lidar_stamp_ns']=close if abs(close-m['stamp_ns'])<=cfg['pair_tolerance_ms']*1e6 else None
   selected.append(m)
 out=run/'offline';out.mkdir(exist_ok=True);inputs=out/'inputs';inputs.mkdir(exist_ok=True)
 wanted={m['stamp_ns']:m for m in selected};scan_wanted={m['lidar_stamp_ns'] for m in selected};scan_data={};evalrows=[];infos={}
 for t,msg,receipt in readbag(run):
  s=stamp(msg)
  if t=='/go2w/lidar/points' and s in scan_wanted:
   arr=np.frombuffer(bytes(msg.data),dtype=dtype);scan_data[s]={n:arr[n].copy() for n in arr.dtype.names}
  if s not in wanted:continue
  folder=inputs/wanted[s]['sample_id'];folder.mkdir(exist_ok=True)
  if t.endswith('/image_raw'):
   name={'/go2w/front_camera/image_raw':'public','/go2w/stereo/left/image_raw':'left','/go2w/stereo/right/image_raw':'right','/go2w/stereo/depth/image_raw':'depth'}[t]
   if msg.encoding=='rgb8':
    im=np.frombuffer(bytes(msg.data),np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
    cv2.imwrite(str(folder/(name+'.png')),cv2.cvtColor(im,cv2.COLOR_RGB2BGR))
   elif msg.encoding=='32FC1':np.save(folder/'depth.npy',np.frombuffer(bytes(msg.data),'<f4').reshape(msg.height,msg.width))
   else:raise ValueError(msg.encoding)
  elif t.endswith('/camera_info'):infos.setdefault(s,{})[t]={'K':list(msg.k),'P':list(msg.p),'D':list(msg.d),'width':msg.width,'height':msg.height,'frame_id':msg.header.frame_id}
 for m in selected:
  folder=inputs/m['sample_id'];scan=scan_data.get(m['lidar_stamp_ns']);m['missing_lidar']=scan is None
  if scan is not None:np.savez_compressed(folder/'lidar.npz',**scan)
  m['camera_info']=infos.get(m['stamp_ns'],{});m['mounts']=mounts
  m['input_rgb_sha256']=hashlib.sha256((folder/'public.png').read_bytes()).hexdigest()
  dump(folder/'metadata.json',m)
  # Evaluation-only data is never written into inference input folders.
  pose=min(poses,key=lambda x:abs(x['simulation_time_s']-m['stamp_ns']*1e-9));q=pose['public']['orientation_world_wxyz'];p=pose['public']['position_world_m'];rot=Rotation.from_quat(np.asarray(q)[[1,2,3,0]])
  true=rot.inv().apply(np.asarray(pose['object']['center_world_m'])-p)
  evalrows.append({'sample_id':m['sample_id'],'true_position_optical_m':true.tolist(),'pose_time_minus_image_ms':(pose['simulation_time_s']-m['stamp_ns']*1e-9)*1000,'public_pose':pose['public'],'object':pose['object']})
 rows(out/'pairing.csv',pairs);rows(out/'scan_timing.csv',scan_spans);dump(out/'evaluation_only.json',evalrows)
 timing=np.asarray([p['lidar_minus_image_ms'] for p in pairs if p['lidar_minus_image_ms'] is not None])
 validation={'image_bundles':len(pairs),'exact_image_info_bundles':sum(p['all_images_info_exact'] for p in pairs),'lidar_pairs_within_50ms':sum(p['within_50ms'] for p in pairs),'lidar_abs_delta_p50_ms':float(np.percentile(abs(timing),50)),'lidar_abs_delta_p95_ms':float(np.percentile(abs(timing),95)),'selected_samples':len(selected),'clock_resets_recording':clock_resets,'lidar_schema':schema,'scan_span_p50_ms':float(np.median([s['span_ms'] for s in scan_spans])),'scan_header_matches_end':all(s['header_end_delta_ns']==0 for s in scan_spans),'tf_mounts':mounts,'dynamic_deskew_used':False,'position_reference':'visible surface center, not volumetric object centroid'}
 dump(out/'validation.json',validation)

def transform(points,T):return points@T[:3,:3].T+T[:3,3]
def infer(inputs,out,mode,config):
 """This function reads inputs/config only. No evaluation path or pose enters geometry."""
 out.mkdir(parents=True,exist_ok=False);cfg=json.loads(config.read_text());events=[];latencies=[];start_rss=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss;start=time.time()
 for folder in sorted(inputs.iterdir()):
  if not folder.is_dir():continue
  t0=time.perf_counter();m=json.loads((folder/'metadata.json').read_text());rgb=cv2.cvtColor(cv2.imread(str(folder/'public.png')),cv2.COLOR_BGR2RGB)
  mask=red_component_mask(rgb,cfg['manual_red_mask']);K=np.array(m['camera_info']['/go2w/front_camera/camera_info']['K']).reshape(3,3)
  Tbc=np.array(m['mounts']['front_camera_optical_frame']['T_base_sensor']);Tcl=np.linalg.inv(Tbc)@np.array(m['mounts']['go2w_lidar']['T_base_sensor']);opts=[];lidar=None;stereo=None;ls={};ss={};selected_points=np.empty((0,3))
  def estimate(points,minimum):
   uv,valid=project_optical(points,K,np.zeros(5),(640,480));proj={'points_optical':points,'uv':uv,'valid':valid,'raw_indices':np.arange(len(points))}
   return lidar_object(proj,mask,{**cfg['association'],'min_lidar_points':minimum})
  if not m['missing_lidar']:
   a=np.load(folder/'lidar.npz');points=transform(np.column_stack([a['x'],a['y'],a['z']]),Tcl);lidar,ls=estimate(points,cfg['association']['min_lidar_points']);lidar_points=points[np.asarray(ls.get('chosen_indices',[]),int)]
  else:lidar_points=np.empty((0,3))
  if mode=='B':
   depth=np.load(folder/'depth.npy');yy,xx=np.where(np.isfinite(depth)&(depth>0));z=depth[yy,xx];Kl=np.array(m['camera_info']['/go2w/stereo/depth/camera_info']['K']).reshape(3,3)
   pts=np.column_stack([(xx-Kl[0,2])*z/Kl[0,0],(yy-Kl[1,2])*z/Kl[1,1],z]);Tcs=np.linalg.inv(Tbc)@np.array(m['mounts']['go2w_stereo_left_optical_frame']['T_base_sensor']);spoints=transform(pts,Tcs)
   stereo,ss=estimate(spoints,cfg['association']['min_stereo_pixels'])
   if stereo:stereo['modality']='stereo'
   choice,source,conflict,unknown=decide_hybrid(lidar,stereo,cfg['association']['max_lidar_stereo_disagreement_m'])
  else:choice,source,conflict,unknown=lidar,'lidar_only' if lidar else 'unknown',False,lidar is None
  if choice is not None:
   selected_points=spoints[np.asarray(ss['chosen_indices'],int)] if choice['modality']=='stereo' else lidar_points
  geometry_ms=(time.perf_counter()-t0)*1000;latencies.append(geometry_ms)
  sid=m['sample_id'];np.save(out/(sid+'_evidence.npy'),selected_points)
  cv2.imwrite(str(out/(sid+'_mask.png')),mask*255)
  row={'sample_id':sid,'scene_id':m['scene_id'],'object_id':m['object_id'],'split':m['split'],'frame_id':'front_camera_optical_frame','stamp_ns':m['stamp_ns'],'mode':mode,'source':source,'unknown':bool(unknown),'conflict':bool(conflict),'position_m':choice['position_m'] if choice else None,'valid_points':choice['valid_point_count'] if choice else 0,'coverage':choice['coverage_fraction'] if choice else 0.,'uncertainty_m':choice['uncertainty_m'] if choice else None,'input_rgb_sha256':m['input_rgb_sha256'],'mask_sha256':hashlib.sha256(mask.tobytes()).hexdigest(),'geometry_latency_ms':geometry_ms,'matched_depth_precompute_ms':m['depth_runtime_ms'] if mode=='B' else 0.,'total_compute_ms':geometry_ms+(m['depth_runtime_ms'] if mode=='B' else 0.),'attempt':1,'evidence_file':sid+'_evidence.npy','lidar_stamp_ns':m['lidar_stamp_ns']}
  events.append(row)
 (out/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in events));dump(out/'config.json',cfg)
 dump(out/'metadata.json',{'mode':mode,'input_directory':str(inputs.resolve()),'evaluation_access':'none in infer function','started_epoch_s':start,'opencv':cv2.__version__,'numpy':np.__version__,'cuda_context':False,'seed':'fixed scene seeds in config','semantic_model':'same frozen classical red-component mask','fixed_mounts':'T_base_sensor; no world pose input','retries':0})
 dump(out/'metrics.json',{'samples':len(events),'known':sum(not r['unknown'] for r in events),'geometry_p50_ms':float(np.percentile(latencies,50)),'geometry_p95_ms':float(np.percentile(latencies,95)),'peak_rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,'initial_peak_rss_mib':start_rss/1024,'vram_mib':0,'vram_scope':'CPU inference; no CUDA context; simulator separate'})

def score(run):
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 out=run/'offline';truth={r['sample_id']:r for r in json.loads((out/'evaluation_only.json').read_text())};paired={};allrows=[]
 for mode in ['A','B']:
  rs=[json.loads(s) for s in (out/mode/'events.jsonl').read_text().splitlines()]
  for r in rs:
   gt=truth[r['sample_id']];r['position_error_m']=float(np.linalg.norm(np.asarray(r['position_m'])-gt['true_position_optical_m'])) if r['position_m'] is not None else None
   evidence=np.load(out/mode/r['evidence_file']);purity=None
   if len(evidence):
    pose=gt['public_pose'];q=np.asarray(pose['orientation_world_wxyz']);world=Rotation.from_quat(q[[1,2,3,0]]).apply(evidence)+pose['position_world_m'];obj=gt['object'];c=np.asarray(obj['center_world_m']);good=(abs(world[:,0]-c[0])<.08)&(abs(world[:,1]-c[1])<obj['width_m']/2+.02)&(abs(world[:,2]-c[2])<obj['height_m']/2+.02);purity=float(good.mean())
   r['association_purity']=purity;allrows.append(r);paired.setdefault(r['sample_id'],{})[mode]=r
 rows(out/'per_sample_metrics.csv',allrows);summary=[]
 for scene in json.loads((run/'config.json').read_text())['scenes']:
  row={'scene_id':scene['scene_id'],'split':scene['split'],'samples_per_mode':6}
  for mode in ['A','B']:
   group=[r for r in allrows if r['scene_id']==scene['scene_id'] and r['mode']==mode];errs=[r['position_error_m'] for r in group if r['position_error_m'] is not None];assoc=[r['association_purity'] for r in group if r['association_purity'] is not None]
   row.update({mode+'_known_fraction':sum(not r['unknown'] for r in group)/len(group),mode+'_position_mae_m':float(np.mean(errs)) if errs else None,mode+'_support_coverage':float(np.mean([r['coverage'] for r in group])),mode+'_association_purity':float(np.mean(assoc)) if assoc else None,mode+'_latency_p50_ms':float(np.percentile([r['total_compute_ms'] for r in group],50)),mode+'_latency_p95_ms':float(np.percentile([r['total_compute_ms'] for r in group],95)),mode+'_conflicts':sum(r['conflict'] for r in group)})
  summary.append(row)
 rows(out/'paired_summary.csv',summary)
 equal=all(p['A']['input_rgb_sha256']==p['B']['input_rgb_sha256'] and p['A']['mask_sha256']==p['B']['mask_sha256'] and p['A']['lidar_stamp_ns']==p['B']['lidar_stamp_ns'] for p in paired.values())
 dump(out/'fairness_checks.json',{'identical_rgb_masks_lidar':equal,'paired_samples':len(paired),'attempts_per_mode':1,'test_tuning':False,'truth_used_during_inference':False,'scope':'static geometry and perception only; not autonomous task success','association_definition':'selected measured points within 8cm target plane and 2cm expanded extent; evaluation only','position_definition':'visible planar surface center','unknown_not_free_space':True})
 fig,axs=plt.subplots(1,3,figsize=(15,4));x=np.arange(len(summary))
 for mode,offset,color in [('A',-.18,'steelblue'),('B',.18,'darkorange')]:
  for ax,key,label in zip(axs,['position_mae_m','known_fraction','support_coverage'],['Center error (m), known only','Known samples / all samples','Mask support coverage']):
   vals=[r[mode+'_'+key] if r[mode+'_'+key] is not None else np.nan for r in summary];ax.bar(x+offset,vals,.34,label=mode,color=color);ax.set_xticks(x);ax.set_xticklabels([r['scene_id'] for r in summary],rotation=20);ax.set_title(label)
 axs[0].legend();fig.tight_layout();fig.savefig(out/'geometry_coverage.png',dpi=150);plt.close(fig)
 # Actual sensor RGB and evidence overlays, distinct from GUI screenshots.
 samples=list(paired);fig,axs=plt.subplots(2,2,figsize=(12,8))
 for ax,sid in zip(axs.flat,[samples[i] for i in [0,6,12,18]]):
  folder=out/'inputs'/sid;m=json.loads((folder/'metadata.json').read_text());K=np.array(m['camera_info']['/go2w/front_camera/camera_info']['K']).reshape(3,3);rgb=cv2.cvtColor(cv2.imread(str(folder/'public.png')),cv2.COLOR_BGR2RGB);ax.imshow(rgb)
  evidence=np.load(out/'A'/(sid+'_evidence.npy'));uv,valid=project_optical(evidence,K,np.zeros(5),(640,480));ax.scatter(uv[valid,0],uv[valid,1],s=3,c='lime');ax.set_title(sid);ax.axis('off')
 fig.tight_layout();fig.savefig(out/'sensor_projection_samples.png',dpi=130);plt.close(fig)
 print(json.dumps({'paired_samples':len(paired),'fair':equal,'summary':summary},indent=2))

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path);p.add_argument('--infer',choices=['A','B']);p.add_argument('--inputs',type=Path);p.add_argument('--out',type=Path);p.add_argument('--config',type=Path);a=p.parse_args()
 if a.infer:infer(a.inputs,a.out,a.infer,a.config);return
 run=a.run.resolve();extract(run)
 for mode in ['A','B']:subprocess.run([sys.executable,__file__,'--infer',mode,'--inputs',str(run/'offline/inputs'),'--out',str(run/'offline'/mode),'--config',str(run/'config.json')],check=True)
 score(run)
if __name__=='__main__':main()
