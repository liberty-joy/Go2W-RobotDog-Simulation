"""Bounded inspection task executor; perception/model output cannot issue arbitrary commands."""
import json,time,hashlib,math
from pathlib import Path
import numpy as np,cv2
from scipy.spatial.transform import Rotation
from fusion_geometry import red_component_mask,project_optical,decide_hybrid
from analyze_stage5_fusion import lidar_object
from stage7_geometry import rot2

def task_from_attributes(registry):
 obj=next((o for o in registry if o['attributes']['kind']=='inspection_panel' and o['attributes']['color']=='red'),None)
 if obj is None:return None
 return {'schema_version':1,'object_id':obj['object_id'],'operations':[{'op':'inspect','object_id':obj['object_id']},{'op':'snapshot','object_id':obj['object_id']},{'op':'wait','seconds':.5}],'origin':'manual object properties + deterministic rules; no object coordinates supplied'}

class TaskLayer:
 def task_init(self):
  self.task_initialized=True;self.task=task_from_attributes(self.cfg['object_registry']);self.exp=self.cfg.get('experiment',{'sensor_mode':'A','planner':'direct','vision_model':'rule_only','delay_s':0});self.obs_count=0;self.task_state='settle';self.task_wait_start=self.now;self.evidence=None;self.reference_embedding=None;self.pending_vision=None;self.task_start=self.now;self.inspection_success=False;self.task_result=None;self.observations=[];self.obs_dir=self.out/'observations';self.obs_dir.mkdir();(self.out/'task.json').write_text(json.dumps(self.task,indent=2));self.event('structured_task_created',task=self.task,experiment=self.exp)
 def task_end(self,success,reason):
  self.task_result={'task_success':bool(success),'reason':reason,'sensor_mode':self.exp['sensor_mode'],'planner':self.exp['planner'],'vision_model':self.exp['vision_model'],'observations':self.obs_count,'extra_observations':max(0,self.obs_count-2),'manual_interventions':0,'task_elapsed_sim_s':self.now-self.task_start,'snapshot':str(self.out/'inspection_snapshot.png') if (self.out/'inspection_snapshot.png').exists() else None,'observation_budget':self.cfg['task_budget']['max_observations'],'task_timeout_s':self.cfg['task_budget']['task_timeout_s'],'condition':self.exp.get('condition','normal'),'seed':self.exp.get('seed')};(self.out/'task_metrics.json').write_text(json.dumps(self.task_result,indent=2));self.event('inspection_task_result',**self.task_result);self.finish(success)
 def request_observation(self,purpose):
  if self.obs_count>=self.cfg['task_budget']['max_observations']:self.task_end(False,'observation_budget_exhausted');return False
  evidence=self.collect_observation()
  if evidence is None:return False
  self.obs_count+=1;self.evidence=evidence;self.pending_purpose=purpose;self.event('observation_captured',index=self.obs_count,**{k:v for k,v in evidence.items() if k not in ['rgb','embedding']})
  if self.exp['vision_model']=='rule_only':
   evidence['appearance_pass']=True;evidence['model']='rule_only';self.observe_completed(purpose);return True
  folder=Path(self.exp['vision_io']);folder.mkdir(parents=True,exist_ok=True);job=hashlib.sha256(str(self.out.resolve()).encode()).hexdigest()[:16]+'_'+self.out.parent.name+f'_obs{self.obs_count:02}';request=folder/(job+'.request.json');self.pending_vision=folder/(job+'.response.json');temporary=request.with_suffix('.tmp');temporary.write_text(json.dumps({'crop_path':evidence['crop_path'],'source_rgb_sha256':evidence['source_rgb_sha256']}));temporary.replace(request);self.task_state='vision_wait';self.vision_wait_wall=time.monotonic();return True
 def collect_observation(self):
  prefixes=['/go2w/front_camera','/go2w/stereo/left','/go2w/stereo/right','/go2w/stereo/depth']
  if any(p not in self.images or p not in self.infos for p in prefixes) or len(self.mounts)<5:return None
  def stamp(m):return m.header.stamp.sec*1000000000+m.header.stamp.nanosec
  stamps=[stamp(self.images[p]) for p in prefixes]+[stamp(self.infos[p]) for p in prefixes]
  if len(set(stamps))!=1:return None
  s=stamps[0]
  if self.now-s*1e-9>.5:return None
  # Causal scans only, with the same 50ms tolerance; no look-ahead or ground truth deskew.
  scans=[(t,p) for t,p in self.scan_history if 0<=s-t<=50000000]
  if not scans:return None
  scan_stamp,pc=max(scans,key=lambda x:x[0]);rgb_msg=self.images[prefixes[0]];rgb=np.frombuffer(bytes(rgb_msg.data),np.uint8).reshape(480,640,3).copy();mask=red_component_mask(rgb,self.cfg['manual_red_mask']);K=np.array(self.infos[prefixes[0]].k).reshape(3,3);Tbc=self.mounts['front_camera_optical_frame'];Tcl=np.linalg.inv(Tbc)@self.mounts['go2w_lidar']
  def apply(p,T):return p@T[:3,:3].T+T[:3,3]
  def estimate(points,minimum):
   uv,valid=project_optical(points,K,np.zeros(5),(640,480));return lidar_object({'points_optical':points,'uv':uv,'valid':valid,'raw_indices':np.arange(len(points))},mask,{**self.cfg['association'],'min_lidar_points':minimum})[0]
  lidar=estimate(apply(pc,Tcl),self.cfg['association']['min_lidar_points']);stereo=None
  if self.exp['sensor_mode']=='B':
   depth=np.frombuffer(bytes(self.images[prefixes[3]].data),'<f4').reshape(480,640);yy,xx=np.where(np.isfinite(depth)&(depth>0));z=depth[yy,xx];Kl=np.array(self.infos[prefixes[3]].k).reshape(3,3);pts=np.column_stack([(xx-Kl[0,2])*z/Kl[0,0],(yy-Kl[1,2])*z/Kl[1,1],z]);stereo=estimate(apply(pts,np.linalg.inv(Tbc)@self.mounts['go2w_stereo_left_optical_frame']),self.cfg['association']['min_stereo_pixels'])
   if stereo:stereo['modality']='stereo'
   chosen,source,conflict,unknown=decide_hybrid(lidar,stereo,self.cfg['association']['max_lidar_stereo_disagreement_m'])
  else:chosen,source,conflict,unknown=lidar,'lidar_only' if lidar else 'unknown',False,lidar is None
  object_map=None
  if chosen is not None:
   base=apply(np.array([chosen['position_m']]),Tbc)[0];object_map=(rot2(self.pose[2])@base[:2]+self.pose[:2]).tolist()
  index=self.obs_count+1;folder=self.obs_dir/f'{index:02}';folder.mkdir(exist_ok=False);cv2.imwrite(str(folder/'public_rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));cv2.imwrite(str(folder/'semantic_mask.png'),mask*255)
  for name,p in [('left',prefixes[1]),('right',prefixes[2])]:cv2.imwrite(str(folder/(name+'.png')),cv2.cvtColor(np.frombuffer(bytes(self.images[p].data),np.uint8).reshape(480,640,3),cv2.COLOR_RGB2BGR))
  np.save(folder/'estimated_depth_z_m.npy',np.frombuffer(bytes(self.images[prefixes[3]].data),'<f4').reshape(480,640));np.savez_compressed(folder/'raw_lidar.npz',points=pc,scan_end_ns=scan_stamp)
  ys,xs=np.where(mask>0);crop=rgb[max(0,int(ys.min())):int(ys.max())+1,max(0,int(xs.min())):int(xs.max())+1] if len(xs) else rgb;crop_path=folder/'model_crop.png';cv2.imwrite(str(crop_path),cv2.cvtColor(crop,cv2.COLOR_RGB2BGR));sha=hashlib.sha256(rgb.tobytes()).hexdigest()
  evidence={'object_id':self.task['object_id'],'object_exists':bool(mask.sum()>=self.cfg['manual_red_mask']['min_pixels']),'stamp_ns':s,'lidar_stamp_ns':scan_stamp,'causal_pair_delta_ms':(s-scan_stamp)*1e-6,'object_map_xy':object_map,'unknown':bool(unknown),'conflict':bool(conflict),'depth_source':source,'lidar':lidar,'stereo':stereo,'coverage':chosen['coverage_fraction'] if chosen else 0.,'uncertainty_m':chosen['uncertainty_m'] if chosen else None,'source_rgb_sha256':sha,'mask_sha256':hashlib.sha256(mask.tobytes()).hexdigest(),'crop_path':str(crop_path.resolve()),'evidence_directory':str(folder.resolve()),'localization_source':self.mode,'model':'pending','appearance_pass':False}
  (folder/'evidence.json').write_text(json.dumps(evidence,indent=2));self.observations.append(evidence);return evidence
 def observe_completed(self,purpose):
  self.evidence['finalized_sim_s']=self.now
  (Path(self.evidence['evidence_directory'])/'evidence.json').write_text(json.dumps(self.evidence,indent=2))
  if purpose=='arrival':
   if not self.evidence['object_exists'] or not self.evidence['appearance_pass']:self.task_end(False,'arrival_object_not_confirmed');return
   # Photo only after navigation success and a fresh stopped observation.
   import shutil
   shutil.copyfile(Path(self.evidence['evidence_directory'])/'public_rgb.png',self.out/'inspection_snapshot.png');self.event('inspection_snapshot_saved',evidence=self.evidence['evidence_directory']);self.task_state='final_wait';self.task_wait_start=self.now;return
  self.task_state='plan_wait';self.task_wait_start=self.now
 def verify(self,e):
  if self.task_canceled:return False,'task_canceled'
  registry={o['object_id'] for o in self.cfg['object_registry']}
  if e['object_id'] not in registry or not e['object_exists']:return False,'object_not_present'
  if self.now-e['stamp_ns']*1e-9>self.cfg['task_budget']['evidence_max_age_s']:return False,'stale_evidence'
  if not e.get('appearance_pass',False):return False,'appearance_unconfirmed'
  if e['unknown'] or e['conflict'] or e['object_map_xy'] is None:return False,'unknown_or_conflicting_depth'
  objectxy=np.asarray(e['object_map_xy'],dtype=float)
  if objectxy.shape!=(2,) or not np.isfinite(objectxy).all():return False,'invalid_geometry'
  delta=objectxy-self.pose[:2];norm=np.linalg.norm(delta)
  if norm<1e-6:return False,'invalid_geometry'
  goal=objectxy-delta/norm*self.cfg['task_budget']['inspection_standoff_m'];heading=math.atan2(delta[1],delta[0]);route=self.map.plan(self.pose[:2],goal)
  if route is None:return False,'unreachable'
  return True,{'id':'inspect_'+e['object_id'],'x':float(goal[0]),'y':float(goal[1]),'yaw':heading,'timeout_s':self.cfg['task_budget']['goal_timeout_s']}
 def task_tick(self):
  if not hasattr(self,'task_initialized'):self.task_init()
  if self.task_canceled:self.task_end(False,'task_canceled');return
  if self.now-self.task_start>self.cfg['task_budget']['task_timeout_s']:self.task_end(False,'task_timeout');return
  if self.task_state=='settle':
   if self.now-self.task_wait_start>=.8:self.request_observation('initial')
  elif self.task_state=='vision_wait':
   if self.pending_vision.exists():
    result=json.loads(self.pending_vision.read_text());folder=Path(self.evidence['evidence_directory']);(folder/'model_result.json').write_text(json.dumps(result,indent=2));self.evidence['model']='squeezenet1_1';passed=False;cosine=None
    if result['status']=='ok' and result['source_rgb_sha256']==self.evidence['source_rgb_sha256']:
     emb=np.asarray(result['embedding']);cosine=float(np.dot(emb,self.reference_embedding)) if self.reference_embedding is not None else 1.;passed=cosine>=.65
     if self.reference_embedding is None and self.evidence['object_exists']:self.reference_embedding=emb
    self.evidence['appearance_pass']=passed;self.evidence['appearance_cosine_to_initial']=cosine;self.event('model_data_received',appearance_pass=passed,cosine=cosine,status=result['status']);self.observe_completed(self.pending_purpose)
   elif time.monotonic()-self.vision_wait_wall>8:self.task_end(False,'visual_model_timeout')
  elif self.task_state=='plan_wait':
   delay=self.exp.get('delay_s',0.) if self.obs_count==1 else 0.
   if self.now-self.task_wait_start<delay:return
   valid,goal_or_reason=self.verify(self.evidence)
   if not valid and self.exp['planner']=='evidence':
    self.event('evidence_planner_requests_observation',reason=goal_or_reason);self.task_state='refresh';self.task_wait_start=self.now;return
   if not valid:
    # Direct proposal is independently rejected by the executor. It never reaches cmd_vel.
    self.event('executor_rejected_direct_plan',reason=goal_or_reason);self.task_end(False,goal_or_reason);return
   self.event('plan_validated',goal=goal_or_reason,planner=self.exp['planner']);self.start_goal(goal_or_reason);self.task_state='navigate'
  elif self.task_state=='refresh':
   if self.now-self.task_wait_start>.4:self.request_observation('refresh')
  elif self.task_state=='navigate':
   if self.active is not None:return
   result=next((r for r in reversed(self.results) if r['id'].startswith('inspect_')),None)
   if result is None or result['status']!='succeeded':self.task_end(False,'navigation_'+(result['reason'] if result else 'missing_result'));return
   self.task_state='arrival_settle';self.task_wait_start=self.now
  elif self.task_state=='arrival_settle':
   if self.now-self.task_wait_start>=self.cfg['task_budget']['snapshot_wait_s']:self.request_observation('arrival')
  elif self.task_state=='final_wait':
   if self.now-self.task_wait_start>=self.cfg['task_budget']['wait_s']:self.task_end(True,'inspected_snapshot_wait_complete')
