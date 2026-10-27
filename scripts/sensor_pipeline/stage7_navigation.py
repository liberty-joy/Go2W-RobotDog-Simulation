"""Minimal actual closed-loop known-map navigation. No Nav2/SLAM claim."""
import argparse,json,time,math,csv,hashlib
from collections import deque
from stage7_tasks import TaskLayer
from scipy.spatial.transform import Rotation
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist,PoseStamped,PolygonStamped,Point32,TransformStamped
from nav_msgs.msg import OccupancyGrid,Odometry,Path as RosPath
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import PointCloud2,Image,CameraInfo
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time
from stage7_geometry import KnownMap,angle,rot2

def ns(t):return t.sec*1000000000+t.nanosec
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def posemsg(x,y,a,stamp):
 p=PoseStamped();p.header.frame_id='map';p.header.stamp=stamp if stamp is not None else Time();p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2);return p

def readpc(m):
 dtype=np.dtype({'names':['x','y','z','sec','nsec'],'formats':['<f4','<f4','<f4','<u4','<u4'],'offsets':[0,4,8,24,28],'itemsize':m.point_step});a=np.frombuffer(bytes(m.data),dtype=dtype);return np.column_stack([a['x'],a['y'],a['z']]),a['sec'].astype(np.uint64)*1000000000+a['nsec']
class TruthEvaluator:
 """Evaluation sink. Its pose is only exposed to navigation in explicit oracle mode."""
 def __init__(self):self.latest=None;self.rows=[]
 def receive(self,m):self.latest=m
 def row(self,sim_s,estimate,source):
  if self.latest is None:return
  p=self.latest.pose.pose.position;q=self.latest.pose.pose.orientation
  self.rows.append({'time_s':sim_s,'true_x':p.x,'true_y':p.y,'true_yaw':yaw(q),'est_x':float(estimate[0]),'est_y':float(estimate[1]),'est_yaw':float(estimate[2]),'position_error_m':float(np.linalg.norm(np.asarray([p.x,p.y])-estimate[:2])),'yaw_error_rad':abs(angle(yaw(q)-estimate[2])),'source':source})
class Navigation(Node):
 def __init__(self,cfg,out,mode,suite):
  super().__init__('go2w_known_map_navigation');self.cfg=cfg;self.nc=cfg['navigation'];self.out=out;self.out.mkdir(parents=True,exist_ok=True);self.mode=mode;self.suite=suite;self.map=KnownMap(self.nc);self.now=0.;self.clock_stamp=None;self.last_clock_wall=time.monotonic();self.pose=None;self.last_scan_wall=-1e9;self.pc=None;self.scan_stamp_ns=-1;self.localization={};self.pose_seed=np.array(self.nc['initial_pose_estimate']);self.active=None;self.results=[];self.task_phase='warmup';self.next_test=0;self.phase_start=None;self.finished=False;self.blocked_since=None;self.last_cmd=(0.,0.);self.nav_started=None;self.truth=TruthEvaluator();self.initialized=False;self.images={};self.infos={};self.scene_meta=None;self.canceled_ids=set();self.task_canceled=False;self.scan_history=deque(maxlen=15);self.mounts={}
  self.events=(out/'events.jsonl').open('w');self.tf=self.create_publisher(TFMessage,'/tf',20);self.odom=self.create_publisher(Odometry,'/go2w/navigation/odom',20);self.vel=self.create_publisher(Twist,'/go2w/cmd_vel',10);self.status=self.create_publisher(String,'/go2w/navigation/status',20);self.path_pub=self.create_publisher(RosPath,'/go2w/navigation/path',10);q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.map_pub=self.create_publisher(OccupancyGrid,'/map',q);self.cost_pub=self.create_publisher(OccupancyGrid,'/go2w/navigation/costmap',q);self.foot=self.create_publisher(PolygonStamped,'/go2w/navigation/footprint',10)
  self.create_subscription(TFMessage,'/tf_static',self.mount_cb,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));self.goal_input=self.create_publisher(String,'/go2w/navigation/goal',10)
  self.create_subscription(Clock,'/clock',self.clock_cb,50);self.create_subscription(Odometry,'/evaluation/go2w/odom',self.truth.receive,50);self.create_subscription(PointCloud2,'/go2w/lidar/points',self.scan_cb,20)
  self.create_subscription(String,'/go2w/navigation/goal',self.goal_cb,10);self.create_subscription(PoseStamped,'/goal_pose',self.pose_goal_cb,10);self.create_service(Trigger,'/go2w/navigation/cancel',self.cancel_cb);self.create_service(GetPlan,'/go2w/navigation/get_plan',self.plan_cb)
  for prefix in ['/go2w/front_camera','/go2w/stereo/left','/go2w/stereo/right','/go2w/stereo/depth']:
   self.create_subscription(Image,prefix+'/image_raw',lambda m,p=prefix:self.images.__setitem__(p,m),20)
   self.create_subscription(CameraInfo,prefix+'/camera_info',lambda m,p=prefix:self.infos.__setitem__(p,m),20)
  self.create_subscription(String,'/go2w/sensors/scene',lambda m:setattr(self,'scene_meta',json.loads(m.data)),20)
  self.timer=self.create_timer(.05,self.tick)
  (out/'config.json').write_text(json.dumps(cfg,indent=2));(out/'metadata.json').write_text(json.dumps({'localization':mode,'navigator':'known-map ICP+A*+pure-pursuit-like velocity feedback; not Nav2; not SLAM','oracle_scope':'development only' if mode=='oracle' else 'truth only evaluator sink; never passed to ICP/controller','suite':suite,'time_definition':'goals use sim clock, stale sensor protection wall clock','goal_interface':'JSON goal topic and /goal_pose; GetPlan service; cancel Trigger','manual_intervention':0},indent=2))
 def event(self,kind,**kw):
  row={'event':kind,'simulation_time_s':self.now,'wall_epoch_ns':time.time_ns(),**kw};self.events.write(json.dumps(row)+'\n');self.events.flush();self.status.publish(String(data=json.dumps(row)))
 def clock_cb(self,m):
  t=ns(m.clock)*1e-9
  if t<self.now:self.images.clear();self.infos.clear();self.scan_history.clear();self.pc=None;self.pose=None;self.pose_seed=np.array(self.nc['initial_pose_estimate']);self.active=None;self.event('clock_reset_caches_cleared')
  self.now=t;self.clock_stamp=m.clock;self.last_clock_wall=time.monotonic()
 def mount_cb(self,m):
  for tf in m.transforms:
   if tf.header.frame_id!='base_link':continue
   q=tf.transform.rotation;p=tf.transform.translation;T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();T[:3,3]=[p.x,p.y,p.z];self.mounts[tf.child_frame_id]=T
 def scan_cb(self,m):
  pc,pt=readpc(m);self.pc=pc;self.scan_stamp_ns=ns(m.header.stamp);self.scan_history.append((self.scan_stamp_ns,pc.copy()));self.last_scan_wall=time.monotonic()
  if self.mode=='estimated':
   pose,info=self.map.localize(pc,self.pose if self.pose is not None else self.pose_seed);self.localization=info
   if pose is not None:self.pose=pose;self.publish_pose(m.header.stamp);self.truth.row(self.now,pose,self.mode)
   else:self.pose=None;self.event('localization_rejected',details=info)
 def publish_pose(self,stamp):
  if self.pose is None:return
  x,y,a=self.pose;p=posemsg(x,y,a,stamp);tf1=TransformStamped();tf1.header.stamp=stamp;tf1.header.frame_id='map';tf1.child_frame_id='odom';tf1.transform.rotation.w=1.;tf2=TransformStamped();tf2.header.stamp=stamp;tf2.header.frame_id='odom';tf2.child_frame_id='base_link';tf2.transform.translation.x=float(x);tf2.transform.translation.y=float(y);tf2.transform.translation.z=.39;tf2.transform.rotation=p.pose.orientation;self.tf.publish(TFMessage(transforms=[tf1,tf2]));od=Odometry();od.header=p.header;od.child_frame_id='base_link';od.pose.pose=p.pose;od.pose.pose.position.z=.39;self.odom.publish(od)
 def publish_map(self):
  for pub,grid in [(self.map_pub,self.map.occupied),(self.cost_pub,self.map.inflated)]:
   m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.clock_stamp;m.info.resolution=self.map.res;m.info.width=self.map.w;m.info.height=self.map.h;m.info.origin.position.x=float(self.map.bounds[0]);m.info.origin.position.y=float(self.map.bounds[1]);m.info.origin.orientation.w=1.;m.data=(grid.astype(np.int8)*100).ravel().tolist();pub.publish(m)
 def pose_goal_cb(self,m):
  if m.header.frame_id!='map':self.event('goal_rejected',reason='requires map frame');return
  self.start_goal({'id':'rviz_'+str(ns(m.header.stamp)),'x':m.pose.position.x,'y':m.pose.position.y,'yaw':yaw(m.pose.orientation),'timeout_s':20.})
 def goal_cb(self,m):
  try:self.start_goal(json.loads(m.data))
  except (ValueError,KeyError,TypeError) as e:self.event('goal_rejected',reason=str(e))
 def plan_cb(self,request,response):
  if self.clock_stamp is None or self.pose is None:
   response.plan.header.frame_id='map';return response
  route=self.map.plan([request.start.pose.position.x,request.start.pose.position.y],[request.goal.pose.position.x,request.goal.pose.position.y]);response.plan.header.frame_id='map'
  if route is not None:response.plan.poses=[posemsg(x,y,0.,self.clock_stamp) for x,y in route]
  return response
 def cancel_cb(self,req,res):
  if self.suite=='task':self.task_canceled=True
  res.success=self.active is not None or self.suite=='task';res.message='canceled' if res.success else 'no active goal'
  if self.active:self.canceled_ids.add(self.active['id']);self.end_goal('canceled','cancel_service')
  return res
 def cancel(self):
  if self.active:self.canceled_ids.add(self.active['id']);self.end_goal('canceled','cancel_request')
 def command(self,v,w):
  msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.vel.publish(msg);self.last_cmd=(float(v),float(w))
 def start_goal(self,g):
  if self.suite=='task' and self.task_canceled:self.event('goal_rejected',reason='task_canceled');return
  allowed={'id','x','y','yaw','timeout_s'}
  if not set(g)<=allowed or not all(k in g for k in ['id','x','y']):raise ValueError('invalid goal schema')
  if not all(np.isfinite(float(g[k])) for k in ['x','y']+([k for k in ['yaw','timeout_s'] if k in g])):raise ValueError('nonfinite goal')
  if self.active:self.end_goal('canceled','superseded')
  g={**g,'yaw':g.get('yaw',0.),'timeout_s':min(30.,max(.1,g.get('timeout_s',20.)))}
  self.active=g;self.active['start_s']=self.now;self.blocked_since=None
  route=self.map.plan(self.pose[:2], [g['x'],g['y']]) if self.pose is not None else None
  self.event('goal_accepted',goal=g,localization=self.mode)
  if route is None:self.end_goal('failed','unreachable_or_unlocalized');return
  self.route=route;path=RosPath();path.header.frame_id='map';path.header.stamp=self.clock_stamp;path.poses=[posemsg(x,y,0.,self.clock_stamp) for x,y in route];self.path_pub.publish(path)
 def end_goal(self,status,reason):
  if self.active is None:return
  self.command(0.,0.);g=self.active;result={'id':g['id'],'status':status,'reason':reason,'elapsed_sim_s':self.now-g['start_s'],'goal':[g['x'],g['y'],g['yaw']],'estimated_pose':self.pose.tolist() if self.pose is not None else None};self.results.append(result);self.event('goal_result',**result);self.active=None
 def obstacle(self):
  if self.pc is None:return True
  p=self.pc;good=np.isfinite(p).all(1)&(np.linalg.norm(p[:,:2],axis=1)>.65)&(p[:,0]>.15)&(p[:,0]<self.nc['stop_distance_lidar_m'])&(abs(p[:,1])<self.nc['obstacle_half_width_m'])&(p[:,2]>-.30)&(p[:,2]<.45)
  return int(good.sum())>=8
 def follow(self):
  if self.active is None:self.command(0.,0.);return
  if self.pose is None or time.monotonic()-self.last_scan_wall>self.nc['sensor_stale_wall_s'] or time.monotonic()-self.last_clock_wall>.7:self.end_goal('failed','sensor_or_localization_stale');return
  g=self.active
  if g['id'] in self.canceled_ids:self.end_goal('canceled','canceled_id');return
  if self.now-g['start_s']>=g['timeout_s']:self.end_goal('failed','timeout');return
  if self.obstacle():
   self.command(0.,0.);self.blocked_since=self.now if self.blocked_since is None else self.blocked_since
   if self.now-self.blocked_since>self.nc['blocked_timeout_s']:self.end_goal('failed','obstacle_stop')
   return
  self.blocked_since=None;delta=np.array([g['x'],g['y']])-self.pose[:2];dist=float(np.linalg.norm(delta));heading=angle(math.atan2(delta[1],delta[0])-self.pose[2]);end_heading=angle(g['yaw']-self.pose[2])
  if dist<=self.nc['goal_tolerance_m']:
   if abs(end_heading)<=self.nc['yaw_tolerance_rad']:self.end_goal('succeeded','within_tolerance');return
   self.command(0.,np.clip(self.nc['kp_heading']*end_heading,-self.nc['max_yaw_rad_s'],self.nc['max_yaw_rad_s']));return
  nearest=int(np.argmin(np.linalg.norm(self.route-self.pose[:2],axis=1)));target=self.route[min(len(self.route)-1,nearest+5)];d=target-self.pose[:2];heading=angle(math.atan2(d[1],d[0])-self.pose[2]);w=np.clip(self.nc['kp_heading']*heading,-self.nc['max_yaw_rad_s'],self.nc['max_yaw_rad_s']);v=min(self.nc['max_v_m_s'],self.nc['kp_distance']*dist)*max(0.,math.cos(heading))
  if abs(heading)>.6:v=0.
  self.command(v,w)
 def tick(self):
  if self.finished:return
  if self.mode=='oracle' and self.truth.latest is not None:
   p=self.truth.latest.pose.pose.position;q=self.truth.latest.pose.pose.orientation;self.pose=np.array([p.x,p.y,yaw(q)]);self.publish_pose(self.truth.latest.header.stamp);self.truth.row(self.now,self.pose,self.mode)
  if self.clock_stamp is None:return
  if not self.initialized and self.pose is not None and self.now>2.5:
   self.initialized=True;self.nav_started=self.now;self.phase_start=self.now;self.publish_map();self.event('navigation_ready',localization=self.mode,fit=self.localization)
  foot=PolygonStamped();foot.header.frame_id='base_link';foot.header.stamp=self.clock_stamp;foot.polygon.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in self.nc['footprint_m']];self.foot.publish(foot)
  self.follow()
  if self.initialized:
   if self.suite=='validation':self.validation()
   elif self.suite=='task':self.task_tick()
  if self.initialized and self.now-self.nav_started>100:self.event('session_timeout');self.finish(False)
 def validation(self):
  # Real ROS goal publication and cancel service exercise the public interfaces.
  if self.phase_start is None:return
  if self.active and self.active['id']=='cancel' and self.now-self.active['start_s']>=.8:
   if not hasattr(self,'cancel_client'):self.cancel_client=self.create_client(Trigger,'/go2w/navigation/cancel')
   if self.cancel_client.service_is_ready():self.cancel_client.call_async(Trigger.Request())
   return
  if self.active:return
  if self.now-self.phase_start<.7:return
  if self.next_test>len(self.results):return
  if self.next_test==0:g={'id':'arrive','x':.70,'y':0.,'timeout_s':15.}
  elif self.next_test==1:g={'id':'cancel','x':1.4,'y':0.,'timeout_s':15.}
  elif self.next_test==2:g={'id':'timeout','x':1.4,'y':0.,'timeout_s':.2}
  elif self.next_test==3:
   (self.out.parent/'publisher/add_obstacle').write_text('add physical obstacle');g={'id':'obstacle','x':2.0,'y':0.,'timeout_s':12.}
  elif self.next_test==4:g={'id':'unreachable','x':8.,'y':0.,'timeout_s':5.}
  else:
   expected={'arrive':'succeeded','cancel':'canceled','timeout':'failed','obstacle':'failed','unreachable':'failed'};checks={r['id']:r['status']==expected[r['id']] for r in self.results};checks['obstacle_reason']=next((r['reason']=='obstacle_stop' for r in self.results if r['id']=='obstacle'),False)
   if self.truth.latest is not None:checks['stopped_after_cancel_failure']=abs(self.truth.latest.twist.twist.linear.x)<.04
   self.event('validation_checks',checks=checks);self.finish(all(checks.values()));return
  self.next_test+=1;self.phase_start=self.now
  self.goal_input.publish(String(data=json.dumps(g)))
 def task_tick(self):return TaskLayer.task_tick(self)
 def finish(self,success):
  if self.finished:return
  if self.active:self.end_goal('canceled','session_ended')
  self.command(0.,0.);self.finished=True
  if self.truth.rows:
   with (self.out/'trajectory.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(self.truth.rows[0]));w.writeheader();w.writerows(self.truth.rows)
  errs=[r['position_error_m'] for r in self.truth.rows];result={'success':bool(success),'localization':self.mode,'suite':self.suite,'results':self.results,'position_error_p50_m':float(np.median(errs)) if errs else None,'position_error_p95_m':float(np.percentile(errs,95)) if errs else None,'manual_interventions':0,'simulation_end_s':self.now,'nav_elapsed_s':self.now-self.nav_started if self.nav_started is not None else None}
  (self.out/'metrics.json').write_text(json.dumps(result,indent=2));(self.out.parent/'publisher/navigation_done.json').write_text(json.dumps(result));self.events.close();print('STAGE7_NAVIGATION_COMPLETE',json.dumps(result),flush=True)

for name in ['task_init','task_end','request_observation','collect_observation','observe_completed','verify']:
 setattr(Navigation,name,getattr(TaskLayer,name))

def main():
 p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--localization',choices=['oracle','estimated'],default='estimated');p.add_argument('--suite',choices=['validation','task'],default='validation');a=p.parse_args();rclpy.init();n=Navigation(json.loads(a.config.read_text()),a.out,a.localization,a.suite);start=time.monotonic()
 try:
  while rclpy.ok() and not n.finished and time.monotonic()-start<240:rclpy.spin_once(n,timeout_sec=.05)
  if not n.finished:n.event('wall_timeout');n.finish(False)
 finally:n.command(0.,0.);n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
