"""Native Jazzy recorder/verifier. Bag receipt times and sensor exposure times stay distinct."""
import argparse,hashlib,json,time,subprocess,os,struct,array
import numpy as np
from pathlib import Path
import rclpy,rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.serialization import serialize_message,deserialize_message
from rosidl_runtime_py.utilities import get_message
TOPICS={'/clock':'rosgraph_msgs/msg/Clock','/tf_static':'tf2_msgs/msg/TFMessage',
 '/go2w/lidar/points':'sensor_msgs/msg/PointCloud2','/go2w/imu/data':'sensor_msgs/msg/Imu',
 '/go2w/sensors/scene':'std_msgs/msg/String','/evaluation/go2w/poses':'std_msgs/msg/String',
 '/evaluation/tf':'tf2_msgs/msg/TFMessage','/evaluation/go2w/odom':'nav_msgs/msg/Odometry'}
for prefix in ['/go2w/front_camera','/go2w/stereo/left','/go2w/stereo/right','/go2w/stereo/depth']:
 TOPICS[prefix+'/image_raw']='sensor_msgs/msg/Image';TOPICS[prefix+'/camera_info']='sensor_msgs/msg/CameraInfo'
def ns(s):return s.sec*1000000000+s.nanosec
def stamp(msg):
 if hasattr(msg,'header'):return ns(msg.header.stamp)
 if hasattr(msg,'clock'):return ns(msg.clock)
 if hasattr(msg,'transforms'):return ns(msg.transforms[0].header.stamp) if msg.transforms else 0
 try:
  d=json.loads(msg.data);return d.get('stamp_ns',int(d.get('simulation_time_s',0)*1e9))
 except Exception:return 0
def canonical_update(h,v):
 # CDR alignment padding is unspecified. Compare typed fields and raw payloads.
 if hasattr(v,'get_fields_and_field_types'):
  for field in v.get_fields_and_field_types():h.update(field.encode()+b'\0');canonical_update(h,getattr(v,field))
 elif isinstance(v,(np.ndarray,array.array)):
  a=np.asarray(v);h.update(str(a.dtype).encode());h.update(struct.pack('<Q',a.size));h.update(a.tobytes())
 elif isinstance(v,(bytes,bytearray)):h.update(v)
 elif isinstance(v,str):h.update(struct.pack('<Q',len(v.encode())));h.update(v.encode())
 elif isinstance(v,(list,tuple)):
  h.update(struct.pack('<Q',len(v)))
  for item in v:canonical_update(h,item)
 elif isinstance(v,(float,np.floating)):h.update(struct.pack('<d',float(v)))
 elif isinstance(v,(int,bool,np.integer)):h.update(int(v).to_bytes(16,'little',signed=True))
 else:raise TypeError(type(v))
def bag_digests(bag):
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'));raw={};semantic={};counts={};types={t:get_message(typ) for t,typ in TOPICS.items()}
 while reader.has_next():
  t,b,_=reader.read_next();raw.setdefault(t,hashlib.sha256()).update(b);canonical_update(semantic.setdefault(t,hashlib.sha256()),deserialize_message(b,types[t]));counts[t]=counts.get(t,0)+1
 return {t:{'sha256_stream':raw[t].hexdigest(),'canonical_sha256':semantic[t].hexdigest(),'count':counts[t]} for t in raw}
class Probe(Node):
 def __init__(self,out,writer=None):
  super().__init__('stage6_bag_probe');self.out=out;self.writer=writer;self.counts={};self.hashes={};self.canonical={};self.bytes={};self.stamps={};self.last={};self.backward={};self.clock_ns=-1;self.resets=0;self.sync_queue={};self.first_wall=None;self.last_wall=None;self.max_clock_publishers=0
  self.events=(out/'events.jsonl').open('w');self.subs=[]
  for topic,typ in TOPICS.items():
   q=QoSProfile(depth=100)
   if topic=='/tf_static':q.durability=DurabilityPolicy.TRANSIENT_LOCAL
   self.subs.append(self.create_subscription(get_message(typ),topic,lambda m,t=topic:self.cb(t,m),q))
 def cb(self,t,m):
  now=time.time_ns();data=serialize_message(m);s=stamp(m)
  if self.writer:self.writer.write(t,data,now)
  self.counts[t]=self.counts.get(t,0)+1;self.bytes[t]=self.bytes.get(t,0)+len(data)
  self.hashes.setdefault(t,hashlib.sha256()).update(data);canonical_update(self.canonical.setdefault(t,hashlib.sha256()),m)
  if t in self.last and s<self.last[t]:self.backward[t]=self.backward.get(t,0)+1
  self.last[t]=s;self.stamps.setdefault(t,[]).append(s)
  if t=='/clock':
   if s<self.clock_ns:self.sync_queue.clear();self.resets+=1
   self.clock_ns=s
  elif t.endswith('/image_raw'):self.sync_queue[t]=s
  self.events.write(json.dumps({'topic':t,'stamp_ns':s,'receipt_ns':now,'bytes':len(data),'generation':self.resets})+'\n')
  self.first_wall=self.first_wall or now;self.last_wall=now
 def result(self):
  duration=(self.last_wall-self.first_wall)*1e-9 if self.first_wall else 0
  topics={}
  for t,n in self.counts.items():
   st=self.stamps[t];span=(max(st)-min(st))*1e-9
   topics[t]={'count':n,'serialized_bytes':self.bytes[t],'sha256_stream':self.hashes[t].hexdigest(),'canonical_sha256':self.canonical[t].hexdigest(),'wall_hz':n/max(duration,1e-9),'stamp_hz':(n-1)/span if span else None,'first_stamp_ns':st[0],'last_stamp_ns':st[-1],'backward_stamps':self.backward.get(t,0)}
  return {'duration_wall_s':duration,'topics':topics,'clock_resets':self.resets,'max_clock_publishers':self.max_clock_publishers,'queue_clear_on_clock_reset':True}
def main():
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['record','replay']);p.add_argument('--out',type=Path,required=True);p.add_argument('--publisher-output',type=Path);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
 rclpy.init();writer=None
 if a.mode=='record':
  writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(a.out/'bag'),storage_id='mcap',storage_preset_profile='zstd_fast'),rosbag2_py.ConverterOptions('cdr','cdr'))
  for t,typ in TOPICS.items():writer.create_topic(rosbag2_py.TopicMetadata(id=0,name=t,type=typ,serialization_format='cdr'))
 probe=Probe(a.out,writer);start=time.monotonic();(a.out/'ready').write_text('ready');player=None;full=None;ended=None
 if a.mode=='replay':
  qos={t:{'reliability':'reliable','durability':'transient_local' if t=='/tf_static' else 'volatile','history':'keep_last','depth':100} for t in TOPICS}
  import yaml
  qp=a.out/'qos.yaml';qp.write_text(yaml.safe_dump(qos))
  log=(a.out/'player.log').open('w')
  def play(short=False):
   cmd=['ros2','bag','play',str(a.out.parent/'record'/'bag'),'--disable-keyboard-controls','--delay','1','--qos-profile-overrides-path',str(qp)]
   if short:cmd+=['--playback-duration','2']
   return subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
  player=play()
 try:
  while time.monotonic()-start<240:
   rclpy.spin_once(probe,timeout_sec=.02)
   probe.max_clock_publishers=max(probe.max_clock_publishers,probe.count_publishers('/clock'))
   if a.mode=='record':
    done=a.publisher_output/'publisher_summary.json'
    if done.exists():
     ended=ended or time.monotonic()
     if time.monotonic()-ended>2:break
   elif player.poll() is not None:
    ended=ended or time.monotonic()
    if time.monotonic()-ended>2:
     if full is None:
      full=probe.result();(a.out/'full_replay_metrics.json').write_text(json.dumps(full,indent=2));player=play(True);ended=None
     else:break
  else:raise TimeoutError('record/replay deadline exceeded')
 finally:
  if player and player.poll() is None:player.terminate();player.wait(timeout=10)
  result=probe.result();(a.out/'metrics.json').write_text(json.dumps(result,indent=2));probe.events.close();probe.destroy_node();rclpy.shutdown()
  if writer:writer.close()
 print(json.dumps({'mode':a.mode,'duration':result['duration_wall_s'],'counts':{k:v['count'] for k,v in result['topics'].items()},'clock_resets':result['clock_resets']}),flush=True)
 if a.mode=='replay':
  original=json.loads((a.out.parent/'record'/'metrics.json').read_text());stored=bag_digests(a.out.parent/'record'/'bag');checks={t:full['topics'].get(t,{}).get('canonical_sha256')==stored[t]['canonical_sha256'] and full['topics'].get(t,{}).get('count')==v['count'] for t,v in original['topics'].items()};persistence={t:stored[t]['sha256_stream']==v['sha256_stream'] and stored[t]['count']==v['count'] for t,v in original['topics'].items()}
  verification={'all_topics_exact':all(checks.values()),'bag_matches_original_cdr':all(persistence.values()),'persistence_per_topic':persistence,'comparison':'canonical typed fields and exact byte arrays; excludes unspecified CDR alignment padding','per_topic':checks,'one_clock_source':result['max_clock_publishers']==1,'reset_cleared_queues':result['clock_resets']>=1,'player_clock_generation':False}
  (a.out/'verification.json').write_text(json.dumps(verification,indent=2))
  if not all([verification['all_topics_exact'],verification['bag_matches_original_cdr'],verification['one_clock_source'],verification['reset_cleared_queues']]):raise RuntimeError('Replay verification failed; inspect verification.json')
if __name__=='__main__':main()
