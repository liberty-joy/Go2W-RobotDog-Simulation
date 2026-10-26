"""Read-only localhost ROS baseline probe and optional short MCAP recording."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image, CameraInfo, JointState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

p=argparse.ArgumentParser()
p.add_argument('--out',type=Path,required=True)
p.add_argument('--seconds',type=float,default=30)
p.add_argument('--bag',action='store_true')
p.add_argument('--reliable-images',action='store_true')
a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1','This probe must stay on localhost'
rclpy.init();node=rclpy.create_node('sensor_stage0_probe')
state={};counts={};times={};sizes={};image_stamps=set();info_stamps=set();poses=[];trace=[];recording=False
def seconds(t):return t.sec+t.nanosec/1e9
def callback(key):
    def receive(msg):
        state[key]=msg
        if not recording:return
        counts[key]=counts.get(key,0)+1
        stamp=seconds(msg.clock) if key=='clock' else (seconds(msg.header.stamp) if hasattr(msg,'header') else None)
        if stamp is not None:times.setdefault(key,[]).append(stamp)
        sizes[key]=sizes.get(key,0)+len(serialize_message(msg))
        if key=='image':image_stamps.add(stamp)
        if key=='info':info_stamps.add(stamp)
        if key=='odom':
            pos=msg.pose.pose.position;q=msg.pose.pose.orientation
            poses.append((pos.x,pos.y,pos.z,1-2*(q.x*q.x+q.y*q.y)))
            trace.append({'t':stamp,'z':pos.z,'upright':poses[-1][3],
                'speed':math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y)})
    return receive
definitions=[('image',Image,'/go2w/front_camera/image_raw',QoSProfile(depth=10) if a.reliable_images else qos_profile_sensor_data),
    ('info',CameraInfo,'/go2w/front_camera/camera_info',qos_profile_sensor_data),
    ('joints',JointState,'/go2w/joint_states',qos_profile_sensor_data),
    ('odom',Odometry,'/go2w/odom',10),('clock',Clock,'/clock',10),('tf',TFMessage,'/tf',10),
    ('static',TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
subs=[node.create_subscription(typ,topic,callback(key),qos) for key,typ,topic,qos in definitions]
bag=None;bag_log=None
try:
    deadline=time.monotonic()+180
    while len(state)<7 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if len(state)<7:raise RuntimeError('Missing data: '+str(set(d[0] for d in definitions)-state.keys()))
    assert node.count_publishers('/clock')==1 and node.count_publishers('/go2w/odom')==1,'Multiple simulator publishers'
    warm=time.monotonic()+1
    while time.monotonic()<warm:rclpy.spin_once(node,timeout_sec=.05)
    graph={topic:list(types) for topic,types in node.get_topic_names_and_types()}
    (a.out/'topics.json').write_text(json.dumps(graph,indent=2))
    (a.out/'READY').write_text('All 7 streams ready; single simulator publisher\n')
    if a.bag:
        qos=a.out/'bag_qos.yaml'
        qos.write_text('/go2w/front_camera/image_raw:\n  reliability: best_effort\n  history: keep_last\n  depth: 10\n/go2w/front_camera/camera_info:\n  reliability: best_effort\n  history: keep_last\n  depth: 10\n/go2w/joint_states:\n  reliability: best_effort\n  history: keep_last\n  depth: 10\n/tf_static:\n  durability: transient_local\n  reliability: reliable\n  history: keep_last\n  depth: 1\n')
        if a.reliable_images:qos.write_text(qos.read_text().replace('reliability: best_effort','reliability: reliable',1))
        cmd=['/opt/ros/jazzy/bin/ros2','bag','record','--storage','mcap','--output',str(a.out/'bag'),
            '--use-sim-time','--disable-keyboard-controls','--qos-profile-overrides-path',str(qos),
            '--topics',*[d[2] for d in definitions]]
        (a.out/'bag_command.json').write_text(json.dumps(cmd,indent=2))
        bag_log=(a.out/'bag_record.log').open('w')
        bag=subprocess.Popen(cmd,stdout=bag_log,stderr=subprocess.STDOUT)
    recording=True;wall_start=time.monotonic();sim_start=seconds(state['clock'].clock)
    while time.monotonic()-wall_start<a.seconds:rclpy.spin_once(node,timeout_sec=.02)
    elapsed=time.monotonic()-wall_start;recording=False
    if bag:
        bag.send_signal(signal.SIGINT)
        bag.wait(timeout=30)
        assert bag.returncode==0,f'Bag recorder exit {bag.returncode}'
        info=subprocess.run(['/opt/ros/jazzy/bin/ros2','bag','info',str(a.out/'bag')],capture_output=True,text=True,check=True)
        (a.out/'bag_info.txt').write_text(info.stdout+info.stderr)
    img=state['image'];info=state['info']
    rgb=np.frombuffer(img.data,dtype=np.uint8).reshape(img.height,img.step)[:,:img.width*3].reshape(img.height,img.width,3)
    PILImage.fromarray(rgb).save(a.out/'rgb.png')
    for key in ['info','static','tf','odom']:
        (a.out/f'{key}.json').write_text(json.dumps(message_to_ordereddict(state[key]),indent=2))
    edges={(t.header.frame_id,t.child_frame_id):t for group in ['tf','static'] for t in state[group].transforms}
    required=[('odom','base_link'),('base_link','front_camera_link'),('front_camera_link','front_camera_optical_frame')]
    optical=edges[required[2]].transform.rotation
    q=np.array([optical.w,optical.x,optical.y,optical.z]);w,x,y,z=q
    optical_z=np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
    xyz=edges[required[1]].transform.translation
    checks={'single_robot_publishers':node.count_publishers('/go2w/odom')==1,
        'rgb_640x480':rgb.shape==(480,640,3) and img.encoding=='rgb8',
        'rgb_nonuniform':float(rgb.std())>5,
        'camera_info':info.width==640 and info.height==480 and abs(info.k[0]-640/(2*math.tan(math.radians(40))))<1,
        'image_info_stamps':len(image_stamps)>10 and image_stamps<=info_stamps,
        'tf_chain':all(key in edges for key in required),
        'static_mount':np.allclose([xyz.x,xyz.y,xyz.z],[.32,0,.08],atol=1e-6),
        'optical_axes':np.allclose(optical_z,[1,0,0],atol=1e-6),
        'clock_monotonic':len(times['clock'])>10 and all(b>a for a,b in zip(times['clock'],times['clock'][1:])),
        'stable_standing':min(row[2] for row in poses)>.3 and min(row[3] for row in poses)>.98,
        'finite_state':bool(np.isfinite(poses).all()),'joints_16':len(state['joints'].name)==16}
    sim_elapsed=seconds(state['clock'].clock)-sim_start
    rates={key:{'messages':count,'wall_hz':count/elapsed,'serialized_bytes':sizes[key],
        'sim_hz':count/sim_elapsed if sim_elapsed else None} for key,count in counts.items()}
    bag_bytes=sum(f.stat().st_size for f in (a.out/'bag').rglob('*') if f.is_file()) if a.bag else None
    result={'checks':{k:bool(v) for k,v in checks.items()},'all_passed':all(checks.values()),
        'wall_seconds':elapsed,'sim_seconds':sim_elapsed,'real_time_factor':sim_elapsed/elapsed,
        'rates':rates,'bag_bytes':bag_bytes,'bag_bytes_per_wall_second':bag_bytes/elapsed if bag_bytes else None,
        'min_height':min(row[2] for row in poses),'min_upright':min(row[3] for row in poses),
        'max_standing_speed':max(row['speed'] for row in trace),
        'ros_domain_id':os.environ['ROS_DOMAIN_ID'],'gui_human_interaction':'NOT_TESTED',
        'rgb_qos':'reliable' if a.reliable_images else 'best_effort',
        'limitations':['Image headers are publication simulation time, not verified exposure time.',
            'Odometry and dynamic TF use simulator ground truth; no SLAM/odometry estimator.',
            'One RGB camera only; no stereo depth or lidar measurements.']}
    (a.out/'probe.json').write_text(json.dumps(result,indent=2))
    (a.out/'stability.json').write_text(json.dumps(trace))
    print('STAGE0_PROBE',json.dumps(result),flush=True)
    assert result['all_passed'],checks
finally:
    if bag and bag.poll() is None:
        bag.send_signal(signal.SIGINT);bag.wait(timeout=30)
    if bag_log:bag_log.close()
    node.destroy_node();rclpy.shutdown()
