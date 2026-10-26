"""Exercise the simulation from a second DDS process, including command expiry."""
import json
import math
import time
import os
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, JointState
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

rclpy.init()
n=rclpy.create_node('go2w_integration_check')
state={}
counts={}
subscriptions=[]
def receive(key):
    def cb(msg):
        state[key]=msg
        counts[key]=counts.get(key,0)+1
    return cb
for key,typ,topic,qos in [
    ('odom',Odometry,'/go2w/odom',10),('image',Image,'/go2w/front_camera/image_raw',qos_profile_sensor_data),
    ('info',CameraInfo,'/go2w/front_camera/camera_info',qos_profile_sensor_data),
    ('joints',JointState,'/go2w/joint_states',qos_profile_sensor_data),('clock',Clock,'/clock',10),
    ('tf',TFMessage,'/tf',10),('static',TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]:
    subscriptions.append(n.create_subscription(typ,topic,receive(key),qos))
pub=n.create_publisher(Twist,'/go2w/cmd_vel',10)
def spin_until(deadline,command=None):
    while time.monotonic()<deadline:
        if command is not None:pub.publish(command)
        rclpy.spin_once(n,timeout_sec=.02)
try:
    deadline=time.monotonic()+180
    while len(state)<7 and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
    if len(state)<7:raise RuntimeError('Missing topics: '+str(set(['odom','image','info','joints','clock','tf','static'])-state.keys()))
    if os.environ.get('ROBOTDOG_REQUIRE_SINGLE_SIM')=='1':
        assert n.count_publishers('/go2w/odom')==1 and n.count_publishers('/clock')==1,'Ambiguous simulator publishers'
    spin_until(time.monotonic()+1.)
    start=state['odom'].pose.pose.position
    start=(start.x,start.y)
    first_clock=state['clock'].clock.sec+state['clock'].clock.nanosec/1e9
    wall_start=time.monotonic()
    command=Twist();command.linear.x=.12
    spin_until(time.monotonic()+3.,command)
    # Stop publishing without a zero message: the simulator watchdog must stop it.
    spin_until(time.monotonic()+1.5)
    odom=state['odom'];img=state['image'];info=state['info'];joints=state['joints']
    delta=math.hypot(odom.pose.pose.position.x-start[0],odom.pose.pose.position.y-start[1])
    speed=math.hypot(odom.twist.twist.linear.x,odom.twist.twist.linear.y)
    checks={
        'external_cmd_moves_robot':delta>.05,
        'expired_command_stops_robot':speed<.04,
        'image_rgb8':img.width==640 and img.height==480 and img.encoding=='rgb8' and len(img.data)==640*480*3,
        'intrinsics':abs(info.k[0]-640/(2*math.tan(math.radians(40))))<1.,
        'joint_state':len(joints.name)==16 and len(joints.position)==16,
        'tf':state['tf'].transforms[0].header.frame_id=='odom',
        'optical_tf':len(state['static'].transforms)==2 and state['static'].transforms[-1].child_frame_id=='front_camera_optical_frame',
        'clock_advances':state['clock'].clock.sec+state['clock'].clock.nanosec/1e9>first_clock,
    }
    checks={key:bool(value) for key,value in checks.items()}
    result={'checks':checks,'counts':counts,'travel_m':delta,'speed_after_timeout_m_s':speed,'sim_elapsed_s':state['clock'].clock.sec+state['clock'].clock.nanosec/1e9-first_clock,'wall_elapsed_s':time.monotonic()-wall_start}
    out=Path(os.environ.get('ROBOTDOG_OUTPUT_DIR',str(Path(__file__).resolve().parents[2]/'outputs/ros2')))
    out.mkdir(parents=True,exist_ok=True)
    dest=out/'verification.json'
    if os.environ.get('ROBOTDOG_OUTPUT_DIR'):
        import numpy as np
        from PIL import Image as PILImage
        pixels=np.frombuffer(img.data,dtype=np.uint8).reshape(img.height,img.step)[:,:img.width*3].reshape(img.height,img.width,3)
        PILImage.fromarray(pixels).save(out/'rgb.png')
    dest.write_text(json.dumps(result,indent=2))
    print('ROS2_INTEGRATION',json.dumps(result),flush=True)
    assert all(checks.values()),checks
    print('ROS2_INTEGRATION_PASS',flush=True)
finally:
    pub.publish(Twist())
    n.destroy_node();rclpy.shutdown()
