"""Verify the stationary mode-switch interlock through a second ROS process."""
import json
import os
import time
import math
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

rclpy.init()
node=rclpy.create_node('go2w_mode_check')
state={}
node.create_subscription(String,'/go2w/control_mode',lambda msg:state.update(mode=msg.data),10)
node.create_subscription(Odometry,'/go2w/odom',lambda msg:state.update(odom=msg),10)
velocity=node.create_publisher(Twist,'/go2w/cmd_vel',10)
mode=node.create_publisher(String,'/go2w/control_mode/cmd',10)

def spin(seconds,v=0.):
    command=Twist();command.linear.x=v
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        velocity.publish(command)
        rclpy.spin_once(node,timeout_sec=.02)

try:
    deadline=time.monotonic()+180
    while len(state)<2 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    assert len(state)==2,'Missing mode/odometry topics'
    spin(1.,.1)
    mode.publish(String(data='walk'))
    spin(.5,.1)
    checks={'moving_request_rejected':state['mode']=='wheel'}
    spin(2.)
    mode.publish(String(data='walk'))
    spin(3.)
    checks['stationary_walk_accepted']=state['mode']=='walk'
    checks['walking_stand_stable']=state['odom'].pose.pose.position.z>.30
    start=state['odom'].pose.pose.position
    start=(start.x,start.y)
    initial_q=state['odom'].pose.pose.orientation
    heading=math.atan2(2*(initial_q.w*initial_q.z+initial_q.x*initial_q.y),1-2*(initial_q.y**2+initial_q.z**2))
    spin(3.,.08)
    end=state['odom'].pose.pose.position
    travel=math.hypot(end.x-start[0],end.y-start[1])
    forward=(end.x-start[0])*math.cos(heading)+(end.y-start[1])*math.sin(heading)
    q=state['odom'].pose.pose.orientation
    checks['walk_command_moves_robot']=forward>.08
    checks['walking_upright']=state['odom'].pose.pose.position.z>.30 and 1-2*(q.x*q.x+q.y*q.y)>.95
    spin(2.)
    t=state['odom'].twist.twist.linear
    checks['walking_stops']=math.hypot(t.x,t.y)<.04
    mode.publish(String(data='wheel'))
    spin(2.)
    checks['stationary_wheel_accepted']=state['mode']=='wheel'
    checks['final_stand_stable']=state['odom'].pose.pose.position.z>.30
    out=Path(os.environ.get('ROBOTDOG_OUTPUT_DIR',str(Path(__file__).resolve().parents[2]/'outputs/ros2')))
    out.mkdir(parents=True,exist_ok=True)
    dest=out/'mode-verification.json'
    dest.write_text(json.dumps({'checks':checks,'walking_travel_m':travel,'walking_forward_m':forward,
        'test':'stationary transitions and straight walking at 0.08m/s; no turning/terrain coverage'},indent=2))
    print('MODE_CHECK',json.dumps(checks),flush=True)
    assert all(checks.values()),checks
    print('GO2W_MODE_CHECK_PASS',flush=True)
finally:
    velocity.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
