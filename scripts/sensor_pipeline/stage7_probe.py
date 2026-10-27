"""Independent read-only ROS process checks real navigation interfaces."""
import argparse,time,json
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Odometry,Path as RosPath
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PolygonStamped
from tf2_msgs.msg import TFMessage
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();rclpy.init();n=rclpy.create_node('stage7_readonly_interface_probe');state={};tf_pairs=set();subs=[]
for k,t,topic,q in [('map',OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),('costmap',OccupancyGrid,'/go2w/navigation/costmap',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),('odom',Odometry,'/go2w/navigation/odom',20),('footprint',PolygonStamped,'/go2w/navigation/footprint',20),('path',RosPath,'/go2w/navigation/path',20)]:subs.append(n.create_subscription(t,topic,lambda m,key=k:state.__setitem__(key,m),q))
subs.append(n.create_subscription(TFMessage,'/tf',lambda m:tf_pairs.update((x.header.frame_id,x.child_frame_id) for x in m.transforms),50));client=n.create_client(GetPlan,'/go2w/navigation/get_plan');start=time.monotonic();future=None
try:
 while time.monotonic()-start<90:
  rclpy.spin_once(n,timeout_sec=.05)
  if future is None and 'map' in state and 'odom' in state and client.service_is_ready():
   req=GetPlan.Request();req.start.header.frame_id=req.goal.header.frame_id='map';req.start.pose.orientation.w=req.goal.pose.orientation.w=1.;req.goal.pose.position.x=.7;future=client.call_async(req)
  if len(state)==5 and future is not None and future.done():break
 checks={'map_received':'map' in state,'costmap_received':'costmap' in state,'path_received':'path' in state,'footprint_received':'footprint' in state,'estimated_odom_received':'odom' in state,'tf_map_odom_base':{('map','odom'),('odom','base_link')}<=tf_pairs,'get_plan_service':future is not None and future.done() and len(future.result().plan.poses)>2,'single_clock':n.count_publishers('/clock')==1,'single_navigation_odom':n.count_publishers('/go2w/navigation/odom')==1}
 result={'checks':checks,'tf_pairs':sorted(tf_pairs),'goal_pose_publishers':n.count_publishers('/goal_pose'),'map_size':[state['map'].info.width,state['map'].info.height] if 'map' in state else None,'footprint_vertices':len(state['footprint'].polygon.points) if 'footprint' in state else None,'result':'PASS' if all(checks.values()) else 'FAIL','scope':'read-only independent ROS process; GetPlan request does not command motion'};a.out.write_text(json.dumps(result,indent=2));print(json.dumps(result));assert all(checks.values())
finally:n.destroy_node();rclpy.shutdown()
