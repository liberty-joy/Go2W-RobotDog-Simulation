"""Adversarial execution gates and startup response serialization regression."""
import json,argparse
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from rclpy.serialization import serialize_message
from nav_msgs.srv import GetPlan
from stage7_geometry import KnownMap
from stage7_tasks import TaskLayer
from stage7_navigation import Navigation,posemsg
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();cfg=json.loads(Path('configs/sensors/stage7_closed_loop.json').read_text());m=KnownMap(cfg['navigation']);checks={}
response=Navigation.plan_cb(SimpleNamespace(clock_stamp=None,pose=None),GetPlan.Request(),GetPlan.Response());checks['unready_getplan_safe_empty_serializable']=len(response.plan.poses)==0 and len(serialize_message(response))>0
checks['missing_pose_stamp_safe']=len(serialize_message(posemsg(0.,0.,0.,None)))>0
n=SimpleNamespace(task_canceled=False,cfg=cfg,now=5.,pose=np.array([0.,0.,0.]),map=m)
e={'object_id':'equipment_red_01','object_exists':True,'stamp_ns':5000000000,'appearance_pass':True,'unknown':False,'conflict':False,'object_map_xy':[2.4,0.]}
checks['valid_evidence_reachable']=TaskLayer.verify(n,e)[0]
for key,change in [('missing_object',{'object_exists':False}),('stale',{'stamp_ns':1000000000}),('conflict',{'conflict':True}),('unknown',{'unknown':True}),('untrusted_object_id',{'object_id':'motor_command'}),('appearance_unconfirmed',{'appearance_pass':False}),('nonfinite_geometry',{'object_map_xy':[float('nan'),0]}),('unreachable',{'object_map_xy':[9.,0.]})]:checks[key+'_rejected']=not TaskLayer.verify(n,{**e,**change})[0]
n.task_canceled=True;checks['canceled_task_rejected']=not TaskLayer.verify(n,e)[0]
checks['known_obstacle_goal_rejected']=m.plan([0,0],[-.8,.95]) is None
result={'checks':checks,'all_pass':all(checks.values()),'scope':'adversarial deterministic execution and serialization; actual navigation validated separately'};a.out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));assert all(checks.values())
