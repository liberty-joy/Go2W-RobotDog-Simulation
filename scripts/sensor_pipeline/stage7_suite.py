"""Run frozen 2x2 cells as separate real Isaac simulations, preserving every run."""
import argparse,json,subprocess,hashlib,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--vision-io',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False);protocol=json.loads(Path('configs/sensors/stage7_protocol.json').read_text());(a.out/'protocol.json').write_text(json.dumps(protocol,indent=2));base=json.loads(Path('configs/sensors/stage7_closed_loop.json').read_text());entries=[]
files=[Path('scripts/go2w_teleop.py'),Path('scripts/go2w_ros2.py')]+[Path('scripts/sensor_pipeline')/n for n in ['stage6_ros_sensors.py','stage5_fusion_capture.py','stage4_lidar_smoke.py','process_stage3_stereo.py','fusion_geometry.py','analyze_stage5_fusion.py']]+sorted(Path('scripts/sensor_pipeline').glob('stage7_*.py'))+[Path('configs/sensors/stage7_protocol.json'),Path('configs/sensors/stage7_closed_loop.json'),Path('configs/models/stage7_squeezenet_SOURCE.json')]
(a.out/'frozen_sources.json').write_text(json.dumps([{'path':str(f),'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in files],indent=2))
for condition in protocol['conditions']:
 for group in protocol['groups']:
  name=condition['condition']+'_'+group['sensor_mode']+'_'+group['planner'];config=json.loads(json.dumps(base));config['scenes'][0].update({k:condition[k] for k in ['scene_id','object_id','seed','center_world_m','texture']});config['scenes'][0]['split']='fixed_task_regression';config['experiment']={**group,'vision_model':'squeezenet','vision_io':str(a.vision_io.resolve()),'condition':condition['condition'],'delay_s':condition['delay_s'],'seed':condition['seed']};cp=a.out/(name+'_config.json');cp.write_text(json.dumps(config,indent=2));run=a.out/name
  start=time.time();print('START',name,flush=True)
  with (a.out/(name+'_launch.log')).open('w') as log:ret=subprocess.run(['scripts/run_stage7.sh',str(run),'estimated','task',str(cp)],stdout=log,stderr=subprocess.STDOUT).returncode
  entry={'run':name,'returncode':ret,'wall_s':time.time()-start,'condition':condition['condition'],**group,'seed':condition['seed']};entries.append(entry);(a.out/'launches.json').write_text(json.dumps(entries,indent=2));print('END',json.dumps(entry),flush=True)
(a.out/'complete.json').write_text(json.dumps({'runs':len(entries),'all_launches_completed':all(e['returncode']==0 for e in entries),'not_all_tasks_expected_to_succeed':True},indent=2))
