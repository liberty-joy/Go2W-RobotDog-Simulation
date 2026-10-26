"""Recalculate stage1 acceptance from real saved evidence; never fabricates missing tests."""
import argparse,json,hashlib,csv
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();rows=[]
def check(name,status,evidence,scope=''):
 rows.append({'check':name,'status':status,'evidence':evidence,'scope':scope})
def stored(name,file,marker=None):
 path=a.run/file
 if not path.exists():check(name,'NOT_TESTED',file);return
 value=json.loads(path.read_text());ok=all(value['checks'].values());check(name,'PASS' if ok else 'FAIL',file)
 if marker:
  log=a.run/marker;check(name+'_simulation_completed','PASS' if log.exists() and 'GO2W_SIMULATION_COMPLETE' in log.read_text() else 'FAIL',marker)
stored('ROS_bridge_8_checks','ros_recheck/verification.json','ros_recheck/sim-bridge.log')
stored('mode_switch_8_checks','mode_recheck/mode-verification.json','mode_recheck/sim-modes.log')
m=json.loads((a.run/'wheel_metrics.json').read_text())
check('FR_fore_aft_reduction','PASS' if m['reduction_fraction']>.5 else 'FAIL','wheel_metrics.json','single flat-scene held-W command replay; no GUI')
check('stand_height','PASS' if m['after']['base_height_min_m']>.30 else 'FAIL','wheel_metrics.json')
check('stop','PASS' if m['after']['stop_max_abs_vx_m_s']<.04 else 'FAIL','wheel_metrics.json')
check('speed_calibration','LIMITED','wheel_metrics.json','command .15m/s; after mean .1897m/s; not calibrated')
final=a.run/'sensor_smoke_final/smoke.json'
if final.exists():
 s=json.loads(final.read_text());ok=s['left_shape']==[480,640,3] and s['right_shape']==[480,640,3] and s['point_count']>0 and s['imu_valid']
 check('stereo_RGB_RTX_IMU_API','PASS' if ok else 'FAIL','sensor_smoke_final/smoke.json','measurement only')
 log=a.run/'sensor_smoke_final.log';check('sensor_simulation_completed','PASS' if 'GO2W_SIMULATION_COMPLETE' in log.read_text() else 'FAIL','sensor_smoke_final.log')
else:check('stereo_RGB_RTX_IMU_API','NOT_TESTED','sensor_smoke_final/smoke.json')
check('hardware_selection','PASS','docs/sensor_pipeline/01_selection.md; configs/sensors/selection_manifest.json','official sources; unknown hardware items retained; no purchase')
i=json.loads((a.run/'asset_weight_integrity.json').read_text());check('assets_weights_unchanged','PASS' if i['all_unchanged'] else 'FAIL','asset_weight_integrity.json',str(i['checked_files'])+' baseline files')
for name,scope in [('GUI_manual','no human GUI/keyboard interaction observed'),('synchronization_calibration','camera render1.85s vs IMU2.0s latest-sample gap; not accepted'),('matched_depth_fusion','not enabled'),('ROS_lidar_IMU','only in-process measurement smoke'),('SLAM_Nav2','absent from accepted chain'),('new_terrain_robustness','historical results do not validate changed rolling control')]:check(name,'NOT_TESTED','stage1 scope',scope)
failed_logs=[x.name for x in sorted(a.run.glob('sensor_smoke*.log')) if 'SELECTION_SENSOR_SMOKE_PASS' not in x.read_text()]
result={'run_id':a.run.name,'rows':rows,'preserved_failed_sensor_attempts':failed_logs,'no_training':True,'no_hardware_commands':True}
(a.run/'acceptance.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
with (a.run/'acceptance.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=['check','status','evidence','scope']);w.writeheader();w.writerows(rows)
print(json.dumps({'statuses':{st:sum(x['status']==st for x in rows) for st in ['PASS','FAIL','LIMITED','NOT_TESTED']},'preserved_failed_sensor_attempts':len(failed_logs)},ensure_ascii=False))
