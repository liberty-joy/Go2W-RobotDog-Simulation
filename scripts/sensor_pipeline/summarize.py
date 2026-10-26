"""Recalculate stage-0 acceptance tables/figures and capacity estimates offline."""
import argparse
import csv
import json
from pathlib import Path
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
run=a.run.resolve();rows=[];details={}
for name in ['headless_sample_retry1','headless_reliable','gui_capture']:
    path=run/name/'probe.json'
    if not path.exists():continue
    d=json.loads(path.read_text());details[name]=d
    for check,passed in d['checks'].items():rows.append({'test':name,'check':check,'status':'PASS' if passed else 'FAIL','evidence':str(path.relative_to(run))})
    meta=run/name/'bag/metadata.yaml'
    if meta.exists():
        m=yaml.safe_load(meta.read_text())['rosbag2_bagfile_information']
        d['bag_duration_sim_seconds']=m['duration']['nanoseconds']/1e9
        d['bag_topic_counts']={t['topic_metadata']['name']:t['message_count'] for t in m['topics_with_message_count']}
for candidate in ['headless_recheck','headless']:
    path=run/candidate/'verification.json'
    if path.exists():
        for check,passed in json.loads(path.read_text())['checks'].items():rows.append({'test':'existing_bridge_'+candidate,'check':check,'status':'PASS' if passed else 'FAIL','evidence':str(path.relative_to(run))})
        if candidate=='headless_recheck':break
rows.append({'test':'gui_human_interaction','check':'keyboard_focus_and_manual_navigation','status':'NOT_TESTED','evidence':'No native GUI observation/control performed'})
for test,path in [('initial_online_ground_failure','failed_initial/sim-bridge.log'),
        ('completion_marker_added_during_attempt','headless/runner.log'),
        ('native_setup_nounset_failure','headless_sample_runner.log')]:
    evidence=run/path
    if evidence.exists() and any(token in evidence.read_text(errors='replace') for token in ['RuntimeError','completion marker','unbound variable']):
        rows.append({'test':test,'check':'failure_retained_then_repaired','status':'FAILED_REPAIRED','evidence':path})
if 'headless_sample_retry1' in details:
    rows.append({'test':'default_best_effort','check':'image_reception_below_info_rate','status':'LIMITED','evidence':'headless_sample_retry1/probe.json; opt-in reliable mode measured separately'})
with (run/'acceptance.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=['test','check','status','evidence']);writer.writeheader();writer.writerows(rows)
GiB=2**30
scenarios=[]
for name,fps,streams,lidar,depth in [('RGB_15Hz',15,1,0,False),('RGB_30Hz',30,1,0,False),
        ('A_RGB10_Lidar_assumed',10,1,100000,False),('B_stereo10_Lidar_assumed',10,2,100000,False),
        ('B_plus_float32_depth10',10,2,100000,True)]:
    bps=640*480*3*fps*streams+lidar*24+(640*480*4*fps if depth else 0)
    scenarios.append({'scenario':name,'fps':fps,'image_streams':streams,'lidar_points_per_second_assumed':lidar,
        'lidar_bytes_per_point_assumed':24,'float32_depth_saved':depth,'raw_bytes_per_second':bps,
        'GiB_per_hour':bps*3600/GiB,'GiB_per_30_seconds':bps*30/GiB,'planning_GiB_per_hour_with_25pct_margin':bps*3600/GiB*1.25})
with (run/'storage_budget.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=list(scenarios[0]));writer.writeheader();writer.writerows(scenarios)
resources={}
for name in details:
    log=run/name/'resources.log'
    gpu=[];rss=[]
    if log.exists():
        for line in log.read_text().splitlines():
            if ',' in line:
                try:gpu.append(float(line.split(',')[0]))
                except ValueError:pass
            else:
                parts=line.split()
                if len(parts)==2:
                    try:rss.append(float(parts[0])/1024)
                    except ValueError:pass
    resources[name]={'peak_total_gpu_MiB':max(gpu) if gpu else None,'peak_sim_RSS_MiB':max(rss) if rss else None}
result={'run_id':run.name,'measurements':details,'resources':resources,'theoretical_budgets':scenarios,
    'assumptions':['Storage scenarios are uncompressed estimates, not sensor product specifications.',
        'Future lidar: hypothetical 100k points/s, 24 bytes/point; not implemented.',
        'A/B share public RGB, localization/navigation/semantics; geometric ground truth is evaluation-only.',
        'Current header pairing does not prove exposure synchronization.'],
    'GUI':'Automated GUI process/render/ROS check only; manual interaction NOT_TESTED',
    'missing':['stereo pair','matched depth','rotating lidar measurements','SLAM/localization estimator','Nav2 autonomous navigation','fixed semantic model']}
(run/'summary.json').write_text(json.dumps(result,indent=2))
fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
if 'headless_reliable' in details:
    image=plt.imread(run/'headless_reliable/rgb.png');axes[0].imshow(image);axes[0].set_title('Current front RGB: 640 x 480');axes[0].axis('off')
names=list(details)
axes[1].bar(range(len(names)),[details[name]['rates']['image']['wall_hz'] for name in names],color=['#a57e4c','#328777','#547db5'][:len(names)])
axes[1].set_xticks(range(len(names)),[name.replace('headless_sample_retry1','best-effort').replace('headless_reliable','reliable').replace('gui_capture','GUI reliable') for name in names],rotation=20)
axes[1].axhline(15,color='gray',ls='--');axes[1].set(ylabel='Received RGB messages / wall second',title='Measured ROS reception')
trace_path=run/'headless_reliable/stability.json'
if trace_path.exists():
    trace=json.loads(trace_path.read_text());t0=trace[0]['t']
    axes[2].plot([r['t']-t0 for r in trace],[r['z'] for r in trace],color='#328777')
    axes[2].set(xlabel='Simulation time (s)',ylabel='Base height (m)',title='Stationary single robot',ylim=(.35,.43))
fig.suptitle(f'Stage 0 baseline | {run.name}\nNo stereo, lidar, SLAM or autonomous navigation claim',fontsize=11)
fig.savefig(run/'stage0_summary.png',dpi=150)
print(json.dumps({'run':str(run),'acceptance_rows':len(rows),'measured_resources':resources,'status':'Offline reports regenerated'},indent=2))
