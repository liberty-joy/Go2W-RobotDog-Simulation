"""Finite classical matching tuning, independent validation and frozen final evaluation.
No optimizer, neural network fitting, motor adaptation or online inference service.
"""
import argparse,csv,datetime,hashlib,json,os,platform,resource,shutil,subprocess,time
from pathlib import Path
from zoneinfo import ZoneInfo
import cv2,numpy as np
from process_stage3_stereo import load_image_pair,run_matcher

ROOT=Path(__file__).resolve().parents[2]
DEV=ROOT/'outputs/sensor_pipeline/stage3_20261026T2110_clean_dev'
BASE=ROOT/'configs/sensors/stage3_sgbm.json'
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,d):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,indent=2,allow_nan=False))
def csvout(p,rows):
 if not rows:return
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def now():return datetime.datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()
def event(out,kind,**kw):
 with (out/'events.jsonl').open('a') as f:f.write(json.dumps({'event':kind,'time':now(),'epoch_ns':time.time_ns(),**kw})+'\n')
def frozen_paths():
 paths=list((ROOT/'outputs/training').rglob('*.pt'))+list((ROOT/'outputs/training').rglob('*.pth'))
 paths += list((ROOT/'models/perception/stage7_squeezenet').glob('*.pth'))
 paths += [ROOT/p for p in ['scripts/go2w_ros2.py','scripts/go2w_teleop.py','scripts/go2w_policy.py','configs/sensors/stage7_closed_loop.json','configs/sensors/stage7_protocol.json','configs/sensors/stage3_sgbm.json','sim/assets/go2w/go2w_description/go2w_description.usda','RobotDog.code-workspace']]
 return sorted(set(p for p in paths if p.is_file()))
def audit_inputs(out):
 snapshots=[]
 for p in frozen_paths():snapshots.append({'path':str(p.relative_to(ROOT)),'bytes':p.stat().st_size,'sha256':digest(p)})
 dump(out/'protected_before.json',snapshots)
 history={}
 for name,path in [('stage0','stage0_20261026T151923/summary.json'),('stage1','stage1_20261026T154717/acceptance.json'),('stage2','stage2_20261026T172917/final_acceptance.json'),('stage3','stage3_20261026T2118_clean_final/analysis/results.json'),('stage4','stage4_20261026T214509_15/capture_summary.json'),('stage5','stage5_20261026T223623_accepted/object_geometry.csv'),('stage6','stage6_20261027T_stage6_first/offline/paired_summary.csv'),('stage7','stage7_20261027_2x2_v4/group_summary.csv')]:
  p=ROOT/'outputs/sensor_pipeline'/path;history[name]={'path':str(p.relative_to(ROOT)),'exists':p.exists(),'sha256':digest(p) if p.exists() else None,'scope':'historical evidence, not rerun acceptance'}
 dump(out/'history_evidence.json',history)
 files=[Path(__file__),ROOT/'scripts/sensor_pipeline/process_stage3_stereo.py',ROOT/'scripts/sensor_pipeline/stage3_stereo_smoke.py',ROOT/'experiments/protocols/sensor_training.md']+list((ROOT/'configs/sensors').glob('stage8_*.json'))
 source=[]
 for p in files:
  q=out/'source_snapshot'/p.relative_to(ROOT);q.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,q);source.append({'path':str(p.relative_to(ROOT)),'sha256':digest(p)})
 dump(out/'source_manifest.json',source)
 (out/'dirty_before.patch').write_text(subprocess.check_output(['git','diff','--binary'],cwd=ROOT,text=True))
 dump(out/'metadata.json',{'started':now(),'git_parent':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'python':platform.python_version(),'python_path':os.sys.executable,'opencv':cv2.__version__,'numpy':np.__version__,'cpu_threads':cv2.getNumThreads(),'disk_free_before_bytes':shutil.disk_usage(out).free,'training_steps':0,'neural_inference':False,'new_weights':False,'best_last_checkpoints':'not applicable; parameter JSON only','user_dirty_file_preserved':'RobotDog.code-workspace','hardware_note':'CPU matching; fresh capture separately uses existing Isaac environment'})

def score(result,truth,record):
 z=result['depth_z_m'];g=np.isfinite(truth)&(truth>0);valid=np.isfinite(z)&(result['valid_mask']>0)&g;n=int(g.sum());ok=int(valid.sum());err=np.abs(z[valid]-truth[valid]);band=g&(abs(truth-float(record['distance_m']))<=.04);bv=band&valid;be=np.abs(z[bv]-truth[bv]);oc=g&(truth<record['distance_m']-.02)
 return {'ground_truth_pixels':n,'accepted_pixels':ok,'good10_fraction':float((err<=.1).sum()/max(n,1)),'bad20_fraction':float((err>.2).sum()/max(n,1)),'coverage':ok/max(n,1),'mae_m':float(err.mean()) if ok else None,'rmse_m':float(np.sqrt(np.mean(err**2))) if ok else None,'bad20_of_accepted':float((err>.2).mean()) if ok else None,'plane_band_pixels':int(band.sum()),'plane_band_coverage':float(bv.sum()/max(1,band.sum())),'plane_band_mae_m':float(be.mean()) if len(be) else None,'occluded_coverage':float((valid&oc).sum()/max(1,oc.sum())),'non_occluded_coverage':float((valid&~oc).sum()/max(1,(g&~oc).sum())),'invalid_zero_count':int(((result['valid_mask']==0)&(z==0)).sum()),'invalid_non_nan_count':int(((result['valid_mask']==0)&~np.isnan(z)).sum())}
def aggregate(rows):
 def mean(k):
  v=[r[k] for r in rows if r[k] is not None];return float(np.mean(v)) if v else None
 return {'scenes':len(rows),**{k:mean(k) for k in ['good10_fraction','bad20_fraction','coverage','mae_m','rmse_m','bad20_of_accepted','plane_band_coverage','plane_band_mae_m']}}
def selected_rows(run):return json.loads((run/'captures.json').read_text())['rows']
def compute(run,record,params):
 left,right=load_image_pair(run,record['scenario_id']);cal=json.loads((run/'calibration.json').read_text());result=run_matcher(left,right,cal,params)
 # Truth opened only after estimated depth is produced, never passed into run_matcher.
 truth=np.load(run/record['truth_path']/'left_optical_z_m.npy');return result,truth

def scan(out):
 out.mkdir(parents=True,exist_ok=False);assert shutil.disk_usage(out).free>8*1024**3;audit_inputs(out);base=json.loads(BASE.read_text());grid=json.loads((ROOT/'configs/sensors/stage8_scan.json').read_text())['candidates'];records=selected_rows(DEV);assert len(records)==8 and all(r['split']=='development' for r in records);start=time.monotonic();allrows=[];summaries=[]
 # Explicit small smoke before enlargement, not training iterations.
 estimate,truth=compute(DEV,records[0],base);assert np.isfinite(estimate['depth_z_m']).any();smoke=score(estimate,truth,records[0]);assert smoke['invalid_zero_count']==smoke['invalid_non_nan_count']==0;dump(out/'smoke.json',{'status':'PASS','scene':records[0]['scenario_id'],'metrics':smoke,'training_iterations':0});event(out,'smoke_pass',scene=records[0]['scenario_id'])
 input_manifest=[]
 for r in records:
  for name in ['left_rgb.png','right_rgb.png']:
   p=DEV/'captures'/r['scenario_id']/name;input_manifest.append({'scene_id':r['scenario_id'],'kind':name,'path':str(p.relative_to(ROOT)),'sha256':digest(p)})
 dump(out/'development_input_manifest.json',input_manifest)
 for candidate in grid:
  if time.monotonic()-start>180:raise TimeoutError('Development scan budget 180s exceeded; partial events retained')
  params={**base,**{k:v for k,v in candidate.items() if k!='name'}};case_rows=[];timings=[]
  for record in records:
   compute(DEV,record,params) # one uncounted warmup
   first_hash=None;repro=True;first_metrics=None
   for repeat in range(2):
    result,truth=compute(DEV,record,params);timings.append(result['elapsed_s']*1000);h=hashlib.sha256(result['depth_z_m'].tobytes()+result['valid_mask'].tobytes()).hexdigest();repro&=first_hash is None or first_hash==h;first_hash=first_hash or h
    if first_metrics is None:first_metrics=score(result,truth,record)
   row={'candidate':candidate['name'],'scene_id':record['scenario_id'],'scene_class':record['scene_class'],'distance_m':record['distance_m'],**first_metrics,'repeat_outputs_identical':repro};case_rows.append(row);allrows.append(row)
  summary={'candidate':candidate['name'],**aggregate(case_rows),'runtime_p50_ms':float(np.percentile(timings,50)),'runtime_p95_ms':float(np.percentile(timings,95)),'params':params,'repeat_outputs_identical':all(r['repeat_outputs_identical'] for r in case_rows)};summaries.append(summary);event(out,'candidate_complete',candidate=candidate['name'],good10_fraction=summary['good10_fraction'],bad20_fraction=summary['bad20_fraction'])
 baseline=next(x for x in summaries if x['candidate']=='b5_n128_u10_lr1.5');best=max(summaries,key=lambda r:(r['good10_fraction'],r['coverage'],-(r['mae_m'] or 1e9)));eligible=best['good10_fraction']>=baseline['good10_fraction']+.01 and best['bad20_fraction']<=baseline['bad20_fraction']+.01 and best['runtime_p95_ms']<=max(baseline['runtime_p95_ms']*1.5,50.)
 dump(out/'candidate_results.json',summaries);csvout(out/'candidate_results.csv',[{k:v for k,v in r.items() if k!='params'} for r in summaries]);csvout(out/'development_per_scene.csv',allrows);dump(out/'baseline_params.json',base);dump(out/'selected_params.json',best['params']);dump(out/'development_selection.json',{'selected_candidate':best['candidate'],'baseline_candidate':baseline['candidate'],'development_eligible':bool(eligible),'best':best,'baseline':baseline,'selection_time':now(),'test_data_opened':False,'development_source':str(DEV),'candidate_count':len(grid),'scan_elapsed_s':time.monotonic()-start,'mode':'parameter tuning only; no neural training'});event(out,'development_selection_frozen',selected=best['candidate'],eligible=bool(eligible))
 print(json.dumps({'candidate_count':len(grid),'best':best['candidate'],'eligible':bool(eligible),'scan_wall_s':time.monotonic()-start}),flush=True)

def evaluate(out,run,partition):
 directory=out/partition;directory.mkdir(parents=True,exist_ok=False);records=selected_rows(run);assert len(records)==8
 selection=json.loads((out/'development_selection.json').read_text())
 if partition=='final_test':
  frozen=json.loads((out/'final_selection.json').read_text());assert digest(out/'final_params.json')==frozen['final_params_sha256'];assert time.time_ns()>frozen['freeze_epoch_ns']
  assert min((run/'captures'/r['scenario_id']/'left_rgb.png').stat().st_mtime_ns for r in records)>frozen['freeze_epoch_ns'],'Test must be captured after final selection freeze'
 results=[];summaries={}
 for method,params_file in [('baseline','baseline_params.json'),('challenger','selected_params.json')]:
  params=json.loads((out/params_file).read_text());method_rows=[];lat=[]
  for record in records:
   result,truth=compute(run,record,params);lat.append(result['elapsed_s']*1000);folder=directory/method/record['scenario_id'];folder.mkdir(parents=True);np.save(folder/'estimated_z_m.npy',result['depth_z_m']);np.save(folder/'valid_mask.npy',result['valid_mask']);np.save(folder/'disparity_px.npy',result['disparity_px']);r={'partition':partition,'method':method,'scene_id':record['scenario_id'],'scene_class':record['scene_class'],'distance_m':record['distance_m'],**score(result,truth,record),'runtime_ms':lat[-1],'left_sha256':digest(run/'captures'/record['scenario_id']/'left_rgb.png'),'right_sha256':digest(run/'captures'/record['scenario_id']/'right_rgb.png'),'truth_path':str((run/record['truth_path']).relative_to(ROOT))};results.append(r);method_rows.append(r)
  summaries[method]={**aggregate(method_rows),'runtime_p50_ms':float(np.percentile(lat,50)),'runtime_p95_ms':float(np.percentile(lat,95))}
 csvout(directory/'per_scene.csv',results);dump(directory/'metrics.json',summaries);dump(directory/'capture_manifest.json',{'run':str(run),'captured_scenes':records,'config_sha256':digest(run/'capture_config.json'),'calibration_sha256':digest(run/'calibration.json'),'matcher_inputs':'left/right RGB only; truth scoring after matching','no_test_tuning':True})
 class_rows=[]
 for method in ['baseline','challenger']:
  for cls in sorted({r['scene_class'] for r in results}):class_rows.append({'method':method,'scene_class':cls,**aggregate([r for r in results if r['method']==method and r['scene_class']==cls])})
 csvout(directory/'per_class.csv',class_rows)
 if partition=='validation':
  b,c=summaries['baseline'],summaries['challenger'];passed=c['good10_fraction']>=b['good10_fraction']+.01 and c['bad20_fraction']<=b['bad20_fraction']+.01;promote=selection['development_eligible'] and passed;source='selected_params.json' if promote else 'baseline_params.json';shutil.copyfile(out/source,out/'final_params.json');freeze={'freeze_epoch_ns':time.time_ns(),'freeze_time':now(),'primary_method':'challenger' if promote else 'baseline','promoted':bool(promote),'development_eligible':selection['development_eligible'],'validation_gate_passed':bool(passed),'selection_basis':'pre-registered dev + independent validation gates; test never used','final_params_sha256':digest(out/'final_params.json'),'baseline_params_sha256':digest(out/'baseline_params.json'),'challenger_params_sha256':digest(out/'selected_params.json'),'test_methods_frozen':['baseline','challenger'],'no_neural_training':True};dump(out/'final_selection.json',freeze);event(out,'final_selection_frozen_before_test_capture',**freeze)
 event(out,'partition_evaluation_complete',partition=partition,summary=summaries);print(json.dumps({'partition':partition,'metrics':summaries}),flush=True)

def report(out):
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 summaries=json.loads((out/'candidate_results.json').read_text());selection=json.loads((out/'development_selection.json').read_text());freeze=json.loads((out/'final_selection.json').read_text());test=json.loads((out/'final_test/metrics.json').read_text());val=json.loads((out/'validation/metrics.json').read_text());protected=json.loads((out/'protected_before.json').read_text());integrity=[{**r,'unchanged':(ROOT/r['path']).is_file() and digest(ROOT/r['path'])==r['sha256']} for r in protected];dump(out/'protected_after.json',integrity);assert all(r['unchanged'] for r in integrity)
 all_metrics=[]
 for part in ['validation','final_test']:
  with (out/part/'per_scene.csv').open() as f:all_metrics+=list(csv.DictReader(f))
 assert all(int(r['invalid_zero_count'])==int(r['invalid_non_nan_count'])==0 for r in all_metrics)
 hist=json.loads((out/'history_evidence.json').read_text());decision={'decision':'当前不需要训练','neural_training_steps':0,'neural_inference':False,'new_weights_downloaded':False,'motion_policy_adaptation':False,'mode':'classical StereoSGBM parameter tuning + inference + final evaluation','reasons':['Stage7 primary failures are evidence age, not motor/perception learning failures','Stereo failure modes include missing/periodic texture and conservative conflict gates; current independent scene sample is too small to establish learned-model benefit','Parameter and independent validation evidence must precede model fitting; no representative industrial attribute dataset','Known-map navigation and ideal synthetic sensor domain remain evaluation limitations'],'primary_method':freeze['primary_method'],'promoted':freeze['promoted'],'candidate':selection['selected_candidate'],'final_evaluation':test,'validation':val,'training_fields':{'network':None,'pretrained_new_model':None,'loss':None,'optimizer':None,'best_checkpoint':None,'last_checkpoint':None},'historical_results_not_rerun':hist,'remaining_limits':['8 scenes per new partition only; no powered statistical/generalization claim','simple ideal RGB pinhole, no real IR/sensor noise','object task utility not re-tested in new closed loop; deployment not changed','no full raw Stage7 PC2 archive; dynamic deskew cannot be learned/evaluated from selected XYZ snapshots'],'peak_offline_rss_MiB':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024};dump(out/'decision.json',decision)
 fig,axs=plt.subplots(2,2,figsize=(11,8))
 for ax,(n,lr) in zip(axs.flat,[(64,1.),(64,1.5),(128,1.),(128,1.5)]):
  values=np.array([[next(r['good10_fraction'] for r in summaries if r['params']['block_size']==b and r['params']['num_disparities']==n and r['params']['uniqueness_ratio']==u and r['params']['lr_tolerance_px']==lr) for u in [5,10,20]] for b in [3,5,7]]);im=ax.imshow(values,vmin=min(r['good10_fraction'] for r in summaries),vmax=max(r['good10_fraction'] for r in summaries),cmap='viridis');ax.set(xticks=[0,1,2],xticklabels=[5,10,20],yticks=[0,1,2],yticklabels=[3,5,7],xlabel='Uniqueness',ylabel='Block size',title=f'Development only: disparities={n}, LR={lr}')
  for i in range(3):
   for j in range(3):ax.text(j,i,f'{values[i,j]:.3f}',ha='center',va='center',color='white')
 fig.colorbar(im,ax=axs.ravel().tolist(),fraction=.025,label='Correct <=10cm / all GT-valid pixels');fig.subplots_adjust(hspace=.38,wspace=.3,right=.84);fig.savefig(out/'parameter_scan.png',dpi=145);plt.close(fig)
 class_rows=list(csv.DictReader((out/'final_test/per_class.csv').open()));classes=sorted({r['scene_class'] for r in class_rows});fig,axs=plt.subplots(1,3,figsize=(15,4));x=np.arange(len(classes))
 for method,off,color in [('baseline',-.18,'steelblue'),('challenger',.18,'darkorange')]:
  for ax,key,label in zip(axs,['good10_fraction','coverage','mae_m'],['Correct <=10cm / all GT','Valid coverage','MAE among accepted (m)']):
   ys=[float(next(r for r in class_rows if r['method']==method and r['scene_class']==c)[key]) for c in classes];ax.bar(x+off,ys,.34,label=method,color=color);ax.set(title=label,xticks=x,xticklabels=classes);ax.tick_params(axis='x',rotation=25)
 axs[0].legend();fig.suptitle('Fresh frozen final evaluation; challenger is not automatically deployed');fig.tight_layout();fig.savefig(out/'final_comparison.png',dpi=145);plt.close(fig)
 capture=json.loads((out/'final_test/capture_manifest.json').read_text());run=Path(capture['run']);records=capture['captured_scenes'];fig,axs=plt.subplots(4,3,figsize=(12,12))
 for i,cls in enumerate(classes):
  worst=max((r for r in all_metrics if r['partition']=='final_test' and r['method']=='baseline' and r['scene_class']==cls),key=lambda r:float(r['mae_m']) if r['mae_m'] else -1);r=next(r for r in records if r['scenario_id']==worst['scene_id']);left,_=load_image_pair(run,r['scenario_id']);axs[i,0].imshow(cv2.cvtColor(left,cv2.COLOR_BGR2RGB));axs[i,0].set_title(f"{cls} | actual RGB\nZ={r['distance_m']:g}m, worst baseline MAE in class",fontsize=9);truth=np.load(run/r['truth_path']/'left_optical_z_m.npy')
  for j,m in enumerate(['baseline','challenger'],1):
   z=np.load(out/'final_test'/m/r['scenario_id']/'estimated_z_m.npy');error=abs(z-truth);cmap=plt.colormaps['magma'].copy();cmap.set_bad('#bdbdbd');im=axs[i,j].imshow(np.ma.masked_invalid(error),vmin=0,vmax=.5,cmap=cmap);axs[i,j].set_title(m+' abs error (m)\ngrey = invalid',fontsize=10)
  for ax in axs[i]:ax.axis('off')
 fig.colorbar(im,ax=axs[:,1:].ravel().tolist(),fraction=.02,label='Error color capped at 0.5m; CSV includes full errors');fig.subplots_adjust(wspace=.04,hspace=.2,right=.85);fig.savefig(out/'failure_examples.png',dpi=140);plt.close(fig)
 checks={'all_protected_files_unchanged':True,'protected_count':len(integrity),'baseline_config_not_overwritten':json.loads(BASE.read_text())==json.loads((out/'baseline_params.json').read_text()),'workspace_user_change_preserved':all(r['unchanged'] for r in integrity if r['path']=='RobotDog.code-workspace'),'no_invalid_zero_or_non_nan':True,'test_after_final_selection':True,'all_scene_shapes_sync':all(r['timestamp_delta_s']<=1e-9 for part in ['validation','final_test'] for r in json.loads((out/part/'capture_manifest.json').read_text())['captured_scenes']),'neural_training_steps':0,'auto_promote_from_test':False};dump(out/'acceptance.json',checks)
 html='<!doctype html><meta charset="utf-8"><title>阶段8：当前不需要训练</title><style>body{font-family:sans-serif;max-width:1100px;margin:30px auto}img{max-width:100%}pre{white-space:pre-wrap}</style><h1>阶段8：当前不需要训练</h1><p>36项经典SGBM参数扫描，独立验证，再冻结最终测试；训练0步、未下载模型、运动策略未改。</p><p>主配置：'+freeze['primary_method']+'；开发最佳：'+selection['selected_candidate']+'；晋升：'+str(freeze['promoted'])+'</p><img src="parameter_scan.png"><img src="final_comparison.png"><img src="failure_examples.png"><h2>数值与限制</h2><pre>'+json.dumps(decision,ensure_ascii=False,indent=2)+'</pre>'
 (out/'results.html').write_text(html);event(out,'stage8_complete',decision='当前不需要训练',primary=freeze['primary_method']);print(json.dumps({'decision':'当前不需要训练','primary':freeze['primary_method'],'promoted':freeze['promoted'],'final_metrics':test,'checks':checks},ensure_ascii=False,indent=2))

def main():
 cv2.setNumThreads(4)
 p=argparse.ArgumentParser();p.add_argument('action',choices=['scan','evaluate','report']);p.add_argument('--out',type=Path,required=True);p.add_argument('--capture',type=Path);p.add_argument('--partition',choices=['validation','final_test']);a=p.parse_args();out=a.out.resolve()
 try:
  if a.action=='scan':scan(out)
  elif a.action=='evaluate':evaluate(out,a.capture.resolve(),a.partition)
  else:report(out)
 except Exception as e:
  if out.exists():dump(out/('failure_'+a.action+'.json'),{'time':now(),'error_type':type(e).__name__,'error':str(e)});event(out,'failure',action=a.action,error=str(e))
  raise
if __name__=='__main__':main()
