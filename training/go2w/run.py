"""Train or inspect the local Go2W flat-ground policy with RSL-RL PPO."""
import argparse
import json
import copy
import csv
import math
from pathlib import Path
from datetime import datetime
import charset_normalizer
from isaaclab.app import add_launcher_args, launch_simulation
p=argparse.ArgumentParser()
p.add_argument('--headless',action='store_true',help='Alias for --visualizer none')
p.add_argument('--mode', choices=['wheel','walk'],default='wheel')
p.add_argument('--num_envs',type=int,default=256)
p.add_argument('--iterations',type=int,default=1500)
p.add_argument('--play',action='store_true')
p.add_argument('--evaluate',action='store_true',help='Replay fixed stop/forward/turn commands and save metrics')
p.add_argument('--evaluate-suite',action='store_true',help='Evaluate stop, forward, reverse, turns and arcs separately')
p.add_argument('--friction',action='store_true',help='Randomize wheel contact materials at startup')
p.add_argument('--push',type=float,default=0.,help='XY velocity impulse bound in m/s every 6 seconds')
p.add_argument('--slope',type=float,default=0.,help='Inclined plane angle in degrees around world Y')
p.add_argument('--raw-residuals',action='store_true',help='Disable the deployment stop gate for ablation')
p.add_argument('--suite-speed',type=float,default=.15,help='Forward speed for the fixed evaluation suite')
p.add_argument('--suite-yaw',type=float,default=.3,help='Yaw rate magnitude for the fixed evaluation suite')
p.add_argument('--walk-action-scale',type=float,default=.12,help='Walking residual joint-angle bound in radians')
p.add_argument('--checkpoint',type=Path)
p.add_argument('--smoke',action='store_true')
p.add_argument('--steps',type=int,default=600)
p.add_argument('--log_dir',type=Path)
add_launcher_args(p)
a=p.parse_args()
if a.evaluate_suite:a.evaluate=True
if a.evaluate:a.play=True
if a.headless:a.visualizer="none"
elif a.visualizer is None:a.visualizer="kit"
from env import Go2WEnv, Go2WEnvCfg, prepare_asset, FrictionEventsCfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, RslRlOnPolicyRunnerCfg, RslRlMLPModelCfg, RslRlPpoAlgorithmCfg
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl.utils import handle_deprecated_rsl_rl_cfg, check_rsl_rl_version
import torch


def main():
    torch.set_num_threads(4)
    cfg=Go2WEnvCfg()
    cfg.mode=a.mode
    cfg.scene.num_envs=a.num_envs
    cfg.sim.device=a.device
    cfg.stop_gate=not a.raw_residuals
    cfg.push_velocity=a.push
    cfg.slope_degrees=a.slope
    cfg.walk_action_scale=a.walk_action_scale
    if a.friction:cfg.events=FrictionEventsCfg()
    if a.slope:
        from isaaclab.assets import AssetBaseCfg
        import isaaclab.sim as sim_utils
        angle=math.radians(a.slope)
        cfg.scene.env_spacing=10.
        cfg.scene.ground=AssetBaseCfg(prim_path='{ENV_REGEX_NS}/SlopedGround',
            spawn=sim_utils.CuboidCfg(size=(8.,8.,.2),collision_props=sim_utils.CollisionPropertiesCfg(),
                physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.,dynamic_friction=1.)),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.,0.,-.1/math.cos(angle)),
                rot=(0.,math.sin(angle/2),0.,math.cos(angle/2))))
    if a.evaluate:cfg.episode_length_s=max(12.,(10 if a.evaluate_suite else 1)*a.steps/30+5)
    if a.mode=='walk':
        cfg.action_space=12
        cfg.observation_space=54
        cfg.scene.robot.actuators['wheels'].stiffness=30.
        cfg.scene.robot.actuators['wheels'].damping=2.
    log=a.log_dir or Path(__file__).resolve().parents[2]/'outputs/training'/f'{a.mode}_{datetime.now():%Y%m%d_%H%M%S}'
    log.mkdir(parents=True,exist_ok=True)
    with launch_simulation(cfg.sim,a):
        prepare_asset()
        env=Go2WEnv(cfg)
        if a.slope:
            from pxr import UsdGeom, Gf
            import omni.usd
            ground=omni.usd.get_context().get_stage().GetPrimAtPath('/World/envs/env_0/SlopedGround')
            normal=UsdGeom.Xformable(ground).ComputeLocalToWorldTransform(0).TransformDir(Gf.Vec3d(0,0,1)).GetNormalized()
            assert abs(normal[0]-math.sin(angle))<1e-5 and abs(normal[2]-math.cos(angle))<1e-5,normal
            print('SLOPE_NORMAL_VERIFIED',list(normal),flush=True)
        wrapper=RslRlVecEnvWrapper(env,clip_actions=1.)
        if a.smoke:
            obs=wrapper.get_observations()
            assert obs['policy'].shape==(a.num_envs,cfg.observation_space)
            min_height=1e6
            early_resets=0
            velocity_error=0.
            for _ in range(a.steps):
                obs,reward,done,_=wrapper.step(torch.zeros((a.num_envs,cfg.action_space),device=env.device))
                assert torch.isfinite(obs['policy']).all() and torch.isfinite(reward).all()
                min_height=min(min_height,float(env.robot.data.root_link_pos_w.torch[:,2].min()))
                early_resets+=int(env.reset_terminated.sum())
                velocity_error+=float(torch.linalg.vector_norm(env.robot.data.root_link_lin_vel_b.torch[:,:2]-env.commands[:,:2],dim=1).mean())
            print('GO2W_SMOKE_PASS',{'mode':a.mode,'steps':a.steps,'envs':a.num_envs,'min_height':min_height,
                'early_resets':early_resets,'mean_velocity_error':velocity_error/max(a.steps,1)},flush=True)
        else:
            agent=RslRlOnPolicyRunnerCfg(
                seed=42, num_steps_per_env=24,max_iterations=a.iterations,save_interval=50,
                experiment_name='go2w_'+a.mode,obs_groups={'actor':['policy'],'critic':['policy']},
                actor=RslRlMLPModelCfg(hidden_dims=[128,128,64],activation='elu',obs_normalization=True,
                    distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=.3)),
                critic=RslRlMLPModelCfg(hidden_dims=[128,128,64],activation='elu',obs_normalization=True),
                algorithm=RslRlPpoAlgorithmCfg(value_loss_coef=1.,use_clipped_value_loss=True,clip_param=.2,
                    entropy_coef=.005,num_learning_epochs=5,num_mini_batches=4,learning_rate=3e-4,
                    schedule='adaptive',gamma=.99,lam=.95,desired_kl=.01,max_grad_norm=1.),
            )
            agent=handle_deprecated_rsl_rl_cfg(agent,check_rsl_rl_version())
            runner=OnPolicyRunner(wrapper,agent.to_dict(),log_dir=str(log),device=env.device)
            metadata={'mode':a.mode,'num_envs':a.num_envs,'iterations_requested':0 if a.play else a.iterations,
                'physics_dt':cfg.sim.dt,'control_dt':env.step_dt,'leg_order':__import__('env').LEG_NAMES,
                'wheel_order':__import__('env').WHEEL_NAMES,'isaaclab':'3.0.0','seed':42,
                'task':'residual locomotion control; terrain and perturbations recorded below; not validated for hardware'}
            metadata.update(stop_gate=cfg.stop_gate,friction_randomization=a.friction,push_velocity=a.push,slope_degrees=a.slope)
            if a.mode=='walk':metadata.update(gait_reference='ik_v1',leg_action_scale=cfg.walk_action_scale)
            metadata['reward_version']='walk_clearance_v4' if a.mode=='walk' else 'wheel_stop_v1'
            if a.checkpoint:
                runner.load(str(a.checkpoint))
                metadata['loaded_checkpoint']=str(a.checkpoint.resolve())
                prior_contract=a.checkpoint.parent/'policy_jit.json'
                metadata['loaded_training_iterations']=(json.loads(prior_contract.read_text())['training_iterations_completed']
                    if prior_contract.exists() else runner.current_learning_iteration+1)
            (log/'run.json').write_text(json.dumps(metadata,indent=2))
            def export_policy():
                actor=runner.get_inference_policy(device=env.device)
                network=torch.nn.Sequential(copy.deepcopy(actor.obs_normalizer),copy.deepcopy(actor.mlp)).cpu().eval()
                example=wrapper.get_observations()['policy'].cpu()
                with torch.inference_mode():
                    expected=actor(wrapper.get_observations()).cpu()
                    exported=torch.jit.trace(network,example)
                    assert torch.allclose(exported(example),expected,atol=1e-5)
                exported.save(str(log/'policy_jit.pt'))
                metadata['last_runner_iteration']=runner.current_learning_iteration
                metadata['training_iterations_completed']=metadata.get('loaded_training_iterations',0)+(0 if a.play else a.iterations)
                metadata.update({'observation_joint_order':list(env.robot.joint_names),'actions':cfg.action_space,'observations':cfg.observation_space})
                (log/'policy_jit.json').write_text(json.dumps(metadata,indent=2))
                print('POLICY_EXPORT_PASS',str(log/'policy_jit.pt'),flush=True)
            if a.play:
                if not a.checkpoint:raise ValueError('--play requires --checkpoint')
                policy=runner.get_inference_policy(device=env.device)
                obs=wrapper.get_observations()
                if a.evaluate_suite:
                    cases=[('stand',0.,0.),('forward',a.suite_speed,0.),('stop_after_forward',0.,0.),
                        ('reverse',-min(.1,a.suite_speed),0.),('stop_after_reverse',0.,0.),
                        ('left_turn',0.,a.suite_yaw),('right_turn',0.,-a.suite_yaw),
                        ('left_arc',a.suite_speed,a.suite_yaw),('right_arc',a.suite_speed,-a.suite_yaw),('final_stop',0.,0.)]
                    results=[]
                    trace=[]
                    with torch.inference_mode():
                        for name,vx,wz in cases:
                            values={key:[] for key in ['velocity_error','yaw_error','upright','height','speed','tail_speed','wheel_speed','foot_lift']}
                            resets=0
                            timeouts=0
                            foot_lift_per_leg=[]
                            for step in range(a.steps):
                                if not env.sim.is_running():raise RuntimeError('Simulation stopped during evaluation')
                                env.commands[:]=torch.tensor([vx,0.,wz],device=env.device)
                                env.command_age[:]=0.
                                obs['policy']=env._get_observations()['policy']
                                obs,_,_,_=wrapper.step(policy(obs))
                                resets+=int(env.reset_terminated.sum())
                                timeouts+=int(env.reset_time_outs.sum())
                                assert torch.isfinite(obs['policy']).all()
                                d=env.robot.data
                                speed=torch.linalg.vector_norm(d.root_link_lin_vel_b.torch[:,:2],dim=1)
                                foot_pos=d.body_link_pose_w.torch[:,env.foot_ids,:3]-env.scene.env_origins[:,None,:]
                                foot_height=foot_pos[:,:,2]+foot_pos[:,:,0]*math.tan(math.radians(a.slope))
                                trace.append([name,step,float(speed.mean()),float(speed.max()),int(speed.argmax()),
                                    float(d.root_link_ang_vel_b.torch[:,2].mean()),float(env.terrain_height().min()),
                                    *foot_height.mean(0).tolist()])
                                # Allow one second for command transitions; count all resets.
                                if step<30:continue
                                d=env.robot.data
                                values['velocity_error'].append(float(torch.linalg.vector_norm(d.root_link_lin_vel_b.torch[:,:2]-torch.tensor([vx,0.],device=env.device),dim=1).mean()))
                                values['yaw_error'].append(float((d.root_link_ang_vel_b.torch[:,2]-wz).abs().mean()))
                                values['upright'].append(float((-d.projected_gravity_b.torch[:,2]).mean()))
                                values['height'].append(float(env.terrain_height().min()))
                                values['speed'].append(float(torch.linalg.vector_norm(d.root_link_lin_vel_b.torch[:,:2],dim=1).max()))
                                values['wheel_speed'].append(float(d.joint_vel.torch[:,env.wheel_ids].abs().mean()))
                                values['foot_lift'].append(float((foot_height>.106).float().mean()))
                                foot_lift_per_leg.append((foot_height>.106).float().mean(0).tolist())
                                if step>=a.steps-30:values['tail_speed'].append(float(speed.max()))
                            if not values['speed']:raise ValueError('Suite requires --steps > 30')
                            result={'case':name,'command_vx':vx,'command_wz':wz,'early_resets':resets,'timeout_resets':timeouts,
                                'mean_velocity_error':sum(values['velocity_error'])/len(values['velocity_error']),
                                'mean_yaw_error':sum(values['yaw_error'])/len(values['yaw_error']),
                                'mean_upright':sum(values['upright'])/len(values['upright']),
                                'min_height':min(values['height']),'peak_speed_after_settling':max(values['speed']),
                                'last_second_peak_speed':max(values['tail_speed'])}
                            result['mean_wheel_joint_speed']=sum(values['wheel_speed'])/len(values['wheel_speed'])
                            result['foot_center_above_106mm_fraction']=sum(values['foot_lift'])/len(values['foot_lift'])
                            result['foot_lift_fraction_per_leg']=[sum(row[i] for row in foot_lift_per_leg)/len(foot_lift_per_leg) for i in range(4)]
                            result['passed']=(resets==0 and timeouts==0 and result['mean_velocity_error']<.06 and result['mean_yaw_error']<.15
                                and result['mean_upright']>.98 and result['min_height']>.30
                                and (vx!=0. or wz!=0. or result['last_second_peak_speed' if a.push else 'peak_speed_after_settling']<.05))
                            if a.mode=='walk':
                                result['gait_clearance_passed']=(vx==0. and wz==0.) or (result['foot_center_above_106mm_fraction']>.1 and min(result['foot_lift_fraction_per_leg'])>.05)
                                result['gait_passed']=result['passed'] and result['gait_clearance_passed']
                            results.append(result)
                            print('SUITE_CASE',json.dumps(result),flush=True)
                    report={'checkpoint':str(a.checkpoint.resolve()),'num_envs':a.num_envs,
                        'steps_per_case':a.steps,'settling_steps':30,'control_dt':env.step_dt,
                        'seed':42,'thresholds':{'mean_velocity_error_max':.06,'mean_yaw_error_max':.15,
                            'mean_upright_min':.98,'height_min':.30,'stop_peak_speed_max':.05,'early_resets_max':0},
                        'time_basis':'simulation','cases':results,'all_passed':all(r['passed'] for r in results),
                        'stop_gate':cfg.stop_gate,'friction_randomization':a.friction,'push_velocity':a.push,
                        'push_events':env.push_count,'slope_degrees':a.slope,
                        'stop_metric':'last_second_peak_speed' if a.push else 'peak_speed_after_settling',
                        'note':'Full training controller including command-dependent residual gate; use --raw-residuals for ablation.'}
                    if a.mode=='walk':report['all_gait_passed']=all(r['gait_passed'] for r in results)
                    (log/'suite.json').write_text(json.dumps(report,indent=2))
                    with (log/'trace.csv').open('w') as f:
                        writer=csv.writer(f)
                        writer.writerow(['case','step','mean_speed','max_speed','worst_env','mean_yaw_rate','min_height',
                            'FL_height','FR_height','RL_height','RR_height'])
                        writer.writerows(trace)
                    print('GO2W_SUITE_COMPLETE',report['all_passed'],flush=True)
                    wrapper.close()
                    return
                metrics={'steps':0,'early_resets':0,'mean_velocity_error':0.,'mean_upright':0.,'min_height':1e6}
                with torch.inference_mode():
                    for step in range(a.steps):
                        if not env.sim.is_running():break
                        if a.evaluate:
                            fraction=step/a.steps
                            env.commands[:]=0.
                            if .2<=fraction<.6:env.commands[:,0]=.15
                            elif .6<=fraction<.8:env.commands[:,2]=.3
                            env.command_age[:]=0.
                            obs['policy']=env._get_observations()['policy']
                        obs,_,_,_=wrapper.step(policy(obs))
                        d=env.robot.data
                        metrics['steps']+=1
                        metrics['early_resets']+=int(env.reset_terminated.sum())
                        metrics['mean_velocity_error']+=float(torch.linalg.vector_norm(d.root_link_lin_vel_b.torch[:,:2]-env.commands[:,:2],dim=1).mean())
                        metrics['mean_upright']+=float((-d.projected_gravity_b.torch[:,2]).mean())
                        metrics['min_height']=min(metrics['min_height'],float(d.root_link_pos_w.torch[:,2].min()))
                        assert torch.isfinite(obs['policy']).all()
                for key in ['mean_velocity_error','mean_upright']:metrics[key]/=max(1,metrics['steps'])
                (log/'evaluation.json').write_text(json.dumps(metrics,indent=2))
                print('POLICY_EVALUATION',json.dumps(metrics),flush=True)
                export_policy()
                print('GO2W_PLAY_PASS',flush=True)
            else:
                runner.learn(a.iterations)
                runner.save(str(log/'model_final.pt'))
                export_policy()
                print('GO2W_TRAIN_COMPLETE',str(log/'model_final.pt'),flush=True)
        wrapper.close()

if __name__=='__main__':main()
