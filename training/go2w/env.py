"""Local Go2W flat-ground residual locomotion tasks for Isaac Lab 3 / PhysX."""
from pathlib import Path
import math
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.envs import mdp

ROOT = Path(__file__).resolve().parents[2]
ASSET = ROOT/'sim/assets/go2w/training/go2w.usda'
LEGS = ['FL', 'FR', 'RL', 'RR']
LEG_NAMES = [f'{leg}_{kind}_joint' for leg in LEGS for kind in ['hip','thigh','calf']]
WHEEL_NAMES = [f'{leg}_foot_joint' for leg in LEGS]


def prepare_asset():
    """Write a small override layer; preserve the original imported model."""
    from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema
    ASSET.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(ASSET))
    robot = stage.DefinePrim('/Go2W','Xform')
    robot.GetReferences().AddReference(str(ROOT/'sim/assets/go2w/go2w_description/go2w_description.usda'))
    stage.SetDefaultPrim(robot)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    robot.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
    UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0,0,0))
    base=stage.GetPrimAtPath('/Go2W/Geometry/base')
    UsdGeom.XformCommonAPI(base).SetTranslate(Gf.Vec3d(0,0,0))
    solver=PhysxSchema.PhysxArticulationAPI.Apply(base)
    solver.CreateSolverPositionIterationCountAttr(32)
    solver.CreateSolverVelocityIterationCountAttr(4)
    solver.CreateEnabledSelfCollisionsAttr(False)
    for path in ['/Go2W/World','/Go2W/GroundPlane','/Go2W/Geometry/Camera']:
        prim=stage.GetPrimAtPath(path)
        if prim:prim.SetActive(False)
    # USD targets use degrees. Isaac Lab actuator gains/targets use radians.
    for name in LEG_NAMES+WHEEL_NAMES:
        joint=stage.GetPrimAtPath('/Go2W/Physics/'+name)
        drive=UsdPhysics.DriveAPI.Apply(joint,'angular')
        drive.CreateTargetPositionAttr(0.)
        drive.CreateTargetVelocityAttr(0.)
    stage.GetRootLayer().Save()


@configclass
class Go2WSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path='/World/ground', spawn=sim_utils.GroundPlaneCfg(usd_path=str(ROOT/'sim/assets/ground/default_ground_plane.usda')))
    light = AssetBaseCfg(prim_path='/World/Light', spawn=sim_utils.DomeLightCfg(intensity=700.))
    robot = ArticulationCfg(
        prim_path='{ENV_REGEX_NS}/Robot',
        spawn=sim_utils.UsdFileCfg(usd_path=str(ASSET)),
        articulation_root_prim_path='/Geometry/base',
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.,0.,.42),
            joint_pos={'.*_hip_joint':0., '.*_thigh_joint':math.pi/4,
                       '.*_calf_joint':-math.pi/2, '.*_foot_joint':0.},
        ),
        soft_joint_pos_limit_factor=.95,
        actuators={
            'legs':ImplicitActuatorCfg(joint_names_expr=['.*_(hip|thigh|calf)_joint'],
                stiffness=229.18, damping=8.02,
                joint_effort_limit={'.*_hip_joint':23.7,'.*_thigh_joint':23.7,'.*_calf_joint':35.55},
                joint_velocity_limit={'.*_hip_joint':30.1,'.*_thigh_joint':30.1,'.*_calf_joint':20.07}),
            'wheels':ImplicitActuatorCfg(joint_names_expr=['.*_foot_joint'],
                stiffness=0., damping=4.58, joint_effort_limit=23.7, joint_velocity_limit=30.1),
        },
    )


@configclass
class FrictionEventsCfg:
    friction=EventTermCfg(func=mdp.randomize_rigid_body_material,mode='startup',params={
        'asset_cfg':SceneEntityCfg('robot',body_names=['.*_foot']),
        'static_friction_range':(.4,1.),'dynamic_friction_range':(.35,.9),
        'restitution_range':(0.,0.),'num_buckets':32,'make_consistent':True})


@configclass
class Go2WEnvCfg(DirectRLEnvCfg):
    decimation=4
    episode_length_s=12.
    action_space=16
    observation_space=58
    state_space=0
    mode='wheel'
    stop_gate=True
    push_velocity=0.
    slope_degrees=0.
    walk_action_scale=.12
    sim=SimulationCfg(dt=1/120, render_interval=4, physics=PhysxCfg())
    scene=Go2WSceneCfg(num_envs=64,env_spacing=4.,replicate_physics=True)
    seed=42


class Go2WEnv(DirectRLEnv):
    def __init__(self,cfg,**kwargs):
        super().__init__(cfg,**kwargs)
        self.robot=self.scene['robot']
        ids,_=self.robot.find_joints(LEG_NAMES,preserve_order=True)
        self.leg_ids=torch.tensor(ids,device=self.device)
        ids,_=self.robot.find_joints(WHEEL_NAMES,preserve_order=True)
        self.wheel_ids=torch.tensor(ids,device=self.device)
        ids,_=self.robot.find_bodies([f'{leg}_foot' for leg in LEGS],preserve_order=True)
        self.foot_ids=torch.tensor(ids,device=self.device)
        self.commands=torch.zeros((self.num_envs,3),device=self.device)
        self.actions=torch.zeros((self.num_envs,cfg.action_space),device=self.device)
        self.previous_actions=torch.zeros_like(self.actions)
        self.phase=torch.zeros(self.num_envs,device=self.device)
        self.phase_offsets=torch.tensor([0.,math.pi,math.pi,0.],device=self.device)
        self.default_legs=self.robot.data.default_joint_pos.torch[:,self.leg_ids].clone()
        self.leg_targets=self.default_legs.clone()
        self.wheel_targets=torch.zeros((self.num_envs,4),device=self.device)
        self.command_age=torch.zeros(self.num_envs,device=self.device)
        self.push_steps=0
        self.push_count=0
        print('GO2W_ENV_READY',{'mode':cfg.mode,'envs':self.num_envs,'legs':LEG_NAMES,'wheels':WHEEL_NAMES},flush=True)

    def _pre_physics_step(self,actions):
        self.previous_actions.copy_(self.actions)
        self.actions=actions.clamp(-1,1).clone()
        self.command_age+=self.step_dt
        self.resample_commands(torch.where(self.command_age>=4.)[0])
        if self.cfg.stop_gate:
            # The same command-dependent residual gate is used in deployment.
            # It prevents phase-dependent leg/wheel actions from exciting a stop.
            scale=((self.commands[:,0].abs()+.18*self.commands[:,2].abs())/.05).clamp(0,1)
            self.actions*=scale[:,None]
        self.push_steps+=1
        if self.cfg.push_velocity and self.push_steps%180==90:
            velocity=self.robot.data.root_com_vel_w.torch.clone()
            velocity[:,:2]+=(2*torch.rand((self.num_envs,2),device=self.device)-1)*self.cfg.push_velocity
            self.robot.write_root_velocity_to_sim_index(root_velocity=velocity)
            self.push_count+=self.num_envs
        self.phase=(self.phase+2*math.pi*1.5*self.step_dt)%(2*math.pi)
        reference=self.default_legs.clone()
        if self.cfg.mode=='walk':
            side=torch.tensor([-1.,1.,-1.,1.],device=self.device)
            stride=(self.commands[:,0,None]+side*self.commands[:,2,None]*.18)/(4*1.5)
            phase=self.phase[:,None]+self.phase_offsets
            moving=(stride.abs()>.002).float()
            foot_x=stride*torch.sin(phase)
            foot_z=-.301+.05*torch.cos(phase).clamp_min(0)*moving
            calf=-torch.acos(((foot_x**2+foot_z**2-2*.213**2)/(2*.213**2)).clamp(-.99,.99))
            reference[:,1::3]=torch.atan2(-foot_x,-foot_z)-calf/2
            reference[:,2::3]=calf
        self.leg_targets=reference+(.2 if self.cfg.mode=='wheel' else self.cfg.walk_action_scale)*self.actions[:,:12]
        limits=self.robot.data.soft_joint_pos_limits.torch[:,self.leg_ids]
        self.leg_targets=self.leg_targets.clamp(limits[:,:,0],limits[:,:,1])
        if self.cfg.mode=='wheel':
            side=torch.tensor([-1.,1.,-1.,1.],device=self.device)
            self.wheel_targets=((self.commands[:,0,None]+side*self.commands[:,2,None]*.18)/.05+3*self.actions[:,12:]).clamp(-20,20)

    def _apply_action(self):
        self.robot.set_joint_position_target_index(target=self.leg_targets,joint_ids=self.leg_ids)
        if self.cfg.mode=='wheel':
            self.robot.set_joint_velocity_target_index(target=self.wheel_targets,joint_ids=self.wheel_ids)
        else:
            self.robot.set_joint_position_target_index(target=torch.zeros_like(self.wheel_targets),joint_ids=self.wheel_ids)
            self.robot.set_joint_velocity_target_index(target=torch.zeros_like(self.wheel_targets),joint_ids=self.wheel_ids)

    def _get_observations(self):
        d=self.robot.data
        obs=torch.cat([d.root_link_lin_vel_b.torch, d.root_link_ang_vel_b.torch,
            d.projected_gravity_b.torch, self.commands,
            d.joint_pos.torch[:,self.leg_ids]-self.default_legs,
            d.joint_vel.torch*.05,self.actions,
            torch.sin(self.phase[:,None]),torch.cos(self.phase[:,None])],dim=-1)
        return {'policy':obs}

    def _get_rewards(self):
        d=self.robot.data
        vel=d.root_link_lin_vel_b.torch
        ang=d.root_link_ang_vel_b.torch
        gravity=d.projected_gravity_b.torch
        height=self.terrain_height()
        planar_error=((vel[:,:2]-self.commands[:,:2])**2).sum(-1)
        yaw_error=(ang[:,2]-self.commands[:,2])**2
        reward=2*torch.exp(-planar_error/(.04 if self.cfg.mode=='walk' else .16))
        reward+=(1.5 if self.cfg.mode=='walk' else 1.)*torch.exp(-yaw_error/(.09 if self.cfg.mode=='walk' else .25))+.5
        reward-=2*(gravity[:,:2]**2).sum(-1)+(100 if self.cfg.mode=='walk' else 8)*(height-.389)**2
        reward-=.2*vel[:,2]**2+.05*(ang[:,:2]**2).sum(-1)
        reward-=.03*((self.actions-self.previous_actions)**2).sum(-1)
        reward-=.02*(self.actions**2).sum(-1)
        reward-=.00002*(d.joint_vel.torch**2).sum(-1)
        stopped=(self.commands.abs().sum(-1)<1e-6).float()
        reward-=stopped*(8*(vel[:,:2]**2).sum(-1)+2*ang[:,2]**2)
        if self.cfg.mode=='walk':
            foot_pos=d.body_link_pose_w.torch[:,self.foot_ids,:3]-self.scene.env_origins[:,None,:]
            foot_height=foot_pos[:,:,2]+foot_pos[:,:,0]*math.tan(math.radians(self.cfg.slope_degrees))
            swing=torch.cos(self.phase[:,None]+self.phase_offsets).clamp_min(0)
            moving=(torch.linalg.vector_norm(self.commands[:,:2],dim=1)+self.commands[:,2].abs()>.03).float()
            desired=.086+.05*swing*moving[:,None]
            reward+=2*torch.exp(-((foot_height-desired)**2)/.0004).mean(-1)
            reward-=.08*(self.actions**2).sum(-1)
        reward-=3*self.reset_terminated.float()
        self.extras['log']={'Metrics/velocity_error':planar_error.sqrt().mean(),
            'Metrics/height':height.mean(),'Metrics/upright':(-gravity[:,2]).mean()}
        return reward*self.step_dt

    def _get_dones(self):
        d=self.robot.data
        height=self.terrain_height()
        fallen=(height<.20)|(height>.65)|(d.projected_gravity_b.torch[:,2]>-.65)
        escaped=torch.linalg.vector_norm(d.root_link_pos_w.torch[:,:2]-self.scene.env_origins[:,:2],dim=1)>3.5
        return fallen|escaped, self.episode_length_buf>=self.max_episode_length-1

    def terrain_height(self):
        pos=self.robot.data.root_link_pos_w.torch-self.scene.env_origins
        return pos[:,2]+pos[:,0]*math.tan(math.radians(self.cfg.slope_degrees))

    def resample_commands(self,ids):
        if len(ids)==0:return
        n=len(ids)
        self.commands[ids,0]=torch.rand(n,device=self.device)*(.5 if self.cfg.mode=='wheel' else .4)-.15
        self.commands[ids,1]=0. # First stage: straight travel + yaw; no lateral command.
        self.commands[ids,2]=torch.rand(n,device=self.device)*.6-.3
        stop=torch.rand(n,device=self.device)<.15
        self.commands[ids[stop]]=0.
        self.command_age[ids]=0.

    def _reset_idx(self,env_ids):
        if env_ids is None:env_ids=self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        pose=self.robot.data.default_root_pose.torch[env_ids].clone()
        pose[:,:3]+=self.scene.env_origins[env_ids]
        joints=self.robot.data.default_joint_pos.torch[env_ids].clone()
        # Mild pose randomization; reset all physical and controller states.
        joints[:,self.leg_ids]+=(torch.rand((len(env_ids),12),device=self.device)-.5)*.04
        self.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((len(env_ids),6),device=self.device),env_ids=env_ids)
        self.robot.write_joint_position_to_sim_index(position=joints,env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(joints),env_ids=env_ids)
        self.robot.set_joint_position_target_index(target=joints,env_ids=env_ids)
        self.robot.set_joint_velocity_target_index(target=torch.zeros_like(joints),env_ids=env_ids)
        self.actions[env_ids]=0
        self.previous_actions[env_ids]=0
        self.leg_targets[env_ids]=joints[:,self.leg_ids]
        self.wheel_targets[env_ids]=0
        self.phase[env_ids]=0
        self.resample_commands(env_ids)
