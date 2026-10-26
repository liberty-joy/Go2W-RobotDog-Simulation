"""Replay wheel/walk policy contracts without importing Isaac Lab into Sim."""
import json
import math
from pathlib import Path
import numpy as np
import torch
from pxr import UsdPhysics


def inverse_rotate(q,v):
    w,x,y,z=q
    u=np.array([x,y,z])
    return v+2*np.cross(u,np.cross(u,v)-w*v)


class WheelPolicy:
    def __init__(self,path,robot,stage,robot_path,wheel_control='smooth'):
        path=Path(path).resolve()
        self.contract=json.loads(path.with_suffix('.json').read_text())
        self.mode=self.contract['mode']
        self.smooth_wheels=self.mode=='wheel' and wheel_control=='smooth'
        if (self.mode,self.contract['observations']) not in [('wheel',58),('walk',54)]:
            raise ValueError('Unsupported locomotion policy contract')
        if self.mode=='walk' and self.contract.get('gait_reference')!='ik_v1':
            raise ValueError('Walking policy must use the ik_v1 gait reference')
        self.model=torch.jit.load(str(path),map_location='cpu').eval()
        self.robot=robot
        names=list(robot.dof_names)
        self.leg_names=self.contract['leg_order']
        self.leg_ids=[names.index(n) for n in self.leg_names]
        self.velocity_ids=[names.index(n) for n in self.contract['observation_joint_order']]
        self.default=np.array([0.,math.pi/4,-math.pi/2]*4)
        self.drives=[UsdPhysics.DriveAPI(stage.GetPrimAtPath(robot_path+'/Physics/'+n),'angular') for n in self.leg_names]
        self.wheel_ids=[names.index(n) for n in self.contract['wheel_order']]
        self.wheel_drives=[UsdPhysics.DriveAPI(stage.GetPrimAtPath(robot_path+'/Physics/'+n),'angular') for n in self.contract['wheel_order']]
        self.reset()
        print('LOCOMOTION_POLICY_READY',self.mode,str(path),flush=True)

    def reset(self):
        self.actions=np.zeros(self.contract['actions'],dtype=np.float32)
        self.phase=0.
        self.targets=self.default.copy()
        self.speeds=np.zeros(4)

    def activate(self):
        self.reset()
        positions=self.robot.get_joint_positions()[self.wheel_ids]
        for drive,position in zip(self.wheel_drives,positions):
            drive.GetStiffnessAttr().Set(30*math.pi/180 if self.mode=='walk' else 0.)
            drive.GetDampingAttr().Set(2*math.pi/180 if self.mode=='walk' else (.02 if self.smooth_wheels else .08))
            drive.GetTargetPositionAttr().Set(math.degrees(float(position)))
            drive.GetTargetVelocityAttr().Set(0.)

    def step(self,v,yaw,update):
        if update:
            _,quat=self.robot.get_world_pose()
            obs=np.concatenate([
                inverse_rotate(quat,np.asarray(self.robot.get_linear_velocity())),
                inverse_rotate(quat,np.asarray(self.robot.get_angular_velocity())),
                inverse_rotate(quat,np.array([0.,0.,-1.])),[v,0.,yaw],
                self.robot.get_joint_positions()[self.leg_ids]-self.default,
                self.robot.get_joint_velocities()[self.velocity_ids]*.05,self.actions,
                [math.sin(self.phase),math.cos(self.phase)],
            ]).astype(np.float32)
            with torch.inference_mode():
                new_actions=self.model(torch.from_numpy(obs[None])).numpy()[0].clip(-1,1)
            if not np.isfinite(new_actions).all():raise RuntimeError('Non-finite policy action')
            if self.contract.get('stop_gate',False):
                new_actions*=min(1.,(abs(v)+.18*abs(yaw))/.05)
            if abs(v)+abs(yaw)<1e-6:new_actions[:]=0.
            if self.smooth_wheels:
                # Rolling needs no periodic stepping excitation. Keep the trained
                # network but damp residual feedback and pair each side's wheels.
                new_actions[12:]=np.tile((new_actions[12:14]+new_actions[14:16])/2,2)
                self.actions+=.15*(new_actions-self.actions)
                # In rolling mode the leg servo holds the established stand.
                # The learned stepping residual was pumping the front wheels.
                self.actions[:12]=0.
            else:self.actions=new_actions
            reference=self.default.copy()
            if self.mode=='walk':
                phase=self.phase+2*math.pi*1.5/30+np.array([0.,math.pi,math.pi,0.])
                stride=(v+np.array([-1.,1.,-1.,1.])*yaw*.18)/6
                foot_x=stride*np.sin(phase)
                foot_z=-.301+.05*np.maximum(0,np.cos(phase))*(np.abs(stride)>.002)
                calf=-np.arccos(np.clip((foot_x**2+foot_z**2-2*.213**2)/(2*.213**2),-.99,.99))
                reference[1::3]=np.arctan2(-foot_x,-foot_z)-calf/2
                reference[2::3]=calf
            self.targets=reference+(.2 if self.mode=='wheel' else self.contract.get('leg_action_scale',.35))*self.actions[:12]
            desired_speeds=(((v+np.array([-1.,1.,-1.,1.])*yaw*.18)/.05+3*self.actions[12:]).clip(-20,20)
                if self.mode=='wheel' else np.zeros(4))
            self.speeds=(self.speeds+np.clip(desired_speeds-self.speeds,-4/30,4/30)
                if self.smooth_wheels else desired_speeds)
            # A zero operator command always brakes, even with a biased short-trained policy.
            if abs(v)+abs(yaw)<1e-6:self.speeds[:]=0.
            self.phase=0. if self.smooth_wheels else (self.phase+2*math.pi*1.5/30)%(2*math.pi)
        # Apply braking immediately, without waiting for the next 30 Hz inference tick.
        if abs(v)+abs(yaw)<1e-6:self.speeds[:]=0.
        for drive,target in zip(self.drives,self.targets):
            drive.GetTargetPositionAttr().Set(math.degrees(float(target)))
        return self.speeds
