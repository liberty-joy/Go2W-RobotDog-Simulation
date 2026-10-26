"""Local Go2W wheel teleoperation and RGB capture. Run through run_go2w.sh."""
import argparse
from pathlib import Path
import math
import time
import faulthandler
faulthandler.enable()
faulthandler.dump_traceback_later(120, repeat=False)
import charset_normalizer  # Load consistently before Kit extension paths.
p = argparse.ArgumentParser()
p.add_argument('--headless', action='store_true')
p.add_argument('--test', action='store_true')
p.add_argument('--ros2', action='store_true')
p.add_argument('--ros-image-reliable',action='store_true',help='Opt-in reliable RGB DDS transport for recording')
p.add_argument('--policy', type=Path, help='Exported wheel policy_jit.pt (optional)')
p.add_argument('--walk-policy',type=Path,help='Optional ik_v1 walking policy; M switches while stationary')
p.add_argument('--steps', type=int, default=0, help='Stop after this many physics steps; zero runs continuously')
p.add_argument('--wheel-replay',action='store_true',help='Reproduce held-W: stand 2s, forward 6s, stop 3s, then exit')
p.add_argument('--diagnostics',type=Path,help='Write 30Hz control/joint trace CSV to a new file')
p.add_argument('--wheel-control',choices=['smooth','legacy'],default='smooth',help='Rolling residual stabilization; legacy is the pre-fix controller')
p.add_argument('--sensor-selection',type=Path,help='Optional parameterized sensor/API smoke configuration')
p.add_argument('--sensor-output',type=Path,help='New output directory for selection smoke')
p.add_argument('--sensor-rig-config',type=Path,help='Optional Stage 2 local sensor-rig assembly configuration')
p.add_argument('--sensor-rig-output',type=Path,help='New output directory for Stage 2 rig smoke')
a = p.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({'headless': a.headless, 'width': 1280, 'height': 800,
    'extra_args': ['--/renderer/raytracingMotion/enabled=true'] if (a.sensor_selection or a.sensor_rig_config) else []})
try:
    import numpy as np
    import carb.input
    import omni.appwindow
    import omni.usd
    from pxr import UsdGeom, UsdLux, Gf, UsdPhysics, PhysxSchema
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.sensors.camera import Camera
    from isaacsim.core.utils.viewports import set_camera_view
    from PIL import Image

    if a.ros2:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.ros2.bridge')
        app.update()
    root = Path(__file__).resolve().parents[1]
    world = World(stage_units_in_meters=1.0, physics_dt=1/120, rendering_dt=1/60)
    stage = omni.usd.get_context().get_stage()
    robot_path = '/World/Go2W'
    prim = stage.DefinePrim(robot_path, 'Xform')
    prim.GetReferences().AddReference(str(root/'sim/assets/go2w/go2w_description/go2w_description.usda'))
    prim.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(0,0,0))
    base_path = robot_path+'/Geometry/base'
    base = stage.GetPrimAtPath(base_path)
    UsdGeom.XformCommonAPI(base).SetTranslate(Gf.Vec3d(0,0,0.42))
    # Exclude user-created ground/cameras inside the referenced robot.
    for path in [robot_path+'/World', robot_path+'/GroundPlane', robot_path+'/Geometry/Camera']:
        q=stage.GetPrimAtPath(path)
        if q: q.SetActive(False)
    # Use the already-cached official ground; the default helper probes Nucleus
    # even when the robot itself is local. Preserve its contact coefficients.
    from isaacsim.core.api.objects import GroundPlane
    from isaacsim.core.api.materials import PhysicsMaterial
    ground_path='/World/defaultGroundPlane'
    stage.DefinePrim(ground_path,'Xform').GetReferences().AddReference(str(root/'sim/assets/ground/default_ground_plane.usda'))
    material=PhysicsMaterial(prim_path='/World/GroundPhysicsMaterial',static_friction=.5,dynamic_friction=.5,restitution=.8)
    world.scene.add(GroundPlane(prim_path=ground_path,name='default_ground_plane',physics_material=material))
    print('LOCAL_GROUND_READY',str(root/'sim/assets/ground/default_ground_plane.usda'),flush=True)
    light=UsdLux.DomeLight.Define(stage,'/World/Light')
    light.CreateIntensityAttr(700)
    solver=PhysxSchema.PhysxArticulationAPI.Apply(base)
    solver.CreateSolverPositionIterationCountAttr(32)
    solver.CreateSolverVelocityIterationCountAttr(4)
    wheels=[]
    for leg in ['FL','FR','RL','RR']:
        for kind, target in [('hip',0),('thigh',45),('calf',-90),('foot',0)]:
            joint=stage.GetPrimAtPath(f'{robot_path}/Physics/{leg}_{kind}_joint')
            if not joint: raise RuntimeError(f'Missing joint {leg}_{kind}')
            drive=UsdPhysics.DriveAPI.Apply(joint,'angular')
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(0 if kind=='foot' else 4)
            drive.CreateDampingAttr(0.08 if kind=='foot' else 0.14)
            drive.CreateMaxForceAttr(35.55 if kind=='calf' else 23.7)
            drive.CreateTargetPositionAttr(target)
            drive.CreateTargetVelocityAttr(0)
            if kind=='foot': wheels.append(drive)
    for name, pos, color in [('RedBox',(2.5,0,0.25),(0.8,0.1,0.1)),('BlueBox',(3,0.9,0.25),(0.1,0.2,0.8))]:
        box=UsdGeom.Cube.Define(stage,'/World/'+name)
        box.CreateSizeAttr(0.5)
        box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdGeom.XformCommonAPI(box).SetTranslate(Gf.Vec3d(*pos))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    robot=world.scene.add(SingleArticulation(prim_path=base_path,name='go2w'))
    camera_frequency=30
    if a.sensor_selection:
        import json
        camera_frequency=json.loads(a.sensor_selection.read_text())['stereo']['frequency_hz']
    camera=Camera(prim_path=base_path+'/FrontCamera', resolution=(640,480), frequency=camera_frequency,
                  translation=np.array([0.32,0,0.08]))
    world.reset()
    q=np.zeros(robot.num_dof)
    for i,name in enumerate(robot.dof_names):
        if '_thigh_joint' in name:q[i]=math.pi/4
        elif '_calf_joint' in name:q[i]=-math.pi/2
    robot.set_default_state(position=np.array([0,0,0.42]), orientation=np.array([1.,0,0,0]))
    robot.set_joints_default_state(positions=q, velocities=np.zeros(robot.num_dof))
    robot.set_joint_positions(q)
    def reset_robot():
        keys.clear()
        for drive in wheels: drive.GetTargetVelocityAttr().Set(0)
        world.reset()
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros(robot.num_dof))
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))
        configure_front_camera()
        if ros is not None:ros.clear_command()
        if policy is not None:policy.activate()
        print('ROBOT_RESET', flush=True)
    camera.initialize()
    def configure_front_camera():
        # +X forward, +Z up. A slight downward pitch keeps the horizon in view.
        half_pitch=math.radians(5)/2
        camera.set_local_pose(translation=np.array([0.32,0,0.08]),
                              orientation=np.array([math.cos(half_pitch),0,math.sin(half_pitch),0]),
                              camera_axes='world')
        camera.set_focal_length(0.018)
        camera.set_horizontal_aperture(2*0.018*math.tan(math.radians(80)/2))
        camera.set_clipping_range(0.02,100)
    configure_front_camera()
    selection_smoke=None
    if a.sensor_selection:
        if not a.sensor_output:raise ValueError('--sensor-output is required')
        from sensor_pipeline.selection_smoke import SelectionSmoke
        selection_smoke=SelectionSmoke(a.sensor_selection,a.sensor_output,stage,base_path,camera)
    asset_rig_smoke=None
    if a.sensor_rig_config:
        if not a.sensor_rig_output:raise ValueError('--sensor-rig-output is required')
        from sensor_pipeline.asset_rig_smoke import AssetRigSmoke
        asset_rig_smoke=AssetRigSmoke(a.sensor_rig_config,a.sensor_rig_output,root,stage,base_path)
    # initialize() attaches RGB by default.
    faulthandler.cancel_dump_traceback_later()
    set_camera_view(eye=np.array([2,2,1.3]),target=np.array([0,0,0.3]))
    ros=None
    keys=set(); flags={'capture':False,'camera':False}; keyboard=None; subscription=None
    interface=carb.input.acquire_input_interface()
    def on_key(event,*_):
        key=event.input.name
        if event.type==carb.input.KeyboardEventType.KEY_PRESS:
            keys.add(key)
            if key=='R':flags['reset']=True
            if key=='P':flags['capture']=True
            if key=='M':flags['mode']=True
            if key=='C':
                from omni.kit.viewport.utility import get_active_viewport
                flags['camera']=not flags['camera']
                get_active_viewport().camera_path = camera.prim_path if flags['camera'] else '/OmniverseKit_Persp'
            if key=='SPACE':
                keys.clear();keys.add('SPACE')
                if ros is not None:ros.clear_command()
        elif event.type==carb.input.KeyboardEventType.KEY_RELEASE:keys.discard(key)
        return True
    if a.test:
        from types import SimpleNamespace
        for key in ['W','SPACE','C','C']:
            on_key(SimpleNamespace(input=SimpleNamespace(name=key),type=carb.input.KeyboardEventType.KEY_PRESS))
        keys.clear()
        print('KEY_CALLBACK_SMOKE_OK',flush=True)
    if not a.headless:
        keyboard=omni.appwindow.get_default_app_window().get_keyboard()
        subscription=interface.subscribe_to_keyboard_events(keyboard,on_key)
    out=root/'outputs/camera';out.mkdir(parents=True,exist_ok=True)
    print('W/S forward/back, A/D turn, SPACE stop, R reset, C switch camera, P save image, M request wheel/walk switch. Camera: '+base_path+'/FrontCamera',flush=True)
    policy=None
    policies={}
    if a.policy:
        from go2w_policy import WheelPolicy
        policy=WheelPolicy(a.policy,robot,stage,robot_path,wheel_control=a.wheel_control)
        if policy.mode!='wheel':raise ValueError('--policy must be a wheel policy')
        policies['wheel']=policy
    if a.walk_policy:
        if not a.policy:raise ValueError('--walk-policy requires --policy')
        policies['walk']=WheelPolicy(a.walk_policy,robot,stage,robot_path)
        if policies['walk'].mode!='walk':raise ValueError('--walk-policy must be a walk policy')
    control_mode='wheel'
    ros=None
    if a.ros2:
        from go2w_ros2 import Go2WRos2
        ros=Go2WRos2(robot,camera,image_reliable=a.ros_image_reliable)
    steps=0; saved=0; before=None; cycle=0; needs_reset=False
    reset_robot()
    next_policy_time=0.;next_ros_time=0.;next_image_time=0.
    stationary_since=None
    diagnostic_file=None
    if a.diagnostics:
        import csv
        a.diagnostics.parent.mkdir(parents=True,exist_ok=True)
        diagnostic_file=a.diagnostics.open('x')
        diagnostic_writer=csv.writer(diagnostic_file)
        diagnostic_writer.writerow(['time','v_cmd','yaw_cmd','base_vx','base_vy','base_z',
            *['target_'+n for n in ['FL','FR','RL','RR']],
            *['vel_'+n for n in ['FL','FR','RL','RR']],
            *['q_'+n for n in (policy.leg_names if policy else [])],
            *['target_'+n for n in (policy.leg_names if policy else [])]])
    while app.is_running():
        tick=time.monotonic()
        if ros is not None:ros.spin()
        if not world.is_playing():
            keys.clear()
            if ros is not None:ros.clear_command()
            if world.is_stopped():needs_reset=True
            world.step(render=True);continue
        if needs_reset or flags.pop('reset',False):
            reset_robot();needs_reset=False
            next_policy_time=next_ros_time=next_image_time=0.
            stationary_since=None
        # Low speed differential steering. All wheel axes are +Y in official URDF.
        v=0.15*(('W' in keys)-('S' in keys)); yaw=0.35*(('A' in keys)-('D' in keys))
        if ros is not None and not (keys & {'W','A','S','D'}):v,yaw=ros.velocity()
        if 'SPACE' in keys:
            v=yaw=0
            if ros is not None:ros.clear_command()
        speed=float(np.linalg.norm(robot.get_linear_velocity()[:2]))
        angular=float(np.linalg.norm(robot.get_angular_velocity()))
        if abs(v)+abs(yaw)<1e-6 and speed<.03 and angular<.08:
            if stationary_since is None:stationary_since=world.current_time
        else:stationary_since=None
        requested=None
        if flags.pop('mode',False):requested='walk' if control_mode=='wheel' else 'wheel'
        if ros is not None and ros.mode_request is not None:
            requested=ros.mode_request;ros.mode_request=None
        if requested is not None:
            if requested not in policies:print('MODE_REJECTED missing policy',requested,flush=True)
            elif stationary_since is None or world.current_time-stationary_since<.5:
                print('MODE_REJECTED stop and wait 0.5s before switching',flush=True)
            else:
                control_mode=requested;policy=policies[requested];policy.activate()
                next_policy_time=0.
                print('MODE_ACTIVE',control_mode,flush=True)
        if ros is not None:ros.control_mode=control_mode
        if control_mode=='walk':v=float(np.clip(v,-.1,.1));yaw=float(np.clip(yaw,-.2,.2))
        if a.wheel_replay:
            v=.15 if 2<=world.current_time<8 else 0.
            yaw=0.
            if world.current_time>=11:break
        if a.test:
            v=0.12 if 240<=steps<480 else 0
            yaw=0.3 if 600<=steps<780 else 0
            if steps==240:before=robot.get_world_pose()[0].copy()
            if steps==600:turn_start=robot.get_world_pose()[1].copy()
        update_policy=world.current_time+1e-6>=next_policy_time
        speeds=policy.step(v,yaw,update_policy) if policy is not None else [
            (v+(-1 if leg[1]=='L' else 1)*yaw*0.18)/.05 for leg in ['FL','FR','RL','RR']]
        if update_policy:next_policy_time=world.current_time+1/30
        if update_policy and diagnostic_file:
            wheel_ids=[list(robot.dof_names).index(n+'_foot_joint') for n in ['FL','FR','RL','RR']]
            diagnostic_writer.writerow([world.current_time,v,yaw,*robot.get_linear_velocity()[:2],robot.get_world_pose()[0][2],
                *speeds,*robot.get_joint_velocities()[wheel_ids],
                *(robot.get_joint_positions()[policy.leg_ids] if policy else []),*(policy.targets if policy else [])])
        for drive,speed in zip(wheels,speeds):
            drive.GetTargetVelocityAttr().Set(math.degrees(float(speed)))
        sim_before=world.current_time
        world.step(render=True);steps+=1
        if selection_smoke is not None:selection_smoke.update(world.current_time)
        if asset_rig_smoke is not None:asset_rig_smoke.update(world.current_time)
        if ros is not None:
            if world.current_time+1e-6>=next_ros_time:
                with_image=world.current_time+1e-6>=next_image_time
                ros.publish(world.current_time,with_image=with_image)
                next_ros_time=world.current_time+1/30
                if with_image:next_image_time=world.current_time+1/15
            time.sleep(max(0.,world.current_time-sim_before-(time.monotonic()-tick)))
        if a.steps and steps>=a.steps:break
        if flags['capture'] or (a.test and steps==500):
            rgb=camera.get_rgb()
            if a.test:print('CAMERA_POSE',camera.get_world_pose(),camera.get_local_pose(),flush=True)
            if rgb is not None and rgb.size:
                dest=out/('test_front.png' if a.test else f'front_{__import__("time").time_ns()}.png')
                Image.fromarray(np.asarray(rgb,dtype=np.uint8)).save(dest)
                print('IMAGE_SAVED',dest,flush=True);saved+=1
            flags['capture']=False
        if a.test and steps>=900:
            pos,ori=robot.get_world_pose()
            print('TEST_RESULT',{'position':pos.tolist(),'orientation':ori.tolist(),'delta':(pos-before).tolist(),'images':saved},flush=True)
            actual=robot.get_joint_positions()
            leg_indices=[i for i,n in enumerate(robot.dof_names) if '_foot_joint' not in n]
            error=float(np.max(np.abs(actual[leg_indices]-q[leg_indices])))
            w,x,y,z=ori
            roll=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)))
            pitch=math.degrees(math.asin(np.clip(2*(w*y-z*x),-1,1)))
            print('STANCE',{'cycle':cycle,'roll':roll,'pitch':pitch,'max_leg_error_deg':math.degrees(error),'joints':dict(zip(robot.dof_names,actual.tolist()))},flush=True)
            local_orientation=camera.get_local_pose(camera_axes='world')[1]
            expected=np.array([math.cos(math.radians(2.5)),0,math.sin(math.radians(2.5)),0])
            checks = {'camera_direction':bool(abs(float(np.dot(local_orientation,expected)))>0.99999),
                      'standing':bool(np.isfinite(pos).all() and pos[2]>0.30 and abs(roll)<10 and abs(pitch)<10 and error<0.15),
                      'forward':bool(pos[0]-before[0]>0.05),
                      'turn':bool(abs(float(np.dot(ori,turn_start)))<0.999),
                      'rgb':bool(saved and np.max(np.abs(np.asarray(rgb,dtype=float)[:,:,0]-np.asarray(rgb,dtype=float)[:,:,2]))>30)}
            print('CHECKS',checks, 'PASS' if all(checks.values()) else 'FAIL',flush=True)
            if not all(checks.values()):break
            cycle+=1
            if cycle>=3:break
            world.stop()
            for _ in range(5):world.step(render=True)
            world.play()
            reset_robot()
            steps=0
    if diagnostic_file:diagnostic_file.close()
    if selection_smoke is not None:selection_smoke.finish()
    if asset_rig_smoke is not None:asset_rig_smoke.finish()
    if ros is not None:ros.close()
    if subscription is not None:interface.unsubscribe_to_keyboard_events(keyboard,subscription)
    print('GO2W_SIMULATION_COMPLETE',{'steps_in_last_cycle':steps,'mode':control_mode},flush=True)
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
