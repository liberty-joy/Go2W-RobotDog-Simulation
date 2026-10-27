"""Opt-in ROS sensor mode using the existing Go2-W controller and sensor models."""
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np

from sensor_pipeline.stage5_fusion_capture import Stage5FusionCapture
from sensor_pipeline.process_stage3_stereo import run_matcher


class Stage6RosSensors(Stage5FusionCapture):
    def __init__(self, config_path, output_path, root, stage, base_path, public_camera):
        self.scene_index = 0
        super().__init__(config_path, output_path, root, stage, base_path, public_camera)
        self.last_image_ns = -1
        self.last_imu_s = -1.
        self.last_sim_s = -1.
        self.static_sent = False
        self.counts = {}
        self.scene_started_s = float(self.config['warmup_s'])
        self.events = (self.out/'publisher_events.jsonl').open('w')
        self.params = json.loads((self.root/self.config['sgbm_config']).read_text())
        self.depth_times_ms = []
        self.mismatches = 0
        paths = [self.config_path, self.root/self.config['sgbm_config'], self.root/self.config['stereo_config'],
                 self.root/self.config['lidar_profile'], Path(__file__), self.root/'scripts/go2w_ros2.py']
        (self.out/'provenance.json').write_text(json.dumps({
            'files':[{ 'path':str(p.relative_to(self.root)), 'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in paths],
            'isaac_sim':'6.2.0','opencv':cv2.__version__,'numpy':np.__version__,
            'pose_scope':'evaluation only; no world localization input to A/B estimators'},indent=2))

    def _create_panel(self, Gf, UsdGeom, Vt):
        scene = self.config['scenes'][self.scene_index]
        ny,nz = 24,20
        width,height = scene['width_m'],scene['height_m']
        points=[Gf.Vec3f(0,-width/2+width*i/ny,-height/2+height*j/nz) for j in range(nz+1) for i in range(ny+1)]
        rng=np.random.default_rng(scene['seed']); counts=[];indices=[];colors=[]
        for j in range(nz):
            for i in range(ny):
                a=j*(ny+1)+i;b,c,d=a+1,a+ny+1,a+ny+2
                if scene['texture']=='textureless': color=(.7,.06,.04)
                elif scene['texture']=='repetitive':color=(.9,.08,.04) if (i+j)%2 else (.4,.03,.02)
                else:color=(float(rng.uniform(.5,.95)),float(rng.uniform(.025,.12)),float(rng.uniform(.02,.1)))
                for tri in ((a,d,c),(a,b,d)):
                    counts.append(3);indices.extend(tri);colors.append(Gf.Vec3f(*color))
        mesh=UsdGeom.Mesh.Define(self.stage,'/World/Stage5RedPanel')
        mesh.CreatePointsAttr(Vt.Vec3fArray(points));mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices));mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform).Set(Vt.Vec3fArray(colors))
        UsdGeom.XformCommonAPI(mesh.GetPrim()).SetTranslate(Gf.Vec3d(*map(float,scene['center_world_m'])))

    def command(self): return 0.,0.

    def _event(self, kind, **values):
        self.events.write(json.dumps({'event':kind,**values})+'\n');self.events.flush()

    def _count(self,topic):self.counts[topic]=self.counts.get(topic,0)+1

    def _mounts(self, ros):
        from tf2_msgs.msg import TFMessage
        from go2w_ros2 import stamp
        transforms=[]
        for frame,cam in [('go2w_stereo_left_optical_frame',self.left),('go2w_stereo_right_optical_frame',self.right)]:
            p,q=cam.get_local_pose(camera_axes='ros')
            transforms.append(ros.transform('base_link',frame,p,q,stamp(0)))
        transforms += [ros.transform('base_link','go2w_lidar',self.rig_config['lidar']['translation_base_m'],[1,0,0,0],stamp(0)),
                       ros.transform('base_link','go2w_imu_link',self.rig_config['imu']['translation_base_m'],[1,0,0,0],stamp(0))]
        # Include the legacy public camera mount in the same transient-local sample.
        p,q=self.public.get_local_pose(camera_axes='ros')
        transforms.append(ros.transform('base_link','front_camera_optical_frame',p,q,stamp(0)))
        ros.static.publish(TFMessage(transforms=transforms));self.static_sent=True

    def update(self, simulation_time, robot, ros):
        from std_msgs.msg import String
        from pxr import Gf,UsdGeom,Vt
        t=float(simulation_time)
        if self.done:return
        if t < self.last_sim_s:
            self.last_image_ns=-1;self.seen_scan.clear();self.last_imu_s=-1.
            self._event('clock_reset_sync_queues_cleared',previous_s=self.last_sim_s,new_s=t)
        self.last_sim_s=t
        if not self.static_sent:self._mounts(ros)
        duration=self.config['scene_duration_s'];warmup=self.config['warmup_s']
        index=max(0,int((t-warmup)//duration))
        if index>=len(self.config['scenes']):
            self.done=True;self.finish();return
        if index!=self.scene_index:
            self.scene_index=index
            self.stage.RemovePrim('/World/Stage5RedPanel');self._create_panel(Gf,UsdGeom,Vt)
            self._event('scene_transition',sim_time_s=t,scene_id=self.config['scenes'][index]['scene_id'])
        scene=self.config['scenes'][index]
        elapsed=t-(warmup+index*duration)
        stable=elapsed>=self.config['scene_settle_s']
        reading=self.imu.get_data()
        if reading and self.imu.get_sensor_reading().is_valid and float(reading['time'])>self.last_imu_s:
            ros.publish_sensor_imu(reading);self.last_imu_s=float(reading['time']);self._count('/go2w/imu/data')
        poses={'simulation_time_s':t,'scene_id':scene['scene_id'],'object_id':scene['object_id'],
               'public':self._pose(self.public,camera=True),'left':self._pose(self.left,camera=True),
               'lidar':self._pose(self.lidar),'base':self._pose(robot),
               'object':{k:scene[k] for k in ('center_world_m','width_m','height_m')}}
        ros.sensor_publisher('/evaluation/go2w/poses',String).publish(String(data=json.dumps(poses)))
        self._count('/evaluation/go2w/poses')
        item=self._new_scan()
        if item is not None:
            arrays,frame,scan=item
            ps,qs=self.lidar.get_world_pose();pb,qb=robot.get_world_pose()
            end_s=float(np.max(arrays['timestamp']))*1e-9
            self.latest_lidar_scan_end_ns=int(np.max(arrays['timestamp']))
            msg=ros.publish_lidar_points(arrays['data'],arrays['timestamp'],arrays['emitterId'],end_s,
                                        ps,qs,pb,qb,exact_time_fields=True,publish_tf=False)
            if msg is not None:self._count('/go2w/lidar/points')
        if hasattr(self,'image_publish_ready') and not self.image_publish_ready(t):return
        cameras=[('public',self.public),('left',self.left),('right',self.right)]
        times=[cam.get_current_frame().get('rendering_time') for name,cam in cameras]
        if any(x is None for x in times):return
        image_ns=int(round(float(times[0])*1e9))
        if image_ns<=self.last_image_ns:return
        if max(times)-min(times)>1e-9:
            self.mismatches+=1;return
        images={name:np.asarray(cam.get_rgb()) for name,cam in cameras}
        if any(im.shape!=(480,640,3) for im in images.values()):return
        self.last_image_ns=image_ns
        start=time.perf_counter()
        calib={'K_left':self.K['left'],'K_right':self.K['right'],'D_left':[0.]*5,'D_right':[0.]*5,
               'R_left_to_right':np.eye(3).tolist(),'T_left_to_right_m':[-.1,0,0]}
        result=run_matcher(cv2.cvtColor(images['left'],cv2.COLOR_RGB2BGR),cv2.cvtColor(images['right'],cv2.COLOR_RGB2BGR),calib,self.params)
        elapsed_ms=(time.perf_counter()-start)*1000;self.depth_times_ms.append(elapsed_ms)
        for name,topic,frame_id in [('public','/go2w/front_camera/image_raw','front_camera_optical_frame'),
                                   ('left','/go2w/stereo/left/image_raw','go2w_stereo_left_optical_frame'),
                                   ('right','/go2w/stereo/right/image_raw','go2w_stereo_right_optical_frame')]:
            ros.publish_sensor_image(topic,images[name],times[0],frame_id,'rgb8',self.K[name],.1 if name=='right' else 0.)
            self._count(topic);self._count(topic.rsplit('/',1)[0]+'/camera_info')
        topic='/go2w/stereo/depth/image_raw'
        ros.publish_sensor_image(topic,result['depth_z_m'],times[0],'go2w_stereo_left_optical_frame','32FC1',result['P1'][:3,:3])
        self._count(topic);self._count('/go2w/stereo/depth/camera_info')
        metadata={'stamp_ns':image_ns,'scene_id':scene['scene_id'],'object_id':scene['object_id'],'split':scene['split'],
                  'scene_elapsed_s':elapsed,'stable':stable,'depth_runtime_ms':elapsed_ms,'depth_source':'StereoSGBM left/right RGB',
                  'publication_sim_s':t,'generation':0}
        ros.sensor_publisher('/go2w/sensors/scene',String).publish(String(data=json.dumps(metadata)))
        self._count('/go2w/sensors/scene')
        self._event('synchronized_image_bundle',**metadata)

    def finish(self):
        if self.results_written:return
        if not self.done:
            self._event('incomplete',last_sim_s=self.last_sim_s)
            raise RuntimeError('Stage6 sensor session ended before all frozen scenes')
        result={'status':'complete','counts':self.counts,'final_sim_time_s':self.last_sim_s,
                'rejected_image_sync_attempts':self.mismatches,
                'depth_compute_p50_ms':float(np.percentile(self.depth_times_ms,50)),
                'depth_compute_p95_ms':float(np.percentile(self.depth_times_ms,95)),
                'truth_namespace':'/evaluation/','imu':'ideal local gyro rad/s and specific force m/s^2; orientation unavailable',
                'gui':'not tested'}
        (self.out/'publisher_summary.json').write_text(json.dumps(result,indent=2));self.events.close()
        self.results_written=True
        print('STAGE6_SENSOR_PUBLICATION_COMPLETE',json.dumps(result),flush=True)
