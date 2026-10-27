"""Stage 7 optional scene and sensors; motor control remains in the existing ROS loop."""
import json,time
import numpy as np
from pathlib import Path
from sensor_pipeline.stage6_ros_sensors import Stage6RosSensors

class Stage7Sensors(Stage6RosSensors):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  from pxr import UsdGeom,UsdPhysics,Gf
  from isaacsim.sensors.camera import Camera
  from scipy.spatial.transform import Rotation
  self.image_tick=0;self.image_stride=int(round(60/self.stereo_config['frequency_hz']))
  for cam in (self.public,self.left,self.right):cam.set_frequency(-1)
  self.next_video=0.;self.video_index=0;self.dynamic_added=False
  self.video_dir=self.out/'video_frames';self.video_dir.mkdir()
  bounds=self.config['navigation']['map_bounds_m'];x0,y0,x1,y1=bounds
  boxes=[('wall_s',[(x0+x1)/2,y0,.7],[x1-x0+.12,.12,1.4]),('wall_n',[(x0+x1)/2,y1,.7],[x1-x0+.12,.12,1.4]),('wall_w',[x0,(y0+y1)/2,.7],[.12,y1-y0,1.4]),('wall_e',[x1,(y0+y1)/2,.7],[.12,y1-y0,1.4])]
  boxes += [(b['id'],b['center'],b['size']) for b in self.config['navigation']['static_boxes']]
  for name,center,size in boxes:self._box(name,center,size)
  self.overview=Camera(prim_path='/World/Stage7Overview',resolution=(640,480),frequency=5)
  self.overview.initialize();eye=np.array([2.1,3.5,3.5]);target=np.array([.8,0,.1]);forward=(target-eye)/np.linalg.norm(target-eye);right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);down=np.cross(forward,right);q=Rotation.from_matrix(np.column_stack([right,down,forward])).as_quat()
  self.overview.set_world_pose(position=eye,orientation=q[[3,0,1,2]],camera_axes='ros');self.overview.set_clipping_range(.05,30.)
  self._event('camera_shared_frame_schedule',readout='all actual render frames via Camera.set_frequency(-1)',publish_selection='one actual common exposure after each new lidar scan end, causal <=50ms',nominal_output_hz=self.stereo_config['frequency_hz'],stamp='unmodified actual rendering_time; equality still required')
  self._event('stage7_scene_ready',map_source='explicit known floorplan; not reconstructed ground truth pose',controller='existing ROS velocity/watchdog')
 def image_publish_ready(self,t):
  # Select a real common frame immediately after a new completed scan, not an arbitrary phase.
  end=getattr(self,'latest_lidar_scan_end_ns',None)
  if end is None or end==getattr(self,'last_selected_scan_end_ns',None):return False
  times=[c.get_current_frame().get('rendering_time') for c in (self.public,self.left,self.right)]
  if any(x is None for x in times) or max(times)-min(times)>1e-9:return False
  exposure=int(round(float(times[0])*1e9))
  if not 0<=exposure-end<=50000000:return False
  self.last_selected_scan_end_ns=end
  return True
 def _box(self,name,center,size):
  from pxr import UsdGeom,UsdPhysics,Gf
  box=UsdGeom.Cube.Define(self.stage,'/World/Stage7_'+name);box.CreateSizeAttr(1);box.CreateDisplayColorAttr([Gf.Vec3f(.4,.45,.5)]);xf=UsdGeom.XformCommonAPI(box);xf.SetScale(Gf.Vec3f(*size));xf.SetTranslate(Gf.Vec3d(*center));UsdPhysics.CollisionAPI.Apply(box.GetPrim())
 def update(self,t,robot,ros):
  import cv2
  if (self.out/'add_obstacle').exists() and not self.dynamic_added:
   b=self.config['navigation']['dynamic_test_box'];self._box('dynamic_blocker',b['center'],b['size']);self.dynamic_added=True;self._event('physical_obstacle_added',simulation_time_s=t)
  if (self.out/'navigation_done.json').exists():
   ros.clear_command();self.done=True;self.finish();return
  super().update(t,robot,ros)
  if t>=self.next_video:
   rgb=self.overview.get_rgb()
   if rgb is not None and np.asarray(rgb).shape==(480,640,3):
    bgr=cv2.cvtColor(np.asarray(rgb),cv2.COLOR_RGB2BGR);cv2.putText(bgr,f'Actual Isaac simulation t={t:.2f}s',(8,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1);cv2.imwrite(str(self.video_dir/f'{self.video_index:06d}.jpg'),bgr);self.video_index+=1
   self.next_video=t+.25
