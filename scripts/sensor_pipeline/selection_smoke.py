"""Small optional sensor/API smoke; no SDK, ROS driver, matching or fusion."""
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image


class SelectionSmoke:
    def __init__(self,config,out,stage,base_path,left):
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.sensors.rtx')
        enable_extension('isaacsim.sensors.experimental.physics')
        enable_extension('isaacsim.sensors.experimental.rtx')
        from isaacsim.sensors.camera import Camera
        from isaacsim.sensors.rtx import LidarRtx
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor
        self.cfg=json.loads(Path(config).read_text());self.out=Path(out);self.out.mkdir(parents=True,exist_ok=False)
        self.left=left;s=self.cfg['stereo'];pitch=math.radians(s['pitch_down_deg'])/2
        if (s['width'],s['height'],s['horizontal_fov_deg'],s['pitch_down_deg'],s['left_translation_base_m']) != (640,480,80,5,[.32,0,.08]):
            raise ValueError('Stage1 public-left viewpoint is fixed to stage0; adapt both camera paths before changing it')
        if not np.isclose(np.linalg.norm(np.array(s['right_translation_base_m'])-np.array(s['left_translation_base_m'])),s['baseline_m']):
            raise ValueError('Stereo translation must match baseline_m')
        self.right=Camera(prim_path=base_path+'/SelectionRightCamera',resolution=(s['width'],s['height']),frequency=s['frequency_hz'],translation=np.array(s['right_translation_base_m']))
        self.right.initialize()
        self.right.set_local_pose(translation=np.array(s['right_translation_base_m']),orientation=np.array([math.cos(pitch),0,math.sin(pitch),0]),camera_axes='world')
        self.right.set_focal_length(.018)
        self.right.set_horizontal_aperture(2*.018*math.tan(math.radians(s['horizontal_fov_deg']/2)))
        self.right.set_clipping_range(.02,100)
        from isaacsim.sensors.experimental.rtx import Lidar
        profile=json.loads(Path(self.cfg['lidar']['profile']).read_text())['profile']
        attrs={f'omni:sensor:Core:{k}':v for k,v in {
            'scanType':profile['scanType'].upper(),'rotationDirection':profile['rotationDirection'],
            'numberOfEmitters':profile['numberOfEmitters'],'numberOfChannels':profile['numberOfEmitters'],
            'patternFiringRateHz':int(profile['reportRateBaseHz']),
            'scanRateBaseHz':int(profile['scanRateBaseHz']),
            'nearRangeM':profile['nearRangeM'],'farRangeM':profile['farRangeM'],
            'azimuthErrorStd':0.,'elevationErrorStd':0.,'rangeAccuracyM':0.,
            'accumulateOutputs':True}.items()}
        for k,v in profile['emitterStates'][0].items():attrs['omni:sensor:Core:emitterState:s001:'+k]=v
        attrs['omni:sensor:Core:emitterState:s001:channelId']=list(range(1,profile['numberOfEmitters']+1))
        attrs['omni:sensor:Core:emitterState:s001:bank']=[]
        authored=Lidar.create(base_path+'/SelectionLidar',tick_rate=profile['scanRateBaseHz'],aux_output_level='FULL',
            translations=[self.cfg['lidar']['translation_base_m']],attributes={k:v for k,v in attrs.items() if not isinstance(v,list)})
        for k,v in attrs.items():
            if isinstance(v,list):
                attr=stage.GetPrimAtPath(base_path+'/SelectionLidar').GetAttribute(k)
                assert attr.IsValid(),k
                attr.Set(v)
        self.authored_attributes=attrs
        self.lidar=LidarRtx(prim_path=base_path+'/SelectionLidar')
        self.lidar.attach_annotator('IsaacCreateRTXLidarScanBuffer',outputTimestamp=True,outputEmitterId=True)
        self.lidar.initialize()
        imu_path=base_path+'/SelectionIMU'
        self.imu=IMUSensor(IMU.create(imu_path,translations=[self.cfg['imu']['translation_base_m']]))
        self.done=False;self.last_keys=[]

    def update(self,t):
        if self.done or t<2:return
        left=self.left.get_rgb();right=self.right.get_rgb()
        frame=self.lidar.get_current_frame();scan=frame.get('IsaacCreateRTXLidarScanBuffer',{})
        self.last_keys=list(scan) if isinstance(scan,dict) else []
        xyz=scan.get('data',np.empty((0,3))) if isinstance(scan,dict) else np.empty((0,3))
        imu=self.imu.get_data()
        imu['is_valid']=bool(self.imu.get_sensor_reading().is_valid)
        self.last_status={'left':None if left is None else list(left.shape),'right':None if right is None else list(right.shape),'points':len(xyz),'imu_valid':imu.get('is_valid')}
        if left is None or right is None or not left.size or not right.size or not len(xyz) or not imu.get('is_valid'):return
        Image.fromarray(np.asarray(left,dtype=np.uint8)).save(self.out/'left.png')
        Image.fromarray(np.asarray(right,dtype=np.uint8)).save(self.out/'right.png')
        np.save(self.out/'lidar_xyz.npy',xyz)
        saved={}
        for name,value in scan.items():
            if isinstance(value,np.ndarray):
                saved[name]={'shape':list(value.shape),'dtype':str(value.dtype)}
                if name in ['timestamp','emitterId']:np.save(self.out/(name+'.npy'),value)
        result={'simulation_time':t,'left_shape':list(left.shape),'right_shape':list(right.shape),'point_count':len(xyz),
            'lidar_fields':saved,'imu_valid':bool(imu['is_valid']),
            'imu':{k:np.asarray(v).tolist() for k,v in imu.items()},
            'left_rendering_frame':str(self.left.get_current_frame().get('rendering_frame')),
            'right_rendering_frame':str(self.right.get_current_frame().get('rendering_frame')),
            'left_rendering_time':str(self.left.get_current_frame().get('rendering_time')),
            'right_rendering_time':str(self.right.get_current_frame().get('rendering_time')),
            'scope':'API/measurement smoke only; no matching, calibration, synchronization acceptance, ROS lidar or fusion',
            'config':self.cfg,'authored_lidar_attributes':self.authored_attributes}
        (self.out/'smoke.json').write_text(json.dumps(result,indent=2));self.done=True
        print('SELECTION_SENSOR_SMOKE_PASS',len(xyz),flush=True)

    def finish(self):
        if not self.done:
            (self.out/'failure.json').write_text(json.dumps({'error':'No complete sensor sample before exit','last_lidar_keys':self.last_keys,'last_status':getattr(self,'last_status',{})},indent=2))
            raise RuntimeError('Selection sensor smoke did not produce data')
