"""ROS 2 transport for the local Go2W simulation (standard Jazzy messages)."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Image, CameraInfo, PointCloud2, PointField, Imu
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String


def rotate_inverse(q, v):
    w, x, y, z = q
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) - w * v)


def stamp(seconds):
    ns = max(0, int(round(seconds * 1e9)))
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


class Go2WRos2:
    def __init__(self, robot, camera, image_reliable=False, evaluation_only_odometry=False):
        rclpy.init()
        self.node = Node('go2w_sim')
        self.robot, self.camera = robot, camera
        self.command = (0., 0.)
        self.mode_request=None
        self.control_mode='wheel'
        self.evaluation_only_odometry = evaluation_only_odometry
        self.node.create_subscription(String,'/go2w/control_mode/cmd',self.on_mode,10)
        self.mode_pub=self.node.create_publisher(String,'/go2w/control_mode',10)
        self.received = -math.inf
        self.node.create_subscription(Twist, '/go2w/cmd_vel', self.on_command, 10)
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.odom = self.node.create_publisher(Odometry, '/evaluation/go2w/odom' if evaluation_only_odometry else '/go2w/odom', 10)
        self.joints = self.node.create_publisher(JointState, '/go2w/joint_states', qos_profile_sensor_data)
        image_qos=QoSProfile(depth=20) if evaluation_only_odometry else (QoSProfile(depth=5) if image_reliable else qos_profile_sensor_data)
        self.image = self.node.create_publisher(Image, '/go2w/front_camera/image_raw', image_qos)
        self.info = self.node.create_publisher(CameraInfo, '/go2w/front_camera/camera_info', image_qos)
        self.tf = self.node.create_publisher(TFMessage, '/tf', 10)
        self.eval_tf = self.node.create_publisher(TFMessage, '/evaluation/tf', 10) if evaluation_only_odometry else self.tf
        self.sensor_publishers = {'/go2w/front_camera/image_raw':self.image,'/go2w/front_camera/camera_info':self.info}
        self.lidar_points = self.node.create_publisher(PointCloud2, '/go2w/lidar/points', QoSProfile(depth=20) if evaluation_only_odometry else qos_profile_sensor_data)
        self.static = self.node.create_publisher(TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if not evaluation_only_odometry:self.publish_static()
        print('ROS2_READY /go2w/cmd_vel -> simulation; watchdog=0.5s', flush=True)

    def on_command(self, msg):
        values = (msg.linear.x, msg.angular.z)
        if not all(math.isfinite(x) for x in values):
            self.clear_command()
            return
        self.command = (float(np.clip(values[0], -.3, .3)), float(np.clip(values[1], -.6, .6)))
        self.received = time.monotonic()

    def on_mode(self,msg):
        if msg.data in ('wheel','walk'):self.mode_request=msg.data
        else:self.node.get_logger().warning('Mode must be wheel or walk')

    def clear_command(self):
        self.command = (0., 0.)
        self.received = -math.inf

    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.)

    def velocity(self):
        return self.command if time.monotonic() - self.received <= .5 else (0., 0.)

    def transform(self, parent, child, pos, quat, t):
        msg = TransformStamped()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = t, parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, pos)
        msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z = map(float, quat)
        return msg

    def publish_static(self):
        t = stamp(0)
        self.static.publish(TFMessage(transforms=[
            self.transform('base_link', 'front_camera_link', [.32, 0, .08], [math.cos(math.radians(2.5)), 0, math.sin(math.radians(2.5)), 0], t),
            self.transform('front_camera_link', 'front_camera_optical_frame', [0, 0, 0], [.5, -.5, .5, -.5], t),
        ]))

    def publish(self, sim_time, with_image=False):
        t = stamp(sim_time)
        self.clock.publish(Clock(clock=t))
        self.mode_pub.publish(String(data=self.control_mode))
        pos, quat = self.robot.get_world_pose()
        msg = Odometry()
        root_frame = 'simulation_world' if self.evaluation_only_odometry else 'odom'
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = t, root_frame, 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, pos)
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = map(float, quat)
        linear = rotate_inverse(quat, np.asarray(self.robot.get_linear_velocity()))
        angular = rotate_inverse(quat, np.asarray(self.robot.get_angular_velocity()))
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, linear)
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, angular)
        self.odom.publish(msg)  # Simulator ground truth, not a wheel/IMU estimator.
        self.eval_tf.publish(TFMessage(transforms=[self.transform(root_frame, 'base_link', pos, quat, t)]))
        joints = JointState()
        joints.header.stamp, joints.header.frame_id = t, 'base_link'
        joints.name = list(self.robot.dof_names)
        joints.position = self.robot.get_joint_positions().astype(float).tolist()
        joints.velocity = self.robot.get_joint_velocities().astype(float).tolist()
        self.joints.publish(joints)
        if not with_image:
            return
        rgb = self.camera.get_rgb()
        if rgb is None or not rgb.size:
            return
        rgb = np.ascontiguousarray(rgb[:, :, :3], dtype=np.uint8)
        h, w = rgb.shape[:2]
        image = Image()
        image.header.stamp, image.header.frame_id = t, 'front_camera_optical_frame'
        image.height, image.width, image.encoding, image.step = h, w, 'rgb8', w*3
        image.data = rgb.tobytes()
        self.image.publish(image)
        info = CameraInfo()
        info.header = image.header
        info.height, info.width = h, w
        fx = w * self.camera.get_focal_length() / self.camera.get_horizontal_aperture()
        fy = h * self.camera.get_focal_length() / self.camera.get_vertical_aperture()
        cx, cy = w/2, h/2
        info.distortion_model, info.d = 'plumb_bob', [0.]*5
        info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
        self.info.publish(info)

    def publish_lidar_points(self, xyz, timestamps_ns, emitter_ids, sim_time,
                             sensor_position_world, sensor_orientation_world,
                             base_position_world, base_orientation_world,
                             exact_time_fields=False, publish_tf=True):
        """Publish only verified raw fields; no intensity or channel/ring is synthesized."""
        points = np.asarray(xyz, dtype=np.float32)
        point_time = np.asarray(timestamps_ns, dtype=np.uint64)
        emitter = np.asarray(emitter_ids, dtype=np.uint32)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) != len(point_time) or len(points) != len(emitter):
            raise ValueError('RTX lidar xyz/timestamp/emitterId arrays must have matching point counts')
        valid = np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > 1e-6)
        points, point_time, emitter = points[valid], point_time[valid], emitter[valid]
        if not len(points):
            return None
        names = ['x', 'y', 'z', 'time_from_scan_start', 'emitter_id']
        formats, offsets, itemsize = ['<f4', '<f4', '<f4', '<f8', '<u4'], [0,4,8,12,20], 24
        if exact_time_fields:
            names += ['time_sec','time_nanosec']; formats += ['<u4','<u4']; offsets += [24,28]; itemsize = 32
        dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': itemsize})
        records = np.empty(len(points), dtype=dtype)
        records['x'], records['y'], records['z'] = points.T
        records['time_from_scan_start'] = (point_time.astype(np.float64) - float(point_time.min())) * 1e-9
        records['emitter_id'] = emitter
        if exact_time_fields:
            records['time_sec'] = point_time // 1000000000
            records['time_nanosec'] = point_time % 1000000000
        msg = PointCloud2()
        msg.header.stamp = stamp(sim_time)
        msg.header.frame_id = 'go2w_lidar'
        msg.height, msg.width = 1, int(len(records))
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='time_from_scan_start', offset=12, datatype=PointField.FLOAT64, count=1),
            PointField(name='emitter_id', offset=20, datatype=PointField.UINT32, count=1),
        ]
        if exact_time_fields:
            msg.fields += [PointField(name='time_sec',offset=24,datatype=PointField.UINT32,count=1),
                           PointField(name='time_nanosec',offset=28,datatype=PointField.UINT32,count=1)]
        msg.is_bigendian = False
        msg.point_step, msg.row_step = itemsize, itemsize * len(records)
        msg.data = records.tobytes()
        msg.is_dense = True
        self.lidar_points.publish(msg)
        if not publish_tf: return msg

        # Publish the sensor's measured run-time mount relative to base_link.
        base_q = np.asarray(base_orientation_world, dtype=float)
        sensor_q = np.asarray(sensor_orientation_world, dtype=float)
        bw, bx, by, bz = base_q / np.linalg.norm(base_q)
        inverse_base = np.array([bw, -bx, -by, -bz])
        sw, sx, sy, sz = sensor_q / np.linalg.norm(sensor_q)
        iw, ix, iy, iz = inverse_base
        relative_q = np.array([iw*sw-ix*sx-iy*sy-iz*sz,
                               iw*sx+ix*sw+iy*sz-iz*sy,
                               iw*sy-ix*sz+iy*sw+iz*sx,
                               iw*sz+ix*sy-iy*sx+iz*sw])
        relative_p = rotate_inverse(base_q, np.asarray(sensor_position_world, dtype=float) - np.asarray(base_position_world, dtype=float))
        self.tf.publish(TFMessage(transforms=[self.transform('base_link', 'go2w_lidar', relative_p, relative_q, msg.header.stamp)]))
        return msg

    def sensor_publisher(self, topic, message_type):
        if topic not in self.sensor_publishers:
            self.sensor_publishers[topic] = self.node.create_publisher(message_type, topic, QoSProfile(depth=20))
        return self.sensor_publishers[topic]

    def publish_sensor_image(self, topic, array, exposure_s, frame_id, encoding, K=None, baseline_m=0.):
        image = Image()
        image.header.stamp, image.header.frame_id = stamp(exposure_s), frame_id
        data = np.ascontiguousarray(array, dtype=np.float32 if encoding=='32FC1' else np.uint8)
        image.height, image.width = data.shape[:2]
        image.encoding, image.is_bigendian = encoding, False
        image.step = image.width * (4 if encoding=='32FC1' else 3)
        image.data = data.tobytes()
        self.sensor_publisher(topic, Image).publish(image)
        if K is not None:
            info = CameraInfo(); info.header = image.header
            info.height, info.width = image.height, image.width
            info.distortion_model, info.d = 'plumb_bob', [0.]*5
            info.k = np.asarray(K,dtype=float).reshape(-1).tolist()
            info.r = np.eye(3).reshape(-1).tolist()
            k = np.asarray(K,dtype=float)
            info.p = [k[0,0],0.,k[0,2],-k[0,0]*baseline_m,0.,k[1,1],k[1,2],0.,0.,0.,1.,0.]
            self.sensor_publisher(topic.rsplit('/',1)[0]+'/camera_info', CameraInfo).publish(info)

    def publish_sensor_imu(self, reading):
        msg = Imu(); msg.header.stamp = stamp(float(reading['time'])); msg.header.frame_id='go2w_imu_link'
        msg.orientation.w=1.; msg.orientation_covariance[0]=-1.
        msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=map(float,reading['linear_acceleration'])
        msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=map(float,reading['angular_velocity'])
        self.sensor_publisher('/go2w/imu/data',Imu).publish(msg)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
