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
from sensor_msgs.msg import JointState, Image, CameraInfo
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
    def __init__(self, robot, camera, image_reliable=False):
        rclpy.init()
        self.node = Node('go2w_sim')
        self.robot, self.camera = robot, camera
        self.command = (0., 0.)
        self.mode_request=None
        self.control_mode='wheel'
        self.node.create_subscription(String,'/go2w/control_mode/cmd',self.on_mode,10)
        self.mode_pub=self.node.create_publisher(String,'/go2w/control_mode',10)
        self.received = -math.inf
        self.node.create_subscription(Twist, '/go2w/cmd_vel', self.on_command, 10)
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.odom = self.node.create_publisher(Odometry, '/go2w/odom', 10)
        self.joints = self.node.create_publisher(JointState, '/go2w/joint_states', qos_profile_sensor_data)
        image_qos=QoSProfile(depth=5) if image_reliable else qos_profile_sensor_data
        self.image = self.node.create_publisher(Image, '/go2w/front_camera/image_raw', image_qos)
        self.info = self.node.create_publisher(CameraInfo, '/go2w/front_camera/camera_info', qos_profile_sensor_data)
        self.tf = self.node.create_publisher(TFMessage, '/tf', 10)
        self.static = self.node.create_publisher(TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publish_static()
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
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = t, 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, pos)
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = map(float, quat)
        linear = rotate_inverse(quat, np.asarray(self.robot.get_linear_velocity()))
        angular = rotate_inverse(quat, np.asarray(self.robot.get_angular_velocity()))
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, linear)
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, angular)
        self.odom.publish(msg)  # Simulator ground truth, not a wheel/IMU estimator.
        self.tf.publish(TFMessage(transforms=[self.transform('odom', 'base_link', pos, quat, t)]))
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

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
