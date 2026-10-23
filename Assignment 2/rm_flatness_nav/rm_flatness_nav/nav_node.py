"""ROS 2 node: AprilTag search, in-place alignment, flat-output pursuit, standoff brake,
and 1 Hz motor-voltage logging for the RoboMaster EP.

Subscribes : camera/image_color, camera/camera_info (sensor_msgs), range_0 (ToF, optional), TF
Publishes  : cmd_vel (geometry_msgs/Twist), target_arm_position (geometry_msgs/Point, at start)
"""
import csv
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Range
import tf2_ros

from .state_machine import ARRIVED, NavStateMachine, Observation, Params
from .tag_detector import TagDetector, camera_rotation, ground_range_bearing
from .wheel_voltage import MotorVoltageModel, wheel_rates

CSV_HEADER = ['t_s', 'V1_V', 'V2_V', 'V3_V', 'V4_V', 'state',
              'v_mps', 'w_radps', 'distance_m', 'bearing_rad', 'tof_m',
              'w1_radps', 'w2_radps', 'w3_radps', 'w4_radps']


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class GrabThread(threading.Thread):
    """Optional direct OpenCV capture (camera_source:=cv2).  Implements the assignment hint:
    a short sleep between cap.grab() calls lets the decoder recycle its buffers."""

    def __init__(self, url):
        super().__init__(daemon=True)
        self.cap, self.frame, self.lock, self.running = cv2.VideoCapture(url), None, threading.Lock(), True

    def run(self):
        while self.running:
            if self.cap.grab():
                ok, frame = self.cap.retrieve()
                if ok:
                    with self.lock:
                        self.frame = frame
            time.sleep(0.010)

    def latest(self):
        with self.lock:
            return self.frame


class FlatnessNav(Node):
    def __init__(self):
        super().__init__('flatness_nav')
        d = self.declare_parameter
        self.tag_family = d('tag_family', 'tag36h11').value
        self.tag_id = d('tag_id', 3).value
        self.tag_size = d('tag_size', 0.10).value          # metres, printed black-square side: MEASURE IT
        self.image_topic = d('image_topic', 'camera/image_color').value
        self.info_topic = d('info_topic', 'camera/camera_info').value
        self.tof_topic = d('tof_topic', 'range_0').value
        self.use_tof = d('use_tof', True).value
        self.base_frame = d('base_frame', 'base_link').value
        self.camera_frame = d('camera_frame', 'camera_optical_link').value
        # The camera is fixed to the chassis (verified from images), so the arm/URDF TF is NOT used by default.
        self.fallback_pitch_deg = d('camera_pitch_deg', 20.0).value   # measured: about 17-23 deg below horizontal
        self.use_tf = d('use_tf_camera', False).value
        self.control_rate = d('control_rate', 20.0).value
        self.csv_path = d('csv_path', 'motor_voltage_log.csv').value
        self.fast_path = d('fast_csv_path', 'motor_voltage_fast.csv').value   # 20 Hz log for the v(t), V1(t) plot
        self.set_arm = d('set_arm_pose', True).value
        self.arm_x, self.arm_z = d('arm_x_m', 0.105).value, d('arm_z_m', 0.030).value
        self.camera_source = d('camera_source', 'ros').value          # 'ros' or 'cv2'
        self.camera_url = d('camera_url', '').value
        self.detect_fresh = d('detection_fresh_s', 0.20).value
        fsm_params = Params()
        for name in Params.__dataclass_fields__:
            setattr(fsm_params, name, d(f'fsm.{name}', getattr(fsm_params, name)).value)
        self.fsm = NavStateMachine(fsm_params)
        self.voltage = MotorVoltageModel(d('kv', 0.38).value, d('ka', 0.06).value)
        self.r = d('wheel_radius', 0.05).value
        self.lc = d('chassis_radius', 0.20).value

        self.detector = TagDetector(self.tag_family, self.tag_id, self.tag_size)
        self.bridge = CvBridge()
        self.K, self.dist = None, None
        self.det_time, self.bearing, self.distance = -1e9, 0.0, float('inf')
        self.tof = None
        self.r_bc, self.tf_ok = camera_rotation(math.radians(self.fallback_pitch_deg)), False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.grab = None

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_pub = self.create_publisher(Point, 'target_arm_position', 1)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, 1)
        if self.camera_source == 'ros':
            self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        else:
            self.grab = GrabThread(self.camera_url)
            self.grab.start()
        if self.use_tof:
            self.create_subscription(Range, self.tof_topic, self.on_range, qos_profile_sensor_data)

        self.t_start = self.now()
        self.prev_tick = self.t_start
        self.next_log = 0.0
        self.volts, self.omegas = (0.0,) * 4, (0.0,) * 4
        self.v_cmd = self.w_cmd = 0.0
        self.stop_ticks = 0
        self.finished = False
        self._csv_file = open(self.csv_path, 'w', newline='')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_HEADER)
        self._fast_file = open(self.fast_path, 'w', newline='')
        self._fast = csv.writer(self._fast_file)
        self._fast.writerow(['t_s', 'v_mps', 'w_radps', 'V1_V', 'V2_V', 'V3_V', 'V4_V', 'state'])
        self.create_timer(1.0 / self.control_rate, self.tick)
        if self.set_arm:
            self._arm_sends = 0
            self.create_timer(1.0, self.send_arm)
        self.get_logger().info(
            f'Tag {self.tag_family} id={self.tag_id} size={self.tag_size} m. '
            f'Table header: t(s) V1 V2 V3 V4 state. CSV -> {os.path.abspath(self.csv_path)}')
        if self.tag_size == 0.10:
            self.get_logger().warn('tag_size is the 0.10 m default: set it to the MEASURED tag side length.')

    # ---- helpers -------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send_arm(self):
        """Arm pose (X=105 mm, Y=30 mm): the driver's second coordinate is the vertical `z`."""
        self.arm_pub.publish(Point(x=self.arm_x, y=0.0, z=self.arm_z))
        self._arm_sends += 1

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=float) if len(msg.d) else None

    def on_range(self, msg):
        self.tof = None if msg.range >= msg.max_range - 1e-3 else float(msg.range)

    def update_camera_rotation(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rclpy.time.Time())
            self.r_bc, self.tf_ok = quat_to_matrix(tf.transform.rotation), True
        except Exception:
            pass   # keep the fixed-pitch fallback

    def process_frame(self, frame):
        if self.K is None:
            return
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        det = self.detector.detect(gray, self.K, self.dist)
        if det is None:
            return
        if self.use_tf:
            self.update_camera_rotation()
        self.distance, self.bearing = ground_range_bearing(det.tvec, self.r_bc)
        self.det_time = self.now()

    def on_image(self, msg):
        try:
            self.process_frame(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        except Exception as exc:    # a bad frame must never stop the control loop
            self.get_logger().warn(f'frame skipped: {exc}', throttle_duration_sec=2.0)

    # ---- control loop -------------------------------------------------------------------
    def tick(self):
        if self.finished:
            return
        if self.grab is not None and self.grab.latest() is not None:
            self.process_frame(self.grab.latest())
        now = self.now()
        dt = max(now - self.prev_tick, 1e-3)
        self.prev_tick = now
        visible = (now - self.det_time) <= self.detect_fresh
        obs = Observation(now=now, visible=visible, bearing=self.bearing if visible else 0.0,
                          distance=self.distance if visible else float('inf'), tof=self.tof)
        v, w = self.fsm.update(obs)
        self.v_cmd, self.w_cmd = v, w
        tw = Twist()
        tw.linear.x, tw.angular.z = float(v), float(w)
        self.cmd_pub.publish(tw)
        self.omegas = wheel_rates(v, w, self.r, self.lc)
        self.volts = self.voltage.step(self.omegas, dt)

        t_rel = now - self.t_start
        self._fast.writerow([f'{t_rel:.3f}', f'{v:.4f}', f'{w:.4f}', *[f'{x:.4f}' for x in self.volts], self.fsm.state])
        if self.fsm.state == ARRIVED:
            self.stop_ticks += 1
            if self.stop_ticks == 3:      # let the derivative term settle: voltages are exactly 0
                self.log_row(t_rel, f'ARRIVED ({self.fsm.arrival_reason})', visible)
                self.get_logger().info(f'Halted at {t_rel:.1f} s ({self.fsm.arrival_reason}); '
                                       f'log saved to {self.csv_path}')
                self.finished = True
                self._csv_file.close()
                self._fast_file.close()
            return
        if t_rel >= self.next_log:
            self.log_row(math.floor(t_rel + 1e-6), self.fsm.state, visible)
            self.next_log += 1.0

    def log_row(self, t, state, visible):
        row = [f'{t:.1f}' if isinstance(t, float) else t, *[f'{x:.3f}' for x in self.volts], state,
               f'{self.v_cmd:.3f}', f'{self.w_cmd:.3f}',
               f'{self.distance:.3f}' if visible else '', f'{self.bearing:.3f}' if visible else '',
               f'{self.tof:.3f}' if self.tof is not None else '',
               *[f'{x:.3f}' for x in self.omegas]]
        self._csv.writerow(row)
        self._csv_file.flush()
        self.get_logger().info(f'{row[0]:>5} s | V1..V4 = ' + ' '.join(f'{x:7.3f}' for x in self.volts)
                               + f' V | {state}')

    def shutdown(self):
        """Always leave the robot stopped."""
        for _ in range(3):
            self.cmd_pub.publish(Twist())
        if self.grab is not None:
            self.grab.running = False
        for f in (self._csv_file, self._fast_file):
            if not f.closed:
                f.close()


def main(args=None):
    rclpy.init(args=args)
    node = FlatnessNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
