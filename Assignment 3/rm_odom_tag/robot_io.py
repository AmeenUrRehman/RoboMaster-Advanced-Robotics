"""Robot interface used by the trial runner.

SdkIO  - the real RoboMaster EP through the DJI `robomaster` Python SDK (USB rndis or Wi-Fi AP).
SimIO  - an offline simulator with the same interface: true pose, wheel-odometry scale error and noise,
         first-order speed response, command delay, and camera frames RENDERED from the true pose
         and delivered late (camera latency).  It exists only to test the code; its numbers are never
         written to logs/ or used in the report results.

Interface:  now(), odom_s(), yaw_deg(), next_frame(timeout) -> (t_rx, gray, bgr) | None,
            drive(v, wz), stop(), battery(), close()
"""
import math
import threading
import time
from collections import deque

import numpy as np


class RobotLinkLost(RuntimeError):
    """No camera frame for LINK_TIMEOUT_S seconds: the Wi-Fi/USB link to the robot is gone."""


class SdkIO:
    LINK_TIMEOUT_S = 6.0

    def __init__(self, conn='rndis', resolution='720p'):
        from robomaster import robot
        self.ep = robot.Robot()
        if self.ep.initialize(conn_type=conn) is False:
            raise ConnectionError('the robot did not answer')
        self._last_frame = time.monotonic()
        self._lock = threading.Lock()
        self._pos = (0.0, 0.0, 0.0)
        self._yaw = float('nan')
        self._batt = float('nan')
        self.ep.battery.sub_battery_info(freq=1, callback=self._on_batt)
        self.ep.chassis.sub_attitude(freq=20, callback=self._on_att)
        self.ep.camera.start_video_stream(display=False, resolution=resolution)
        self._pos_sub = False

    # --- subscriptions (SDK threads)
    def _on_pos(self, info):
        with self._lock:
            self._pos = tuple(info[:3])

    def _on_att(self, info):
        with self._lock:
            self._yaw = float(info[0])

    def _on_batt(self, p):
        self._batt = p

    def reset_odom(self):
        """(Re)subscribe the chassis position at 50 Hz with cs=0.  The SDK then subtracts the position at
        subscription time but keeps the axes of the power-on frame (robomaster/chassis.py, PositionSubject),
        so x alone is NOT the forward travel once the robot was turned.  The robot only drives forward on a
        straight line, so the travel is the length of the displacement, s = sqrt(x^2 + y^2)."""
        if self._pos_sub:
            self.ep.chassis.unsub_position()
        with self._lock:
            self._pos = (0.0, 0.0, 0.0)
        self.ep.chassis.sub_position(cs=0, freq=50, callback=self._on_pos)
        self._pos_sub = True
        time.sleep(0.3)

    # --- interface
    def now(self):
        return time.monotonic()

    def odom_s(self):
        with self._lock:
            return float(math.hypot(self._pos[0], self._pos[1]))

    def odom_xy(self):
        with self._lock:
            return float(self._pos[0]), float(self._pos[1])

    def yaw_deg(self):
        with self._lock:
            return self._yaw

    def next_frame(self, timeout=0.3):
        try:
            bgr = self.ep.camera.read_cv2_image(timeout=timeout, strategy='newest')
        except Exception:
            bgr = None
        if bgr is None:
            if time.monotonic() - self._last_frame > self.LINK_TIMEOUT_S:
                raise RobotLinkLost(f'no camera frame for {self.LINK_TIMEOUT_S:.0f} s')
            return None
        self._last_frame = time.monotonic()
        import cv2
        return time.monotonic(), cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), bgr

    def drive(self, v, wz):
        # timeout: the chassis stops by itself if the laptop stops sending commands
        self.ep.chassis.drive_speed(x=float(v), y=0.0, z=float(wz), timeout=0.5)

    def stop(self):
        # never raise here: stop() runs in finally blocks, also after the link is gone
        # (the chassis stops by itself 0.5 s after the last drive command anyway)
        for _ in range(3):
            try:
                self.ep.chassis.drive_speed(x=0.0, y=0.0, z=0.0)
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.02)

    def battery(self):
        return self._batt

    def close(self):
        self.stop()
        steps = [self.ep.chassis.unsub_attitude, self.ep.battery.unsub_battery_info,
                 self.ep.camera.stop_video_stream, self.ep.close]
        if self._pos_sub:
            steps.insert(0, self.ep.chassis.unsub_position)
        for f in steps:
            try:
                f()
            except Exception:  # noqa: BLE001
                pass


class SimIO:
    """Simulated EP facing an upright tag.  All randomness comes from `seed`."""

    def __init__(self, K, start_distance=1.50, tag_size=0.15, tag_id=3, seed=0, fps=30.0,
                 latency_s=0.15, cmd_delay_s=0.06, tau_s=0.12, odom_scale_err=None, cam_back_m=0.08,
                 cam_above_tag_m=0.10, pitch_deg=20.0, start_lateral_m=None, start_yaw_deg=None,
                 pixel_noise=3.0, size_wh=(1280, 720)):
        from .tag import make_tag_image
        self.rng = np.random.default_rng(seed)
        self.K, self.size_wh, self.fps = np.asarray(K, float), size_wh, fps
        self.tag_img, self.tag_size = make_tag_image(tag_id), tag_size
        self.L, self.cmd_delay, self.tau = latency_s, cmd_delay_s, tau_s
        self.eps = self.rng.normal(0.0, 0.02) if odom_scale_err is None else odom_scale_err
        self.cam_back, self.cam_above, self.pitch = cam_back_m, cam_above_tag_m, math.radians(pitch_deg)
        self.pixel_noise = pixel_noise
        lat = self.rng.normal(0.0, 0.01) if start_lateral_m is None else start_lateral_m
        yaw = self.rng.normal(0.0, 1.0) if start_yaw_deg is None else start_yaw_deg
        # world: x toward the tag, y left; tag plane at x = 0, robot reference point at x = -start_distance
        self.x, self.y, self.psi = -start_distance, lat, math.radians(yaw)
        self.v = 0.0
        self.t = 0.0
        self.s_true = 0.0
        self.odom_noise = 0.0
        self.cmd_queue = deque()
        self.cmd = (0.0, 0.0)
        self.pose_hist = deque(maxlen=200)
        self.pose_hist.append((self.t, self.x, self.y, self.psi))

    # --- truth (for tests only)
    def true_distance(self):
        return -self.x

    # --- interface
    def now(self):
        return self.t

    def reset_odom(self):
        self.s_true = 0.0
        self.odom_noise = 0.0
        self.y0 = self.y

    @property
    def y_since_reset(self):
        return self.y - getattr(self, 'y0', self.y)

    def odom_s(self):
        return (1.0 + self.eps) * self.s_true + self.odom_noise

    def odom_xy(self):
        """Displacement since reset_odom in a frame that is NOT aligned with the start heading (like the SDK)."""
        c, s = math.cos(0.7), math.sin(0.7)
        return self.odom_s() * c + self.y_since_reset * s, self.odom_s() * s - self.y_since_reset * c

    def yaw_deg(self):
        return math.degrees(self.psi)

    def drive(self, v, wz):
        self.cmd_queue.append((self.t + self.cmd_delay, v, wz))

    def stop(self):
        self.cmd_queue.clear()
        self.cmd_queue.append((self.t + self.cmd_delay, 0.0, 0.0))

    def battery(self):
        return 100

    def close(self):
        pass

    def _step(self, dt):
        while self.cmd_queue and self.cmd_queue[0][0] <= self.t:
            _, v, wz = self.cmd_queue.popleft()
            self.cmd = (v, wz)
        v_cmd, wz_cmd = self.cmd
        self.v += (v_cmd - self.v) * (1.0 - math.exp(-dt / self.tau))
        ds = self.v * dt
        self.psi -= math.radians(wz_cmd) * dt          # SDK z > 0 = clockwise
        self.x += ds * math.cos(self.psi)
        self.y += ds * math.sin(self.psi)
        self.s_true += ds
        self.odom_noise += self.rng.normal(0.0, 0.002 * abs(ds) / max(dt, 1e-6) * math.sqrt(dt)) if ds else 0.0
        self.t += dt
        self.pose_hist.append((self.t, self.x, self.y, self.psi))

    def _pose_at(self, t):
        best = self.pose_hist[0]
        for p in self.pose_hist:
            if p[0] <= t:
                best = p
        return best

    def _render(self, pose):
        from .tag import render_tag
        _, x, y, psi = pose
        cx, cy = x - self.cam_back * math.cos(psi), y - self.cam_back * math.sin(psi)
        c, s = math.cos(-psi), math.sin(-psi)

        def to_opt(vw):  # world vector -> level optical frame (robot frame: x fwd, y left, z up)
            xr, yr, zr = c * vw[0] - s * vw[1], s * vw[0] + c * vw[1], vw[2]
            return np.array([-yr, -zr, xr])

        Rx = np.array([[1, 0, 0], [0, math.cos(self.pitch), -math.sin(self.pitch)],
                       [0, math.sin(self.pitch), math.cos(self.pitch)]])
        t_ct = Rx @ to_opt(np.array([0.0 - cx, 0.0 - cy, -self.cam_above]))
        R_ct = Rx @ np.column_stack([to_opt(np.array([0.0, -1.0, 0.0])), to_opt(np.array([0.0, 0.0, 1.0])),
                                     to_opt(np.array([-1.0, 0.0, 0.0]))])
        img = render_tag(self.K, self.size_wh, self.tag_img, self.tag_size, R_ct, t_ct)
        if self.pixel_noise:
            img = np.clip(img + self.rng.normal(0, self.pixel_noise, img.shape), 0, 255).astype(np.uint8)
        return img

    def next_frame(self, timeout=0.3):
        period = 1.0 / self.fps
        for _ in range(3):
            self._step(period / 3)
        gray = self._render(self._pose_at(self.t - self.L))
        return self.t, gray, None
