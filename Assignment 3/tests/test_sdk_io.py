"""SdkIO against a stub of the robomaster SDK (checks the calls and the odometry travel without a robot)."""
import math
import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class _Chassis:
    def __init__(self):
        self.cmds, self.pos_cb, self.subs = [], None, []

    def sub_position(self, cs, freq, callback):
        self.subs.append(('pos', cs, freq))
        self.pos_cb = callback

    def unsub_position(self):
        self.subs.append(('unpos',))

    def sub_attitude(self, freq, callback):
        callback((12.0, 0.0, 0.0))

    def unsub_attitude(self):
        pass

    def drive_speed(self, x=0.0, y=0.0, z=0.0, timeout=None):
        self.cmds.append((x, y, z, timeout))


class _Camera:
    def __init__(self):
        self.started = None

    def start_video_stream(self, display, resolution):
        self.started = (display, resolution)

    def read_cv2_image(self, timeout, strategy):
        assert strategy == 'newest'
        return np.zeros((720, 1280, 3), np.uint8)

    def stop_video_stream(self):
        self.started = None


class _Battery:
    def sub_battery_info(self, freq, callback):
        callback(87)

    def unsub_battery_info(self):
        pass


class _Robot:
    def __init__(self):
        self.chassis, self.camera, self.battery, self.closed = _Chassis(), _Camera(), _Battery(), False

    def initialize(self, conn_type):
        self.conn = conn_type

    def close(self):
        self.closed = True


@pytest.fixture
def io(monkeypatch):
    pkg = types.ModuleType('robomaster')
    pkg.robot = types.SimpleNamespace(Robot=_Robot)
    monkeypatch.setitem(sys.modules, 'robomaster', pkg)
    monkeypatch.setitem(sys.modules, 'robomaster.robot', pkg.robot)
    from rm_odom_tag.robot_io import SdkIO
    return SdkIO('rndis', '720p')


def test_connects_streams_720p_and_reads_battery_and_yaw(io):
    assert io.ep.conn == 'rndis' and io.ep.camera.started == (False, '720p')
    assert io.battery() == 87 and io.yaw_deg() == 12.0
    t, gray, bgr = io.next_frame()
    assert gray.shape == (720, 1280) and bgr.shape == (720, 1280, 3)


def test_travel_is_displacement_length_in_unaligned_frame(io, monkeypatch):
    monkeypatch.setattr('time.sleep', lambda *_: None)
    io.reset_odom()
    assert io.ep.chassis.subs[-1] == ('pos', 0, 50)
    io.ep.chassis.pos_cb((0.3, -0.4, 0.0))      # 0.5 m forward, power-on axes rotated
    assert io.odom_s() == pytest.approx(0.5)
    io.reset_odom()                              # next trial: resubscribe, travel restarts
    assert ('unpos',) in io.ep.chassis.subs and io.odom_s() == 0.0


def test_drive_uses_watchdog_and_close_stops(io):
    io.drive(0.2, -3.0)
    assert io.ep.chassis.cmds[-1] == (0.2, 0.0, -3.0, 0.5)
    io.close()
    assert io.ep.chassis.cmds[-1][:3] == (0.0, 0.0, 0.0) and io.ep.closed


def test_link_lost_raises_after_timeout_and_stop_never_raises(io, monkeypatch):
    from rm_odom_tag.robot_io import RobotLinkLost
    io.ep.camera.read_cv2_image = lambda timeout, strategy: None
    assert io.next_frame() is None                       # a single missed frame is fine
    io._last_frame -= io.LINK_TIMEOUT_S + 1
    with pytest.raises(RobotLinkLost):
        io.next_frame()

    def boom(**_):
        raise OSError(101, 'Network is unreachable')
    io.ep.chassis.drive_speed = boom
    monkeypatch.setattr('time.sleep', lambda *_: None)
    io.stop()
    io.close()
    assert io.ep.closed
