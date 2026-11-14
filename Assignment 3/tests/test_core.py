"""Unit tests: filter, latency compensation, control law, tag measurement, calibration fit."""
import math
import os
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rm_odom_tag import load_config  # noqa: E402
from rm_odom_tag.control import should_stop, speed_command, yaw_rate_command  # noqa: E402
from rm_odom_tag.kf import DistanceKF, OdomHistory, latency_compensate  # noqa: E402
from rm_odom_tag.robot_io import SimIO  # noqa: E402
from rm_odom_tag.tag import CameraModel, TagDetector  # noqa: E402

CFG = load_config()
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_predict_moves_estimate_and_grows_variance():
    kf = DistanceKF(1.5, 0.01, odom_sigma_frac=0.03, process_sigma_mps=0.0)
    kf.predict(0.10, 0.1)
    assert kf.d == pytest.approx(1.40)
    assert kf.P == pytest.approx(0.01 ** 2 + (0.03 * 0.10) ** 2)


def test_correct_matches_scalar_kalman_formulas():
    kf = DistanceKF(1.0, 0.02)
    P, R, z = 0.02 ** 2, 0.01 ** 2, 1.01
    out = kf.correct(z, R)
    K = P / (P + R)
    assert out['accepted'] and out['K'] == pytest.approx(K)
    assert kf.d == pytest.approx(1.0 + K * 0.01)
    assert kf.P == pytest.approx((1 - K) * P)


def test_gate_rejects_outlier_and_keeps_state():
    kf = DistanceKF(1.0, 0.01)
    out = kf.correct(1.5, 0.01 ** 2)
    assert not out['accepted'] and kf.d == 1.0 and kf.n_reject == 1


def test_repeated_corrections_converge_to_measurement_mean():
    kf = DistanceKF(1.0, 0.05)
    rng = np.random.default_rng(1)
    for _ in range(400):
        kf.correct(0.9 + rng.normal(0, 0.005), 0.005 ** 2)
    assert kf.d == pytest.approx(0.9, abs=1e-3)


def test_odom_history_interpolates_and_latency_shift():
    h = OdomHistory()
    for t in range(11):
        h.add(t * 0.1, t * 0.02)       # 0.2 m/s
    assert h.at(0.55) == pytest.approx(0.11)
    # frame captured at t=0.85 (s=0.17) received at t=1.0 (s=0.20): robot is 3 cm closer now
    assert latency_compensate(0.80, 0.20, h.at(0.85)) == pytest.approx(0.77)


def test_speed_law_limits_and_slew():
    assert speed_command(1.5, 0.5, 0.0, 0.05, CFG) == pytest.approx(CFG['accel_mps2'] * 0.05)
    assert speed_command(1.5, 0.5, 0.2, 0.05, CFG) == pytest.approx(CFG['v_max_mps'])
    assert speed_command(0.51, 0.5, CFG['v_min_mps'], 0.05, CFG) == pytest.approx(CFG['v_min_mps'])


def test_stop_rule_and_yaw_sign():
    assert should_stop(0.505, 0.5, 0.1, CFG)          # 5 mm left < 0.1 m/s * 0.1 s = 10 mm
    assert not should_stop(0.6, 0.5, 0.1, CFG)
    assert yaw_rate_command(5.0, CFG) > 0             # tag to the right -> clockwise (SDK z > 0)
    assert yaw_rate_command(-50.0, CFG) == -CFG['wz_max_dps']
    assert yaw_rate_command(None, CFG) == 0.0


@pytest.mark.parametrize('d', [0.5, 1.0, 1.5])
def test_plane_distance_from_rendered_tag(d):
    io = SimIO(CFG['K'], start_distance=d, seed=3, start_lateral_m=0.03, start_yaw_deg=3.0, pixel_noise=2.0)
    det = TagDetector('tag36h11', 3, 0.15)
    m = det.detect(io.next_frame()[1], CFG['K'])
    assert m is not None and m.tag_id == 3
    # simulator camera sits 0.08 m behind the reference point
    assert m.plane_distance == pytest.approx(d + 0.08, abs=0.02)
    assert m.bearing_deg > 0                          # robot yawed left (CCW) -> tag appears right


def test_other_tag_id_is_ignored():
    io = SimIO(CFG['K'], start_distance=1.0, tag_id=2, seed=0, pixel_noise=0)
    assert TagDetector('tag36h11', 3, 0.15).detect(io.next_frame()[1], CFG['K']) is None


def test_calibration_fit_recovers_line(tmp_path):
    rows = ['time,tape_m,n,z_plane_mean,z_plane_std,z_plane_median,bearing_mean_deg,side_px_mean']
    for d in (0.5, 0.75, 1.0, 1.25, 1.5):
        z = (d + 0.09) / 1.02
        rows.append(f'x,{d},60,{z:.6f},{0.002 + 0.004 * d * d:.6f},{z:.6f},0,100')
    p = tmp_path / 'calib_raw.csv'
    p.write_text('\n'.join(rows) + '\n')
    subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'fit_calib.py'), str(p), '--out-config', str(tmp_path),
                    '--fig', str(tmp_path), '--tab', str(tmp_path)], check=True, capture_output=True)
    cam = CameraModel.load(str(tmp_path))
    assert cam.calibrated
    assert cam.a == pytest.approx(1.02, abs=1e-4) and cam.b == pytest.approx(-0.09, abs=1e-4)
    assert cam.distance((1.0 + 0.09) / 1.02) == pytest.approx(1.0, abs=1e-4)
    assert cam.c0 == pytest.approx(0.002, abs=1e-4) and cam.c1 == pytest.approx(0.004, abs=1e-4)
