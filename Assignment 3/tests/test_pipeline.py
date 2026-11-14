"""End-to-end on the simulator: trial runner -> trials CSV -> analysis (no robot, no files in logs/)."""
import csv
import os
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rm_odom_tag import load_config  # noqa: E402
from rm_odom_tag.robot_io import SimIO  # noqa: E402
from rm_odom_tag.tag import CameraModel, TagDetector  # noqa: E402
from rm_odom_tag.trial import run_trial  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from run_trials import FIELDS, fill_errors  # noqa: E402


def test_simulated_trials_stop_near_target_and_analysis_runs(tmp_path):
    cfg = load_config()
    cam = CameraModel(a=1.0, b=-0.08, latency_s=0.15, calibrated=True)
    det = TagDetector('tag36h11', 3, 0.15)
    rows = []
    for i in (1, 2, 3):
        io = SimIO(cfg['K'], cfg['start_distance_m'], seed=10 + i, odom_scale_err=0.03 * (i - 2))
        res = run_trial(io, cfg, cam, det, cfg['K'], str(tmp_path), f'trial_{i:02d}', video=False, echo=lambda *_: None)
        assert res['ok'] and res['reason'] == 'estimate reached stop distance'
        true_d = io.true_distance()
        assert true_d == pytest.approx(cfg['stop_distance_m'], abs=0.02)       # stopped where it should
        assert abs(res['est_kf'] - true_d) < 0.005                               # fused estimate within 5 mm
        assert abs(res['est_odom'] - true_d) > abs(res['est_kf'] - true_d) or i == 2  # correction helps when odometry is biased
        res['tape_m'] = true_d
        res['time'], res['battery_pct'] = 'sim', 100
        rows.append(fill_errors({k: (f'{v:.5f}' if isinstance(v, float) else v) for k, v in res.items()}))
    csv_path = tmp_path / 'trials.csv'
    with open(csv_path, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in FIELDS})
    out = subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'analyze.py'), '--trials', str(csv_path),
                          '--logs', str(tmp_path), '--fig', str(tmp_path), '--tab', str(tmp_path), '--label', 'TEST'],
                         check=True, capture_output=True, text=True).stdout
    assert '3 trials used' in out
    for f in ('fig_final_scatter.png', 'fig_trials_grid.png', 'fig_trial_01.png', 'tab_trials.tex', 'tab_stats.tex',
              'results.tex'):
        assert (tmp_path / f).exists(), f
    lat = subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'estimate_latency.py'), str(tmp_path),
                          '--write', str(tmp_path / 'latency.json')], check=True, capture_output=True, text=True).stdout
    assert 'latency estimate from 3 run(s)' in lat
