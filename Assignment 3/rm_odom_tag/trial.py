"""One trial: DETECT -> APPROACH -> SETTLE -> DONE, with odometry prediction and camera correction.

DETECT    robot at rest at the start distance; waits until the tag is seen in `detect_frames` frames
          in a row (camera corrections already run, the robot is not moving).
APPROACH  every camera frame: predict with the wheel-odometry increment, correct with the
          latency-compensated camera distance, command v from the estimate and ω from the tag bearing.
          Stops when the estimate reaches the stopping distance (or a safety limit trips).
SETTLE    robot halted; prediction (catches the braking roll-out) and correction keep running for
          `settle_s`.  The final estimate is the filter value at the end of this window.

Per trial it writes trial_XX.csv (every loop) and returns a summary dict with three final estimates:
    est_kf    fused odometry + camera (the method)
    est_odom  odometry only:  d0 - s_final      (prediction without correction)
    est_cam   camera only:    median calibrated camera distance while at rest after the stop
"""
import csv
import math
import os
import statistics

import cv2

from .control import should_stop, speed_command, yaw_rate_command
from .kf import DistanceKF, OdomHistory, latency_compensate

COLS = ['t', 'phase', 's_odom', 'd_odom_only', 'd_pred', 'd_kf', 'sigma_kf', 'detected', 'z_plane', 'z_cal',
        'z_comp', 'R', 'innov', 'K', 'accepted', 'bearing_deg', 'side_px', 'v_cmd', 'wz_cmd', 'v_odom', 'yaw_deg',
        'odom_x', 'odom_y']
NAN = float('nan')


def _annotate(bgr_or_gray, det, row, trial_name):
    img = bgr_or_gray if bgr_or_gray.ndim == 3 else cv2.cvtColor(bgr_or_gray, cv2.COLOR_GRAY2BGR)
    img = img.copy()
    if det is not None:
        cv2.polylines(img, [det.corners.astype(int).reshape(-1, 1, 2)], True, (0, 255, 0), 3)
    lines = [f'{trial_name}  {row["phase"]}  t={row["t"]:.2f}s',
             f'KF d = {row["d_kf"]:.3f} m  (sigma {100 * row["sigma_kf"]:.1f} cm)',
             f'odom-only d = {row["d_odom_only"]:.3f} m',
             'camera z = ' + ('---' if math.isnan(row['z_comp']) else f'{row["z_comp"]:.3f} m'),
             f'v = {row["v_cmd"]:.2f} m/s   wz = {row["wz_cmd"]:+.1f} deg/s']
    for i, txt in enumerate(lines):
        cv2.putText(img, txt, (20, 45 + 40 * i), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(img, txt, (20, 45 + 40 * i), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
    return cv2.resize(img, (640, 360))


def run_trial(io, cfg, cam, detector, K, out_dir, trial_name, video=True, echo=print):
    os.makedirs(out_dir, exist_ok=True)
    d0, d_stop = cfg['start_distance_m'], cfg['stop_distance_m']
    kf = DistanceKF(d0, cfg['start_sigma_m'], cfg['odom_sigma_frac'], cfg['process_sigma_mps'], cfg['gate_sigma'])
    hist = OdomHistory()
    io.reset_odom()
    t0 = io.now()
    s_prev, t_prev, v_cmd, v_odom = 0.0, 0.0, 0.0, 0.0
    phase, n_seen, t_stop, reason = 'DETECT', 0, None, ''
    stop_snapshot, cam_rest, bearings, rows = None, [], [], []
    writer = None
    fh = open(os.path.join(out_dir, f'{trial_name}.csv'), 'w', newline='')
    w = csv.DictWriter(fh, fieldnames=COLS)
    w.writeheader()
    try:
        while True:
            fr = io.next_frame(0.3)
            t = io.now() - t0
            s = io.odom_s()
            hist.add(t, s)
            dt = max(t - t_prev, 1e-3)
            v_odom = 0.7 * v_odom + 0.3 * (s - s_prev) / dt
            kf.predict(s - s_prev, dt)
            s_prev, t_prev = s, t
            d_pred = kf.d
            det, z_plane, z_cal, z_comp, R, upd = None, NAN, NAN, NAN, NAN, None
            if fr is not None:
                t_rx, gray, bgr = fr
                det = detector.detect(gray, K)
                if det is not None:
                    t_cap = (t_rx - t0) - cam.latency_s
                    z_plane = det.plane_distance
                    z_cal = cam.distance(z_plane)
                    z_comp = latency_compensate(z_cal, s, hist.at(t_cap))
                    R = cam.sigma(z_cal) ** 2 + (cfg['latency_sigma_s'] * v_odom) ** 2
                    upd = kf.correct(z_comp, R)
            wz_cmd = 0.0
            if phase == 'DETECT':
                n_seen = n_seen + 1 if det is not None else 0
                v_cmd = 0.0
                if n_seen >= cfg['detect_frames'] and abs(z_cal - d0) > cfg.get('start_check_m', 0.10):
                    # the filter starts from d0; never drive if the camera says the robot is not on the start mark
                    reason = f'not on the start mark (camera {z_cal:.2f} m, expected {d0:.2f} m)'
                    echo(f'  ABORT: {reason} - robot did not move')
                    break
                if n_seen >= cfg['detect_frames']:
                    phase = 'APPROACH'
                    echo(f'  tag detected ({n_seen} frames): camera {z_cal:.3f} m, filter {kf.d:.3f} m -> approach')
                elif t > cfg['detect_timeout_s']:
                    reason = 'tag not detected'
                    break
            elif phase == 'APPROACH':
                safety = (s > d0 - d_stop + cfg['max_extra_travel_m']) or (kf.d < cfg['min_safe_distance_m'])
                if should_stop(kf.d, d_stop, v_cmd, cfg) or safety:
                    reason = 'safety limit' if safety else 'estimate reached stop distance'
                    io.stop()
                    v_cmd, phase, t_stop = 0.0, 'SETTLE', t
                    stop_snapshot = {'t': t, 'd_kf': kf.d, 's': s}
                    echo(f'  stop at t={t:.2f}s: filter {kf.d:.3f} m ({reason})')
                else:
                    v_cmd = speed_command(kf.d, d_stop, v_cmd, dt, cfg)
                    wz_cmd = yaw_rate_command(det.bearing_deg if det is not None else None, cfg)
                    if det is not None:
                        bearings.append(det.bearing_deg)
            elif phase == 'SETTLE':
                if det is not None and t - t_stop >= 0.6:
                    cam_rest.append(z_cal)
                if t - t_stop >= cfg['settle_s']:
                    break
            if phase != 'SETTLE':
                io.drive(v_cmd, wz_cmd)
            row = {'t': t, 'phase': phase, 's_odom': s, 'd_odom_only': d0 - s, 'd_pred': d_pred, 'd_kf': kf.d,
                   'sigma_kf': kf.sigma, 'detected': int(det is not None), 'z_plane': z_plane, 'z_cal': z_cal,
                   'z_comp': z_comp, 'R': R, 'innov': upd['innov'] if upd else NAN, 'K': upd['K'] if upd else NAN,
                   'accepted': int(upd['accepted']) if upd else '', 'bearing_deg': det.bearing_deg if det else NAN,
                   'side_px': det.side_px if det else NAN, 'v_cmd': v_cmd, 'wz_cmd': wz_cmd, 'v_odom': v_odom,
                   'yaw_deg': io.yaw_deg(), 'odom_x': io.odom_xy()[0], 'odom_y': io.odom_xy()[1]}
            w.writerow({k: (f'{v:.5f}' if isinstance(v, float) else v) for k, v in row.items()})
            rows.append(row)
            if video and fr is not None and fr[2] is not None:
                frame = _annotate(fr[2], det, row, trial_name)
                if writer is None:
                    writer = cv2.VideoWriter(os.path.join(out_dir, f'{trial_name}.mp4'),
                                             cv2.VideoWriter_fourcc(*'mp4v'), 25, (640, 360))
                writer.write(frame)
    finally:
        io.stop()
        fh.close()
        if writer is not None:
            writer.release()

    s_final = s_prev
    # path straightness: largest distance of the odometry path from the straight start-to-end line
    lateral = NAN
    if rows:
        xe, ye = rows[-1]['odom_x'], rows[-1]['odom_y']
        n = math.hypot(xe, ye)
        if n > 0.05:
            lateral = max(abs(r['odom_x'] * ye - r['odom_y'] * xe) / n for r in rows)
    duration = rows[-1]['t'] if rows else 0.0
    n_frames = sum(1 for r in rows if r['detected'])
    return {
        'trial': trial_name, 'ok': phase == 'SETTLE', 'reason': reason, 'd0': d0, 'd_stop': d_stop,
        'est_kf': kf.d, 'sigma_kf': kf.sigma,
        'est_kf_at_stop': stop_snapshot['d_kf'] if stop_snapshot else NAN,
        'est_odom': d0 - s_final, 'est_cam': statistics.median(cam_rest) if cam_rest else NAN,
        'travel_m': s_final, 'rollout_m': s_final - stop_snapshot['s'] if stop_snapshot else NAN,
        'n_accept': kf.n_accept, 'n_reject': kf.n_reject, 'duration_s': duration,
        'detect_rate': n_frames / max(len(rows), 1), 'loop_hz': len(rows) / max(duration, 1e-3),
        'max_abs_bearing_deg': max((abs(b) for b in bearings), default=NAN),
        'max_lateral_dev_m': lateral,
        'latency_s': cam.latency_s, 'calibrated': int(cam.calibrated),
    }
