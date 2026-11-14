#!/usr/bin/env python3
"""Phase 3 robot session driver.  Every number written to logs/ comes from the robot and the tape measure.

    source ~/rm_setup.sh && cd ~/robomasters/assignment3
    python run_trials.py check            # link, battery, camera, live tag distance (robot does not move)
    python run_trials.py calib            # camera calibration: robot at rest at 5 tape-measured distances
    python run_trials.py warmup           # 2 warm-up runs (logs/warmup/, NOT in the results) -> latency estimate
    python run_trials.py trials --n 10    # the 10 measured trials (logs/trials/ + logs/trials.csv)
    python run_trials.py tape 4 0.512     # enter / fix a tape-measured final distance later
    python run_trials.py sim --n 10       # same code on the simulator (sim_out/, never logs/)
    python run_trials.py report           # analysis + figures + report PDF from the real logs
    python run_trials.py session          # everything above in order, skipping finished steps

Connection: --conn rndis (USB cable, robot 192.168.42.2, default) or --conn ap (robot Wi-Fi).
"""
import argparse
import csv
import datetime
import json
import math
import os
import statistics
import sys
import time

from rm_odom_tag import CONFIG_DIR, ROOT, load_config
from rm_odom_tag.tag import CameraModel, TagDetector
from rm_odom_tag.trial import run_trial

LOGS = os.path.join(ROOT, 'logs')
TRIALS_CSV = os.path.join(LOGS, 'trials.csv')
FIELDS = ['trial', 'time', 'ok', 'reason', 'd0', 'd_stop', 'est_kf', 'sigma_kf', 'est_kf_at_stop', 'est_odom',
          'est_cam', 'tape_m', 'err_kf', 'err_odom', 'err_cam', 'travel_m', 'rollout_m', 'n_accept', 'n_reject',
          'duration_s', 'detect_rate', 'loop_hz', 'max_abs_bearing_deg', 'max_lateral_dev_m', 'battery_pct',
          'latency_s', 'calibrated']


def say(msg, colour='33'):
    print(f'\033[1;{colour}m{msg}\033[0m', flush=True)


def ask_distance(prompt):
    """Tape value in metres or centimetres (a value above 3 is read as cm).  Blank = enter later."""
    while True:
        raw = input(f'  {prompt} (m or cm, blank = later): ').strip().replace(',', '.')
        if not raw:
            return None
        unit = None
        for suf, f in (('cm', 0.01), ('mm', 0.001), ('m', 1.0)):
            if raw.lower().endswith(suf):
                raw, unit = raw[:-len(suf)].strip(), f
                break
        try:
            v = float(raw) * (unit or 1.0)
        except ValueError:
            print('  not a number, try again')
            continue
        m = v if unit else (v / 100.0 if v > 3.0 else v)
        if not 0.05 <= m <= 2.5:
            print(f'  {m:.3f} m is outside 0.05..2.5 m, try again')
            continue
        if input(f'  = {m:.3f} m ({100 * m:.1f} cm). Correct? [Y/n]: ').strip().lower() in ('', 'y', 'yes'):
            return m


_SHARED = {}


def open_robot(args, cfg):
    """Connect once.  In a session the same connection is reused by every step: the SDK over Wi-Fi often
    refuses a new connection right after the previous one was closed."""
    if _SHARED.get('io') is not None:
        return _SHARED['io']
    from rm_odom_tag.robot_io import SdkIO
    n_try = 6
    for attempt in range(1, n_try + 1):
        say(f'connecting to the robot ({args.conn}) ...' + (f' attempt {attempt}/{n_try}' if attempt > 1 else ''), '36')
        try:
            io = SdkIO(args.conn, cfg['resolution'])
            break
        except Exception:  # noqa: BLE001
            # usual cause: the robot still holds the previous (dropped) connection for ~10-30 s
            say('  robot did not answer yet - waiting 8 s and trying again', '33')
            if attempt == n_try:
                say('Robot not reachable. Turn the robot off and on, wait for its Wi-Fi, run ./go.sh again.\n'
                    'Your saved calibration points and finished trials are kept.', '31')
                os._exit(1)
            time.sleep(8)
    time.sleep(1.5)
    print(f'  connected, battery {io.battery()} %')
    if getattr(args, 'keep_conn', False):
        _SHARED['io'] = io
    return io


def close_robot(args, io):
    if getattr(args, 'keep_conn', False):
        io.stop()
    else:
        io.close()


def cmd_check(args, cfg):
    io = open_robot(args, cfg)
    det = TagDetector(cfg['tag_family'], cfg['tag_id'], cfg['tag_size_m'])
    cam = CameraModel.load(CONFIG_DIR)
    print(f'  calibration loaded: {cam.calibrated} (a={cam.a:.4f}, b={cam.b:+.4f}), latency {cam.latency_s:.3f} s '
          f'({"measured" if cam.latency_measured else "default"})')
    try:
        t0, n, seen, zs = time.monotonic(), 0, 0, []
        while time.monotonic() - t0 < args.seconds:
            fr = io.next_frame(1.0)
            if fr is None:
                continue
            n += 1
            d = det.detect(fr[1], cfg['K'])
            if d is not None:
                seen += 1
                zs.append(d.plane_distance)
                if seen % 10 == 1:
                    print(f'  frame {n:4d}: tag {d.tag_id}  z_plane {d.plane_distance:.3f} m  calibrated '
                          f'{cam.distance(d.plane_distance):.3f} m  bearing {d.bearing_deg:+.1f} deg  side {d.side_px:.0f} px  '
                          f'img {fr[1].shape[1]}x{fr[1].shape[0]}')
        dt = time.monotonic() - t0
        print(f'  {n} frames in {dt:.1f} s ({n / dt:.1f} fps), tag in {seen} frames', end='')
        print(f', z_plane mean {statistics.mean(zs):.4f} m, std {1000 * statistics.pstdev(zs):.1f} mm' if zs else '')
        print(f'  battery {io.battery()} %, yaw {io.yaw_deg():.1f} deg')
        print(f'  slow fallback detector used in {det.n_fallback} frames')
        say('CHECK OK' if seen > 0.8 * n and n > 0 else 'CHECK: tag not seen reliably - check light, aim and tag ID',
            '32' if seen > 0.8 * n and n > 0 else '31')
        return seen > 0.8 * n and n > 0
    finally:
        close_robot(args, io)


def cmd_calib(args, cfg):
    io = open_robot(args, cfg)
    det = TagDetector(cfg['tag_family'], cfg['tag_id'], cfg['tag_size_m'])
    out = os.path.join(LOGS, 'calib_raw.csv')
    os.makedirs(LOGS, exist_ok=True)
    new = not os.path.exists(out)
    say('Place the robot at rest facing the tag; measure tag plane -> front of the chassis armour (centre line).')
    print('  Suggested distances: 1.50, 1.25, 1.00, 0.75, 0.50 m.  Blank distance = finish.')
    if not new:
        have = sorted(set(round(float(r['tape_m']), 2) for r in csv.DictReader(open(out))))
        print(f'  already saved (kept from before): {", ".join(f"{d:.2f}" for d in have)} m '
              f'- only the missing distances are needed')
    try:
        with open(out, 'a', newline='') as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(['time', 'tape_m', 'n', 'z_plane_mean', 'z_plane_std', 'z_plane_median', 'bearing_mean_deg',
                            'side_px_mean'])
            while True:
                tape = ask_distance('tape distance now')
                if tape is None:
                    taken = sorted(set(round(float(r['tape_m']), 2) for r in csv.DictReader(open(out))))
                    spread = (taken[-1] - taken[0]) if taken else 0.0
                    if len(taken) >= 4 and spread >= 0.6:
                        break
                    say(f'  not enough yet: {len(taken)} distances {taken}, spread {spread:.2f} m. Need at least 4 '
                        f'distances spread over 0.6 m or more (e.g. 1.50, 1.25, 1.00, 0.75, 0.50). Move the robot closer.', '31')
                    continue
                near = [float(r['tape_m']) for r in csv.DictReader(open(out)) if abs(float(r['tape_m']) - tape) < 0.05]
                if near:
                    say(f'  note: already have a point at {near[0]:.2f} m. Move the robot about 25 cm closer for the next one.', '33')
                zs, bs, ps, tries = [], [], [], 0
                while len(zs) < args.frames and tries < 4 * args.frames:
                    tries += 1
                    fr = io.next_frame(1.0)
                    d = det.detect(fr[1], cfg['K']) if fr is not None else None
                    if d is not None:
                        zs.append(d.plane_distance), bs.append(d.bearing_deg), ps.append(d.side_px)
                if len(zs) < 10:
                    say(f'  only {len(zs)} detections - not saved, fix the view and repeat', '31')
                    continue
                row = [datetime.datetime.now().isoformat(timespec='seconds'), f'{tape:.4f}', len(zs),
                       f'{statistics.mean(zs):.5f}', f'{statistics.pstdev(zs):.5f}', f'{statistics.median(zs):.5f}',
                       f'{statistics.mean(bs):.2f}', f'{statistics.mean(ps):.1f}']
                w.writerow(row)
                fh.flush()
                print(f'  saved: tape {tape:.3f} m  z_plane {statistics.mean(zs):.4f} m  std {1000 * statistics.pstdev(zs):.1f} mm '
                      f' ({len(zs)} frames, {statistics.mean(ps):.0f} px)')
    finally:
        close_robot(args, io)
    os.system(f'{sys.executable} {os.path.join(ROOT, "scripts", "fit_calib.py")}')


def load_rows():
    if not os.path.exists(TRIALS_CSV):
        return []
    return list(csv.DictReader(open(TRIALS_CSV)))


def save_rows(rows):
    with open(TRIALS_CSV, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in FIELDS})


def fill_errors(r):
    tape = float(r['tape_m']) if r.get('tape_m') not in (None, '') else None
    for k in ('kf', 'odom', 'cam'):
        est = float(r[f'est_{k}']) if r.get(f'est_{k}') not in (None, '') else math.nan
        r[f'err_{k}'] = f'{est - tape:.4f}' if tape is not None and not math.isnan(est) else ''
    return r


def warmup_status(out_dir):
    """Finished warm-ups end in the SETTLE phase.  Runs cut off by a link drop are renamed *_incomplete
    (kept as raw data, ignored by the latency estimate).  Returns (number finished, highest index used)."""
    import glob
    import re
    done, last = 0, 0
    for p in sorted(glob.glob(os.path.join(out_dir, 'warmup_[0-9][0-9].csv'))):
        idx = int(re.search(r'_(\d\d)\.csv$', p).group(1))
        last = max(last, idx)
        lines = open(p).read().strip().splitlines()
        if len(lines) > 1 and lines[-1].split(',')[1] == 'SETTLE':
            done += 1
        else:
            for ext in ('.csv', '.mp4'):
                src = p[:-4] + ext
                if os.path.exists(src):
                    os.replace(src, p[:-4] + '_incomplete' + ext)
    for p in glob.glob(os.path.join(out_dir, 'warmup_[0-9][0-9]_incomplete.csv')):
        last = max(last, int(re.search(r'_(\d\d)_incomplete', p).group(1)))
    return done, last


def run_series(args, cfg, warmup):
    cam = CameraModel.load(CONFIG_DIR)
    if not cam.calibrated:
        say('WARNING: config/calib.json missing - camera offset unknown. Run "python run_trials.py calib" first.', '31')
        if input('  continue anyway? [y/N]: ').strip().lower() != 'y':
            return
    out_dir = os.path.join(LOGS, 'warmup' if warmup else 'trials')
    rows = [] if warmup else load_rows()
    first = args.start or (len(rows) + 1)
    if warmup:
        done, last = warmup_status(out_dir)
        first = last + 1
        args.n = max(args.n - done, 0) if getattr(args, 'keep_conn', False) else args.n
        if args.n == 0:
            estimate_latency(out_dir)
            return
        print(f'  {done} warm-up run(s) already finished and kept, {args.n} to go')
    io = open_robot(args, cfg)
    det = TagDetector(cfg['tag_family'], cfg['tag_id'], cfg['tag_size_m'])
    say(f'start distance {cfg["start_distance_m"]:.2f} m, stop distance {cfg["stop_distance_m"]:.2f} m '
        f'(reference: front of the chassis armour), latency {cam.latency_s:.3f} s', '36')
    try:
        for i in range(first, first + args.n):
            name = f'{"warmup" if warmup else "trial"}_{i:02d}'
            say(f'\n=== {name} ===')
            input(f'  Put the robot at the START mark ({cfg["start_distance_m"]:.2f} m, facing the tag). '
                  f'Start the phone video. ENTER = go, Ctrl-C = quit ')
            res = run_trial(io, cfg, cam, det, cfg['K'], out_dir, name, video=True, echo=print)
            res['time'] = datetime.datetime.now().isoformat(timespec='seconds')
            res['battery_pct'] = io.battery()
            print(f'  result: {"OK" if res["ok"] else "FAILED"} ({res["reason"]})  KF {res["est_kf"]:.4f} m '
                  f'(sigma {100 * res["sigma_kf"]:.2f} cm) | odom-only {res["est_odom"]:.4f} | camera-only {res["est_cam"]:.4f} '
                  f'| travel {res["travel_m"]:.3f} m, roll-out {100 * res["rollout_m"]:.1f} cm, {res["loop_hz"]:.1f} Hz')
            if warmup:
                continue
            say('  Do NOT move the robot. Tape-measure tag plane -> front of the chassis armour (centre line).')
            res['tape_m'] = ask_distance('measured final distance')
            res = fill_errors({k: (f'{v:.5f}' if isinstance(v, float) else v) for k, v in res.items()})
            rows = [r for r in rows if r['trial'] != name] + [res]
            save_rows(sorted(rows, key=lambda r: r['trial']))
            if res['err_kf']:
                print(f'  error (estimate - measured): KF {100 * float(res["err_kf"]):+.1f} cm, odom-only '
                      f'{100 * float(res["err_odom"]):+.1f} cm, camera-only {100 * float(res["err_cam"]) if res["err_cam"] else math.nan:+.1f} cm')
    except KeyboardInterrupt:
        print('\n  stopped by user')
    finally:
        close_robot(args, io)
    if warmup:
        estimate_latency(out_dir)


def estimate_latency(out_dir):
    os.system(f'{sys.executable} {os.path.join(ROOT, "scripts", "estimate_latency.py")} {out_dir} '
              f'--fig {os.path.join(ROOT, "figures", "fig_latency.png")} --tab {os.path.join(ROOT, "report")}')


def cmd_tape(args, cfg):
    rows = load_rows()
    name = f'trial_{int(args.trial):02d}'
    m = args.value / 100.0 if args.value > 3 else args.value
    for r in rows:
        if r['trial'] == name:
            r['tape_m'] = f'{m:.4f}'
            fill_errors(r)
            save_rows(rows)
            print(f'{name}: tape {m:.4f} m, err_kf {r["err_kf"]}')
            return
    print(f'{name} not found in {TRIALS_CSV}')


def cmd_sim(args, cfg):
    from rm_odom_tag.robot_io import SimIO
    out_dir = os.path.join(ROOT, 'sim_out')
    cam = CameraModel(a=1.0, b=-0.08, latency_s=0.15, calibrated=True)   # matches the simulator's camera
    det = TagDetector(cfg['tag_family'], cfg['tag_id'], cfg['tag_size_m'])
    rows = []
    for i in range(1, args.n + 1):
        io = SimIO(cfg['K'], cfg['start_distance_m'], cfg['tag_size_m'], cfg['tag_id'], seed=i)
        res = run_trial(io, cfg, cam, det, cfg['K'], out_dir, f'sim_{i:02d}', video=False, echo=lambda *_: None)
        res['tape_m'] = io.true_distance()
        res = fill_errors({k: (f'{v:.5f}' if isinstance(v, float) else v) for k, v in res.items()})
        rows.append(res)
        print(f'SIM {i:02d}: {res["reason"]:<32} true {float(res["tape_m"]):.4f}  KF {float(res["est_kf"]):.4f} '
              f'(err {1000 * float(res["err_kf"]):+5.1f} mm)  odom-only err {1000 * float(res["err_odom"]):+5.1f} mm  '
              f'cam-only err {1000 * float(res["err_cam"]):+5.1f} mm')
    with open(os.path.join(out_dir, 'sim_trials.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in FIELDS})
    e = [abs(float(r['err_kf'])) for r in rows]
    print(f'SIMULATION ONLY: KF MAE {1000 * statistics.mean(e):.1f} mm over {len(e)} runs -> {out_dir}')


def cmd_report(args, cfg):
    py, sc = sys.executable, os.path.join(ROOT, 'scripts')
    rc = os.system(f'{py} {sc}/analyze.py')
    os.system(f'{py} {sc}/make_term_figs.py')
    tect = os.path.expanduser('~/tools/tectonic/tectonic')
    # --only-cached first: on the robot's Wi-Fi there is no internet; fall back to online if a file is missing
    rep = os.path.join(ROOT, 'report')
    if os.system(f'cd {rep} && {tect} -X compile --only-cached report.tex > /dev/null 2>&1') != 0:
        os.system(f'cd {rep} && {tect} -X compile report.tex 2>&1 | grep -E "error|Writing"')
    say('report/report.pdf rebuilt' if rc == 0 else 'analysis failed - see above', '32' if rc == 0 else '31')


def cmd_session(args, cfg):
    """check -> calib -> warmup -> trials -> report, skipping steps that are already done."""
    say('STEP 1/5  check', '36')
    args.seconds = 8.0
    if not cmd_check(args, cfg):
        if input('  tag not reliable. Continue anyway? [y/N]: ').strip().lower() != 'y':
            return
    say('STEP 2/5  camera calibration', '36')
    if not os.path.exists(os.path.join(CONFIG_DIR, 'calib.json')) or \
            input('  calibration exists. Redo it? [y/N]: ').strip().lower() == 'y':
        cmd_calib(args, cfg)
    if not os.path.exists(os.path.join(CONFIG_DIR, 'calib.json')):
        say('Calibration not finished - stopping here. Run ./go.sh again to continue.', '31')
        return
    say('STEP 3/5  warm-up runs (latency)', '36')
    if not os.path.exists(os.path.join(CONFIG_DIR, 'latency.json')) or \
            input('  latency estimate exists. Redo warm-up? [y/N]: ').strip().lower() == 'y':
        args.n, args.start = 2, None
        run_series(args, cfg, True)
    say('STEP 4/5  measured trials', '36')
    done = len([r for r in load_rows() if r.get('tape_m')])
    if done < 10:
        args.n, args.start = 10 - done, None
        run_series(args, cfg, False)
    say('STEP 5/5  analysis and report', '36')
    if _SHARED.get('io') is not None:
        _SHARED.pop('io').close()
    cmd_report(args, cfg)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=['check', 'calib', 'warmup', 'trials', 'tape', 'sim', 'report', 'session'])
    ap.add_argument('trial', nargs='?', help='tape mode: trial number')
    ap.add_argument('value', nargs='?', type=float, help='tape mode: measured distance (m or cm)')
    ap.add_argument('--conn', default='rndis', choices=['rndis', 'ap', 'sta'])
    ap.add_argument('--n', type=int, default=None)
    ap.add_argument('--start', type=int, default=None, help='first trial number (default: next free)')
    ap.add_argument('--seconds', type=float, default=10.0, help='check mode duration')
    ap.add_argument('--frames', type=int, default=60, help='calib frames per distance')
    args = ap.parse_args()
    cfg = load_config()
    args.keep_conn = args.mode == 'session'
    if args.n is None:
        args.n = {'warmup': 2, 'trials': 10, 'sim': 10}.get(args.mode, 1)
    {'check': cmd_check, 'calib': cmd_calib, 'warmup': lambda a, c: run_series(a, c, True),
     'trials': lambda a, c: run_series(a, c, False), 'tape': cmd_tape, 'sim': cmd_sim, 'report': cmd_report,
     'session': cmd_session}[args.mode](args, cfg)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:  # noqa: BLE001
        if type(e).__name__ != 'RobotLinkLost':
            raise
        say(f'\nLINK TO THE ROBOT LOST ({e}). The robot stops by itself.\n'
            'Wait until the laptop is back on the robot Wi-Fi (or use the USB cable: ./go.sh usb), then run ./go.sh again.\n'
            'Everything saved so far is kept: finished calibration points and trials are skipped automatically.', '31')
        sys.stdout.flush()
        os._exit(3)
    finally:
        # the SDK leaves non-daemon threads behind; without this the program can hang after an error
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
