#!/usr/bin/env python3
"""Build every figure of the assessment-2 report from real logs, real camera frames and the package itself.
    source ~/rm_setup.sh && source ws/install/setup.bash && python scripts/make_report_figs.py
"""
import csv, os, sys, math
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(__file__)); sys.path.insert(0, 'ws/src/rm_flatness_nav')
from term_shot import term_shot
from rm_flatness_nav.flatness import FlatTrajectory
from rm_flatness_nav.wheel_voltage import wheel_rates, K_V, K_A
from rm_flatness_nav.tag_detector import TagDetector, camera_rotation, ground_range_bearing
from rm_flatness_nav.sim import simulate

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['cmr10', 'DejaVu Serif'], 'mathtext.fontset': 'cm',
                     'axes.formatter.use_mathtext': True, 'axes.unicode_minus': False, 'font.size': 10})
T = 'figures/term'; rd = lambda p: open(p).read()
def load(p):
    r = list(csv.DictReader(open(p)))
    return r, {k: np.array([float(x[k]) if x[k] not in ('', 'nan') and k != 'state' else np.nan for x in r]) for k in r[0] if k != 'state'}

# ------------------------------------------------------------ terminal captures (real text)
term_shot(rd('logs/captures/pytest_verbose.txt'), 'ameen@ameen-G6: ~/ws  (unit tests)', f'{T}/pytest.png', width_chars=118, fontsize=5.9, line_h=0.105)
term_shot(rd('logs/captures/colcon_build.txt'), 'ameen@ameen-G6: ~/ws  (colcon build)', f'{T}/colcon.png')
term_shot(rd('logs/captures/camera_pose_015.txt'), 'ameen@ameen-G6: ~/ws  (camera_pose helper, 15 cm tag)', f'{T}/camera_pose.png')
term_shot(rd('logs/captures/sim_run.txt'), 'ameen@ameen-G6: ~/ws  (offline simulation, not hardware data)', f'{T}/sim.png', width_chars=118)
term_shot(rd('logs/driver.log'), 'ros2 launch robomaster_ros ep.launch conn_type:=rndis', f'{T}/driver.png', width_chars=150, fontsize=5.3, line_h=0.1, head=12, tail=0)
# console format of the node, generated from the real 1 Hz log (same f-string as nav_node.log_row)
rows, _ = load('logs/flat_run5/motor_voltage_log.csv')
lines = ['$ ros2 launch rm_flatness_nav flatness_nav.launch.py tag_id:=3 tag_size:=0.15 csv_path:=$PWD/motor_voltage_log.csv',
         "[INFO] [flatness_nav]: Tag tag36h11 id=3 size=0.15 m. Table header: t(s) V1 V2 V3 V4 state. CSV -> .../motor_voltage_log.csv"]
for r in rows:
    lines.append(f"[INFO] [flatness_nav]: {r['t_s']:>5} s | V1..V4 = " + ' '.join(f"{float(r[k]):7.3f}" for k in ('V1_V', 'V2_V', 'V3_V', 'V4_V')) + f" V | {r['state']}")
lines.append(f"[INFO] [flatness_nav]: Halted at {rows[-1]['t_s']} s (trajectory end, tag out of view); log saved")
term_shot('\n'.join(lines), 'flatness_nav console output (rendered from motor_voltage_log.csv)', f'{T}/node_table.png', width_chars=122, fontsize=5.6, line_h=0.1)

# ------------------------------------------------------------ real camera frames
fig, ax = plt.subplots(1, 4, figsize=(9.4, 2.6), constrained_layout=True)
for a, (f, lab) in zip(ax, (('cam_z0.03', 'arm read-back\nx=0.105, z=0.030 m'), ('cam_z0.06', 'arm read-back\nx=0.105, z=0.059 m'),
                            ('cam_x0.06', 'arm read-back\nx=0.070, z=0.039 m'), ('cam_x0.15', 'arm read-back\nx=0.148, z=0.029 m'))):
    a.imshow(cv2.cvtColor(cv2.imread(f'figures/camera/{f}.png'), cv2.COLOR_BGR2RGB)); a.set_title(lab, fontsize=8); a.axis('off')
fig.suptitle('Frames from the robot camera at different arm poses (640 x 360)', fontsize=10)
fig.savefig('figures/fig_camera_arm_poses.png', dpi=200); plt.close(fig)

# ------------------------------------------------------------ real detection with the project detector
K = np.array([[317.03, 0, 317.26], [0, 316.26, 182.58], [0, 0, 1]])
img = cv2.imread('figures/camera/tag720_2.png'); gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
det = TagDetector('tag36h11', 3, 0.15).detect(gray, K)
fig, ax = plt.subplots(figsize=(6.4, 3.8)); ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)); ax.axis('off')
if det is not None:
    c = det.corners; ax.plot(*np.vstack([c, c[:1]]).T, '-', color='#00ff66', lw=2)
    for k, (x, y) in enumerate(c): ax.text(x, y, str(k), color='yellow', fontsize=8, fontweight='bold')
    d, b = ground_range_bearing(det.tvec, camera_rotation(math.radians(20)))
    ax.text(10, 22, f'tag36h11 ID {det.tag_id}  |  side {np.linalg.norm(c[0]-c[1]):.0f} px  |  range {np.linalg.norm(det.tvec):.2f} m  |  bearing {math.degrees(b):+.1f} deg',
            color='white', fontsize=8, bbox=dict(fc='black', alpha=0.65, ec='none'))
ax.set_title('Detection by the project detector on a real frame (tag side assumed 0.15 m)', fontsize=9)
fig.savefig('figures/fig_detection_real.png', dpi=200, bbox_inches='tight'); plt.close(fig)
print('detected on real frame:', det is not None and det.tag_id)

# ------------------------------------------------------------ planned trajectories (analytic)
fig, ax = plt.subplots(1, 3, figsize=(9.6, 3.0), constrained_layout=True)
for d0, c in ((0.9, '#2b8a3e'), (1.28, '#1c7ed6'), (1.5, '#e8590c'), (1.8, '#7048e8')):
    tr = FlatTrajectory(d0); t = np.linspace(0, tr.T, 300); s = np.array([tr.sample(x) for x in t])
    ax[0].plot(t, s[:, 2], color=c, label=f'$d_0={d0}$ m, $T={tr.T:.1f}$ s'); ax[1].plot(t, s[:, 0], color=c)
    ax[2].plot(s[:, 2], s[:, 0], color=c)
ax[1].axhline(0.16, color='k', ls='--', lw=0.8); ax[1].text(0.5, 0.163, '$v_{\\max}=0.16$ m/s', fontsize=8)
ax[0].set_xlabel('t (s)'); ax[0].set_ylabel('x(t) (m)'); ax[1].set_xlabel('t (s)'); ax[1].set_ylabel('v(t) (m/s)'); ax[2].set_xlabel('x (m)'); ax[2].set_ylabel('v (m/s)')
ax[0].legend(fontsize=7)
for a in ax: a.grid(alpha=0.3)
fig.suptitle('Planned rest-to-rest cubic trajectories for several start distances (computed with the package)', fontsize=10)
fig.savefig('figures/fig_planned_trajectories.png', dpi=200); plt.close(fig)

# ------------------------------------------------------------ visibility model
fig, ax = plt.subplots(figsize=(6.2, 3.6)); z = np.linspace(0.4, 2.2, 200)
ax.plot(z, 317.03 * 0.15 / z, color='#1c7ed6', lw=2, label='upright 15 cm tag: side in pixels')
h, pitch = 0.25, math.radians(20); R = camera_rotation(pitch); rows = []
for d in z:
    p = (R.T @ np.array([[d + 0.075, 0, -h], [d - 0.075, 0, -h]]).T).T; rows.append(abs(316.26 * (p[0, 1] / p[0, 2] - p[1, 1] / p[1, 2])))
ax.plot(z, rows, color='#e8590c', lw=2, label='flat 15 cm tag: height in pixels (camera 0.25 m high, $20^\\circ$ tilt, assumed)')
ax.axhspan(33, 41, color='#1c7ed6', alpha=0.15); ax.text(2.2, 37, 'native-size limit\n(found 33 to 41 px)', ha='right', va='center', fontsize=7.5)
ax.axhspan(24, 27, color='#2b8a3e', alpha=0.2); ax.text(2.2, 25.5, 'with 2x upscaling\n(found 24 to 27 px)', ha='right', va='center', fontsize=7.5, color='#1a6b30')
ax.set_ylim(0, 100); ax.set_xlabel('distance to the tag (m)'); ax.set_ylabel('apparent size (px)'); ax.grid(alpha=0.3); ax.legend(fontsize=7.5, loc='upper right')
ax.set_title('Why an upright tag was used', fontsize=10)
fig.savefig('figures/fig_visibility.png', dpi=200, bbox_inches='tight'); plt.close(fig)

# ------------------------------------------------------------ voltage components of the real run
_, f = load('logs/upright_run4/motor_voltage_fast.csv'); t = f['t_s']
w1 = np.array([wheel_rates(v, w)[0] for v, w in zip(f['v_mps'], f['w_radps'])])
emf = K_V * w1; ine = f['V1_V'] - emf
ch = np.where(np.array([r['state'].startswith('CHASING') for r in load('logs/upright_run4/motor_voltage_fast.csv')[0]]))[0]
lo = t[ch[0]] - 2.5
fig, ax = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True, constrained_layout=True)
ax[0].plot(t, w1, color='#1c7ed6'); ax[0].set_ylabel('$\\omega_1$ (rad/s)')
ax[1].plot(t, f['V1_V'], color='k', lw=1.6, label='$V_1$ (total)'); ax[1].plot(t, emf, color='#d6336c', lw=1.0, label='$K_v\\omega_1$ (back-EMF term)'); ax[1].plot(t, ine, color='#2b8a3e', lw=1.0, label='$K_a\\,\\Delta\\omega_1/\\Delta t$ (inertia term)')
ax[1].set_ylabel('voltage (V)'); ax[1].set_xlabel('t (s)'); ax[1].legend(fontsize=8, loc='upper right')
for a in ax: a.grid(alpha=0.3); a.set_xlim(lo, t[-1] + 0.3)
fig.suptitle('Decomposition of the motor-1 voltage of the real run (Eq. 5 applied to the 20 Hz log)', fontsize=10)
fig.savefig('figures/fig_voltage_terms.png', dpi=200); plt.close(fig)

# ------------------------------------------------------------ repeatability of the two successful runs
fig, ax = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True, constrained_layout=True)
info = []
for p, lab, c in (('logs/archive_20261023_191249/motor_voltage_fast.csv', 'run 3 (18:35)', '#1c7ed6'), ('logs/motor_voltage_fast.csv', 'run 4 (19:14)', '#e8590c')):
    rr, f = load(p); i0 = next(i for i, r in enumerate(rr) if r['state'].startswith('CHASING')); i1 = next(i for i, r in enumerate(rr) if r['state'].startswith('ARRIVED'))
    t = f['t_s'] - f['t_s'][i0]; ax[0].plot(t, f['v_mps'], color=c, label=lab); ax[1].plot(t, f['V1_V'], color=c, label=lab)
    info.append((lab, f['t_s'][i1] - f['t_s'][i0], f['v_mps'].max(), f['V1_V'].max()))
ax[0].set_ylabel('v (m/s)'); ax[1].set_ylabel('$V_1$ (V)'); ax[1].set_xlabel('time since the start of CHASING (s)'); ax[0].legend(fontsize=8)
for a in ax: a.grid(alpha=0.3); a.set_xlim(-1, 14)
fig.suptitle('Two successful real runs, aligned at the start of the chase', fontsize=10)
fig.savefig('figures/fig_repeatability.png', dpi=200); plt.close(fig)
print('repeatability:', info)

# ------------------------------------------------------------ offline simulation scenarios (NOT hardware)
fig, ax = plt.subplots(1, 2, figsize=(9.2, 3.2), constrained_layout=True)
for hd, c in ((70, '#1c7ed6'), (-70, '#e8590c'), (170, '#7048e8')):
    rows_, h, sm, pose, tag = simulate(distance=1.4, lateral=0.1, heading_deg=hd, detect_range=1.8, min_range=0.35)
    ax[0].plot(h[:, 0], h[:, 1], color=c, label=f'start heading {hd:+d} deg, ends {math.hypot(tag[0]-pose[0], tag[1]-pose[1]):.2f} m from tag'); ax[1].plot(h[:, 0], h[:, 3], color=c)
ax[0].set_xlabel('t (s)'); ax[0].set_ylabel('v (m/s)'); ax[1].set_xlabel('t (s)'); ax[1].set_ylabel('$V_1$ (V)'); ax[0].legend(fontsize=7)
for a in ax: a.grid(alpha=0.3)
fig.suptitle('SIMULATION (not hardware data): the same state machine from three starting headings', fontsize=10)
fig.savefig('figures/fig_sim_scenarios.png', dpi=200); plt.close(fig)
print('done')
