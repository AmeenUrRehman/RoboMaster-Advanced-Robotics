#!/usr/bin/env python3
"""Build every figure of the assessment-1 report from the real logs.  Run with the ROS environment sourced:
    source ~/rm_setup.sh && python scripts/make_report_figs.py
"""
import csv
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, FancyArrowPatch

sys.path.insert(0, os.path.dirname(__file__))
from term_shot import term_shot

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['cmr10', 'DejaVu Serif'], 'mathtext.fontset': 'cm',
                     'axes.formatter.use_mathtext': True, 'axes.unicode_minus': False, 'font.size': 10})
C = 'logs/captures'; T = 'figures/term'; os.makedirs(T, exist_ok=True)
rd = lambda p: open(p).read()

# ---------------------------------------------------------------- terminal captures (real text)
ping = lambda p: rd(p)
term_shot(rd('logs/system_diagnostics.txt'), 'ameen@ameen-G6: ~  (system diagnostics)', f'{T}/diag.png')
term_shot(rd(f'{C}/env_check.txt'), 'ameen@ameen-G6: ~  (virtual environment and ROS 2)', f'{T}/env.png')
term_shot(rd(f'{C}/install_check.txt'), 'ameen@ameen-G6: ~  (installed packages and workspace)', f'{T}/install.png')
term_shot(rd(f'{C}/numpy2_abi_failure.txt'), 'ameen@ameen-G6: ~  (NumPy 2.x in a throwaway venv)', f'{T}/numpy2.png', width_chars=118)
term_shot(rd(f'{C}/kinematics_check.txt'), 'ameen@ameen-G6: ~  (kinematics_check.py)', f'{T}/kin.png')
term_shot(rd(f'{C}/link_rndis.txt'), 'ameen@ameen-G6: ~  (wired link, RNDIS)', f'{T}/link_rndis.png')
term_shot(rd('logs/ap_host_info.txt'), 'ameen@ameen-G6: ~  (host network in AP mode)', f'{T}/host_ap.png', width_chars=100)
term_shot('$ scripts/ping_bench.sh rndis\n' + rd('logs/ping_rndis.txt'), 'ameen@ameen-G6: ~  (ping, wired)', f'{T}/ping_rndis.png', head=6, tail=5)
term_shot('$ scripts/ping_bench.sh ap\n' + rd('logs/ping_ap.txt'), 'ameen@ameen-G6: ~  (ping, access point)', f'{T}/ping_ap.png', head=6, tail=5)
term_shot(rd('logs/excerpt_rndis.txt'), 'ep_teleop_sdk.py --conn rndis', f'{T}/teleop_rndis.png', width_chars=150, fontsize=5.2, line_h=0.098)
term_shot(rd('logs/excerpt_ap.txt'), 'ep_teleop_sdk.py --conn ap', f'{T}/teleop_ap.png', width_chars=150, fontsize=5.2, line_h=0.098)
term_shot(rd('logs/ap_ros_verification.txt'), 'ameen@ameen-G6: ~  (ROS 2 driver over AP)', f'{T}/ros_verify.png', width_chars=100, head=28, tail=14)
term_shot(rd('logs/ap_driver.log'), 'ros2 launch robomaster_ros ep.launch conn_type:=ap', f'{T}/driver_ap.png', width_chars=140, fontsize=5.4, line_h=0.1, head=12, tail=0)
term_shot(rd(f'{C}/bag_info_now.txt'), 'ameen@ameen-G6: ~  (ros2 bag info)', f'{T}/bag_info.png', width_chars=118)
term_shot(rd('logs/ap_tf_info.txt'), 'ameen@ameen-G6: ~  (TF publishers)', f'{T}/tf_info.png', width_chars=100, head=30, tail=0)

# ---------------------------------------------------------------- telemetry time series of the two SDK runs
def load(p):
    r = list(csv.DictReader(open(p))); return {k: np.array([float(x[k]) if x[k] not in ('', 'nan') else np.nan for x in r]) for k in r[0]}
for tag, name in (('rndis', 'wired USB (RNDIS)'), ('ap', 'wireless access point')):
    d = load(f'logs/{tag}_run.csv'); t = d['t_rel_s']
    fig, ax = plt.subplots(3, 1, figsize=(7.2, 6.2), sharex=True, constrained_layout=True)
    ax[0].plot(t, d['cmd_vx'], label='$v_x$ (m/s)'); ax[0].plot(t, d['cmd_vy'], label='$v_y$ (m/s)')
    a2 = ax[0].twinx(); a2.plot(t, d['cmd_wz'], color='g', label=r'$\omega_z$ (deg/s)'); a2.set_ylabel(r'$\omega_z$ (deg/s)')
    ax[0].set_ylabel('command (m/s)'); ax[0].legend(loc='upper left', fontsize=8); a2.legend(loc='upper right', fontsize=8)
    for k, lab in (('w_fr_rpm', 'FR'), ('w_fl_rpm', 'FL'), ('w_rl_rpm', 'RL'), ('w_rr_rpm', 'RR')):
        ax[1].plot(t, d[k], lw=0.9, label=lab)
    ax[1].set_ylabel('wheel speed (rpm)'); ax[1].legend(ncol=4, fontsize=8)
    ax[2].plot(t, d['yaw_deg'], label='yaw'); ax[2].plot(t, d['pitch_deg'], label='pitch'); ax[2].plot(t, d['roll_deg'], label='roll')
    ax[2].set_ylabel('attitude (deg)'); ax[2].set_xlabel('t (s)'); ax[2].legend(ncol=3, fontsize=8)
    for a in ax: a.grid(alpha=0.3)
    fig.suptitle(f'SDK teleoperation log, {name}: battery {d["batt_pct"][1]:.0f} % to {d["batt_pct"][-1]:.0f} %', fontsize=10)
    fig.savefig(f'figures/fig_telemetry_{tag}.png', dpi=200); plt.close(fig)

# ---------------------------------------------------------------- mecanum geometry
lx, ly, R = 0.10, 0.10, 0.05
fig, ax = plt.subplots(figsize=(5.4, 4.6)); ax.set_aspect('equal'); ax.axis('off')
ax.add_patch(Rectangle((-lx, -ly), 2 * lx, 2 * ly, fill=False, lw=1.2))
sig = {'FL': (+1, +1), 'FR': (+1, -1), 'RL': (-1, +1), 'RR': (-1, -1)}   # (sign x, sign y)
gam = {'FL': 45, 'FR': -45, 'RL': -45, 'RR': 45}                          # roller axis angle, deg (assumed layout)
for n, (sx, sy) in sig.items():
    cx, cy = sx * lx, sy * ly
    ax.add_patch(Rectangle((cx - 0.016, cy - 0.034), 0.032, 0.068, fc='#dddddd', ec='k'))
    a = np.radians(gam[n]); ax.plot([cx - 0.03 * np.cos(a), cx + 0.03 * np.cos(a)], [cy - 0.03 * np.sin(a), cy + 0.03 * np.sin(a)], 'r-', lw=1.6)
    ax.text(cx + sx * 0.045, cy + sy * 0.02, n, ha='center', fontsize=10, fontweight='bold')
ax.annotate('', xy=(0.07, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='->', lw=1.6)); ax.text(0.075, -0.012, '$x$', fontsize=11)
ax.annotate('', xy=(0, 0.07), xytext=(0, 0), arrowprops=dict(arrowstyle='->', lw=1.6)); ax.text(0.006, 0.072, '$y$', fontsize=11)
ax.plot(0, 0, 'ko', ms=3)
ax.annotate('', xy=(lx, -ly - 0.055), xytext=(0, -ly - 0.055), arrowprops=dict(arrowstyle='<->')); ax.text(lx / 2, -ly - 0.078, '$l_x$', ha='center')
ax.annotate('', xy=(-lx - 0.075, ly), xytext=(-lx - 0.075, 0), arrowprops=dict(arrowstyle='<->')); ax.text(-lx - 0.098, ly / 2, '$l_y$', va='center')
ax.annotate('', xy=(lx + 0.09, 0), xytext=(lx + 0.03, 0), arrowprops=dict(arrowstyle='-|>', lw=2.2, color='#0b5ed7'))
ax.text(lx + 0.06, 0.012, 'front', ha='center', fontsize=9, color='#0b5ed7')
ax.text(-0.02, -0.045, r'$\omega_z>0$: counter-clockwise', ha='left', fontsize=9)
ax.text(0, -ly - 0.115, 'red segments: assumed ground-contact roller axes ($\\gamma=\\pm45^\\circ$)', ha='center', fontsize=8)
ax.set_xlim(-0.2, 0.26); ax.set_ylim(-0.2, 0.17)
fig.savefig('figures/fig_mecanum_geometry.png', dpi=220, bbox_inches='tight'); plt.close(fig)

# ---------------------------------------------------------------- ROS bag overview (needs the ROS environment)
import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
rdr = rosbag2_py.SequentialReader(); rdr.open(rosbag2_py.StorageOptions(uri='logs/bag_ap'), rosbag2_py.ConverterOptions('', ''))
cmd, od = [], []
while rdr.has_next():
    tp, data, ts = rdr.read_next()
    if tp == '/cmd_vel':
        m = deserialize_message(data, Twist); cmd.append((ts * 1e-9, m.linear.x, m.linear.y, m.angular.z))
    elif tp == '/odom':
        m = deserialize_message(data, Odometry); q = m.pose.pose.orientation
        od.append((ts * 1e-9, m.twist.twist.linear.x, m.twist.twist.angular.z, np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
cmd, od = np.array(cmd), np.array(od); t0 = od[0, 0]; cmd[:, 0] -= t0; od[:, 0] -= t0
fig, ax = plt.subplots(3, 1, figsize=(7.2, 6.4), sharex=True, constrained_layout=True)
ax[0].step(cmd[:, 0], cmd[:, 1], where='post', label='commanded $v_x$'); ax[0].plot(od[:, 0], od[:, 1], lw=0.8, label='odometry $v_x$'); ax[0].set_ylabel('m/s'); ax[0].legend(fontsize=8)
ax[1].step(cmd[:, 0], cmd[:, 3], where='post', label=r'commanded $\omega_z$'); ax[1].plot(od[:, 0], od[:, 2], lw=0.8, label=r'odometry $\omega_z$'); ax[1].set_ylabel('rad/s'); ax[1].legend(fontsize=8)
yaw = np.degrees(np.unwrap(od[:, 3])); ax[2].plot(od[:, 0], yaw - yaw[0]); ax[2].set_ylabel('heading change (deg)'); ax[2].set_xlabel('t (s)')
for a in ax: a.grid(alpha=0.3)
fig.suptitle('ROS 2 teleoperation bag (access point): commands and odometry', fontsize=10)
fig.savefig('figures/fig_bag_overview.png', dpi=200); plt.close(fig)
print('figures written:', sorted(os.listdir(T)))
