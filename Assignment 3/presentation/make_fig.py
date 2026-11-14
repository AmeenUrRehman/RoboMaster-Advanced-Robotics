"""Presentation figures for Assignment 3, made only from the real robot logs (logs/trials.csv, logs/trials/*.csv).

fig_final.pdf       final scatter: estimate vs tape (3 methods) and error per trial
fig_all_trials.pdf  all 10 trials in one graph + the camera correction (fused minus odometry-only) over time
"""
import csv
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, '..', 'logs')
plt.rcParams.update({'font.family': 'serif', 'mathtext.fontset': 'cm', 'font.serif': ['CMU Serif', 'DejaVu Serif'],
                     'font.size': 10})
C_KF, C_ODOM, C_CAM, INK, GRID = '#1F5FBF', '#E4572E', '#17A589', '#222222', '#E6E6E6'

rows = list(csv.DictReader(open(os.path.join(LOGS, 'trials.csv'))))
tape = np.array([float(r['tape_m']) for r in rows])
est = {k: np.array([float(r[f'est_{k}']) for r in rows]) for k in ('kf', 'odom', 'cam')}
err = {k: 1000 * (est[k] - tape) for k in est}
idx = np.arange(1, len(rows) + 1)

# ---------------- final scatter
fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.6, 4.3))
lo, hi = 0.435, 0.535
a1.plot([lo, hi], [lo, hi], color='#999999', lw=0.9, label=r'ideal $\hat d = d$ (zero error)')
a1.scatter(tape, est['odom'], s=40, color=C_ODOM, marker='s', edgecolors='white', lw=0.8, label='odometry only (prediction)')
a1.scatter(tape, est['cam'], s=75, facecolors='none', edgecolors=C_CAM, marker='^', lw=1.3, label='camera only (sensor)')
a1.scatter(tape, est['kf'], s=34, color=C_KF, marker='o', edgecolors='white', lw=0.8, label='fused KF (final answer)', zorder=4)
a1.set_xlim(lo, hi), a1.set_ylim(lo, hi), a1.set_aspect('equal')
a1.set_xlabel('measured final distance $d_i$ (m, tape)')
a1.set_ylabel(r'estimated final distance $\hat d_i$ (m)')
a1.set_title('(a) Estimate vs tape, 10 trials', loc='left', fontsize=10)
a1.grid(color=GRID, lw=0.5)
a1.legend(frameon=False, fontsize=7.5, loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=2)

b, s = err['kf'].mean(), err['kf'].std(ddof=1)
a2.axhspan(b - 2 * s, b + 2 * s, color=C_KF, alpha=0.10, lw=0, label=r'KF bias $\pm2$ SD')
a2.axhline(0, color='#999999', lw=0.9)
a2.axhline(b, color=C_KF, ls='--', lw=1, label=f'KF bias {b:+.1f} mm')
a2.scatter(idx - 0.15, err['odom'], s=40, color=C_ODOM, marker='s', edgecolors='white', lw=0.8, label='odometry only')
a2.scatter(idx + 0.15, err['cam'], s=60, facecolors='none', edgecolors=C_CAM, marker='^', lw=1.2, label='camera only')
a2.scatter(idx, err['kf'], s=34, color=C_KF, marker='o', edgecolors='white', lw=0.8, label='fused KF', zorder=4)
a2.set_xticks(idx)
a2.set_xlabel('trial $i$')
a2.set_ylabel(r'error $e_i=\hat d_i-d_i$ (mm)')
a2.set_title('(b) Error of every trial', loc='left', fontsize=10)
a2.grid(color=GRID, lw=0.5)
a2.legend(frameon=False, fontsize=7.5, loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=3)
fig.tight_layout()
fig.savefig(os.path.join(HERE, 'fig_final.pdf'))
fig.savefig(os.path.join(HERE, 'fig_final.png'), dpi=160)
plt.close(fig)

# ---------------- all trials in one graph
def trace(name):
    rr = list(csv.DictReader(open(os.path.join(LOGS, 'trials', f'{name}.csv'))))
    f = lambda k: np.array([float(r[k]) if r[k] not in ('', 'nan') else np.nan for r in rr])
    return f('t'), f('d_kf'), f('d_odom_only'), f('z_comp'), [r['phase'] for r in rr]

fig, (b1, b2) = plt.subplots(1, 2, figsize=(9.6, 4.0))
cmap = plt.get_cmap('viridis')
for i, r in enumerate(rows):
    t, dkf, dod, z, ph = trace(r['trial'])
    t0 = t[ph.index('APPROACH')] if 'APPROACH' in ph else 0.0     # align the trials at the start of motion
    c = cmap(i / max(len(rows) - 1, 1))
    b1.scatter(t - t0, z, s=3, color=c, alpha=0.35, lw=0)
    b1.plot(t - t0, dkf, color=c, lw=1.2, label=f'{i + 1}')
    b1.plot([t[-1] - t0], [tape[i]], marker='*', ms=8, color=c, mec=INK, mew=0.4, lw=0)
    b2.plot(t - t0, 1000 * (dkf - dod), color=c, lw=1.2)
b1.axhline(0.50, color='#999999', ls=':', lw=0.9)
b1.set_xlabel('time since the robot started moving (s)')
b1.set_ylabel('distance to the tag (m)')
b1.set_title('(a) All 10 trials: fused estimate (line),\n     camera points (dots), tape (star)', loc='left', fontsize=10)
b1.grid(color=GRID, lw=0.5)
b1.legend(title='trial', frameon=False, fontsize=7, title_fontsize=7.5, ncol=2, loc='upper right')
b2.axhline(0, color='#999999', lw=0.9)
b2.set_xlabel('time since the robot started moving (s)')
b2.set_ylabel(r'camera correction $\hat d_k-(d_0-s_k)$ (mm)')
b2.set_title('(b) Error correction: how far the camera\n     moved the estimate away from odometry', loc='left', fontsize=10)
b2.grid(color=GRID, lw=0.5)
fig.tight_layout()
fig.savefig(os.path.join(HERE, 'fig_all_trials.pdf'))
fig.savefig(os.path.join(HERE, 'fig_all_trials.png'), dpi=160)
plt.close(fig)
print('KF  MAE %.1f  bias %+.1f  SD %.1f  range %.1f' % (np.abs(err['kf']).mean(), b, s, np.ptp(err['kf'])))
print('final corrections (mm):', np.round(1000 * (est['kf'] - est['odom']), 1))
