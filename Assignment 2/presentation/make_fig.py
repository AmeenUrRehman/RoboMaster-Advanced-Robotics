"""Reference x(t), v(t) from flatness.py (d0 = 1.053 m) with the real run log (flat_run5)."""
import csv, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
# works in the GitHub layout (Assignment 2/rm_flatness_nav) and in the local workspace (ws/src/rm_flatness_nav)
for pkg in ('../rm_flatness_nav', '../ws/src/rm_flatness_nav'):
    if os.path.isdir(os.path.join(HERE, pkg)):
        sys.path.insert(0, os.path.join(HERE, pkg))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rm_flatness_nav.flatness import FlatTrajectory

LOG = os.path.join(HERE, '../logs/flat_run5/motor_voltage_log.csv')
D0, T0 = 1.053, 9.83   # T0: chase start, fitted by matching logged commands to the planned speed curve
plt.rcParams.update({'font.family': 'serif', 'mathtext.fontset': 'cm', 'font.serif': ['CMU Serif', 'DejaVu Serif'], 'font.size': 10})

tr = FlatTrajectory(D0)
ts = np.linspace(0, tr.T, 400)
ref = np.array([tr.sample(t) for t in ts])
rows = [r for r in csv.DictReader(open(LOG)) if r['state'] == 'CHASING']
tc = np.array([float(r['t_s']) - T0 for r in rows])
vc = np.array([float(r['v_mps']) for r in rows])
cam = [(float(r['t_s']) - T0, D0 - float(r['distance_m'])) for r in rows if r['distance_m']]
tm, xm = np.array(cam).T
xp = np.array([tr.sample(t)[2] for t in tm])
err = (xm - xp) * 100
print('T', tr.T, 'peak v', ref[:, 0].max())
for a, b, c, e in zip(tm + T0, xp, xm, err):
    print(f'{a:.0f} {b:.3f} {c:.3f} {e:.1f}')

blue = '#1f5fbf'
fig, ax = plt.subplots(2, 1, figsize=(6.6, 5.4), sharex=True)
ax[0].plot(ts, ref[:, 2], color=blue, lw=2, label='reference $x(t)$ from flatness.py')
ax[0].plot(tm, xm, 'ko', ms=5, label='real run: $d_0-$ camera distance')
ax[0].axhline(D0 - 0.2, color='gray', ls='--', lw=1)
ax[0].text(0.1, D0 - 0.2 + 0.02, 'goal $x(T)=d_0-0.20=0.853$ m', color='gray', fontsize=9)
ax[0].set_ylabel('position $x$ (m)'); ax[0].set_ylim(0, 0.95)
ax[0].legend(loc='lower right', fontsize=8.5, frameon=False)
ax[1].plot(ts, ref[:, 0], color=blue, lw=2, label='reference $v(t)$ from flatness.py')
ax[1].plot(tc, vc, 'ko', ms=5, label='real run: $v$ sent on cmd_vel')
ax[1].axhline(0.16, color='gray', ls='--', lw=1)
ax[1].text(5.6, 0.163, 'limit $v_{\\max}=0.16$ m/s', color='gray', fontsize=9)
ax[1].set_ylabel('speed $v$ (m/s)'); ax[1].set_ylim(0, 0.18)
ax[1].set_xlabel('time since the chase started (s)'); ax[1].set_xlim(0, 9)
ax[1].legend(loc='lower center', fontsize=8.5, frameon=False)
for a in ax: a.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(os.path.join(HERE, 'fig_reference.pdf')); fig.savefig(os.path.join(HERE, 'fig_reference.png'), dpi=200)

fig, a = plt.subplots(figsize=(6.6, 2.4))
a.bar(tm + T0, err, width=0.5, color=blue)
for x, e in zip(tm + T0, err): a.text(x, e - 0.35, f'{e:.1f}', ha='center', fontsize=9)
a.axhline(0, color='k', lw=0.8)
a.set_xlabel('time in the log (s)'); a.set_ylabel('error (cm)'); a.set_ylim(-3.6, 0.5)
a.grid(axis='y', alpha=0.3)
fig.tight_layout(); fig.savefig(os.path.join(HERE, 'fig_error.pdf')); fig.savefig(os.path.join(HERE, 'fig_error.png'), dpi=200)
