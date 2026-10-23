#!/usr/bin/env python3
"""v(t) and V1(t) plot from the 20 Hz log written by the node.
Usage: python scripts/plot_run.py logs/motor_voltage_fast.csv figures/fig_v_V1.png"""
import csv
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['cmr10', 'DejaVu Serif'], 'mathtext.fontset': 'cm',
                     'axes.formatter.use_mathtext': True, 'axes.unicode_minus': False, 'font.size': 10})
import numpy as np

src, dst = sys.argv[1], sys.argv[2]
rows = list(csv.DictReader(open(src)))
t = np.array([float(r['t_s']) for r in rows])
v = np.array([float(r['v_mps']) for r in rows])
v1 = np.array([float(r['V1_V']) for r in rows])
state = [r['state'] for r in rows]
# zoom to the active part: the search phase can last tens of seconds with constant voltage
first_active = next((k for k, st in enumerate(state) if st != 'SEARCHING'), 0)
t_lo = max(0.0, t[first_active] - 3.0) if first_active else 0.0
fig, ax = plt.subplots(2, 1, figsize=(7, 5), sharex=True, constrained_layout=True)
ax[0].plot(t, v, color='#15803D'); ax[0].set_ylabel('forward speed v (m/s)')
ax[1].plot(t, v1, color='#C2410C'); ax[1].set_ylabel('motor voltage $V_1$ (V)')
ax[1].set_xlabel('t (s)' + (f'   (search phase 0 to {t_lo:.0f} s omitted: v = 0, V$_1$ = 0.456 V)' if t_lo > 5 else ''))
# shade each state; label only segments that last long enough to read
colors = {'SEARCHING': '0.92', 'ALIGNING': '#fef3c7', 'CHASING': '#dcfce7'}
seg = 0
for k in range(1, len(state) + 1):
    if k == len(state) or state[k][:6] != state[seg][:6]:
        name, a, b = state[seg].split(' ')[0], t[seg], t[min(k, len(t) - 1)]
        for a_ in ax:
            a_.axvspan(a, b, color=colors.get(name, '#e5e7eb'), lw=0)
        if b - a >= 1.0 and b > t_lo:
            ax[0].text(max(a, t_lo) + 0.15, ax[0].get_ylim()[1] * 0.92, name, fontsize=7, color='0.35')
        seg = k
ax[0].set_xlim(t_lo, t[-1] + 0.3)
for a_ in ax:
    a_.grid(alpha=0.3)
fig.savefig(dst, dpi=200)
print('wrote', dst, f'({len(t)} samples, {t[-1]:.1f} s, peak v {v.max():.3f} m/s)')
