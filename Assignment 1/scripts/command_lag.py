#!/usr/bin/env python3
"""Command-to-wheel response delay from the two SDK logs (resolution = 0.1 s, the logging period).

For every key press (|command| rises from 0) the delay is the time until the mean absolute wheel speed
first exceeds a threshold.  Prints a table and writes figures/fig_command_lag.png."""
import csv, json, sys
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['cmr10', 'DejaVu Serif'], 'mathtext.fontset': 'cm', 'axes.formatter.use_mathtext': True, 'axes.unicode_minus': False})
TH = 15.0   # rpm
out = {}
for tag in ('rndis', 'ap'):
    r = list(csv.DictReader(open(f'logs/{tag}_run.csv')))
    f = lambda k: np.array([float(x[k]) if x[k] not in ('', 'nan') else np.nan for x in r])
    t = f('t_rel_s'); cmd = np.hypot.reduce([f('cmd_vx'), f('cmd_vy'), np.radians(f('cmd_wz'))]) > 0
    w = np.nanmean(np.abs(np.c_[f('w_fr_rpm'), f('w_fl_rpm'), f('w_rl_rpm'), f('w_rr_rpm')]), axis=1)
    lags = []
    for i in range(1, len(t)):
        if cmd[i] and not cmd[i - 1]:
            j = next((k for k in range(i, min(i + 15, len(t))) if w[k] > TH), None)
            if j is not None:
                lags.append(t[j] - t[i])
    out[tag] = np.array(lags)
    print(f'{tag:6s} key presses with a response: {len(lags):2d}   delay mean {np.mean(lags):.2f} s  median {np.median(lags):.2f} s  max {np.max(lags):.2f} s  min {np.min(lags):.2f} s')
fig, ax = plt.subplots(figsize=(5.6, 3.3), constrained_layout=True)
bins = np.arange(-0.05, 0.85, 0.1)
for tag, c, lab in (('rndis', '#1f6feb', 'wired (RNDIS)'), ('ap', '#d9480f', 'wireless (AP)')):
    ax.hist(out[tag], bins=bins, alpha=0.6, color=c, label=f'{lab}, n={len(out[tag])}, mean {np.mean(out[tag]):.2f} s')
ax.set_xlabel('delay from key press to wheel response (s)'); ax.set_ylabel('count'); ax.grid(alpha=0.3); ax.legend(fontsize=8)
fig.savefig('figures/fig_command_lag.png', dpi=200)
json.dump({k: {'n': int(len(v)), 'mean_s': float(np.mean(v)), 'median_s': float(np.median(v)), 'max_s': float(np.max(v))} for k, v in out.items()}, open('figures/command_lag.json', 'w'), indent=1)
