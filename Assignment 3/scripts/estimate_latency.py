#!/usr/bin/env python3
"""Estimate the camera latency L from logged runs (normally the two warm-up runs).

While the robot moves, a frame received at t shows the scene at t - L.  For each candidate L the
odometry-predicted distance  d(t - L) = c - s(t - L)  is compared with the calibrated camera reading
z_cal(t); c is fitted per run.  L is the value with the smallest residual RMS (grid 0 ... 0.40 s).

Usage: python scripts/estimate_latency.py logs/warmup [--write config/latency.json] [--fig figures/fig_latency.png]
"""
import argparse
import csv
import glob
import json
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(path):
    rows = list(csv.DictReader(open(path)))
    f = lambda k: np.array([float(r[k]) if r[k] not in ('', 'nan') else np.nan for r in rows])
    return f('t'), f('s_odom'), f('z_cal'), f('v_odom')


def residual_rms(runs, L):
    res = []
    for t, s, z, v in runs:
        m = (~np.isnan(z)) & (v > 0.05)
        if m.sum() < 10:
            continue
        s_cap = np.interp(t[m] - L, t, s)
        r = z[m] + s_cap          # = c if L is right
        res.append(r - r.mean())
    return float(np.sqrt(np.mean(np.concatenate(res) ** 2))) if res else np.nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folder', nargs='?', default=os.path.join(ROOT, 'logs', 'warmup'))
    ap.add_argument('--write', default=os.path.join(ROOT, 'config', 'latency.json'))
    ap.add_argument('--fig', default=None)
    ap.add_argument('--tab', default=None, help='folder for latency.tex (report macros)')
    args = ap.parse_args()
    files = [p for p in sorted(glob.glob(os.path.join(args.folder, '*_[0-9][0-9].csv')))
             if open(p).readline().startswith('t,')]
    runs = [load(p) for p in files]
    grid = np.round(np.arange(0.0, 0.401, 0.005), 3)
    rms = np.array([residual_rms(runs, L) for L in grid])
    if np.all(np.isnan(rms)):
        print('no moving samples with camera readings - latency not estimated')
        return
    L = float(grid[np.nanargmin(rms)])
    print(f'latency estimate from {len(files)} run(s): L = {L:.3f} s  (residual RMS {1000 * np.nanmin(rms):.1f} mm; '
          f'at L=0: {1000 * rms[0]:.1f} mm)')
    if args.write:
        json.dump({'latency_s': L, 'rms_m': float(np.nanmin(rms)), 'rms_at_zero_m': float(rms[0]),
                   'runs': [os.path.relpath(p, ROOT) for p in files]}, open(args.write, 'w'), indent=2)
        print(f'  written to {args.write}')
    if args.tab:
        with open(os.path.join(args.tab, 'latency.tex'), 'w') as fh:
            fh.write('% written by scripts/estimate_latency.py\n')
            fh.write(f'\\renewcommand{{\\LatL}}{{{L:.3f}}}\n\\renewcommand{{\\LatRMS}}{{{1000 * np.nanmin(rms):.1f}}}\n'
                     f'\\renewcommand{{\\LatRMSzero}}{{{1000 * rms[0]:.1f}}}\n\\renewcommand{{\\LatRuns}}{{{len(files)}}}\n'
                     '\\renewcommand{\\HaveLatency}{1}\n')
    if args.fig:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(5.0, 3.0))
        ax.plot(grid, 1000 * rms, '-', color='#2a78d6', lw=2)
        ax.axvline(L, color='#52514e', lw=1, ls='--')
        ax.annotate(f'L = {L:.3f} s', (L, np.nanmin(rms) * 1000), textcoords='offset points', xytext=(8, 10), fontsize=9)
        ax.set_xlabel('assumed camera latency L (s)')
        ax.set_ylabel('camera vs odometry residual RMS (mm)')
        ax.grid(alpha=0.25, lw=0.5)
        ax.spines[['top', 'right']].set_visible(False)
        fig.tight_layout()
        fig.savefig(args.fig, dpi=200)


if __name__ == '__main__':
    main()
