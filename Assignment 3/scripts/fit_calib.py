#!/usr/bin/env python3
"""Fit the camera distance model from logs/calib_raw.csv (robot at rest at tape-measured distances).

    d_tape = a z_plane + b            (least squares)
    σ_z(d) = c0 + c1 d²               (least squares on the per-distance frame std, floored at 2 mm)

Writes config/calib.json, figures/fig_calib.png and report/tab_calib.tex.
Usage: python scripts/fit_calib.py [calib_raw.csv] [--out-config DIR] [--fig DIR] [--tab DIR]
"""
import argparse
import csv
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BLUE, ORANGE, INK, MUTED = '#2a78d6', '#eb6834', '#0b0b0b', '#52514e'


def fit(tape, z, std):
    A = np.column_stack([z, np.ones_like(z)])
    (a, b), *_ = np.linalg.lstsq(A, tape, rcond=None)
    resid = tape - (a * z + b)
    B = np.column_stack([np.ones_like(tape), tape ** 2])
    (c0, c1), *_ = np.linalg.lstsq(B, std, rcond=None)
    c0, c1 = max(float(c0), 0.002), max(float(c1), 0.0)
    return float(a), float(b), c0, c1, resid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?', default=os.path.join(ROOT, 'logs', 'calib_raw.csv'))
    ap.add_argument('--out-config', default=os.path.join(ROOT, 'config'))
    ap.add_argument('--fig', default=os.path.join(ROOT, 'figures'))
    ap.add_argument('--tab', default=os.path.join(ROOT, 'report'))
    args = ap.parse_args()
    rows = list(csv.DictReader(open(args.csv)))
    if len(rows) < 3:
        sys.exit(f'need at least 3 calibration distances, found {len(rows)} in {args.csv}')
    tape = np.array([float(r['tape_m']) for r in rows])
    z = np.array([float(r['z_plane_mean']) for r in rows])
    std = np.array([float(r['z_plane_std']) for r in rows])
    n = np.array([int(r['n']) for r in rows])
    a, b, c0, c1, resid = fit(tape, z, std)
    rms = float(np.sqrt(np.mean(resid ** 2)))
    json.dump({'a': a, 'b': b, 'c0': c0, 'c1': c1, 'rms_m': rms, 'n_points': len(rows), 'source': os.path.basename(args.csv)},
              open(os.path.join(args.out_config, 'calib.json'), 'w'), indent=2)
    print(f'calibration: d = {a:.4f} z_plane {b:+.4f} m   residual RMS {1000 * rms:.1f} mm   '
          f'sigma(d) = {1000 * c0:.1f} mm + {1000 * c1:.1f} mm/m^2 d^2   ({len(rows)} distances)')
    for t, zz, s, r in zip(tape, z, std, resid):
        print(f'  tape {t:.3f}  z_plane {zz:.4f}  std {1000 * s:.2f} mm  residual {1000 * r:+.1f} mm')

    os.makedirs(args.fig, exist_ok=True)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.0, 3.4))
    ax1.plot(z, tape, 'o', color=BLUE, ms=7, label='calibration points')
    zz = np.linspace(z.min() - 0.05, z.max() + 0.05, 50)
    ax1.plot(zz, a * zz + b, '-', color=MUTED, lw=1.5, label=f'fit  d = {a:.3f} z {b:+.3f}')
    ax1.set_xlabel('camera plane distance $z_{plane}$ (m)')
    ax1.set_ylabel('tape distance $d$ (m)')
    ax1.legend(frameon=False, fontsize=8)
    ax2.axhline(0, color=MUTED, lw=0.8)
    ax2.plot(tape, 1000 * resid, 's', color=ORANGE, ms=7)
    ax2.set_xlabel('tape distance $d$ (m)')
    ax2.set_ylabel('fit residual (mm)')
    for ax in (ax1, ax2):
        ax.grid(alpha=0.25, lw=0.5)
        ax.spines[['top', 'right']].set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(args.fig, 'fig_calib.png'), dpi=200)

    with open(os.path.join(args.tab, 'tab_calib.tex'), 'w') as fh:
        fh.write('\\begin{tabular}{@{}rrrrr@{}}\\toprule\n'
                 'Tape $d$ (m) & Frames & $\\bar z_{plane}$ (m) & Std (mm) & Residual (mm)\\\\\\midrule\n')
        for t, k, zz, s, r in zip(tape, n, z, std, resid):
            fh.write(f'{t:.3f} & {k} & {zz:.4f} & {1000 * s:.2f} & {1000 * r:+.1f}\\\\\n')
        fh.write('\\bottomrule\\end{tabular}\n')
        fh.write(f'%% a={a:.5f} b={b:.5f} c0={c0:.5f} c1={c1:.5f} rms={rms:.5f}\n')
    with open(os.path.join(args.tab, 'calib.tex'), 'w') as fh:
        fh.write('% written by scripts/fit_calib.py\n')
        for k, v in (('CalA', f'{a:.4f}'), ('CalB', f'{b:+.4f}'), ('CalCzero', f'{1000 * c0:.1f}'),
                     ('CalCone', f'{1000 * c1:.1f}'), ('CalRMS', f'{1000 * rms:.1f}'), ('CalN', f'{len(rows)}')):
            fh.write(f'\\renewcommand{{\\{k}}}{{{v}}}\n')
        fh.write('\\renewcommand{\\HaveCalib}{1}\n')


if __name__ == '__main__':
    main()
