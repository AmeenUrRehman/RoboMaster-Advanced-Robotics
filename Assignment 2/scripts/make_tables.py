#!/usr/bin/env python3
"""LaTeX tables for the report, computed from the package and the real logs (no hand-typed numbers).
Writes report/tab_coeff.tex and report/tab_runs.tex."""
import csv, sys
import numpy as np
sys.path.insert(0, 'ws/src/rm_flatness_nav')
from rm_flatness_nav.flatness import FlatTrajectory

# ---- trajectory coefficients and peak speeds
out = [r'\begin{tabular}{@{}rrrrrr@{}}', r'\toprule', r'$d_0$ (m) & $T$ (s) & $\alpha_2^{[x]}$ (m/s$^2$) & $\alpha_3^{[x]}$ (m/s$^3$) & peak $v$ (m/s) & at $t$ (s)\\', r'\midrule']
for d0 in (0.9, 1.0, 1.28, 1.5, 1.8):
    tr = FlatTrajectory(d0); t = np.linspace(0, tr.T, 4000); v = np.array([tr.sample(x)[0] for x in t])
    out.append(f'{d0:.2f} & {tr.T:.2f} & {tr.alpha_x[2]:.5f} & {tr.alpha_x[3]:.3e} & {v.max():.3f} & {t[v.argmax()]:.2f}\\\\'.replace('e-0', r'\times10^{-').replace('e-', r'\times10^{-') if False else
               f'{d0:.2f} & {tr.T:.2f} & {tr.alpha_x[2]:.5f} & ${tr.alpha_x[3]*1e4:.3f}\\times10^{{-4}}$ & {v.max():.3f} & {t[v.argmax()]:.2f}\\\\')
out += [r'\bottomrule', r'\end{tabular}']
open('report/tab_coeff.tex', 'w').write('\n'.join(out) + '\n')

# ---- per-run statistics of the real runs
runs = [('1', '16:00', 'Wi-Fi AP', 'logs/archive_20261023_155809'), ('2', '16:04', 'Wi-Fi AP', 'logs/archive_20261023_183424'),
        ('3', '18:36', 'USB', 'logs/archive_20261023_191249'), ('4', '19:14', 'USB', 'logs/upright_run4'), ('5', '00:44', 'USB', 'logs/flat_run5')]
out = [r'\begin{tabular}{@{}llllrrrrrl@{}}', r'\toprule',
       r'Run & logged & link & final state & search (s) & first seen (m) & chase $d_0$ (m) & chase (s) & peak $v$ (m/s) & last seen (m)\\', r'\midrule']
for n, when, link, d in runs:
    import os
    L = list(csv.DictReader(open(f'{d}/motor_voltage_log.csv')))
    F = list(csv.DictReader(open(f'{d}/motor_voltage_fast.csv'))) if os.path.exists(f'{d}/motor_voltage_fast.csv') else L   # run 5: no 20 Hz log, 1 Hz rows used
    det = [r for r in L if r['distance_m']]
    ts = [float(r['t_s']) for r in F]; st = [r['state'].split(' ')[0] for r in F]; v = [float(r['v_mps']) for r in F]
    first_non = next((ts[i] for i, s in enumerate(st) if s != 'SEARCHING'), None)
    ch = [i for i, s in enumerate(st) if s == 'CHASING']
    arr = next((ts[i] for i, s in enumerate(st) if s == 'ARRIVED'), None)
    chase = f'{(arr - ts[ch[0]]):.1f}' if (ch and arr) else '--'
    d0 = next((r['distance_m'] for r in L if r['state'] == 'CHASING' and r['distance_m']), '--')
    final = 'ARRIVED' if L[-1]['state'].startswith('ARRIVED') else 'SEARCHING (no halt)'
    out.append(f"{n} & {when} & {link} & {final} & {first_non:.1f} & {det[0]['distance_m']} & {d0} & {chase} & {max(v):.3f} & {det[-1]['distance_m']}\\\\")
out += [r'\bottomrule', r'\end{tabular}']
open('report/tab_runs.tex', 'w').write('\n'.join(out) + '\n')
print(open('report/tab_coeff.tex').read()); print(open('report/tab_runs.tex').read())
