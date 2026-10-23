#!/usr/bin/env python3
"""LaTeX Table 1 from the real 1 Hz log.  Usage: python scripts/make_table.py logs/motor_voltage_log.csv report/table1.tex"""
import csv
import sys

rows = list(csv.DictReader(open(sys.argv[1])))
out = [r'\begin{longtable}{@{}rrrrrp{5.2cm}@{}}',
       r'\caption*{\textbf{Table 1.} Chassis motor armature voltage log (target: Tag 3), from \texttt{motor\_voltage\_log.csv}, one row per second.}\\',
       r'\toprule', r'\textbf{$t$ (s)} & $V_1$ (V) & $V_2$ (V) & $V_3$ (V) & $V_4$ (V) & \textbf{State}\\', r'\midrule', r'\endfirsthead',
       r'\toprule', r'\textbf{$t$ (s)} & $V_1$ (V) & $V_2$ (V) & $V_3$ (V) & $V_4$ (V) & \textbf{State}\\', r'\midrule', r'\endhead',
       r'\bottomrule', r'\endfoot']
for r in rows:
    t = r['t_s'] if not r['state'].startswith('ARRIVED') else r'$T_{\rm stop}$=' + r['t_s']
    st = r['state'].replace('_', r'\_')
    out.append(f"{t} & {r['V1_V']} & {r['V2_V']} & {r['V3_V']} & {r['V4_V']} & \\texttt{{{st}}}\\\\")
out.append(r'\end{longtable}')
open(sys.argv[2], 'w').write('\n'.join(out) + '\n')
print('wrote', sys.argv[2], len(rows), 'rows; last row:', rows[-1]['V1_V'], rows[-1]['V2_V'], rows[-1]['V3_V'], rows[-1]['V4_V'], rows[-1]['state'])
