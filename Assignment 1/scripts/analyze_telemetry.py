#!/usr/bin/env python3
"""Telemetry-interval and ping-RTT comparison between wired RNDIS and Wi-Fi AP runs.

Usage:
    python scripts/analyze_telemetry.py \
        --rndis-csv logs/rndis_run.csv --ap-csv logs/ap_run.csv \
        --rndis-ping logs/ping_rndis.txt --ap-ping logs/ping_ap.txt --out figures
Writes fig_dt.png, fig_ping.png and stats.json (numbers for Table 1 / Problem 2(iv)).
"""
import argparse
import csv
import json
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['cmr10', 'DejaVu Serif'], 'mathtext.fontset': 'cm',
                     'axes.formatter.use_mathtext': True, 'axes.unicode_minus': False, 'font.size': 10})
import numpy as np

COLORS = {'RNDIS (wired)': '#1f6feb', 'AP (wireless)': '#d9480f'}


def load_csv(path):
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def load_ping(path):
    rtts = [float(m.group(1)) for m in re.finditer(r'time=([\d.]+) ms', open(path).read())]
    return np.array(rtts)


def stats(x):
    return {'mean': float(np.mean(x)), 'std': float(np.std(x, ddof=1)),
            'min': float(np.min(x)), 'max': float(np.max(x)),
            'p95': float(np.percentile(x, 95)), 'n': int(len(x))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rndis-csv', required=True)
    ap.add_argument('--ap-csv', required=True)
    ap.add_argument('--rndis-ping', required=True)
    ap.add_argument('--ap-ping', required=True)
    ap.add_argument('--out', default='figures')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    runs = {'RNDIS (wired)': (a.rndis_csv, a.rndis_ping), 'AP (wireless)': (a.ap_csv, a.ap_ping)}
    out = {}
    fig, axes = plt.subplots(2, 2, figsize=(10, 6.5), constrained_layout=True)
    fig2, ax2 = plt.subplots(1, 2, figsize=(10, 3.6), constrained_layout=True)

    for name, (csv_path, ping_path) in runs.items():
        d, rtt = load_csv(csv_path), load_ping(ping_path)
        c = COLORS[name]
        # Δt of newest-ESC-packet arrival times = network-level telemetry interval
        t_rx = np.unique(d['t_esc_rx_unix'][np.isfinite(d['t_esc_rx_unix'])])
        dt_rx = np.diff(t_rx) * 1e3
        dt_log = np.diff(d['t_rel_s']) * 1e3  # logger loop interval (host scheduling only)
        j_rtt = np.abs(np.diff(rtt))          # mean |RTT_i - RTT_{i-1}| (RFC 3550-style jitter)
        out[name] = {'dt_esc_rx_ms': stats(dt_rx), 'dt_logger_ms': stats(dt_log),
                     'ping_rtt_ms': stats(rtt), 'ping_jitter_mean_abs_diff_ms': float(np.mean(j_rtt)),
                     'ping_loss_pct': float(100 * (100 - len(rtt)) / 100)}

        axes[0, 0].plot(np.arange(len(dt_rx)) / 10.0, dt_rx, '.-', ms=3, lw=0.7, color=c, label=name)
        axes[0, 1].hist(dt_rx, bins=40, alpha=0.6, color=c, label=name)
        axes[1, 0].plot(np.arange(len(dt_log)) / 10.0, dt_log, '.-', ms=3, lw=0.7, color=c, label=name)
        axes[1, 1].hist(dt_log, bins=40, alpha=0.6, color=c, label=name)
        ax2[0].plot(np.arange(1, len(rtt) + 1), rtt, '.-', ms=3, lw=0.7, color=c, label=name)
        ax2[1].hist(rtt, bins=30, alpha=0.6, color=c, label=name)

    for ax, title, xl, yl in [
        (axes[0, 0], r'Telemetry arrival interval $\Delta t=t_k-t_{k-1}$ (ESC packets)', 'time (s)', r'$\Delta t$ (ms)'),
        (axes[0, 1], 'Distribution of arrival interval', r'$\Delta t$ (ms)', 'count'),
        (axes[1, 0], r'10 Hz logger loop interval', 'time (s)', r'$\Delta t$ (ms)'),
        (axes[1, 1], 'Distribution of logger interval', r'$\Delta t$ (ms)', 'count')]:
        ax.set_title(title, fontsize=10); ax.set_xlabel(xl); ax.set_ylabel(yl)
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    fig.savefig(os.path.join(a.out, 'fig_dt.png'), dpi=200)

    ax2[0].set_title('Ping RTT, N=100'); ax2[0].set_xlabel('packet #'); ax2[0].set_ylabel('RTT (ms)')
    ax2[1].set_title('RTT distribution'); ax2[1].set_xlabel('RTT (ms)'); ax2[1].set_ylabel('count')
    for ax in ax2:
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    fig2.savefig(os.path.join(a.out, 'fig_ping.png'), dpi=200)

    with open(os.path.join(a.out, 'stats.json'), 'w') as fh:
        json.dump(out, fh, indent=2)
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
