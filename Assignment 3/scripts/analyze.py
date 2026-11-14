#!/usr/bin/env python3
"""Error calculation, statistics, plots and LaTeX tables for the Phase 3 trials.

Error of trial i (brief, item 3):   e_i = d̂_i - d_i     (estimated minus tape-measured final distance)
For each estimator (fused KF, odometry only, camera only):
    bias  ē = (1/n) Σ e_i            SD  s = sqrt(Σ (e_i - ē)² / (n - 1))
    MAE   = (1/n) Σ |e_i|            RMSE = sqrt((1/n) Σ e_i²)
    range = max e_i - min e_i        95 % CI of the bias: ē ± t_{0.975,n-1} s / √n
    one-sample t-test of ē = 0, Shapiro-Wilk normality test,
    paired Wilcoxon signed-rank test of |e_KF| against |e_odom| and |e_cam|.

Usage:
    python scripts/analyze.py                                   # logs/trials.csv -> figures/, report/
    python scripts/analyze.py --trials sim_out/sim_trials.csv --logs sim_out --fig X --tab Y --label SIMULATION
"""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# fixed categorical order (validated default palette): 1 blue, 2 orange, 3 aqua
C_KF, C_ODOM, C_CAM = '#2a78d6', '#eb6834', '#1baf7a'
INK, MUTED, GRID = '#0b0b0b', '#52514e', '#d9d8d4'
METHODS = [('kf', 'Fused KF (odometry + camera)', C_KF, 'o'),
           ('odom', 'Odometry only (prediction)', C_ODOM, 's'),
           ('cam', 'Camera only (correction sensor)', C_CAM, '^')]
plt.rcParams.update({'font.size': 9, 'axes.edgecolor': MUTED, 'axes.labelcolor': INK, 'xtick.color': MUTED,
                     'ytick.color': MUTED, 'axes.spines.top': False, 'axes.spines.right': False})


def fnum(x):
    try:
        v = float(x)
        return v if not math.isnan(v) else None
    except (TypeError, ValueError):
        return None


def describe(e):
    e = np.asarray(e, float)
    n = len(e)
    out = {'n': n, 'bias': e.mean(), 'sd': e.std(ddof=1) if n > 1 else np.nan, 'mae': np.abs(e).mean(),
           'rmse': math.sqrt(np.mean(e ** 2)), 'median': float(np.median(e)), 'min': e.min(), 'max': e.max(),
           'range': e.max() - e.min(), 'maxabs': np.abs(e).max()}
    if n > 1:
        h = stats.t.ppf(0.975, n - 1) * out['sd'] / math.sqrt(n)
        out['ci_lo'], out['ci_hi'] = out['bias'] - h, out['bias'] + h
        out['t'], out['p_t'] = stats.ttest_1samp(e, 0.0)
    else:
        out['ci_lo'] = out['ci_hi'] = out['t'] = out['p_t'] = np.nan
    out['p_sw'] = stats.shapiro(e).pvalue if n >= 3 else np.nan
    return out


def load_trace(path):
    rows = list(csv.DictReader(open(path)))
    g = lambda k: np.array([fnum(r[k]) if fnum(r[k]) is not None else np.nan for r in rows])
    return {k: g(k) for k in ('t', 's_odom', 'd_odom_only', 'd_kf', 'sigma_kf', 'z_comp', 'z_cal', 'v_cmd', 'innov')} | \
        {'phase': [r['phase'] for r in rows], 'accepted': [r['accepted'] for r in rows]}


def zoom_window(ax, tr, row, seconds=3.0):
    """Last `seconds` of the trial (stop + settle) so the final mm-level differences are visible."""
    t = tr['t']
    t1 = t[-1] + 0.15
    t0 = t1 - seconds - 0.15
    m = t >= t0
    vals = [tr['d_kf'][m], tr['d_odom_only'][m], tr['z_comp'][m]]
    tape = fnum(row.get('tape_m'))
    if tape is not None:
        vals.append(np.array([tape]))
    v = np.concatenate([x[~np.isnan(x)] for x in vals[:3]])
    lo, hi = np.percentile(v, 1), np.percentile(v, 99)
    if tape is not None:            # the tape star must always be inside the window
        lo, hi = min(lo, tape), max(hi, tape)
    pad = max(0.004, 0.15 * (hi - lo))
    ax.set_xlim(t0, t1)
    ax.set_ylim(lo - pad, hi + pad)


def plot_trial(ax, tr, row, d_stop, compact=False):
    t = tr['t']
    acc = np.array([a == '1' for a in tr['accepted']])
    rej = np.array([a == '0' for a in tr['accepted']])
    ax.axhline(d_stop, color=MUTED, lw=0.8, ls=':')
    ax.fill_between(t, tr['d_kf'] - 2 * tr['sigma_kf'], tr['d_kf'] + 2 * tr['sigma_kf'], color=C_KF, alpha=0.15, lw=0)
    ax.scatter(t[acc], tr['z_comp'][acc], s=7 if compact else 10, color=C_CAM, marker='^', lw=0,
               label='camera measurement $z_k$ (scatter)', zorder=2)
    if rej.any():
        ax.scatter(t[rej], tr['z_comp'][rej], s=14, facecolors='none', edgecolors=INK, lw=0.7,
                   label='rejected by 3$\\sigma$ gate', zorder=3)
    ax.plot(t, tr['d_odom_only'], '--', color=C_ODOM, lw=1.4, label='odometry only $d_0-s_k$')
    ax.plot(t, tr['d_kf'], '-', color=C_KF, lw=1.6, label='fused KF $\\hat d_k$ ($\\pm2\\sigma$)')
    tape = fnum(row.get('tape_m'))
    if tape is not None:
        ax.plot([t[-1]], [tape], marker='*', ms=11 if not compact else 9, color=INK, lw=0, label='tape measurement', zorder=4)
    if 'APPROACH' in tr['phase'] and 'SETTLE' in tr['phase']:
        ts = t[tr['phase'].index('SETTLE')]
        ax.axvline(ts, color=MUTED, lw=0.6)
    ax.grid(color=GRID, lw=0.5)
    ek = fnum(row.get('err_kf'))
    title = row['trial'].replace('_', ' ').title()
    if ek is not None:
        title += f'   $e$ = {1000 * ek:+.1f} mm'
    ax.set_title(title, fontsize=8.5 if compact else 10, loc='left', color=INK)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', default=os.path.join(ROOT, 'logs', 'trials.csv'))
    ap.add_argument('--logs', default=os.path.join(ROOT, 'logs', 'trials'))
    ap.add_argument('--fig', default=os.path.join(ROOT, 'figures'))
    ap.add_argument('--tab', default=os.path.join(ROOT, 'report'))
    ap.add_argument('--label', default='', help='text stamped on every figure (e.g. SIMULATION)')
    args = ap.parse_args()
    os.makedirs(args.fig, exist_ok=True)
    all_rows = list(csv.DictReader(open(args.trials)))
    rows = [r for r in all_rows if r['ok'] in ('True', '1', 'true') and fnum(r.get('tape_m')) is not None]
    excluded = [r for r in all_rows if r not in rows]
    if not rows:
        raise SystemExit('no completed trial with a tape measurement yet')
    d_stop = float(rows[0]['d_stop'])
    d0 = float(rows[0]['d0'])
    stamp = (lambda fig: fig.text(0.99, 0.01, args.label, ha='right', va='bottom', color='#d03b3b', fontsize=12,
                                  weight='bold')) if args.label else (lambda fig: None)

    # ---------------- per-trial scatter plots (individual + grid)
    traces = {}
    for r in rows:
        p = os.path.join(args.logs, f'{r["trial"]}.csv')
        if os.path.exists(p):
            traces[r['trial']] = load_trace(p)
    for r in rows:
        if r['trial'] not in traces:
            continue
        fig, (ax, az) = plt.subplots(1, 2, figsize=(9.6, 3.4), gridspec_kw={'width_ratios': [1.35, 1]})
        plot_trial(ax, traces[r['trial']], r, d_stop)
        plot_trial(az, traces[r['trial']], r, d_stop, compact=True)
        zoom_window(az, traces[r['trial']], r)
        az.set_title('stop and settle (zoom)', loc='left', fontsize=9, color=INK)
        for a in (ax, az):
            a.set_xlabel('time (s)')
        ax.set_ylabel('distance to tag (m)')
        ax.legend(frameon=False, fontsize=7.5, loc='upper right')
        stamp(fig)
        fig.tight_layout()
        fig.savefig(os.path.join(args.fig, f'fig_{r["trial"]}.png'), dpi=200)
        plt.close(fig)
    names = [r['trial'] for r in rows if r['trial'] in traces]
    if names:
        nc = 2
        nr = math.ceil(len(names) / nc)
        fig, axes = plt.subplots(nr, nc, figsize=(7.2, 1.85 * nr + 0.6), squeeze=False)
        for ax, nm in zip(axes.flat, names):
            rr = next(r for r in rows if r['trial'] == nm)
            plot_trial(ax, traces[nm], rr, d_stop, compact=True)
            zoom_window(ax, traces[nm], rr)
        for ax in list(axes.flat)[len(names):]:
            ax.axis('off')
        for ax in axes[:, 0]:
            ax.set_ylabel('distance (m)')
        for ax in axes[-1, :]:
            ax.set_xlabel('time (s), last 3 s of the trial')
        h, l = axes.flat[0].get_legend_handles_labels()
        fig.legend(h, l, loc='upper center', ncol=3, frameon=False, fontsize=7.5)
        stamp(fig)
        fig.tight_layout(rect=(0, 0, 1, 1 - 0.55 / (1.85 * nr + 0.6)))
        fig.savefig(os.path.join(args.fig, 'fig_trials_grid.png'), dpi=200)
        plt.close(fig)

    # ---------------- statistics
    tape = np.array([float(r['tape_m']) for r in rows])
    E, D = {}, {}
    for k, *_ in METHODS:
        pairs = [(float(r[f'est_{k}']), float(r['tape_m'])) for r in rows if fnum(r.get(f'est_{k}')) is not None]
        D[k] = np.array([p[0] for p in pairs])
        E[k] = np.array([p[0] - p[1] for p in pairs])
    S = {k: describe(E[k]) for k in E if len(E[k])}
    wil = {}
    for k in ('odom', 'cam'):
        if k in S and len(E[k]) == len(E['kf']) and len(E['kf']) >= 5:
            try:
                wil[k] = stats.wilcoxon(np.abs(E['kf']), np.abs(E[k])).pvalue
            except ValueError:
                wil[k] = np.nan
    stop_sd = tape.std(ddof=1) if len(tape) > 1 else np.nan
    corr = np.array([float(r['est_kf']) - float(r['est_odom']) for r in rows])
    hz = np.mean([float(r['loop_hz']) for r in rows])
    det_rate = np.mean([float(r['detect_rate']) for r in rows])
    rollout = np.array([float(r['rollout_m']) for r in rows if fnum(r.get('rollout_m')) is not None])
    sig = np.array([float(r['sigma_kf']) for r in rows])
    lat_dev = np.array([float(r['max_lateral_dev_m']) for r in rows if fnum(r.get('max_lateral_dev_m')) is not None])
    max_bear = np.array([float(r['max_abs_bearing_deg']) for r in rows if fnum(r.get('max_abs_bearing_deg')) is not None])
    z95 = np.mean(np.abs(E['kf']) <= 2 * sig)

    # ---------------- final scatter: estimate vs measurement
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.2, 5.0))
    lo, hi = min(tape.min(), *(D[k].min() for k in D)) - 0.01, max(tape.max(), *(D[k].max() for k in D)) + 0.01
    ax1.plot([lo, hi], [lo, hi], color=MUTED, lw=0.9, label='ideal $\\hat d = d$')
    for k, lab, c, m in METHODS:
        tk = np.array([float(r['tape_m']) for r in rows if fnum(r.get(f'est_{k}')) is not None])
        if k == 'cam':   # hollow and larger: the camera and fused points nearly coincide
            ax1.scatter(tk, D[k], s=70, facecolors='none', edgecolors=c, marker=m, lw=1.3, label=lab, zorder=4)
        else:
            ax1.scatter(tk, D[k], s=34 if k == 'kf' else 42, color=c, marker=m, edgecolors='white', lw=0.8,
                        label=lab, zorder=3)
    ax1.set_xlabel('measured final distance $d_i$ (m, tape)')
    ax1.set_ylabel('estimated final distance $\\hat d_i$ (m)')
    ax1.set_xlim(lo, hi), ax1.set_ylim(lo, hi), ax1.set_aspect('equal')
    ax1.grid(color=GRID, lw=0.5)
    ax1.legend(frameon=False, fontsize=7.5, loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=1)
    ax1.set_title('Final estimate vs measurement, all trials', loc='left', fontsize=10)
    idx = np.arange(1, len(rows) + 1)
    ax2.axhline(0, color=MUTED, lw=0.9)
    sk = S['kf']
    ax2.axhspan(1000 * (sk['bias'] - 2 * sk['sd']), 1000 * (sk['bias'] + 2 * sk['sd']), color=C_KF, alpha=0.10, lw=0,
                label='KF bias $\\pm 2$ SD')
    ax2.axhline(1000 * sk['bias'], color=C_KF, lw=1, ls='--', label=f'KF bias {1000 * sk["bias"]:+.1f} mm')
    off = {'kf': 0.0, 'odom': -0.18, 'cam': 0.18}
    for k, lab, c, m in METHODS:
        ii = np.array([i for i, r in zip(idx, rows) if fnum(r.get(f'est_{k}')) is not None])
        ax2.scatter(ii + off[k], 1000 * E[k], s=38, color=c, marker=m, edgecolors='white', lw=0.8, zorder=3,
                    label=lab.split(' (')[0])
    ax2.set_xticks(idx)
    ax2.set_xlabel('trial')
    ax2.set_ylabel('error $e_i=\\hat d_i-d_i$ (mm)')
    ax2.grid(color=GRID, lw=0.5)
    ax2.legend(frameon=False, fontsize=7.5, loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=2)
    ax2.set_title('Error per trial', loc='left', fontsize=10)
    stamp(fig)
    fig.tight_layout()
    fig.savefig(os.path.join(args.fig, 'fig_final_scatter.png'), dpi=200)
    plt.close(fig)

    # ---------------- error distributions
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    data = [1000 * E[k] for k, *_ in METHODS if k in E]
    bp = ax.boxplot(data, vert=False, widths=0.5, patch_artist=True, showfliers=False,
                    medianprops={'color': INK}, whiskerprops={'color': MUTED}, capprops={'color': MUTED})
    rng = np.random.default_rng(0)
    for i, ((k, lab, c, m), patch) in enumerate(zip(METHODS, bp['boxes']), start=1):
        patch.set_facecolor(c + '33'), patch.set_edgecolor(c)
        ax.scatter(1000 * E[k], i + rng.uniform(-0.12, 0.12, len(E[k])), s=20, color=c, marker=m, zorder=3)
    ax.set_yticks([1, 2, 3], ['fused KF', 'odometry only', 'camera only'])
    ax.axvline(0, color=MUTED, lw=0.8)
    ax.set_xlabel('final distance error (mm)')
    ax.grid(axis='x', color=GRID, lw=0.5)
    stamp(fig)
    fig.tight_layout()
    fig.savefig(os.path.join(args.fig, 'fig_error_box.png'), dpi=200)
    plt.close(fig)

    # ---------------- LaTeX tables and macros
    mm = lambda x: f'{1000 * x:+.1f}'
    with open(os.path.join(args.tab, 'tab_trials.tex'), 'w') as fh:
        fh.write('\\begin{tabular}{@{}lrrrrrrrr@{}}\\toprule\n'
                 '& \\multicolumn{2}{c}{Fused KF} & & \\multicolumn{2}{c}{Odometry only} & \\multicolumn{2}{c}{Camera only} & \\\\\n'
                 '\\cmidrule(lr){2-3}\\cmidrule(lr){5-6}\\cmidrule(lr){7-8}\n'
                 'Trial & $\\hat d_i$ (m) & $e_i$ (mm) & $d_i$ tape (m) & $\\hat d_i$ (m) & $e_i$ (mm) & $\\hat d_i$ (m) & $e_i$ (mm) & $\\sigma_{KF}$ (mm)\\\\\\midrule\n')
        for r in rows:
            g = lambda k: (f'{float(r[f"est_{k}"]):.4f}', mm(float(r[f'err_{k}']))) if fnum(r.get(f'err_{k}')) is not None else ('--', '--')
            fh.write(f'{r["trial"].split("_")[-1]} & {g("kf")[0]} & {g("kf")[1]} & {float(r["tape_m"]):.4f} & '
                     f'{g("odom")[0]} & {g("odom")[1]} & {g("cam")[0]} & {g("cam")[1]} & {1000 * float(r["sigma_kf"]):.1f}\\\\\n')
        fh.write('\\bottomrule\\end{tabular}\n')
    with open(os.path.join(args.tab, 'tab_stats.tex'), 'w') as fh:
        fh.write('\\begin{tabular}{@{}lrrr@{}}\\toprule\n'
                 'Statistic (mm unless stated) & Fused KF & Odometry only & Camera only\\\\\\midrule\n')
        spec = [('Trials $n$', 'n', lambda v: f'{v:d}'), ('Mean error (bias) $\\bar e$', 'bias', mm),
                ('Standard deviation $s$', 'sd', lambda v: f'{1000 * v:.1f}'),
                ('Mean absolute error (MAE)', 'mae', lambda v: f'{1000 * v:.1f}'),
                ('Root mean square error (RMSE)', 'rmse', lambda v: f'{1000 * v:.1f}'),
                ('Median error', 'median', mm), ('Minimum error', 'min', mm), ('Maximum error', 'max', mm),
                ('Range of errors $e_{max}-e_{min}$', 'range', lambda v: f'{1000 * v:.1f}'),
                ('Largest $|e_i|$', 'maxabs', lambda v: f'{1000 * v:.1f}'),
                ('95\\,\\% CI of the bias', None, None),
                ('$t$-test $H_0{:}\\ \\bar e=0$, $p$', 'p_t', lambda v: f'{v:.3f}'),
                ('Shapiro--Wilk normality, $p$', 'p_sw', lambda v: f'{v:.3f}')]
        for lab, key, f in spec:
            cells = []
            for k, *_ in METHODS:
                s = S.get(k)
                if s is None:
                    cells.append('--')
                elif key is None:
                    cells.append(f'[{mm(s["ci_lo"])}, {mm(s["ci_hi"])}]')
                else:
                    v = s[key]
                    cells.append('--' if isinstance(v, float) and math.isnan(v) else f(v))
            fh.write(f'{lab} & ' + ' & '.join(cells) + '\\\\\n')
        fh.write('\\midrule\n')
        fh.write('Wilcoxon $|e_{KF}|$ vs $|e|$ of this method, $p$ & -- & '
                 + ' & '.join('--' if k not in wil or math.isnan(wil[k]) else f'{wil[k]:.3f}' for k in ('odom', 'cam'))
                 + '\\\\\n\\bottomrule\\end{tabular}\n')

    def mac(name, val):
        return f'\\renewcommand{{\\{name}}}{{{val}}}\n'
    k, o, c = S['kf'], S['odom'], S.get('cam', S['kf'])
    impr = 100 * (1 - k['mae'] / o['mae']) if o['mae'] > 0 else float('nan')
    with open(os.path.join(args.tab, 'results.tex'), 'w') as fh:
        fh.write('% written by scripts/analyze.py - do not edit by hand\n')
        for name, val in [
            ('ResN', f'{len(rows)}'), ('ResNexcl', f'{len(excluded)}'),
            ('ResStart', f'{d0:.2f}'), ('ResStop', f'{d_stop:.2f}'),
            ('ResMAE', f'{1000 * k["mae"]:.1f}'), ('ResRange', f'{1000 * k["range"]:.1f}'),
            ('ResMin', mm(k['min'])), ('ResMax', mm(k['max'])), ('ResBias', mm(k['bias'])),
            ('ResSD', f'{1000 * k["sd"]:.1f}'), ('ResRMSE', f'{1000 * k["rmse"]:.1f}'),
            ('ResCI', f'[{mm(k["ci_lo"])}, {mm(k["ci_hi"])}]'), ('ResPt', f'{k["p_t"]:.3f}'),
            ('ResPsw', f'{k["p_sw"]:.3f}'),
            ('ResOdomMAE', f'{1000 * o["mae"]:.1f}'), ('ResOdomRange', f'{1000 * o["range"]:.1f}'),
            ('ResOdomBias', mm(o['bias'])), ('ResOdomSD', f'{1000 * o["sd"]:.1f}'),
            ('ResCamMAE', f'{1000 * c["mae"]:.1f}'), ('ResCamRange', f'{1000 * c["range"]:.1f}'),
            ('ResCamBias', mm(c['bias'])),
            ('ResImprove', f'{impr:.0f}'),
            ('ResWilOdom', f'{wil.get("odom", float("nan")):.3f}'), ('ResWilCam', f'{wil.get("cam", float("nan")):.3f}'),
            ('ResStopMean', f'{tape.mean():.4f}'), ('ResStopSD', f'{1000 * stop_sd:.1f}'),
            ('ResStopMin', f'{tape.min():.4f}'), ('ResStopMax', f'{tape.max():.4f}'),
            ('ResCorrMean', mm(corr.mean())), ('ResCorrMaxAbs', f'{1000 * np.abs(corr).max():.1f}'),
            ('ResHz', f'{hz:.1f}'), ('ResDetect', f'{100 * det_rate:.0f}'),
            ('ResRollout', f'{100 * rollout.mean():.1f}' if len(rollout) else '--'),
            ('ResSigmaMean', f'{1000 * sig.mean():.1f}'), ('ResCover', f'{100 * z95:.0f}'),
            ('ResLatency', f'{float(rows[0]["latency_s"]):.3f}'),
            ('ResLateral', f'{1000 * lat_dev.mean():.1f}' if len(lat_dev) else '--'),
            ('ResLateralMax', f'{1000 * lat_dev.max():.1f}' if len(lat_dev) else '--'),
            ('ResBearingMax', f'{max_bear.max():.1f}' if len(max_bear) else '--'),
        ]:
            fh.write(mac(name, val))
        fh.write('\\renewcommand{\\HaveResults}{1}\n')

    # ---------------- terminal summary (captured for the report)
    lines = [f'{args.label + ": " if args.label else ""}{len(rows)} trials used, {len(excluded)} excluded '
             f'({", ".join(r["trial"] for r in excluded) or "none"})',
             f'start {d0:.2f} m, stop target {d_stop:.2f} m; tape final distance {tape.mean():.4f} m '
             f'+/- {1000 * stop_sd:.1f} mm (SD), {tape.min():.4f} .. {tape.max():.4f} m',
             f'{"method":<14}{"bias":>8}{"SD":>7}{"MAE":>7}{"RMSE":>7}{"min":>8}{"max":>8}{"range":>7}   (mm)']
    for kk, lab, *_ in METHODS:
        s = S.get(kk)
        if s:
            lines.append(f'{lab.split(" (")[0]:<14}{1000 * s["bias"]:>+8.1f}{1000 * s["sd"]:>7.1f}{1000 * s["mae"]:>7.1f}'
                         f'{1000 * s["rmse"]:>7.1f}{1000 * s["min"]:>+8.1f}{1000 * s["max"]:>+8.1f}{1000 * s["range"]:>7.1f}')
    lines.append(f'KF bias 95% CI {1000 * k["ci_lo"]:+.1f} .. {1000 * k["ci_hi"]:+.1f} mm, t-test p = {k["p_t"]:.3f}, '
                 f'Shapiro-Wilk p = {k["p_sw"]:.3f}; Wilcoxon |e_KF| vs |e_odom| p = {wil.get("odom", float("nan")):.3f}')
    lines.append(f'KF MAE is {impr:.0f}% lower than odometry-only; {100 * z95:.0f}% of trials inside +/-2 sigma_KF; '
                 f'loop {hz:.1f} Hz, tag seen in {100 * det_rate:.0f}% of loops')
    txt = '\n'.join(lines)
    print(txt)
    cap = os.path.join(os.path.dirname(args.trials), 'captures')
    os.makedirs(cap, exist_ok=True)
    open(os.path.join(cap, 'analyze.txt'), 'w').write('$ python scripts/analyze.py\n' + txt + '\n')


if __name__ == '__main__':
    main()
