#!/usr/bin/env python3
"""Render REAL captured terminal text as a terminal-window image for the report.

The text is never edited except for (a) cutting overlong lines with an ellipsis and
(b) optionally keeping only the first/last lines of a long capture ("... N lines omitted ...").

    from term_shot import term_shot
    term_shot(open('logs/captures/env_check.txt').read(), 'ameen@ameen-G6: ~', 'figures/term/env.png')
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle

BG, FG, BAR = '#1e1e1e', '#d4d4d4', '#3a3a3a'
GREEN, YELLOW, RED, BLUE, GREY = '#7ee787', '#e3b341', '#ff7b72', '#79c0ff', '#8b949e'


def _colour(line):
    s = line.strip()
    if s.startswith('$') or s.startswith('ameen@'):
        return GREEN
    if any(w in s for w in ('PASSED', 'passed', 'Finished <<<', 'OK', 'Connected', 'Successfully')):
        return GREEN
    if any(w in s for w in ('ERROR', 'Error', 'FAIL', 'failed', 'ImportError', 'Traceback', 'Can not connect')):
        return RED
    if any(w in s for w in ('WARN', 'WARNING', 'SIMULATION', 'NumPy 1.x', 'incompatible')):
        return YELLOW
    if s.startswith('#') or s.startswith('...'):
        return GREY
    return FG


def term_shot(text, title, out, width_chars=112, head=None, tail=None, fontsize=6.6, line_h=0.118):
    lines = text.rstrip('\n').split('\n')
    if head is not None and tail is not None and len(lines) > head + tail + 1:
        omitted = len(lines) - head - tail
        lines = lines[:head] + [f"... {omitted} lines omitted ..."] + (lines[-tail:] if tail else [])
    lines = [l.replace('\t', '    ') for l in lines]
    lines = [l if len(l) <= width_chars else l[:width_chars - 1] + '…' for l in lines]
    w_in = 7.4
    h_in = 0.42 + line_h * len(lines) + 0.12
    fig = plt.figure(figsize=(w_in, h_in), dpi=220)
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, w_in); ax.set_ylim(0, h_in); ax.axis('off')
    ax.add_patch(FancyBboxPatch((0.02, 0.02), w_in - 0.04, h_in - 0.04, boxstyle='round,pad=0,rounding_size=0.08',
                                fc=BG, ec='#555555', lw=0.8))
    ax.add_patch(FancyBboxPatch((0.02, h_in - 0.34), w_in - 0.04, 0.32, boxstyle='round,pad=0,rounding_size=0.08',
                                fc=BAR, ec='none'))
    for i, c in enumerate(('#ff5f56', '#ffbd2e', '#27c93f')):
        ax.add_patch(Circle((0.18 + 0.16 * i, h_in - 0.18), 0.045, fc=c, ec='none'))
    ax.text(w_in / 2, h_in - 0.18, title, color='#cccccc', ha='center', va='center', fontsize=7, family='DejaVu Sans')
    y = h_in - 0.46
    for l in lines:
        ax.text(0.16, y, l, color=_colour(l), ha='left', va='top', fontsize=fontsize, family='DejaVu Sans Mono')
        y -= line_h
    fig.savefig(out, dpi=220)
    plt.close(fig)
    return out
