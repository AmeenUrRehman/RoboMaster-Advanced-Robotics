#!/usr/bin/env python3
"""Render the captured (real) terminal text in logs/captures/*.txt as images figures/term_*.png."""
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from term_shot import term_shot  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in sorted(glob.glob(os.path.join(ROOT, 'logs', 'captures', '*.txt'))):
    name = os.path.splitext(os.path.basename(p))[0]
    term_shot(open(p).read(), 'ameen@ameen: ~/robomasters/assignment3', os.path.join(ROOT, 'figures', f'term_{name}.png'),
              head=40, tail=6)
    print('rendered', name)
