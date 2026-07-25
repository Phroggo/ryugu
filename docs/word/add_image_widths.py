#!/usr/bin/env python3
"""Give every image in paper_docx.md an explicit width.

Pandoc otherwise sizes images from their pixel dimensions and DPI, which
overflows a 3.4 in IEEE column (the 200 dpi diagram renders came out at
~5.8 in). One figure (the dual-panel hop-trajectory plot) is wide enough
that it is marked to span both columns (handled afterwards by
apply_figure_spans.py); everything else is fitted to one column and
additionally capped by height so that portrait images (dashboard crops,
the tall NASA reaction-wheel photo) do not run off the page.
"""
import os
import re
from PIL import Image

MD = 'paper_docx.md'

def resolve(src):
    for cand in (src, os.path.join('..', src)):
        if os.path.exists(cand):
            return cand
    raise FileNotFoundError(src)
COL_W, FULL_W, MAX_H = 3.30, 6.90, 3.60   # inches
FULL_WIDTH = {'fig_hop_trajectory.png'}

md = open(MD, encoding='utf-8').read()

def size(m):
    alt, src = m.group(1), m.group(2)
    w, h = Image.open(resolve(src)).size
    aspect = w / h
    target = FULL_W if src in FULL_WIDTH else min(COL_W, MAX_H * aspect)
    return f'![{alt}]({src}){{width={target:.2f}in}}'

md, n = re.subn(r'!\[([^\]]*)\]\(([^)]+\.(?:png|jpe?g))\)', size, md)
open(MD, 'w', encoding='utf-8').write(md)
print(f'sized {n} images')