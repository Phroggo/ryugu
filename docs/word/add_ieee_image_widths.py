#!/usr/bin/env python3
"""Give every image in paper_ieee.md an explicit width, sized against the
official IEEE A4 template's actual geometry (pgSz 595.30pt, 44.65pt side
margins, 36pt gutter -> 3.264in per column, 7.028in full width across
both columns) rather than the Letter-based estimate used for the earlier
build. Same logic as add_image_widths.py otherwise: one genuinely wide
figure (the dual-panel hop-trajectory plot) spans both columns, the
NASA reaction-wheel photo's extreme portrait aspect is height-capped, and
everything else fits one column."""
import os
import re
from PIL import Image

MD = 'paper_ieee.md'
COL_W, FULL_W, MAX_H = 3.264, 7.028, 3.264
FULL_WIDTH = {'fig_hop_trajectory.png'}

md = open(MD, encoding='utf-8').read()

def resolve(src):
    for cand in (src, os.path.join('..', src)):
        if os.path.exists(cand):
            return cand
    raise FileNotFoundError(src)

def size(m):
    alt, src = m.group(1), m.group(2)
    w, h = Image.open(resolve(src)).size
    aspect = w / h
    target = FULL_W if src in FULL_WIDTH else min(COL_W, MAX_H * aspect)
    return f'![{alt}]({src}){{width={target:.2f}in}}'

md, n = re.subn(r'!\[([^\]]*)\]\(([^)]+\.(?:png|jpe?g))\)', size, md)
open(MD, 'w', encoding='utf-8').write(md)
print(f'sized {n} images')
