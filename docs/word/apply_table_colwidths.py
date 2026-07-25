#!/usr/bin/env python3
"""Redistribute each table's column widths by content need (long-text
columns get more, short-numeric columns get less) WITHOUT changing the
table's total width -- tables stay single-column per explicit
instruction, this only fixes mid-word breaking caused by pandoc's
default equal-width columns."""
import re

PATH = 'word/document.xml'
xml = open(PATH, encoding='utf-8').read()

# caption text (unique substring) -> list of relative weights, one per column
TABLE_WEIGHTS = {
    'Mass Budget by Subsystem': [26, 36, 16, 22],                    # Subsystem | Components | Mass(kg) | Mass Fraction
    'Active Compliance Schemes': [24, 20, 20, 36],                    # Scheme | Impact v | Rebound v | Outcome
    'Joint-Damping Sweep': [22, 38, 40],                              # c | Launch separation velocity | Landing behavior
    'Component-Level Power Budget': [30, 26, 20, 24],                 # Operational State | Subsystem | Peak Power | Avg Continuous Power
}

# Process tables in document order, matching
# each to its caption by proximity (caption text appears shortly before
# the <w:tbl> in our generated doc).
captions_order = list(TABLE_WEIGHTS.keys())
tbl_iter = list(re.finditer(r'<w:tbl>.*?</w:tbl>', xml, re.S))
assert len(tbl_iter) == 4, f"expected 4 tables, found {len(tbl_iter)}"

replacements = []
for i, m in enumerate(tbl_iter):
    block = m.group(0)
    preceding = xml[max(0, m.start()-500):m.start()]
    key = None
    for k in captions_order:
        if k in preceding:
            key = k
            break
    if key is None:
        continue
    weights = TABLE_WEIGHTS[key]
    grid_m = re.search(r'<w:tblGrid>(.*?)</w:tblGrid>', block, re.S)
    cols = re.findall(r'<w:gridCol w:w="(\d+)" />', grid_m.group(1))
    total = sum(int(c) for c in cols)
    wsum = sum(weights)
    new_widths = [round(total * w / wsum) for w in weights]
    new_grid = '<w:tblGrid>' + ''.join(f'<w:gridCol w:w="{w}" />' for w in new_widths) + '</w:tblGrid>'
    new_block = block.replace(grid_m.group(0), new_grid)
    # also set each cell's <w:tcW> if present (pandoc's fixed layout uses
    # tblGrid as the authority when tcW is absent per-cell, which is the
    # case here, so grid alone should suffice)
    replacements.append((m.start(), m.end(), new_block))
    print(f'table {i+1} ({key}): {cols} -> {new_widths}')

for start, end, new_block in reversed(replacements):
    xml = xml[:start] + new_block + xml[end:]

open(PATH, 'w', encoding='utf-8').write(xml)
print(f'applied column-width fix to {len(replacements)} tables')
