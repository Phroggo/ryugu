#!/usr/bin/env python3
"""Convert Research_Paper.md into pandoc-ready markdown for the Word .docx
build: strip the HTML author/abstract/nomenclature divs into plain
markdown, and swap mermaid code fences for the pre-rendered diagram PNGs
(same d1-1..d4-1 renders used by the earlier LaTeX build -- diagram
content hasn't changed this session)."""
import re

SRC = '../Research_Paper.md'
OUT = 'paper_docx.md'

DIAGRAM_PNG = {3: 'd1-1.png', 9: 'd2-1.png', 12: 'd3-1.png', 13: 'd4-1.png'}

md = open(SRC, encoding='utf-8').read()
lines = md.split('\n')
out = []
i = 0
n = len(lines)

while i < n:
    line = lines[i]

    if line.strip().startswith('<div class="authors">'):
        depth = 0
        block = []
        while i < n:
            depth += lines[i].count('<div') - lines[i].count('</div>')
            block.append(lines[i])
            i += 1
            if depth <= 0:
                break
        text = '\n'.join(block)
        name_m = re.search(r'<div class="author-line">(.*?)</div>', text)
        affil_m = re.search(r'<div class="affil">(.*?)</div>', text)
        corr_m = re.search(r'<div class="corr">(.*?)</div>', text)
        if name_m:
            out.append(re.sub(r'&nbsp;', ' ', name_m.group(1)).replace('<sup>*</sup>', '*'))
            out.append('')
        if affil_m:
            out.append(affil_m.group(1))
            out.append('')
        if corr_m:
            out.append(re.sub(r'<sup>\*</sup>', '*', corr_m.group(1)))
            out.append('')
        continue

    if line.strip().startswith('<div class="abstract">'):
        depth = 0
        block = []
        while i < n:
            depth += lines[i].count('<div') - lines[i].count('</div>')
            block.append(lines[i])
            i += 1
            if depth <= 0:
                break
        text = '\n'.join(block)
        em_m = re.search(r'<em>(.*?)</em>', text, re.S)
        if em_m:
            out.append('**Abstract—***' + em_m.group(1).strip() + '*')
            out.append('')
        continue

    if line.strip().startswith('<div class="nomenclature">'):
        depth = 0
        block = []
        while i < n:
            depth += lines[i].count('<div') - lines[i].count('</div>')
            block.append(lines[i])
            i += 1
            if depth <= 0:
                break
        text = '\n'.join(block)
        inner = re.sub(r'</?div[^>]*>', '', text).strip()
        inner = re.sub(r'<strong>(.*?)</strong>', r'**\1**', inner)
        out.append(inner)
        out.append('')
        continue

    if line.strip().startswith('```mermaid'):
        i += 1
        while i < n and not lines[i].strip().startswith('```'):
            i += 1
        i += 1
        j = i
        while j < n and lines[j].strip() == '':
            j += 1
        cm = re.match(r'\*Figure (\d+):', lines[j].strip()) if j < n else None
        if cm:
            fign = int(cm.group(1))
            png = DIAGRAM_PNG.get(fign)
            if png:
                out.append(f'![diagram]({png})')
                out.append('')
        continue

    out.append(line)
    i += 1

open(OUT, 'w', encoding='utf-8').write('\n'.join(out))
print('wrote', OUT, len(out), 'lines')
