#!/usr/bin/env python3
"""Convert the SpaceHopper Research_Paper.md to an IEEEtran LaTeX document."""
import re, sys

SRC = '../Research_Paper.md'
OUT = 'spacehopper_ieee.tex'

# figure N -> image file (photos are PNG; the 4 mermaid diagrams are rendered PDFs)
FIGFILE = {
    1:'heightmap_preview.png', 2:'fig_swarm_terrain.png', 3:'diagram1.pdf',
    4:'fig_robot_closeup.png', 5:'fig_hop_flight.png', 6:'fig_descent_terrain.png',
    7:'diagram2.pdf', 8:'fig_landed_scout.png', 9:'fig_recharge_dashboard.png',
    10:'diagram3.pdf', 11:'fig_dashboard.png', 12:'diagram4.pdf',
    13:'fig_hop_trajectory.png', 14:'fig_sampling_dashboard.png',
    15:'fig_sampling.png', 16:'fig_sampling_dashboard_2.png',
}
DIAGRAMS = {3,7,10,12}

UNI = [
    ('—','---'), ('–','--'),
    ('10⁻⁴', r'\ensuremath{10^{-4}}'),
    ('⁻⁴', r'\textsuperscript{-4}'),
    ('²', r'\textsuperscript{2}'), ('⁴', r'\textsuperscript{4}'),
    ('⁻', r'\textsuperscript{-}'),
    ('§', r'\S'), ('°', r'\textdegree{}'),
    ('·', r'\,\textperiodcentered\,'),
    ('×', r'\ensuremath{\times}'), ('≈', r'\ensuremath{\approx}'),
    ('→', r'\ensuremath{\rightarrow}'), ('−', r'\ensuremath{-}'),
    ('±', r'\ensuremath{\pm}'), ('≤', r'\ensuremath{\leq}'),
    ('ć', r"\'c"), ('Ø', r'\O{}'), ('ö', r'\"o'),
]

def esc_text(s):
    """Escape LaTeX specials in plain text (not math, not code)."""
    s = s.replace('\\', r'\textbackslash{}')
    for a,b in [('&',r'\&'),('%',r'\%'),('#',r'\#'),('_',r'\_'),
                ('{',r'\{'),('}',r'\}'),('~',r'\textasciitilde{}'),
                ('^',r'\textasciicircum{}'),('$',r'\$')]:
        s = s.replace(a,b)
    for a,b in UNI:
        s = s.replace(a,b)
    return s

def inline(s):
    """Process one line/run of inline markdown into LaTeX.
    Protect $math$ and `code`, escape the rest, apply bold/italic."""
    tokens = []
    def stash(t):
        tokens.append(t); return f'\x00{len(tokens)-1}\x00'
    # protect display-ish inline math $...$  (paper has no $ inside code)
    s = re.sub(r'\$([^$]+)\$', lambda m: stash('$'+m.group(1)+'$'), s)
    # protect inline code `...`
    s = re.sub(r'`([^`]+)`', lambda m: stash(r'\texttt{'+esc_text(m.group(1))+'}'), s)
    # bold **...**
    s = re.sub(r'\*\*([^*]+)\*\*', lambda m: '\x01B\x01'+m.group(1)+'\x01/\x01', s)
    # italic *...*  (single star, not part of ** which is gone)
    s = re.sub(r'\*([^*]+)\*', lambda m: '\x01I\x01'+m.group(1)+'\x01/\x01', s)
    # now escape text, then restore bold/italic wrappers
    s = esc_text(s)
    s = s.replace('\x01B\x01', r'\textbf{').replace('\x01I\x01', r'\emph{').replace('\x01/\x01','}')
    # restore protected tokens
    s = re.sub(r'\x00(\d+)\x00', lambda m: tokens[int(m.group(1))], s)
    return s

def convert(md):
    lines = md.split('\n')
    out = []
    i = 0
    n = len(lines)
    # skip the H1 title line (handled in preamble)
    body_start = 0
    while i < n:
        line = lines[i]

        # author / abstract HTML blocks -> skip entirely (handled in preamble).
        # These contain NESTED <div>s, so track depth until the outer div closes.
        if line.strip().startswith('<div class="authors">') or line.strip().startswith('<div class="abstract">'):
            depth = 0
            while i < n:
                depth += lines[i].count('<div') - lines[i].count('</div>')
                i += 1
                if depth <= 0:
                    break
            continue

        # horizontal rule
        if line.strip() == '---':
            i += 1; continue

        # H1 (title) - skip, in preamble
        if line.startswith('# ') and not line.startswith('## '):
            i += 1; continue

        # headers
        m = re.match(r'(#{2,4})\s+(.*)', line)
        if m:
            lvl = len(m.group(1)); title = inline(m.group(2))
            cmd = {2:r'\section*', 3:r'\subsection*', 4:r'\subsubsection*'}[lvl]
            out.append(f'{cmd}{{{title}}}')
            i += 1; continue

        # fenced block: mermaid -> emit a figure with the rendered diagram PDF
        # (using the following *Figure N:* caption); other code -> skip.
        if line.strip().startswith('```'):
            is_mermaid = 'mermaid' in line.strip().lower()
            i += 1
            while i < n and not lines[i].strip().startswith('```'):
                i += 1
            i += 1  # past closing ```
            if is_mermaid:
                j = i
                while j < n and lines[j].strip() == '':
                    j += 1
                cm = re.match(r'\*Figure (\d+):\s*(.*?)\*\s*$', lines[j].strip()) if j < n else None
                if cm:
                    fign = int(cm.group(1)); cap = cm.group(2); f = FIGFILE[fign]
                    out.append(r'\begin{figure}[!t]\centering')
                    out.append(rf'\includegraphics[width=0.9\columnwidth]{{{f}}}')
                    out.append(rf'\caption{{{inline(cap)}}}')
                    out.append(rf'\label{{fig:{fign}}}')
                    out.append(r'\end{figure}')
                    i = j + 1
            continue

        # image  ![alt](file)  possibly followed by *Figure N: caption*
        m = re.match(r'!\[.*?\]\((.+?)\)', line.strip())
        if m:
            # find following caption line
            j = i+1
            while j < n and lines[j].strip()=='':
                j += 1
            cap = ''
            fign = None
            if j < n:
                cm = re.match(r'\*Figure (\d+):\s*(.*?)\*\s*$', lines[j].strip())
                if cm:
                    fign = int(cm.group(1)); cap = cm.group(2)
            if fign is None:
                i += 1; continue
            f = FIGFILE[fign]
            width = '0.85\\columnwidth' if fign in DIAGRAMS else '\\columnwidth'
            out.append(r'\begin{figure}[!t]\centering')
            out.append(rf'\includegraphics[width={width}]{{{f}}}')
            out.append(rf'\caption{{{inline(cap)}}}')
            out.append(rf'\label{{fig:{fign}}}')
            out.append(r'\end{figure}')
            i = j+1; continue

        # table block (lines starting with |)
        if line.strip().startswith('|'):
            tbl = []
            while i < n and lines[i].strip().startswith('|'):
                tbl.append(lines[i]); i += 1
            out.append(render_table(tbl))
            continue

        # bullet list
        if re.match(r'\s*\*\s+', line):
            out.append(r'\begin{itemize}')
            while i < n and re.match(r'\s*\*\s+', lines[i]):
                item = re.sub(r'\s*\*\s+','',lines[i],count=1)
                out.append(r'\item '+inline(item))
                i += 1
            out.append(r'\end{itemize}')
            continue
        # numbered list
        if re.match(r'\s*\d+\.\s+', line):
            out.append(r'\begin{enumerate}')
            while i < n and re.match(r'\s*\d+\.\s+', lines[i]):
                item = re.sub(r'\s*\d+\.\s+','',lines[i],count=1)
                out.append(r'\item '+inline(item))
                i += 1
            out.append(r'\end{enumerate}')
            continue

        # display math $$...$$ (may be on one line or span)
        if line.strip().startswith('$$'):
            buf = line.strip()
            # collect until closing $$ if not closed
            while buf.count('$$') < 2:
                i += 1; buf += ' '+lines[i].strip()
            inner = buf.strip().strip('$').strip()
            out.append(r'\begin{equation*}'+inner+r'\end{equation*}')
            i += 1; continue

        # References section: lines like [1] ...
        m = re.match(r'\[(\d+)\]\s+(.*)', line)
        if m:
            # collect all bib entries
            bib = []
            while i < n:
                mm = re.match(r'\[(\d+)\]\s+(.*)', lines[i].strip())
                if mm:
                    bib.append((int(mm.group(1)), mm.group(2))); i += 1
                elif lines[i].strip()=='':
                    i += 1
                else:
                    break
            out.append(r'\begin{thebibliography}{99}')
            for num,txt in bib:
                out.append(rf'\bibitem{{r{num}}} '+inline(txt))
            out.append(r'\end{thebibliography}')
            continue

        # blank line
        if line.strip()=='':
            out.append('')
            i += 1; continue

        # normal paragraph line
        out.append(inline(line))
        i += 1

    return '\n'.join(out)

def render_table(rows):
    # rows: list of "| a | b |" ; second row is the |---|---| separator
    def cells(r): return [c.strip() for c in r.strip().strip('|').split('|')]
    header = cells(rows[0])
    data = [cells(r) for r in rows[2:]]
    ncol = len(header)
    colspec = '|'+'|'.join(['l']*ncol)+'|'
    out = [r'\begin{table}[!t]\centering\footnotesize', rf'\begin{{tabular}}{{{colspec}}}', r'\hline']
    out.append(' & '.join(inline(h) for h in header)+r' \\ \hline')
    for d in data:
        d = (d+['']*ncol)[:ncol]
        out.append(' & '.join(inline(c) for c in d)+r' \\ \hline')
    out += [r'\end{tabular}', r'\end{table}']
    return '\n'.join(out)

# ---- build ----
md = open(SRC, encoding='utf-8').read()
# title (first H1)
title = re.search(r'^#\s+(.*)', md, re.M).group(1)
title_tex = inline(title)
body = convert(md)

preamble = r'''\documentclass[conference]{IEEEtran}
\usepackage[T1]{fontenc}
\usepackage[utf8]{inputenc}
\usepackage{graphicx}
\usepackage{amsmath,amssymb}
\usepackage{textcomp}
\usepackage{url}
\IEEEoverridecommandlockouts
\begin{document}
\title{''' + title_tex + r'''}
\author{\IEEEauthorblockN{Melvin Jacob Sajan\IEEEauthorrefmark{1}\quad Dr.~Vyshak Sureshkumar\IEEEauthorrefmark{1}\thanks{\IEEEauthorrefmark{1}Corresponding author: Dr.~Vyshak Sureshkumar.}}
\IEEEauthorblockA{Department of Mechatronics Engineering\\ Manipal Academy of Higher Education, Dubai\\ melvinsajan20@gmail.com}}
\maketitle
'''

# abstract text (from the div)
mabs = re.search(r'<div class="abstract">.*?<em>(.*?)</em>\s*</div>', md, re.S)
abstract = inline(mabs.group(1)) if mabs else ''
abstract_block = r'\begin{abstract}'+'\n'+abstract+'\n'+r'\end{abstract}'+'\n'

full = preamble + abstract_block + body + '\n\\end{document}\n'
open(OUT,'w',encoding='utf-8').write(full)
print('wrote', OUT, '(', len(full), 'chars )')
