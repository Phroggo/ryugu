#!/usr/bin/env python3
"""Make the one genuinely wide figure span both columns, like a LaTeX
figure*.

Same continuous-section-break sandwich apply_table_spans.py uses for
tables: 2-col -> 1-col around the image paragraph and its caption -> back
to 2-col. The architecture diagram and state-machine diagram (originally
also full-width candidates at 4.4:1 / 5.6:1) were redesigned vertical
earlier this session and now fit a single column fine (0.86:1 / 0.69:1) --
only the measured hop-trajectory plot (a genuine side-by-side dual panel,
2.2:1) still needs full width.
"""
import re

PATH = 'word/document.xml'
FIGURES = ('Figure 17:',)

xml = open(PATH, encoding='utf-8').read()

PAGE = '<w:pgSz w:w="12240" w:h="15840" />'
MARGIN = ('<w:pgMar w:top="1080" w:right="1080" w:bottom="1080" '
          'w:left="1080" w:header="720" w:footer="720" w:gutter="0" />')


def break_para(ncols):
    cols = '<w:cols w:num="1" />' if ncols == 1 else '<w:cols w:num="2" w:space="288" />'
    return (f'<w:p><w:pPr><w:sectPr><w:type w:val="continuous" />'
            f'{PAGE}{MARGIN}{cols}</w:sectPr></w:pPr></w:p>')


TO_1COL, TO_2COL = break_para(1), break_para(2)

insertions = []
for label in FIGURES:
    m = re.search(re.escape(label), xml)
    assert m, f'caption {label!r} not found -- figures renumbered?'
    # caption paragraph, and the image paragraph before it -- pandoc
    # inserts an intermediate "ImageCaption" alt-text paragraph between
    # the image and our own italic "Figure N:" caption, so search back
    # for the nearest paragraph that actually contains the drawing
    # rather than assuming strict adjacency.
    cap_start = xml.rfind('<w:p>', 0, m.start())
    cap_end = xml.find('</w:p>', m.end()) + len('</w:p>')
    drawing_pos = xml.rfind('<w:drawing>', 0, cap_start)
    assert drawing_pos != -1, f'no preceding image found for {label}'
    img_start = xml.rfind('<w:p>', 0, drawing_pos)
    assert img_start != -1
    insertions.append((cap_end, TO_1COL))
    insertions.append((img_start, TO_2COL))

insertions.sort(key=lambda t: t[0], reverse=True)
for pos, text in insertions:
    xml = xml[:pos] + text + xml[pos:]

open(PATH, 'w', encoding='utf-8').write(xml)
print(f'applied {len(FIGURES)} full-width figure spans')