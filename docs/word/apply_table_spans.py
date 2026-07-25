#!/usr/bin/env python3
"""Make each of the 5 tables (and its caption paragraph) span the full
page width, like a LaTeX table* -- by bracketing each table+caption with
a continuous section break to 1-column, then back to 2-column
afterward. Word cannot span a single table across live columns, so this
is the only way to get a full-width table inside an otherwise
two-column IEEE-style layout."""
import re

PATH = 'word/document.xml'
xml = open(PATH, encoding='utf-8').read()

PAGE = '<w:pgSz w:w="12240" w:h="15840" />'
MARGIN = ('<w:pgMar w:top="1080" w:right="1080" w:bottom="1080" '
          'w:left="1080" w:header="720" w:footer="720" w:gutter="0" />')

def break_para(ncols):
    cols = '<w:cols w:num="1" />' if ncols == 1 else '<w:cols w:num="2" w:space="288" />'
    return f'<w:p><w:pPr><w:sectPr><w:type w:val="continuous" />{PAGE}{MARGIN}{cols}</w:sectPr></w:pPr></w:p>'

TO_1COL = break_para(1)
TO_2COL = break_para(2)

# Find each caption occurrence, then locate the <w:p> that contains it and
# the very next </w:tbl> after it. Process in REVERSE so earlier offsets
# stay valid while we insert text.
captions = list(re.finditer(r'Table [IVX]+\.', xml))
assert len(captions) == 5, f"expected 5 table captions, found {len(captions)}"

insertions = []  # (position, text) to splice in, applied in reverse order
for m in captions:
    # caption paragraph start: nearest preceding <w:p> before the match
    p_start = xml.rfind('<w:p>', 0, m.start())
    assert p_start != -1
    # table end: nearest following </w:tbl> after the match
    tbl_end = xml.find('</w:tbl>', m.end())
    assert tbl_end != -1
    tbl_end += len('</w:tbl>')
    # a break-paragraph's sectPr describes the section ENDING there, not
    # the one starting after it -- so the para before the caption closes
    # out the preceding 2-col section, and the para after the table
    # closes out the 1-col caption+table section (letting whatever
    # follows inherit 2-col from the next break or the final body sectPr).
    insertions.append((tbl_end, TO_1COL))
    insertions.append((p_start, TO_2COL))

insertions.sort(key=lambda t: t[0], reverse=True)
for pos, text in insertions:
    xml = xml[:pos] + text + xml[pos:]

open(PATH, 'w', encoding='utf-8').write(xml)
print(f'applied {len(captions)} full-width table spans')
