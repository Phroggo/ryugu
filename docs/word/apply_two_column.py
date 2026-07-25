#!/usr/bin/env python3
"""Convert the single-column pandoc docx into an IEEE-style layout: title
block + abstract + nomenclature stay single-column, everything from
'1. Introduction' onward (through references) becomes two-column, via a
continuous section break -- the mechanism Word actually uses for this."""
import re

PATH = 'word/document.xml'
xml = open(PATH, encoding='utf-8').read()

PAGE = '<w:pgSz w:w="12240" w:h="15840" />'
MARGIN = ('<w:pgMar w:top="1080" w:right="1080" w:bottom="1080" '
          'w:left="1080" w:header="720" w:footer="720" w:gutter="0" />')

# 1) insert a continuous section break (1-column) right before the
#    "1. Introduction" heading's bookmarkStart -- this paragraph's sectPr
#    describes the section that ENDS here (title/abstract/nomenclature).
break_para = (
    f'<w:p><w:pPr><w:sectPr><w:type w:val="continuous" />{PAGE}{MARGIN}'
    f'<w:cols w:num="1" /></w:sectPr></w:pPr></w:p>'
)
marker = '<w:bookmarkStart w:id="20" w:name="introduction" />'
assert marker in xml, "Introduction bookmark not found -- structure changed?"
xml = xml.replace(marker, break_para + marker, 1)

# 2) the body-level trailing sectPr (self-closing) governs the LAST
#    section -- Introduction through the end (References). Make it 2-col.
final_sectpr = f'<w:sectPr>{PAGE}{MARGIN}<w:cols w:num="2" w:space="288" /></w:sectPr>'
assert '<w:sectPr />' in xml, "expected self-closing trailing sectPr"
xml = xml.replace('<w:sectPr />', final_sectpr, 1)

open(PATH, 'w', encoding='utf-8').write(xml)
print('applied two-column section break')
