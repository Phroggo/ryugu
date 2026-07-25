#!/usr/bin/env python3
"""Reduce table cell font size (9pt) so multi-word cell content wraps at
word boundaries within a single (not stretched) column, instead of
breaking mid-word. Tables stay single-column width per explicit
instruction -- this fixes the word-breaking without stretching."""
import re

PATH = 'word/styles.xml'
xml = open(PATH, encoding='utf-8').read()

old = '''<w:style w:default="1" w:styleId="Table" w:type="table">
    <w:name w:val="Table" />
    <w:basedOn w:val="TableNormal" />
    <w:semiHidden />
    <w:unhideWhenUsed />
    <w:qFormat />
    <w:tblPr>'''
new = '''<w:style w:default="1" w:styleId="Table" w:type="table">
    <w:name w:val="Table" />
    <w:basedOn w:val="TableNormal" />
    <w:semiHidden />
    <w:unhideWhenUsed />
    <w:qFormat />
    <w:rPr><w:sz w:val="17" /><w:szCs w:val="17" /></w:rPr>
    <w:tblPr>'''
assert old in xml, "Table style block not found as expected"
xml = xml.replace(old, new)
open(PATH, 'w', encoding='utf-8').write(xml)
print('reduced table font to 8.5pt')
