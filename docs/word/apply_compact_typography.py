#!/usr/bin/env python3
"""Tighten typography to LaTeX-IEEEtran density: the user's first-draft
LaTeX PDF (14 pages) vs this Word rebuild (35 pages, same content plus
legitimate additions from review rounds) showed the gap is mostly
Word's default 12pt body font + generous 9pt paragraph spacing, not
extra content. Drop body text to 10pt with minimal paragraph gaps,
scale headings/title down proportionally, and shrink table text --
matching IEEEtran's actual compactness rather than Word's defaults."""
import re

PATH = 'word/styles.xml'
xml = open(PATH, encoding='utf-8').read()

# 1) document default font: 12pt -> 10pt
xml = xml.replace('<w:sz w:val="24" />\n        <w:szCs w:val="24" />',
                   '<w:sz w:val="20" />\n        <w:szCs w:val="20" />')

def resize_style(xml, style_id, old_sz_vals, new_sz, old_spacing=None, new_spacing=None):
    m = re.search(r'(<w:style [^>]*w:styleId="' + style_id + r'"[^>]*>.*?</w:style>)', xml, re.S)
    assert m, f'{style_id} not found'
    block = m.group(1)
    new_block = block
    for old in old_sz_vals:
        new_block = new_block.replace(f'<w:sz w:val="{old}" />', f'<w:sz w:val="{new_sz}" />')
        new_block = new_block.replace(f'<w:szCs w:val="{old}" />', f'<w:szCs w:val="{new_sz}" />')
    if old_spacing and new_spacing:
        new_block = new_block.replace(old_spacing, new_spacing)
    assert new_block != block or old_spacing, f'{style_id}: no change applied'
    return xml.replace(block, new_block)

xml = resize_style(xml, 'BodyText', [], None,
                    '<w:spacing w:after="180" w:before="180" />',
                    '<w:spacing w:after="60" w:before="60" />')
xml = resize_style(xml, 'Heading1', ['32'], '26',
                    '<w:spacing w:after="0" w:before="480" />',
                    '<w:spacing w:after="0" w:before="200" />')
xml = resize_style(xml, 'Heading2', ['28'], '22',
                    '<w:spacing w:after="0" w:before="200" />',
                    '<w:spacing w:after="0" w:before="120" />')
xml = resize_style(xml, 'Heading3', ['24'], '22',
                    '<w:spacing w:after="0" w:before="200" />',
                    '<w:spacing w:after="0" w:before="100" />')
xml = resize_style(xml, 'Title', ['36'], '32',
                    '<w:spacing w:after="240" w:before="480" />',
                    '<w:spacing w:after="160" w:before="240" />')
xml = resize_style(xml, 'Abstract', ['20'], '20',
                    '<w:spacing w:after="300" w:before="100" />',
                    '<w:spacing w:after="150" w:before="60" />')

# table text: 10pt (docDefault) -> 8pt, matching a real IEEEtran table's
# small font, and give the Table style itself explicit compact spacing
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
    <w:rPr><w:sz w:val="16" /><w:szCs w:val="16" /></w:rPr>
    <w:tblPr>'''
assert old in xml, "Table style block not found as expected"
xml = xml.replace(old, new)

open(PATH, 'w', encoding='utf-8').write(xml)
print('compact typography applied')
