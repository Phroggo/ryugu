#!/usr/bin/env python3
"""Font and point sizes ONLY, matching the official IEEE conference-
template-A4, applied to the existing content-complete two-column build
-- no structural changes (manual "1. Introduction" numbering, "Figure
N:" captions, "[N]" references all stay exactly as they are; nothing is
re-templated or auto-numbered). This is deliberately narrow in scope
after the previous full re-template rebuild overcorrected.

Template values (measured directly from the template's own styles.xml):
  body / headings: Times New Roman, 10pt (the template's default, no
    explicit size override on Heading1/2/3 or BodyText)
  title: 24pt
  author line: 11pt
  abstract: 9pt
  figure captions, table captions, references: 8pt
"""
import re

DOC = 'word/document.xml'
STYLES = 'word/styles.xml'
TNR = '<w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman" w:cs="Times New Roman" />'

styles = open(STYLES, encoding='utf-8').read()

# 1) document-wide default font -> Times New Roman (cascades to every
# style that doesn't explicitly override rFonts, i.e. almost all of them)
styles = re.sub(
    r'<w:rFonts w:asciiTheme="minorHAnsi"[^/]*/>',
    TNR, styles, count=1)

# 2) explicit per-style overrides that need both font and size fixed
def set_style(styles, style_id, sz_half_pt, extra_rpr=''):
    m = re.search(r'(<w:style [^>]*w:styleId="' + style_id + r'"[^>]*>.*?)(<w:rPr>(.*?)</w:rPr>)?(</w:style>)', styles, re.S)
    assert m, f'{style_id} not found'
    head, rpr_full, rpr_inner, tail = m.groups()
    rpr_inner = rpr_inner or ''
    # strip any existing rFonts/sz/szCs so we don't duplicate
    rpr_inner = re.sub(r'<w:rFonts[^/]*/>', '', rpr_inner)
    rpr_inner = re.sub(r'<w:sz w:val="\d+" ?/>', '', rpr_inner)
    rpr_inner = re.sub(r'<w:szCs w:val="\d+" ?/>', '', rpr_inner)
    new_rpr = f'<w:rPr>{TNR}<w:sz w:val="{sz_half_pt}" /><w:szCs w:val="{sz_half_pt}" />{extra_rpr}{rpr_inner}</w:rPr>'
    new_block = head + new_rpr + tail
    return styles[:m.start()] + new_block + styles[m.end():]

styles = set_style(styles, 'Title', 48)          # 24pt
styles = set_style(styles, 'Author', 22)         # 11pt
styles = set_style(styles, 'Abstract', 18)        # 9pt
styles = set_style(styles, 'Caption', 16)         # 8pt (figure alt-text line + table copy fallback)
styles = set_style(styles, 'BodyText', 20)        # 10pt, explicit (was inheriting)
styles = set_style(styles, 'Heading1', 20)        # 10pt
styles = set_style(styles, 'Heading2', 20)
styles = set_style(styles, 'Heading3', 20)
styles = set_style(styles, 'Heading4', 20)

open(STYLES, 'w', encoding='utf-8').write(styles)
print('styles.xml: Times New Roman + IEEE template point sizes applied')

# 3) the VISIBLE "Figure N:"/"Table N." caption paragraphs use BodyText
# + inline italic run formatting (not the Caption style), so they need
# a direct per-paragraph size override to reach 8pt; same for the
# reference-list paragraphs -- find each by content pattern.
doc = open(DOC, encoding='utf-8').read()

def shrink_paragraph_runs(doc, pattern, sz_half_pt=16):
    n = 0
    for m in re.finditer(pattern, doc):
        p_start = doc.rfind('<w:p>', 0, m.start())
        p_end = doc.find('</w:p>', m.end()) + len('</w:p>')
        block = doc[p_start:p_end]
        # add explicit sz/szCs to every run's rPr in this paragraph
        def add_sz(rm):
            r = rm.group(0)
            if '<w:rPr>' in r:
                return r.replace('<w:rPr>', f'<w:rPr><w:sz w:val="{sz_half_pt}" /><w:szCs w:val="{sz_half_pt}" />', 1)
            return r.replace('<w:r>', f'<w:r><w:rPr><w:sz w:val="{sz_half_pt}" /><w:szCs w:val="{sz_half_pt}" /></w:rPr>', 1)
        new_block = re.sub(r'<w:r>.*?</w:r>', add_sz, block, flags=re.S)
        doc = doc[:p_start] + new_block + doc[p_end:]
        n += 1
    return doc, n

doc, n1 = shrink_paragraph_runs(doc, r'>Figure \d+: ')
doc, n2 = shrink_paragraph_runs(doc, r'>Table [IVX]+\. ')
doc, n3 = shrink_paragraph_runs(doc, r'>\[\d+\] ')
print(f'shrunk to 8pt: {n1} figure captions, {n2} table captions, {n3} references')

open(DOC, 'w', encoding='utf-8').write(doc)
print('done')
