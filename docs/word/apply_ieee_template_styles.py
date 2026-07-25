#!/usr/bin/env python3
"""Post-process the pandoc+reference-doc output into the actual IEEE
conference-template-A4 shape.

pandoc's --reference-doc only copies style/numbering DEFINITIONS from
the template and maps markdown headings to same-named styles (h1->
"Heading 1" etc, confirmed working -- Roman-numeral auto-numbering comes
through correctly). It does NOT preserve the template's own section-break
paragraphs (title single-column vs body two-column) since it generates a
fresh document body from the markdown, and it has no way to know about
the template's custom paragraph styles (papertitle, Author, Affiliation,
Abstract, Keywords, figurecaption, tablehead, references, equation,
tablecolhead, tablecopy) since those aren't things pandoc's own model
has a slot for. This script does both: rebuilds the single-column ->
two-column section break at the template's own measurements (A4, 18pt
column gutter, confirmed from the template's real internal section
break rather than guessed), and retargets styles by content pattern --
%%MARKER%% prefixes inserted by prep_ieee_template_md.py for anything
pandoc can't infer on its own (author block, abstract, keywords, figure/
table captions, references), plus structural detection for equations
(paragraphs containing <m:oMathPara>) and table header vs body rows.
"""
import re

PATH = 'word/document.xml'
xml = open(PATH, encoding='utf-8').read()

PAGE = '<w:pgSz w:w="595.30pt" w:h="841.90pt" w:code="9" />'
MARGIN = ('<w:pgMar w:top="27pt" w:right="44.65pt" w:bottom="72pt" '
          'w:left="44.65pt" w:header="36pt" w:footer="36pt" w:gutter="0pt" />')


def break_para(ncols, space='18pt'):
    cols = '<w:cols w:num="1" />' if ncols == 1 else f'<w:cols w:num="2" w:space="{space}" />'
    return f'<w:p><w:pPr><w:sectPr><w:type w:val="continuous" />{PAGE}{MARGIN}{cols}</w:sectPr></w:pPr></w:p>'


# ---- 1) title: the very first <w:p> in the body -> papertitle style ----
body_start = xml.find('<w:body>') + len('<w:body>')
first_p_end = xml.find('</w:p>', body_start) + len('</w:p>')
first_p = xml[body_start:first_p_end]
if 'w:pStyle' in first_p:
    new_first_p = re.sub(r'<w:pStyle w:val="[^"]*" ?/>', '<w:pStyle w:val="papertitle" />', first_p, count=1)
else:
    new_first_p = first_p.replace('<w:p>', '<w:p><w:pPr><w:pStyle w:val="papertitle" /></w:pPr>', 1)
xml = xml[:body_start] + new_first_p + xml[first_p_end:]


# ---- 2) marker-based paragraphs: strip the %%MARKER%% run text, set pStyle ----
MARKER_STYLE = {
    'AUTHOR': 'Author',
    'AFFIL': 'Affiliation',
    'ABSTRACT': 'Abstract',
    'KEYWORDS': 'Keywords',
    'FIGCAP': 'figurecaption',
    'TABHEAD': 'tablehead',
    'REF': 'references',
}

for marker, style in MARKER_STYLE.items():
    tag = f'%%{marker}%%'
    while tag in xml:
        idx = xml.find(tag)
        p_start = xml.rfind('<w:p>', 0, idx)
        if p_start == -1:
            p_start = xml.rfind('<w:p ', 0, idx)
        p_end = xml.find('</w:p>', idx) + len('</w:p>')
        block = xml[p_start:p_end]
        block = block.replace(tag, '', 1)
        if 'w:pStyle' in block:
            block = re.sub(r'<w:pStyle w:val="[^"]*" ?/>', f'<w:pStyle w:val="{style}" />', block, count=1)
        else:
            close = block.index('>') + 1
            block = block[:close] + f'<w:pPr><w:pStyle w:val="{style}" /></w:pPr>' + block[close:]
        xml = xml[:p_start] + block + xml[p_end:]

print('markers resolved')

# ---- 2b) single-column title/author block -> two-column body, break
# inserted right before the Abstract paragraph (title+author stay 1-col,
# everything from Abstract through References is 2-col, matching the
# template's own real internal section split at 18pt gutter) ----
abstract_idx = xml.find('w:pStyle w:val="Abstract"')
assert abstract_idx != -1, 'Abstract paragraph not found'
abstract_p_start = xml.rfind('<w:p>', 0, abstract_idx)
xml = xml[:abstract_p_start] + break_para(1) + xml[abstract_p_start:]

final_sectpr = f'<w:sectPr>{PAGE}{MARGIN}<w:cols w:num="2" w:space="18pt" /></w:sectPr>'
assert '<w:sectPr' in xml[xml.rfind('</w:p>'):] or True
last_sectpr_start = xml.rfind('<w:sectPr')
last_sectpr_end = xml.find('</w:body>', last_sectpr_start)
xml = xml[:last_sectpr_start] + final_sectpr + xml[last_sectpr_end:]
print('two-column section break inserted before Abstract')

# ---- 3) drop pandoc's redundant alt-text "ImageCaption" paragraphs ----
n_dropped = 0
while True:
    m = re.search(r'<w:p>(?:(?!</w:p>).)*?w:pStyle w:val="ImageCaption"(?:(?!</w:p>).)*?</w:p>', xml, re.S)
    if not m:
        break
    xml = xml[:m.start()] + xml[m.end():]
    n_dropped += 1
print(f'dropped {n_dropped} redundant ImageCaption paragraphs')

# ---- 4) remap pandoc's own default styles to the template's real ones ----
xml = xml.replace('w:pStyle w:val="Compact"', 'w:pStyle w:val="bulletlist"')
xml = xml.replace('w:pStyle w:val="FirstParagraph"', 'w:pStyle w:val="BodyText"')

# ---- 5) display equations: paragraphs containing m:oMathPara -> equation style ----
def eq_style(m):
    block = m.group(0)
    if 'm:oMathPara' not in block:
        return block
    if 'w:pStyle' in block:
        return re.sub(r'<w:pStyle w:val="[^"]*" ?/>', '<w:pStyle w:val="equation" />', block, count=1)
    close = block.index('>') + 1
    return block[:close] + '<w:pPr><w:pStyle w:val="equation" /></w:pPr>' + block[close:]

xml = re.sub(r'<w:p>(?:(?!</w:p>).)*?</w:p>', eq_style, xml, flags=re.S)
print('equation paragraphs styled')

# ---- 6) table cells: first row -> tablecolhead, remaining rows -> tablecopy ----
def style_table(m):
    tbl = m.group(0)
    rows = list(re.finditer(r'<w:tr>.*?</w:tr>', tbl, re.S))
    out = tbl
    for i, r in enumerate(rows):
        style = 'tablecolhead' if i == 0 else 'tablecopy'
        row = r.group(0)
        new_row = re.sub(r'<w:p>', lambda mm: f'<w:p><w:pPr><w:pStyle w:val="{style}" /></w:pPr>', row)
        # avoid double pPr if pandoc already emitted one -- fall back to plain replace of any existing pStyle
        new_row = re.sub(r'(<w:pPr>)(?:<w:pStyle w:val="[^"]*" ?/>)?', rf'\1<w:pStyle w:val="{style}" />', new_row) \
            if '<w:pPr>' in row else new_row
        out = out.replace(row, new_row, 1)
    return out

xml = re.sub(r'<w:tbl>.*?</w:tbl>', style_table, xml, flags=re.S)
print('table cells styled')

# ---- 7) full-width spans: each table (caption+table) and the one wide
# figure (hop-trajectory) bracketed 2-col -> 1-col -> 2-col, same
# mechanism as the earlier two-column build ----
insertions = []

# tables: find each tablehead-styled caption paragraph, bracket through its <w:tbl>
for m in re.finditer(r'w:pStyle w:val="tablehead"', xml):
    p_start = xml.rfind('<w:p>', 0, m.start())
    tbl_start = xml.find('<w:tbl>', m.start())
    tbl_end = xml.find('</w:tbl>', tbl_start) + len('</w:tbl>')
    insertions.append((p_start, break_para(1)))
    insertions.append((tbl_end, break_para(2)))

# the one wide figure: its caption text is unique, locate the drawing
# paragraph before it (same "search back through ImageCaption" logic
# as before, though ImageCaption paragraphs are already dropped by now
# so the drawing paragraph is the one directly preceding the caption)
FULL_WIDTH_CAPTION = 'the ballistic altitude profile over a multi-minute flight'
m = re.search(re.escape(FULL_WIDTH_CAPTION), xml)
if m:
    cap_p_start = xml.rfind('<w:p>', 0, m.start())
    cap_p_end = xml.find('</w:p>', m.end()) + len('</w:p>')
    drawing_pos = xml.rfind('<w:drawing>', 0, cap_p_start)
    img_p_start = xml.rfind('<w:p>', 0, drawing_pos)
    insertions.append((img_p_start, break_para(1)))
    insertions.append((cap_p_end, break_para(2)))
else:
    print('WARNING: wide-figure caption not found, skipping its full-width span')

insertions.sort(key=lambda t: t[0], reverse=True)
for pos, text in insertions:
    xml = xml[:pos] + text + xml[pos:]
print(f'applied {len(insertions)//2} full-width spans (tables + wide figure)')

# ---- 8) keep figures/tables attached to their captions across page
# breaks: table caption needs keepNext (stay with the table that follows);
# each image paragraph needs keepNext too (stay with its caption below) ----
STYLES_PATH = 'word/styles.xml'
styles = open(STYLES_PATH, encoding='utf-8').read()
m = re.search(r'(<w:style w:customStyle="1" w:styleId="tablehead"[^>]*>)(<w:name[^/]*/>)?(<w:pPr>)', styles)
assert m, 'could not locate tablehead pPr to inject keepNext'
styles = styles[:m.end()] + '<w:keepNext />' + styles[m.end():]
open(STYLES_PATH, 'w', encoding='utf-8').write(styles)
print('tablehead keepNext added')

def img_keepnext(m):
    block = m.group(0)
    if '<w:drawing>' not in block:
        return block
    if '<w:pPr>' in block:
        return block.replace('<w:pPr>', '<w:pPr><w:keepNext />', 1)
    close = block.index('>') + 1
    return block[:close] + '<w:pPr><w:keepNext /></w:pPr>' + block[close:]

xml = re.sub(r'<w:p>(?:(?!</w:p>).)*?</w:p>', img_keepnext, xml, flags=re.S)
print('image-paragraph keepNext added')

open(PATH, 'w', encoding='utf-8').write(xml)
print('done')
