# Word build pipeline

`spacehopper_word.docx` is built against the **official IEEE
conference-template-A4** (`ieee_conference_template_a4.docx`, saved
alongside these scripts so the build is reproducible without depending
on wherever the template was originally downloaded from). This is the
current, primary pipeline -- an earlier custom two-column build (not
using the official template) is documented at the bottom of this file
for reference, but is no longer what `spacehopper_word.docx` is built
from.

## IEEE-template pipeline (current)

1. `python3 prep_ieee_template_md.py` — converts `../Research_Paper.md`
   into `paper_ieee.md`. This does more than the old prep script: the
   template's heading/figure/table/reference styles carry their own
   Word auto-numbering (Roman numerals for top-level sections, letters
   for subsections, "Fig. N." for captions, "TABLE N." for table heads,
   "[N]" for references), so every manual number in the source
   ("1. Introduction", "Figure 7:", "[12]", ...) is stripped here or
   auto-numbering would double up. Section heading levels are also
   shifted up by one (source `##`/`###`/`####` -> output `#`/`##`/`###`)
   to line up with pandoc's h1->"Heading 1" etc. name-based style
   mapping; the paper title itself is emitted as a **plain paragraph**,
   not a heading, since it needs the template's `papertitle` style, not
   `Heading 1` — retargeted in post-processing since it's reliably the
   first paragraph in the body. Author/affiliation/abstract/keywords/
   figure-caption/table-head/reference paragraphs are tagged with
   `%%MARKER%%` prefixes for the same reason (pandoc has no way to know
   about the template's custom style names on its own).
   The `DIAGRAM_PNG` map inside is keyed by figure number — re-verify it
   whenever figures are renumbered (grep `` ```mermaid `` and the
   `*Figure N:` line right after each), or diagrams silently fail to
   swap in.
2. `python3 add_ieee_image_widths.py` — sizes every image against the
   template's *actual* A4 column geometry (595.30pt page, 44.65pt side
   margins, 36pt gutter -> 3.264in per column, 7.028in full width),
   computed from the template's own measurements, not guessed. One
   genuinely wide figure (`FULL_WIDTH` set, currently just the
   dual-panel hop-trajectory plot) gets the full page width; everything
   else fits one column, height-capped so extreme-aspect images (the
   portrait NASA reaction-wheel photo) don't overflow.
3. `pandoc paper_ieee.md -o spacehopper_ieee_fresh.docx --from markdown+tex_math_dollars --reference-doc=ieee_conference_template_a4.docx --resource-path=.:..`
   — pandoc copies style/numbering/theme *definitions* from the
   reference doc and maps markdown headings to same-named styles by
   name (`h1` -> a style literally named "heading 1", which the
   template's `Heading1` style is — confirmed this auto-numbers
   correctly). It does **not** carry over the template's own
   single-column/two-column section-break structure (it generates a
   fresh document body from the markdown), and has no way to target the
   template's custom style names (`papertitle`, `Author`, `Affiliation`,
   `Abstract`, `Keywords`, `figurecaption`, `tablehead`, `references`,
   `equation`, `tablecolhead`, `tablecopy`) — both handled in
   post-processing, step 5.
4. Unzip **directly into this directory**: `unzip -q -o
   spacehopper_ieee_fresh.docx -d ieee_root` then `cd ieee_root` for the
   remaining steps — `docs/word/ieee_root/word/document.xml` must be
   sibling to `docs/word/ieee_root/[Content_Types].xml` and
   `docs/word/ieee_root/_rels/`, or the re-zip in step 6 silently drops
   required parts and the docx won't open.
5. `python3 ../apply_ieee_template_styles.py` (run from inside
   `ieee_root/`) does everything pandoc's reference-doc mapping can't:
   - retargets the first paragraph to `papertitle`
   - resolves every `%%MARKER%%` paragraph to its real style, stripping
     the marker text
   - inserts the actual single-column (title/author) -> two-column
     (abstract through references) section break, at the template's
     real 18pt gutter — the template has this break built in originally
     but pandoc doesn't preserve it, so it's rebuilt here from the
     template's own measured section properties, not guessed
   - drops pandoc's redundant `ImageCaption` alt-text paragraphs (would
     otherwise show a duplicate caption-like line above the real one)
   - remaps pandoc's own default styles (`Compact` -> `bulletlist`,
     `FirstParagraph` -> `BodyText`) since those pandoc-internal names
     aren't in the template and would silently fall back to unstyled
     text
   - styles display equations (paragraphs containing `<m:oMathPara>`,
     which distinguishes them from inline math) with the `equation`
     style
   - styles table cells (first row -> `tablecolhead`, rest ->
     `tablecopy`)
   - full-width-spans every table and the one wide figure, same
     continuous-section-break sandwich as the two-column build below
   - adds `keepNext` to table captions and every image paragraph so a
     caption doesn't get orphaned from its table/figure across a page
     break (this **did** happen before the fix — a table caption landed
     alone at the bottom of one page with the table starting fresh on
     the next)
6. Re-zip from inside `ieee_root/`: `zip -q -r -X ../spacehopper_word.docx . -x '.*'`.

**Known source-formatting gotcha**: the reference list in
`Research_Paper.md` has no blank line between consecutive `[N] ...`
entries (natural for a tightly-packed reference list), but markdown
needs a blank line to treat consecutive lines as separate paragraphs —
without one, pandoc merges the *entire* reference list into a single
run-on paragraph with only the first entry's auto-number rendering.
`prep_ieee_template_md.py` inserts a blank line after each `%%REF%%`
line specifically to avoid this; if references ever stop rendering
individually, this is the first thing to check.

Always re-verify visually after a rebuild: render to PDF with
LibreOffice headless (`soffice --headless --convert-to pdf
spacehopper_word.docx`) and check a handful of pages, not just that
each script ran without asserting — a bad section-break offset, a stale
`DIAGRAM_PNG` entry, a missing blank line, or an image sized for the
wrong column width can all silently mis-render without raising an
error.

---

## Earlier custom two-column pipeline (superseded, kept for reference)

This produced a two-column layout *not* based on the official IEEE
template — superseded by the pipeline above once the actual template
was available, but the section-break and full-width-span mechanisms it
introduced are what the current pipeline's `apply_ieee_template_styles.py`
reuses.

1. `python3 prep_docx_md.py` — strips the HTML author/abstract/nomenclature
   divs out of `../Research_Paper.md` and swaps mermaid fences for the
   pre-rendered diagram PNGs, writing `paper_docx.md`.
2. `python3 add_image_widths.py` — sizes images to a generic ~3.3in
   column (Letter-based estimate, not the template's real A4 geometry).
3. `pandoc paper_docx.md -o spacehopper_word.docx --from markdown+tex_math_dollars --resource-path=.:..`
4. Unzip directly into `docs/word/` (same caveat as step 4 above).
5. `python3 apply_two_column.py` — continuous section break before
   "1. Introduction" (title/authors/abstract/nomenclature single-column,
   everything after two-column).
6. `python3 apply_table_spans.py` — full-width table spans.
7. `python3 apply_figure_spans.py` — full-width span for wide figures.
8. `python3 apply_compact_typography.py` — 10pt body text, tightened
   spacing, 8pt table text.
9. Re-zip into `spacehopper_word.docx`.
