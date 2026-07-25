# Word build pipeline

1. `python3 prep_docx_md.py` — strips the HTML author/abstract/nomenclature
   divs out of `../Research_Paper.md` and swaps mermaid fences for the
   pre-rendered diagram PNGs, writing `paper_docx.md`. The `DIAGRAM_PNG`
   map inside is keyed by figure number — re-verify it (grep
   `` ```mermaid `` and the `*Figure N:` line right after each) whenever
   figures are renumbered, or diagrams silently fail to swap in.
2. `python3 add_image_widths.py` — gives every image in `paper_docx.md`
   an explicit `{width=...in}` attribute sized to fit one ~3.3in column
   (capped by height too, so tall portrait images don't run off the
   page), except the images listed in `FULL_WIDTH` which get the full
   ~6.9in page width instead. Without this, pandoc sizes images from
   pixel dimensions/DPI, which overflows a column badly.
3. `pandoc paper_docx.md -o spacehopper_word.docx --from markdown+tex_math_dollars --resource-path=.:..`
   — produces a valid but single-column docx (Word's native math via OMML,
   tables, images all come through, but no IEEE-style column layout).
   `--resource-path=.:..` is required: most source images live in `../`
   (the parent `docs/` dir), only the `dN-1.png` diagram renders live
   alongside the scripts here.
4. Unzip the docx **directly into this directory** (`unzip -q -o
   spacehopper_word.docx -d .`, NOT into a `word/` subfolder — the docx's
   own internal `word/` folder must land at `docs/word/word/`, sibling to
   `docs/word/[Content_Types].xml` and `docs/word/_rels/`, or the
   re-zip in step 8 silently drops required parts and the docx won't open).
5. Run `apply_two_column.py` against the extracted `word/document.xml` —
   inserts a continuous section break right before "1. Introduction" so
   title/authors/abstract/nomenclature stay single-column while everything
   from Introduction through References becomes two-column.
6. Run `apply_table_spans.py` — Word cannot span a single table across
   live columns, so each numbered table (and its caption) gets bracketed
   by a pair of continuous section breaks (2-col -> 1-col for the table ->
   2-col again) so it renders full-width instead of squeezed into one
   ~3.25in column with badly broken words.
7. Run `apply_figure_spans.py` — same section-break trick as tables, for
   the `FIGURES` tuple inside (currently just the dual-panel hop-trajectory
   plot; other historically-wide figures were redesigned vertical instead
   and now fit one column). Note pandoc inserts an intermediate
   `ImageCaption`-styled alt-text paragraph between the image and our own
   italic `Figure N:` caption paragraph — the script searches back through
   that for the actual `<w:drawing>`, don't assume strict adjacency if you
   touch this script.
8. Run `apply_compact_typography.py` against `word/styles.xml` — Word's
   defaults (12pt body, 9pt paragraph spacing) run noticeably longer than
   the same content set in LaTeX/IEEEtran; this drops body text to 10pt,
   tightens paragraph/heading spacing, and shrinks table text to 8pt to
   close most of that gap without touching content.
9. Re-zip (`zip -q -r -X out.docx word "[Content_Types].xml" _rels docProps`)
   into the final `spacehopper_word.docx`.

If `Research_Paper.md` changes, redo all 9 steps — steps 5-8 operate on
the unzipped XML and must be re-applied after every fresh pandoc run.

Notes:
- `apply_table_spans.py` asserts an exact table-caption count
  (`Table [IVX]+\.`) and `apply_figure_spans.py` asserts every entry in
  its `FIGURES` tuple is found — if tables/figures are added, removed, or
  renumbered, update both before running.
- Always re-verify visually after a rebuild: render to PDF with
  LibreOffice headless (`libreoffice --headless --convert-to pdf
  spacehopper_word.docx`) and check a handful of pages, not just that the
  script ran without asserting — a bad section-break offset, a stale
  `DIAGRAM_PNG` entry, or an image sized for the wrong column width can
  all silently mis-render without raising an error.
