# Word build pipeline

1. `python3 prep_docx_md.py` — strips the HTML author/abstract/nomenclature
   divs out of `../Research_Paper.md` and swaps mermaid fences for the
   pre-rendered diagram PNGs, writing `paper_docx.md`.
2. `pandoc paper_docx.md -o spacehopper_word.docx --from markdown+tex_math_dollars`
   — produces a valid but single-column docx (Word's native math via OMML,
   tables, images all come through, but no IEEE-style column layout).
3. Unzip the docx, then run `apply_two_column.py` against the extracted
   `word/document.xml` — inserts a continuous section break right before
   "1. Introduction" so title/authors/abstract/nomenclature stay
   single-column while everything from Introduction through References
   becomes two-column.
4. Run `apply_table_spans.py` against the same `word/document.xml` —
   Word cannot span a single table across live columns, so each of the
   4 numbered tables (and its caption) gets bracketed by a pair of
   continuous section breaks (2-col -> 1-col for the table -> 2-col
   again) so it renders full-width instead of squeezed into one ~3.25in
   column with badly broken words.
5. Re-zip (`zip -q -r -X out.docx word "[Content_Types].xml" _rels docProps`)
   into the final `spacehopper_word.docx`.

If `Research_Paper.md` changes, redo all 5 steps -- steps 3/4 operate on
the unzipped XML and must be re-applied after every fresh pandoc run.

Note: `apply_table_spans.py` asserts exactly 4 table captions
(`Table [IVX]+\.`) are found — if a table is added/removed, update that
count and re-verify visually (render to PDF with LibreOffice headless:
`libreoffice --headless --convert-to pdf spacehopper_word.docx`) before
shipping, since a bad section-break offset can silently mis-render.
