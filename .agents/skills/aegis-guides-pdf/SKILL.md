---
name: aegis-guides-pdf
description: Create, edit or regenerate the A.E.G.I.S. PDF guides (How It Works, Fabrication, Assembly, Wiring, Deployment), their figures, or the FINAL folder; also any request to read, check, merge or produce a PDF in this project. Use whenever the user asks for a guide, document or explanation; they want formatted PDFs, not Markdown.
---

# PDF guides

The user prefers **well-formatted PDF guides**. Markdown is only the source. Never hand them a `.md` file as the deliverable.

## Pipeline
| PDF (in `team_share/`) | Source | Builder |
|---|---|---|
| `AEGIS_How_It_Works.pdf` | `HOW_AEGIS_WORKS.md` | `tools/make_guides_pdf.py` |
| `AEGIS_Fabrication_Guide.pdf` | `FABRICATION_GUIDE.md` | `tools/make_guides_pdf.py` |
| `AEGIS_Deployment_Guide.pdf` | `edge/README.md` | `tools/make_guides_pdf.py` |
| `AEGIS_Assembly_Guide.pdf` | step list inside `tools/assembly_guide.py` + STEP model | `tools/assembly_guide.py` (MuJoCo renders, slow) |
| `AEGIS_Wiring_Guide.pdf` | `tools/wiring_steps.py` (W1-W15), pins from `firmware/aegis_s3/main/aegis_config.h` | `tools/wiring_steps.py` |
| `edge/AEGIS_start_pad.pdf` | `edge/make_start_pad.py` | same |

```bash
cd ~/Documents/Fusion/project
~/aegis_venv/bin/python team_share/tools/make_guides_pdf.py
~/aegis_venv/bin/python team_share/tools/make_final.py        # refresh the FINAL/ copies afterwards
```
`make_guides_pdf.py` converts Markdown → HTML (python-markdown with tables and fenced_code), then HTML → PDF with headless Google Chrome (`--print-to-pdf`). Pages are styled by CSS `@page` rules with running headers. In a source:
- `<!-- fig:NAME -->` inserts a figure drawn by a `fig_NAME()` function in the builder (matplotlib).
- `<!-- img:relative/path.png|Caption -->` inserts an image.

To add a figure, write a `fig_xxx(path)` in the same style as the existing ones (matplotlib, navy/orange palette, `dpi=200`, tight bbox) and register it in `FIGS`.

## Quality bar (the user has rejected guides for these before)
- **Images must be sharp and clearly show the part.** Assembly renders are 2x supersampled. Outlines come from mesh feature edges, so there are no broken or dashed artefact lines and no stray lines widening the crop. The new parts in a step are blue and everything else grey. The camera angle must show the part being added; if it's hidden, change the view for that step.
- **One action per step**, numbered, as simple as a toy-kit manual, in a professional tone. Each step lists exactly the parts and fasteners it uses: M3x6 screws, nuts and pillars from the kit only.
- Wiring is tied to the assembly order: wiring steps happen at assembly steps 16 and 24. Every guide must say clearly *when* to do what.
- No layout breaks: no figure split across pages, no orphaned headings, no table wider than the page. Check this every time.

## Checking a PDF (do this before saying it's done)
```bash
pdftotext -layout team_share/AEGIS_Deployment_Guide.pdf - | less        # text, numbers, order
pdftoppm -r 70 -png -f 3 -l 3 team_share/AEGIS_Assembly_Guide.pdf /tmp/p  # render page 3, then open /tmp/p-3.png and look
```
Look at every page you changed as an image, not just the text.

## Other PDF work
- Read or extract: `pdftotext -layout`; tables: `pdfplumber` (in `~/aegis_venv`).
- Merge, split or rotate: `pypdf` (`PdfReader`, `PdfWriter`).
- Never edit a PDF directly to change content. Change the source and rebuild.
