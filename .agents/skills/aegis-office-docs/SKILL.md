---
name: aegis-office-docs
description: Create, read or edit Word (.docx), PowerPoint (.pptx) or Excel (.xlsx) files for A.E.G.I.S. - the course submission deck, approval form and item list in to_submit/, or any new office document. Use whenever a .docx/.pptx/.xlsx is involved.
---

# Office documents (docx, pptx, xlsx)

## The course submission files are special
`to_submit/ProjectIdea_AEGIS_Thursday_1.pptx`, `ApprovalForm_AEGIS_Thursday_1.docx` and `ItemList_AEGIS_Thursday_1.xlsx` follow the course templates in `course_templates/`. **The project is already approved with them.**
- They are **local only** (git-ignored; the repo is public). Never `git add` them.
- **Keep the template structure exactly.** That means the same slides, fields, columns, fonts, header colours and one-page form. Don't add blanks, sections, extra fields or "helpful" extras. Earlier the user had to undo exactly that kind of over-eager change.
- Don't edit them unless asked. If asked, regenerate them with the scripts:
  - `tools/build_submission.py` builds the deck and the approval form;
  - `tools/bom_excel.py` builds the item list.
  Don't hand-edit the binaries. Team names and roll numbers come from the git-ignored `tools/team.json` (`load_team()`).

## Libraries (in `~/aegis_venv`)
- **docx**: `python-docx`. Edit in place to keep the template's styles. Replace the text of existing runs; don't delete and recreate paragraphs. For fields the API doesn't expose, edit the XML with `docx.oxml.ns.qn`.
- **pptx**: `python-pptx`. Reuse the template's layouts and placeholders. Keep text inside its boxes: check that nothing overflows, and shorten the text rather than shrinking the font below 12 pt.
- **xlsx**: `openpyxl`. Write real formulas (`=SUM(D2:D40)`), not hard-coded totals, and keep the template's header fill, borders and column widths.

## Check what you made (always)
```bash
cd /tmp && soffice --headless --convert-to pdf ~/Documents/Fusion/project/to_submit/ApprovalForm_AEGIS_Thursday_1.docx
pdftoppm -r 70 -png ApprovalForm_AEGIS_Thursday_1.pdf form    # look at the PNGs: one page? nothing cut off?
```
For xlsx, recalculate and check the formulas (open the LibreOffice PDF, or load with `openpyxl` using `data_only=True` after a LibreOffice round-trip). Report what you checked.

## Writing in these documents
Plain, factual, specific numbers, no hype and no emojis. Don't give anyone a title or role they didn't ask for, and don't credit AI tools.
