# A.E.G.I.S. — autonomous search-and-rescue rover

TL101 Tinkering Lab, IIT Ropar (AY 2026-27). Team: Yatharth Shrivastava, Prabhnoor Singh, Shreyansh, Souvik Das, Utkarsh Kumar Gautam and Shubham Maurya (School of AI & Data Engineering).

The rover **hears** a distress call, **searches** a table-top arena, **recognises** the victim with a neural network, **takes it inside** its intake bay, and **returns** by the shortest path it has already proven safe.

| Folder | What is in it |
|---|---|
| [`to_submit/`](to_submit) | Proposal deck (`.pptx`), itemised BOM (`.xlsx`), faculty approval form (`.docx`) |
| [`team_share/`](team_share) | Everything the team works from. **[`FABRICATION_GUIDE.md`](team_share/FABRICATION_GUIDE.md)**: what to cut or print from which material. **Read [`team_share/README.md`](team_share/README.md)** for the full log, results and to-do list |
| [`course_templates/`](course_templates) | Blank templates as issued by the course |

**Status:**
- Mk2 is designed in Autodesk Fusion (41 parts, zero clashes, with a wiring harness and engraved placement marks).
- The ESP32-S3 firmware builds, and its core passes 32/32 unit tests.
- In a 3D simulation (MuJoCo, a YOLO detector trained on the Mac GPU, room acoustics, lossy Wi-Fi), the full mission succeeds in **70%** of hidden-victim runs and **72%** of scream-homing runs, with 1 fall off the table in 120 missions.

Reproduce the results with `team_share/simulation3d/README.md`, then rebuild the submission files:

```bash
~/aegis_venv/bin/python team_share/tools/build_submission.py
```
