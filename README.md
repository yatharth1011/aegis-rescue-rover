# A.E.G.I.S. — autonomous search-and-rescue rover

TL101 Tinkering Lab, IIT Ropar (AY 2026-27). Team: Yatharth Shrivastava, Prabhnoor Singh, Shreyansh, Souvik Das, Utkarsh Kumar Gautam and Shubham Maurya (School of AI & Data Engineering).

The rover **hears** a distress call, **searches** a table-top arena, **recognises** the victim with a neural network, **takes it inside** its intake bay, and **returns** by the shortest path it has already proven safe.

| Folder | What is in it |
|---|---|
| **[`AEGIS_How_It_Works.pdf`](team_share/AEGIS_How_It_Works.pdf)** | **How the whole system works, in plain language** |
| **[`FINAL/`](FINAL)** | **Start here:** submission files, laser-cut DXFs, 3D-print plates, guides and the CAD model, in one place |
| [`to_submit/`](to_submit) | Proposal deck (`.pptx`), itemised BOM (`.xlsx`), faculty approval form (`.docx`) |
| [`team_share/`](team_share) | Everything the team works from. **[`AEGIS_Fabrication_Guide.pdf`](team_share/AEGIS_Fabrication_Guide.pdf)**: what to cut or print from which material. **Read [`team_share/README.md`](team_share/README.md)** for the full log, results and to-do list |
| [`course_templates/`](course_templates) | Blank templates as issued by the course |

**Status:** proposal **approved**; now building.
- Mk2 is designed in Autodesk Fusion (46 parts, zero clashes, with a wiring harness and engraved placement marks).
- All code to run the real rover is written: ESP32-S3 firmware (with Wi-Fi link and the on-robot table-edge reflex), ESP32-CAM firmware, and the laptop edge brain. Both firmwares build; the core passes 41/41 unit tests; the laptop program has been run end to end against a simulator-driven stand-in for the rover. Nothing has run on hardware yet.
- Build and deploy, in order: `AEGIS_Fabrication_Guide.pdf`, `AEGIS_Assembly_Guide.pdf`, `AEGIS_Wiring_Guide.pdf`, `AEGIS_Deployment_Guide.pdf` (all also in `FINAL/4_guides/`).
- In a 3D simulation (MuJoCo, a YOLO detector trained on the Mac GPU, room acoustics, lossy Wi-Fi), the full mission succeeds in **70%** of hidden-victim runs and **72%** of scream-homing runs, with 1 fall off the table in 120 missions.

Reproduce the results with `team_share/simulation3d/README.md`, then rebuild the submission files:

```bash
~/aegis_venv/bin/python team_share/tools/build_submission.py
```
