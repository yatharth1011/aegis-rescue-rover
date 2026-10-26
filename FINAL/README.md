# A.E.G.I.S. — final files

Copies of the files the team works from. Regenerate with `~/aegis_venv/bin/python team_share/tools/make_final.py`
after changing anything; edit the sources in `to_submit/` and `team_share/`, never these copies.

| Folder | What | Use |
|---|---|---|
| `1_submission/` | Project idea deck, item list, approval form | Upload to the course Google Form |
| `2_laser_cut_acrylic/` | Lower and upper deck DXFs (red = cut, blue = engrave); underside file | LaserCAD on the 3 mm acrylic sheet. Print the underside file on paper at 100 % as the cliff-sensor template |
| `3_3d_print_K1_Max/` | Plate 1 (black PLA+), plate 2 (orange PLA) | Creality Print on the K1 Max |
| `4_guides/` | Fabrication, assembly, wiring and deployment guides (PDF); start pad to print; deck as PDF | Use in that order: fabrication, assembly (wiring at its steps 16 and 24), then deployment |
| `5_cad_model/` | Fusion archives (printable and detailed), STEP, two renders | Open the `.f3d` in Fusion; STEP for other CAD tools |

Code: rover firmware `team_share/firmware/aegis_s3/`, camera firmware `team_share/firmware/aegis_cam/`,
laptop edge brain `team_share/edge/` (deployment steps in `4_guides/AEGIS_Deployment_Guide.pdf`).
Simulation: `team_share/simulation3d/`.
