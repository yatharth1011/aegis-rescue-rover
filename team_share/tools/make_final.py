"""
Collect the files the team works from into project/FINAL/ (copies; rerun after regenerating anything).

    ~/aegis_venv/bin/python team_share/tools/make_final.py
"""
import os
import shutil

PROJ = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
TS = os.path.join(PROJ, "team_share")
OUT = os.path.join(PROJ, "FINAL")
B = os.path.join(TS, "Build_AEGIS_Rover")

# the team's own save of the detailed model is preferred while it is newer than the script export
_SAVED = os.path.join(PROJ, "The Most Final AEGIS Body Detailed.f3d")
_EXPORT = os.path.join(B, "output_detailed", "AEGIS_Rover_Detailed.f3d")
DETAILED_F3D = ((_SAVED, "AEGIS_Rover_Detailed.f3d")
                if os.path.exists(_SAVED) and os.path.getmtime(_SAVED) > os.path.getmtime(_EXPORT) else _EXPORT)

FILES = {
    "1_submission": [os.path.join(PROJ, "to_submit", f) for f in
                     ("ProjectIdea_AEGIS_Thursday_1.pptx", "ItemList_AEGIS_Thursday_1.xlsx", "ApprovalForm_AEGIS_Thursday_1.docx")],
    "2_laser_cut_acrylic": [os.path.join(B, "output", f) for f in
                            ("lasercad_lower_deck.dxf", "lasercad_upper_deck.dxf", "lasercad_lower_deck_underside.dxf")],
    "3_3d_print_K1_Max": [os.path.join(TS, "print", f) for f in
                          ("AEGIS_plate1_black_PLA+_rover_parts.3mf", "AEGIS_plate2_orange_PLA_demo_props.3mf")],
    "4_guides": [os.path.join(TS, f) for f in ("FABRICATION_GUIDE.md", "AEGIS_Assembly_Guide.pdf", "AEGIS_Wiring_Guide.pdf",
                                                "AEGIS_Proposal_Deck.pdf")] +
                [(os.path.join(TS, "edge", "README.md"), "DEPLOYMENT_GUIDE.md"), os.path.join(TS, "edge", "AEGIS_start_pad.pdf")],
    "5_cad_model": [os.path.join(B, "output", "AEGIS_Rover.f3d"), os.path.join(B, "output", "AEGIS_Rover.step"),
                    DETAILED_F3D,
                    os.path.join(B, "output", "render_iso_door_open.png"),
                    os.path.join(B, "output_detailed", "render_wiring_lower_deck.png")],
}

README = """# A.E.G.I.S. — final files

Copies of the files the team works from. Regenerate with `~/aegis_venv/bin/python team_share/tools/make_final.py`
after changing anything; edit the sources in `to_submit/` and `team_share/`, never these copies.

| Folder | What | Use |
|---|---|---|
| `1_submission/` | Project idea deck, item list, approval form | Upload to the course Google Form |
| `2_laser_cut_acrylic/` | Lower and upper deck DXFs (red = cut, blue = engrave); underside file | LaserCAD on the 3 mm acrylic sheet. Print the underside file on paper at 100 % as the cliff-sensor template |
| `3_3d_print_K1_Max/` | Plate 1 (black PLA+), plate 2 (orange PLA) | Creality Print on the K1 Max |
| `4_guides/` | Fabrication, assembly and wiring guides; deployment guide (`DEPLOYMENT_GUIDE.md`); start pad to print; deck as PDF | Order of use: fabrication, assembly, wiring, then deployment |
| `5_cad_model/` | Fusion archives (printable and detailed), STEP, two renders | Open the `.f3d` in Fusion; STEP for other CAD tools |

Code: rover firmware `team_share/firmware/aegis_s3/`, camera firmware `team_share/firmware/aegis_cam/`,
laptop edge brain `team_share/edge/` (deployment steps in `4_guides/DEPLOYMENT_GUIDE.md`).
Simulation: `team_share/simulation3d/`.
"""


def main():
    if os.path.isdir(OUT):
        shutil.rmtree(OUT)
    for sub, files in FILES.items():
        os.makedirs(os.path.join(OUT, sub))
        for f in files:
            src, name = f if isinstance(f, tuple) else (f, os.path.basename(f))
            shutil.copy2(src, os.path.join(OUT, sub, name))
    with open(os.path.join(OUT, "README.md"), "w") as f:
        f.write(README)
    for sub in FILES:
        print(sub, sorted(os.listdir(os.path.join(OUT, sub))))


if __name__ == "__main__":
    main()
