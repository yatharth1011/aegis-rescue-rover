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

FILES = {
    "1_submission": [os.path.join(PROJ, "to_submit", f) for f in
                     ("ProjectIdea_AEGIS_Thursday_1.pptx", "ItemList_AEGIS_Thursday_1.xlsx", "ApprovalForm_AEGIS_Thursday_1.docx")],
    "2_laser_cut_acrylic": [os.path.join(B, "output", f) for f in
                            ("lasercad_lower_deck.dxf", "lasercad_upper_deck.dxf", "lasercad_lower_deck_underside.dxf")],
    "3_3d_print_K1_Max": [os.path.join(TS, "print", f) for f in
                          ("AEGIS_plate1_black_PLA+_rover_parts.3mf", "AEGIS_plate2_orange_PLA_demo_props.3mf")],
    "4_guides": [os.path.join(TS, f) for f in ("FABRICATION_GUIDE.md", "AEGIS_Wiring_Guide.pdf", "AEGIS_Proposal_Deck.pdf")],
    "5_cad_model": [os.path.join(B, "output", "AEGIS_Rover.f3d"), os.path.join(B, "output", "AEGIS_Rover.step"),
                    os.path.join(B, "output_detailed", "AEGIS_Rover_Detailed.f3d"),
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
| `4_guides/` | Fabrication guide, wiring guide, deck as PDF | Read `FABRICATION_GUIDE.md` first |
| `5_cad_model/` | Fusion archives (printable and detailed), STEP, two renders | Open the `.f3d` in Fusion; STEP for other CAD tools |

Firmware: `team_share/firmware/aegis_s3/` (build and flash steps in its README).
Simulation: `team_share/simulation3d/`.
"""


def main():
    if os.path.isdir(OUT):
        shutil.rmtree(OUT)
    for sub, files in FILES.items():
        os.makedirs(os.path.join(OUT, sub))
        for f in files:
            shutil.copy2(f, os.path.join(OUT, sub, os.path.basename(f)))
    with open(os.path.join(OUT, "README.md"), "w") as f:
        f.write(README)
    for sub in FILES:
        print(sub, sorted(os.listdir(os.path.join(OUT, sub))))


if __name__ == "__main__":
    main()
