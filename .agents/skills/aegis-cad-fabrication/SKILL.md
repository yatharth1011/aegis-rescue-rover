---
name: aegis-cad-fabrication
description: Anything about the A.E.G.I.S. mechanical design and fabrication - the Fusion 360 script Build_AEGIS_Rover.py, clashes, holes, fasteners, the K1 Max 3MF print plates (make_3mf.py), the LaserCAD acrylic DXFs (lasercad_dxf.py), STEP/STL exports, renders, the bill of materials, or whether the team has the parts and screws needed.
---

# CAD and fabrication

## Hard constraints
- **Nothing more can be ordered.** Only these are available:
  - the BOM in `tools/bom_excel.py` (39 items);
  - reusable lab tools: soldering iron, screwdrivers, the laser cutter running LaserCAD, the Creality K1 Max printer;
  - personal phones and laptops.
- Fasteners come only from the M3 kit: **20 × M3x6, 20 × M3 nuts, M-F and F-F pillars of 6/10/15/20 mm (10 each)**. There are no M3x10 screws. If a joint needs a longer screw, use a pillar stack or redesign. Ask first.
- Acrylic: 3 mm sheet. Printer: K1 Max, 300 × 300 mm bed, black PLA+ for rover parts and orange PLA for the demo props.
- Any change to a dimension, hole, part or material is a **design change: ask the user first.**

## The Fusion script
- `team_share/Build_AEGIS_Rover/Build_AEGIS_Rover.py` builds the whole rover parametrically. It **only runs inside Fusion 360**: Utilities → Scripts and Add-Ins → the green + → this folder → Run.
- You cannot execute it from a terminal. So:
  - read the code carefully;
  - keep the units Fusion uses: the API is in **cm**, and the script's helper functions take mm; check which one a call uses;
  - tell the user exactly what to run and what to look at.
- Outputs:
  - `output/`: printable model, `.f3d`, `.step`, STLs, DXFs, renders.
  - `output_detailed/`: the version with the wiring harness.
  The renders run **after** the f3d/STEP export. Keep that order, or hidden bodies end up missing from the exports.
- The team's own saves, `The Most Final AEGIS Body.f3d` and `The Most Final AEGIS Body Detailed.f3d` in `project/`, are theirs. Never overwrite them. `make_final.py` prefers them when they are newer.
- Clashes fixed before must not come back: the switch cutout corners, the shroud vs `IR_Cliff_FR` (the fin has a notch there), and the door self-overlap.
  After changing geometry, ask the user to run Inspect → Interference in Fusion and send a screenshot.

## Derived files
```bash
cd ~/Documents/Fusion/project
~/aegis_venv/bin/python team_share/tools/make_3mf.py       # print/AEGIS_plate1_black_PLA+_rover_parts.3mf, plate2_orange_PLA_demo_props.3mf + STLs
~/aegis_venv/bin/python team_share/tools/lasercad_dxf.py   # Build_AEGIS_Rover/output/lasercad_*.dxf (R12, mm; CUT layer red 1, ENGRAVE blue 5)
~/aegis_venv/bin/python team_share/tools/bom_excel.py      # item list (course template layout; local only)
~/aegis_venv/bin/python team_share/tools/assembly_guide.py # re-render the assembly guide after geometry changes
~/aegis_venv/bin/python team_share/tools/make_final.py
```
- `make_3mf.py` reads the STLs from Fusion's `output/`. It applies printability patches: pilot holes, hanger cable-tie channels, the mast camera pad, the full-height fin and the ToF L-brackets. Then it lays the parts out centred on the plate. After a change, check that every part lies inside 300 × 300 mm and flat on the bed, and that the part count is unchanged.
- Check meshes with `trimesh`: `is_watertight`, bounds, volume. Booleans use `manifold3d`.
- The DXFs must open in LaserCAD. Use R12 only, with polylines (no splines) and closed cut contours.

## Before you answer "do we have everything?"
Count from the sources, not from memory:
- screws, nuts and pillars per step in `assembly_guide.py`, against the kit contents;
- wires and consumables (foam tape, cable ties, heat-shrink, solder) against the BOM;
- every printed part against the 3MF.
Report the shortages plainly.
