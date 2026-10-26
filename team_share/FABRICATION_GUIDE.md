# A.E.G.I.S. fabrication guide: what to cut or print, from which material

Item numbers (#) refer to the lines in `to_submit/ItemList_AEGIS_Thursday_1.xlsx`.
Build files are in `team_share/Build_AEGIS_Rover/output/` and `team_share/print/`.

> **Before cutting anything:** rerun both Fusion scripts (`Build_AEGIS_Rover`, `Build_AEGIS_Rover_Detailed`), check the clash list is empty, then regenerate the deck files and print plates:
> ```bash
> ~/aegis_venv/bin/python team_share/tools/lasercad_dxf.py
> ```
> ```bash
> ~/aegis_venv/bin/python team_share/tools/make_3mf.py
> ```

## 1. Summary

| What you make | From (item #) | Machine | File |
|---|---|---|---|
| Lower deck, 255 x 120 mm | 3 mm clear acrylic sheet (#16) | Laser cutter (LaserCAD) | `lasercad_lower_deck.dxf` |
| Upper deck, 210 x 120 mm | Same sheet (#16) | Laser cutter (LaserCAD) | `lasercad_upper_deck.dxf` |
| Cliff-sensor template | Plain paper | Normal printer, 100 % scale | `lasercad_lower_deck_underside.dxf` |
| Shroud, door, camera mast, 4 motor mounts, 3 ToF brackets | Black PLA+ (#14) | Creality K1 Max | `print/AEGIS_plate1_black_PLA+_rover_parts.3mf` |
| Victim token, cube decoy, disc decoy | Orange PLA (#39) | Creality K1 Max | `print/AEGIS_plate2_orange_PLA_demo_props.3mf` |
| ESP32-S3 carrier board, 66 x 34 mm | Prototype PCB 8 x 12 cm (#31) | Hacksaw or score-and-snap, file the edges | Size from this guide |
| Fuse board, 30 x 14 mm | Offcut of the same PCB (#31) | Same | Size from this guide |
| Header pins | Male and female strips (#32, #33) | Side cutters | Counts in section 4 |
| Wires | Silicone wire (#24–#30) | Wire strippers | Gauges in `AEGIS_Wiring_Guide.pdf` |

Nothing else is cut or printed: wheels, motors, spacers, screws and every electronic board are bought ready-made.

## 2. Acrylic decks (laser cutter, LaserCAD)

<!-- fig:decks -->

Material: one NEILL 12 x 12 inch (about 305 x 305 mm) clear acrylic sheet, 3 mm (#16). Both decks fit on it.

1. Peel the protective film only after cutting (it stops burn marks), unless the lab says otherwise.
2. In LaserCAD, import `lasercad_lower_deck.dxf` and `lasercad_upper_deck.dxf`. **Choose millimetres** when asked, and check the sizes: 255 x 120 mm and 210 x 120 mm.
3. Place them side by side along the 305 mm edge, about 30 mm apart (some labels reach a few mm past the deck outline). They need roughly 305 x 255 mm.
4. Layers:
   - **Blue (ENGRAVE):** placement marks and labels. Low-power scan or light engrave. Run first.
   - **Red (CUT):** outline, holes, slots. Full cut for 3 mm acrylic. Run second.
   Ask the lab staff for their machine's power and speed for 3 mm acrylic; do a 20 mm test square on a corner of the sheet first.
5. Check after cutting: M3 screws (3.2 mm holes) pass freely; the servo pocket fits the MG946R body; the power-switch cutout (19.4 x 13.2 mm) fits the switch. Measure the switch before cutting if it has arrived.
6. Do **not** laser the underside file. Print `lasercad_lower_deck_underside.dxf` on paper at 100 % (check a dimension with a ruler) and use it as a template for the five cliff sensors under the lower deck.

## 3. 3D-printed parts (Creality K1 Max, Creality Print)

<!-- fig:plates -->

| Plate | Filament | Parts |
|---|---|---|
| 1 | Black PLA+ (#14) | Intake shroud, intake door, camera mast, 4 motor mounts, 3 ToF L-brackets |
| 2 | Orange PLA (#39) | Victim token (40 x 40 mm), cube decoy (40 mm), disc decoy (60 x 12 mm) |

Settings: K1 Max profile, textured PEI plate, 0.2 mm layers, 3 walls, 25 % gyroid infill; 40 % for the motor mounts, camera mast and brackets. Brim on the shroud. No supports needed: every part is already oriented flat.

Order:
1. **Fit test first:** print one motor mount and the door alone. Check the N20 motor slides into the mount (0.2 mm clearance per side; sand lightly if tight) and the servo spline presses into the door socket (warm the spline; if it is loose, glue the servo horn instead).
2. Print the rest of plate 1.
3. Print plate 2 any time. Paint or tape a white band around the middle of the token.

Filament use: at most about 160 g black and 180 g orange (solid-part estimate; less with infill), from 1 kg spools.

## 4. Electronics prep (bench, soldering iron)

**Prototype PCB (#31), 8 x 12 cm:**
- Carrier board: 66 x 34 mm. Holds two 22-pin female sockets for the ESP32-S3 (rows 22.86 mm apart, 2.54 mm pitch), the two 2.2 kΩ I2C pull-ups and the 100 kΩ / 20 kΩ battery divider.
- Fuse board: 30 x 14 mm. Holds the BLX-A fuse holder (#22).
- Score both sides along a hole row with a knife and snap, or use a hacksaw; file the edges smooth.

**Female header strips (#32), 4 x 1x20:** each ESP32-S3 row needs 22 sockets. Use one full strip plus 2 sockets cut from a second strip, per row. Cutting a female strip sacrifices the socket you cut through, so cut on the 3rd socket of the spare strip.

**Male header strips (#33), 3 x 1x40:** break off by hand:

| Board | Pins |
|---|---|
| VL53L0X x 3 | 6 each = 18 |
| MPU-6050 | 8 |
| INMP441 x 2 | 6 each = 12 |
| TB6612FNG x 2 | 16 each = 32 |
| Total | 70 (two strips; the third is spare) |

**Wire (#24–#30):** cut to length as you route, following `AEGIS_Wiring_Guide.pdf`:

| Wire | Use | You have |
|---|---|---|
| 18 AWG red / black | Battery, fuse, switch, buck and driver inputs | 1 m each |
| 22 AWG 3-core | 5 V rails, servo, motor leads | 2 m |
| 26 AWG red / black / white / green | 3.3 V, grounds, signals | 3 m each |

Slide heat-shrink (#35) on before soldering every power joint.

**Foam tape (#38):** two layers on the camera pad of the mast; one layer under each sensor board, the ToF brackets, the mic boards and the PCF8574.

**Cable ties (#36):** one per motor through the channel in its mount; the rest for the harness and to strap the battery to the lower deck.

## 5. Order of work

1. Fusion rerun, then regenerate the DXF and 3MF files (top of this guide).
2. Fit-test prints (motor mount, door), then plate 1 and plate 2.
3. Laser-cut the decks.
4. Cut and solder the carrier board and fuse board; solder headers on the modules.
5. Assemble and wire with `AEGIS_Wiring_Guide.pdf`. Set both XL4015 outputs to 5.0 V with a multimeter **before** connecting any board.
