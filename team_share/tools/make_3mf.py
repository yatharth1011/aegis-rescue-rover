"""
Print-ready 3MF of every PLA part, oriented flat on the bed and laid out on 220 x 220 mm plates.

    ~/aegis_venv/bin/python team_share/tools/make_3mf.py
    -> team_share/print/AEGIS_PLA_parts.3mf (+ one STL per part)

Reads the STLs that Build_AEGIS_Rover exports to Build_AEGIS_Rover/output/. If those were exported
before the printability fixes (hanger cable-tie channels, mast bolt holes and camera pad, full-height
funnel fin, cone under the door hub), the same fixes are applied here, so the file is correct either way.
Also adds the demo props from the simulation: victim token, orange cube decoy, orange disc decoy.
"""
import os

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "Build_AEGIS_Rover", "output")
OUT = os.path.join(HERE, "..", "print")


def box(lo, hi):
    lo, hi = np.array(lo, float), np.array(hi, float)
    return trimesh.creation.box(extents=hi - lo, transform=trimesh.transformations.translation_matrix((lo + hi) / 2))


def cyl(d, z0, z1, x=0.0, y=0.0, sections=64):
    return trimesh.creation.cylinder(radius=d / 2, height=z1 - z0, sections=sections,
                                     transform=trimesh.transformations.translation_matrix((x, y, (z0 + z1) / 2)))


def prism(poly_xy, z0, z1):                      # convex polygon in XY, extruded in Z
    pts = [(x, y, z) for x, y in poly_xy for z in (z0, z1)]
    return trimesh.convex.convex_hull(np.array(pts))


def load(name):
    return trimesh.load(os.path.join(SRC, "print_%s.stl" % name))


def solid_at(m, p):
    return bool(m.contains([p])[0])


def hanger():
    m = load("Motor_Hanger_PLA")
    if solid_at(m, (0, 8.0, 29.5)):                            # cable-tie channels missing
        cuts = [box((-9.3, s * 8 - 1.75, 28.5), (9.3, s * 8 + 1.75, 30.5)) for s in (1, -1)]
        cuts += [box((-9.3, s * 8 - 1.75, 14.9), (9.3, s * 8 + 1.75, 17.2)) for s in (1, -1)]
        m = m.difference(trimesh.util.concatenate(cuts))
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi, (1, 0, 0)))   # bridge face down
    return m


def mast():
    m = load("Camera_Mast_PLA")
    if not solid_at(m, (2.0, 0.0, 55.0)):                      # camera pad missing
        pad = trimesh.convex.convex_hull(np.array([(x, y, z) for x, z in ((-3.5, 38.4), (8.89, 59.0), (-3.5, 59.0))
                                                   for y in (-17.0, 17.0)]))
        m = m.union(pad)
    if solid_at(m, (4.0, 11.0, 1.5)):                          # bolt holes missing
        m = m.difference(trimesh.util.concatenate([cyl(3.2, -1, 4, 4.0, s * 11.0) for s in (1, -1)]))
    return m                                                   # base already on the bed


def shroud():
    m = load("Intake_Shroud_PLA")
    if not solid_at(m, (139.5, -38.8, 48.5)):                  # funnel fin stops 3 mm short of the deck
        m = m.union(prism([(131, -32), (131, -35), (148, -45), (148, -42)], 46.9, 50.0))
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi, (1, 0, 0)))   # flanges on the bed
    return m


def door():
    m = load("Intake_Door_PLA")
    if not solid_at(m, (0.0, 3.5, 28.0)):                      # no support cone under the hub
        cone = trimesh.creation.cone(radius=6.0, height=6.0, sections=64)          # apex up; flip and trim
        cone.apply_transform(trimesh.transformations.rotation_matrix(np.pi, (1, 0, 0)))
        cone.apply_translation((0, 0, 29.5))                                       # base at hub bottom, tip 6 mm below
        m = m.union(cone)
    return m                                                   # knuckle and leaf on the bed, socket opening up


def props():
    token = cyl(40.0, 0, 40.0)                                 # victim token: print orange, add a white band
    cube = box((-20, -20, 0), (20, 20, 40))                    # decoy 1: orange cube
    disc = cyl(60.0, 0, 12.0)                                  # decoy 2: orange disc (decoy 3 is a bought red ball)
    return [("Victim_Token_40mm", token), ("Decoy_Cube_40mm", cube), ("Decoy_Disc_60mm", disc)]


def on_bed(m):
    m = m.copy()
    m.apply_translation((-m.bounds[0][0], -m.bounds[0][1], -m.bounds[0][2]))
    return m


def main():
    os.makedirs(OUT, exist_ok=True)
    parts = [("Intake_Shroud", shroud()), ("Intake_Door", door()), ("Camera_Mast", mast())]
    h = hanger()
    parts += [("Motor_Hanger_%d" % (i + 1), h.copy()) for i in range(4)]
    parts += props()

    # plate 1: rover parts, plate 2: demo props; simple shelf packing on 220 x 220
    plates, report = [[], []], []
    for name, m in parts:
        m = on_bed(m)
        assert m.is_watertight, name
        plates[1 if name.startswith(("Victim", "Decoy")) else 0].append((name, m))
        m.export(os.path.join(OUT, name + ".stl"))
        report.append("%-18s %5.1f x %5.1f x %5.1f mm  %5.1f cm3" % (name, *m.extents, m.volume / 1000))
    scene = trimesh.Scene()
    for k, plate in enumerate(plates):
        x = y = row_h = 0.0
        for name, m in sorted(plate, key=lambda t: -t[1].extents[1]):
            if x + m.extents[0] > 220:
                x, y, row_h = 0.0, y + row_h + 8, 0.0
            assert y + m.extents[1] <= 220, "plate %d overflow" % (k + 1)
            mm = m.copy()
            mm.apply_translation((x + k * 260.0, y, 0))      # plates side by side in the file
            scene.add_geometry(mm, node_name="plate%d_%s" % (k + 1, name), geom_name=name)
            x, row_h = x + m.extents[0] + 8, max(row_h, m.extents[1])
    f = os.path.join(OUT, "AEGIS_PLA_parts.3mf")
    scene.export(f)
    print("\n".join(report))
    print(f)


if __name__ == "__main__":
    main()
