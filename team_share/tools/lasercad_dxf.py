"""
One LaserCAD-ready DXF per deck: R12 format, millimetres, splines flattened to polylines,
cut lines on layer CUT (red, colour 1) and placement marks + labels on layer ENGRAVE (blue, colour 5).

    ~/aegis_venv/bin/python team_share/tools/lasercad_dxf.py
    -> team_share/Build_AEGIS_Rover/output/lasercad_lower_deck.dxf, lasercad_upper_deck.dxf

In LaserCAD: File > Import the DXF, set the blue layer to engrave/scan (low power) and the red layer
to cut, and run the blue layer first so the sheet does not move after cutting.
"""
import os

import ezdxf
from ezdxf import path as ezpath

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Build_AEGIS_Rover", "output")
DECKS = {
    "lower_deck": (["lasercut_lower_deck.dxf"],
                   ["engrave_lower_deck_top.dxf", "engrave_lower_deck_top_labels.dxf"]),
    "upper_deck": (["lasercut_upper_deck.dxf"],
                   ["engrave_upper_deck_top.dxf", "engrave_upper_deck_top_labels.dxf"]),
}
# the lower deck's underside marks (cliff sensors) need the sheet flipped, so they get their own file
UNDERSIDE = ("lower_deck_underside", ["engrave_lower_deck_underside.dxf", "engrave_lower_deck_underside_labels.dxf"])


def copy_into(msp, src, layer, color):
    n = 0
    for e in ezdxf.readfile(os.path.join(OUT, src)).modelspace():
        if e.dxftype() == "CIRCLE":
            msp.add_circle(e.dxf.center, e.dxf.radius, dxfattribs={"layer": layer, "color": color})
        else:
            try:
                p = ezpath.make_path(e)
            except TypeError:
                continue
            for sub in p.sub_paths():
                pts = [(v.x, v.y) for v in sub.flattening(0.02)]
                if len(pts) > 1:
                    msp.add_polyline2d(pts, close=sub.is_closed, dxfattribs={"layer": layer, "color": color})
        n += 1
    return n


def write(name, cut, engrave):
    doc = ezdxf.new("R12")
    doc.header["$INSUNITS"] = 4
    doc.layers.add("CUT", color=1)
    doc.layers.add("ENGRAVE", color=5)
    msp = doc.modelspace()
    counts = [sum(copy_into(msp, f, "ENGRAVE", 5) for f in engrave), sum(copy_into(msp, f, "CUT", 1) for f in cut)]
    f = os.path.join(OUT, "lasercad_%s.dxf" % name)
    doc.saveas(f)
    print("%s: %d engrave + %d cut entities" % (os.path.basename(f), *counts))


if __name__ == "__main__":
    for name, (cut, engrave) in DECKS.items():
        write(name, cut, engrave)
    write(UNDERSIDE[0], [], UNDERSIDE[1])
