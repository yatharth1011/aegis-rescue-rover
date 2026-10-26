"""
Printable start pad: the ArUco marker (DICT_4X4_50, id 0, 100 mm) that aegis_edge.py uses to correct
the rover's pose near home, the rover outline and the forward arrow.

    ~/aegis_venv/bin/python team_share/edge/make_start_pad.py   ->  team_share/edge/AEGIS_start_pad.pdf

Print on A4 at 100 % ("actual size", no fit-to-page) and check the 100 mm marker with a ruler.
Tape it flat on the table so the marker centre is at the --start x, y you give aegis_edge.py, with the
arrow pointing along the --start heading. The rover starts parked on it, centred, facing the arrow.
"""
import os

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "AEGIS_start_pad.pdf")
SIZE = 100.0                     # mm, must match MARKER_SIZE in aegis_edge.py
ROVER = (263.0, 140.0)           # mm, Mk2 footprint (Fusion bounding box, decks and wheels)
W, H = 297.0, 210.0              # A4 landscape


def main():
    img = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 600)
    fig = plt.figure(figsize=(W / 25.4, H / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.axis("off")
    cx, cy = W / 2, H / 2 + 5
    ax.add_patch(patches.Rectangle((cx - ROVER[0] / 2, cy - ROVER[1] / 2), *ROVER, fill=False, ls="--", lw=0.8,
                                   ec="0.5"))
    ax.text(cx - ROVER[0] / 2 + 3, cy + ROVER[1] / 2 - 7, "rover outline (Mk2, 263 x 140 mm)", fontsize=7, color="0.4")
    ax.imshow(img, cmap="gray", extent=(cx - SIZE / 2, cx + SIZE / 2, cy - SIZE / 2, cy + SIZE / 2),
              interpolation="nearest")
    ax.annotate("", xy=(cx + SIZE / 2 + 55, cy), xytext=(cx + SIZE / 2 + 8, cy),
                arrowprops=dict(arrowstyle="-|>", lw=2.5, color="k"))
    ax.text(cx + SIZE / 2 + 31, cy + 5, "FORWARD", ha="center", fontsize=9, weight="bold")
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):         # centre ticks outside the marker
        ax.plot([cx + dx * (SIZE / 2 + 3), cx + dx * (SIZE / 2 + 10)], [cy + dy * (SIZE / 2 + 3), cy + dy * (SIZE / 2 + 10)],
                "k", lw=0.8)
    ax.plot([20, 120], [12, 12], "k", lw=1.2)
    ax.text(70, 15, "100 mm: check with a ruler after printing", ha="center", fontsize=7)
    ax.text(W / 2, H - 8, "A.E.G.I.S. start pad  |  ArUco DICT_4X4_50 id 0, 100 mm  |  print at 100 % (actual size)",
            ha="center", fontsize=9)
    ax.text(W - 10, 12, "Marker centre = --start x, y.  Arrow = --start heading.", ha="right", fontsize=7)
    fig.savefig(OUT)
    print(OUT)


if __name__ == "__main__":
    main()
