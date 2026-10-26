"""
Formatted PDF guides from the Markdown sources, with figures, via headless Chrome.

    ~/aegis_venv/bin/python team_share/tools/make_guides_pdf.py
    -> team_share/AEGIS_Fabrication_Guide.pdf, team_share/AEGIS_Deployment_Guide.pdf

Edit the sources (FABRICATION_GUIDE.md, edge/README.md), then run this again. A line
"<!-- fig:NAME -->" in a source places figure NAME there.
"""
import base64
import html
import os
import re
import subprocess
import tempfile

import ezdxf
import markdown
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from ezdxf import bbox as dxf_bbox
from matplotlib import patches

HERE = os.path.dirname(os.path.abspath(__file__))
TS = os.path.abspath(os.path.join(HERE, ".."))
ASSETS = os.path.join(TS, "deck_assets", "guides")
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
NAVY, ORANGE = "#1E2A44", "#EA6C1C"

GUIDES = [
    dict(src=os.path.join(TS, "FABRICATION_GUIDE.md"), out=os.path.join(TS, "AEGIS_Fabrication_Guide.pdf"),
         title="Fabrication guide", sub="What to cut, print and prepare, and from which material",
         cover=os.path.join(TS, "Build_AEGIS_Rover", "output", "render_iso_door_open.png")),
    dict(src=os.path.join(TS, "edge", "README.md"), out=os.path.join(TS, "AEGIS_Deployment_Guide.pdf"),
         title="Deployment guide", sub="Flashing, network, bring-up and the first autonomous mission",
         cover=os.path.join(TS, "Build_AEGIS_Rover", "output_detailed", "render_wiring_lower_deck.png")),
    dict(src=os.path.join(TS, "HOW_AEGIS_WORKS.md"), out=os.path.join(TS, "AEGIS_How_It_Works.pdf"),
         title="How A.E.G.I.S. works", sub="The whole system explained in plain language: hearing, seeing, thinking, staying safe",
         cover=os.path.join(TS, "Build_AEGIS_Rover", "output", "render_iso_door_open.png")),
]

CSS = """
@page { size: A4; margin: 18mm 17mm 20mm 17mm;
  @bottom-left { content: "A.E.G.I.S. Mk2  |  %(title)s"; font: 8pt 'Helvetica Neue', Arial; color: #8a93a3; }
  @bottom-right { content: "page " counter(page) " of " counter(pages); font: 8pt 'Helvetica Neue', Arial; color: #8a93a3; } }
@page :first { margin: 0; @bottom-left { content: none; } @bottom-right { content: none; } }
* { box-sizing: border-box; }
body { font: 10.2pt/1.5 'Helvetica Neue', Helvetica, Arial, sans-serif; color: #1d2433; margin: 0; }
.cover { height: 297mm; width: 210mm; position: relative; background: %(navy)s; color: white; overflow: hidden;
  page-break-after: always; }
.cover .band { position: absolute; left: 0; top: 0; width: 9mm; height: 100%%; background: %(orange)s; }
.cover .brand { position: absolute; left: 24mm; top: 30mm; font-size: 13pt; letter-spacing: 3px; color: #c9d2e3; }
.cover h1 { position: absolute; left: 24mm; top: 40mm; margin: 0; font-size: 34pt; font-weight: 800; color: white; }
.cover .sub { position: absolute; left: 24mm; top: 62mm; right: 24mm; font-size: 13pt; color: #dfe5ef; }
.cover .img { position: absolute; left: 18mm; right: 18mm; top: 92mm; height: 140mm; background: white;
  border-radius: 6px; display: flex; align-items: center; justify-content: center; }
.cover .img img { max-width: 95%%; max-height: 95%%; }
.cover .foot { position: absolute; left: 24mm; bottom: 22mm; font-size: 9.5pt; color: #c9d2e3; line-height: 1.6; }
h1 { display: none; }
h2 { font-size: 15pt; color: %(navy)s; border-bottom: 2.5px solid %(orange)s; padding-bottom: 3px; margin: 22px 0 10px;
  page-break-after: avoid; }
h3 { font-size: 11.5pt; color: %(navy)s; margin: 16px 0 6px; page-break-after: avoid; }
p { margin: 6px 0; }
strong { color: #0f1626; }
a { color: %(navy)s; text-decoration: none; border-bottom: 1px dotted #8a93a3; }
table { border-collapse: collapse; width: 100%%; margin: 8px 0 12px; font-size: 9.2pt; page-break-inside: avoid; }
th { background: %(navy)s; color: white; text-align: left; padding: 5px 7px; font-weight: 600; }
td { padding: 5px 7px; border-bottom: 1px solid #dde2ea; vertical-align: top; }
tr:nth-child(even) td { background: #f4f6fa; }
code { font: 8.8pt 'SF Mono', Menlo, monospace; background: #eef1f6; padding: 1px 4px; border-radius: 3px; color: #2b3550; }
pre { background: #1d2433; color: #e6ebf3; padding: 9px 12px; border-radius: 5px; overflow: hidden; white-space: pre-wrap;
  page-break-inside: avoid; margin: 8px 0 10px; border-left: 4px solid %(orange)s; }
pre code { background: none; color: inherit; padding: 0; font-size: 8.6pt; }
blockquote { margin: 10px 0; padding: 8px 12px; background: #fff4ec; border-left: 4px solid %(orange)s; border-radius: 3px; }
blockquote p { margin: 4px 0; }
ul, ol { margin: 6px 0 8px; padding-left: 22px; }
li { margin: 3px 0; }
figure { margin: 10px 0 14px; text-align: center; page-break-inside: avoid; }
figure img { max-width: 100%%; border: 1px solid #dde2ea; border-radius: 4px; }
figcaption { font-size: 8.6pt; color: #5b6476; margin-top: 4px; }
"""


# ------------------------------------------------------------------ figures
def fig_decks(path):
    out = os.path.join(TS, "Build_AEGIS_Rover", "output")
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.6))
    for ax, name, title in zip(axs, ("lasercad_lower_deck.dxf", "lasercad_upper_deck.dxf"),
                               ("Lower deck, 255 x 120 mm", "Upper deck, 210 x 120 mm")):
        doc = ezdxf.readfile(os.path.join(out, name))
        for e in doc.modelspace():
            col = "#d0021b" if e.dxf.layer == "CUT" else "#1f5fbf"
            lw = 0.9 if e.dxf.layer == "CUT" else 0.35
            if e.dxftype() == "CIRCLE":
                ax.add_patch(patches.Circle((e.dxf.center.x, e.dxf.center.y), e.dxf.radius, fill=False, ec=col, lw=lw))
            else:
                pts = np.array([(v.x, v.y) for v in e.points()])
                if e.is_closed:
                    pts = np.vstack([pts, pts[:1]])
                ax.plot(pts[:, 0], pts[:, 1], color=col, lw=lw)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(title, fontsize=10)
    axs[0].plot([], [], color="#d0021b", label="red: CUT (full power)")
    axs[0].plot([], [], color="#1f5fbf", label="blue: ENGRAVE (light, run first)")
    axs[0].legend(loc="upper left", bbox_to_anchor=(0, -0.02), ncol=2, fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=200, facecolor="white")
    plt.close(fig)


def fig_plates(path):
    fig, axs = plt.subplots(1, 2, figsize=(10, 4.9))
    for ax, name, col, title in zip(
            axs, ("AEGIS_plate1_black_PLA+_rover_parts.3mf", "AEGIS_plate2_orange_PLA_demo_props.3mf"),
            ("#3a4150", ORANGE), ("Plate 1: black PLA+ (rover parts)", "Plate 2: orange PLA (demo props)")):
        s = trimesh.load(os.path.join(TS, "print", name))
        ax.add_patch(patches.Rectangle((0, 0), 300, 300, fc="#f4f6fa", ec="#8a93a3", lw=1))
        for node in s.graph.nodes_geometry:
            T, g = s.graph[node]
            mm = s.geometry[g].copy()
            mm.apply_transform(T)
            hull = trimesh.path.polygons.projected(mm, normal=(0, 0, 1))
            if hull is not None:
                xs, ys = hull.exterior.xy
                ax.fill(xs, ys, color=col, alpha=0.85, lw=0)
                c = hull.centroid
                ax.text(c.x, c.y, g.replace("_", " ").replace("Motor Hanger", "Mount"), fontsize=5.5,
                        ha="center", va="center", color="white" if col != ORANGE else "black")
        ax.set_xlim(-5, 305)
        ax.set_ylim(-5, 305)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=10)
        ax.set_xticks([0, 150, 300])
        ax.set_yticks([0, 150, 300])
        ax.tick_params(labelsize=7)
    fig.suptitle("Creality K1 Max bed, 300 x 300 mm (top view)", fontsize=9, color="#5b6476", y=0.04)
    fig.tight_layout()
    fig.savefig(path, dpi=200, facecolor="white")
    plt.close(fig)


def fig_system(path):
    fig, ax = plt.subplots(figsize=(10, 3.6))
    ax.axis("off")
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 36)

    def box(x, y, w, h, title, lines, fc):
        ax.add_patch(patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.5", fc=fc, ec=NAVY, lw=1.4))
        ax.text(x + w / 2, y + h - 3, title, ha="center", va="top", fontsize=10.5, weight="bold", color=NAVY)
        for i, t in enumerate(lines):
            ax.text(x + w / 2, y + h - 8.2 - 3.6 * i, t, ha="center", va="top", fontsize=7.6, color="#2b3550")

    box(2, 4, 30, 28, "Rover: ESP32-S3", ["motors, encoders, gyro", "3 x distance, 6 x IR", "edge reflex, stall escape",
                                         "CBF speed limit", "300 ms link watchdog", "scream bearing (2 mics)"], "#eef3fb")
    box(68, 4, 30, 28, "Laptop: edge brain", ["YOLO victim detector", "scream detector", "map, coverage search, A*",
                                             "rescue sequence", "start-pad marker fix", "same brain.py as the 3D sim"], "#fff4ec")
    box(37, 22, 26, 11, "ESP32-CAM", ["MJPEG 320 x 240, ~10 fps"], "#eef3fb")
    arrow = dict(arrowstyle="-|>", lw=1.3, color=NAVY)
    ax.annotate("", xy=(67.5, 17), xytext=(32.5, 17), arrowprops=arrow)
    ax.text(50, 18.3, "telemetry 20 Hz, audio clips  (UDP 4211)", ha="center", fontsize=7.6)
    ax.annotate("", xy=(32.5, 11), xytext=(67.5, 11), arrowprops=arrow)
    ax.text(50, 7.4, "commands v, w, door 20 Hz; listen  (UDP 4210)", ha="center", fontsize=7.6)
    ax.annotate("", xy=(67.5, 27.5), xytext=(63.2, 27.5), arrowprops=arrow)
    ax.text(50, 1.0, "Same Wi-Fi network (a phone hotspot works). Losing it stops the rover within 300 ms.",
            ha="center", fontsize=8, color="#5b6476")
    fig.savefig(path, dpi=200, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def fig_safety(path):
    fig, ax = plt.subplots(figsize=(10, 3.4))
    ax.axis("off")
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 34)
    steps = [("Laptop command", "v, w, door", "#fff4ec"), ("Wi-Fi watchdog", "no command 0.3 s:\nstop", "#eef3fb"),
             ("Edge reflex", "edge seen: back off,\nno turning for 1 s", "#eef3fb"), ("Stall escape", "pinned: back off", "#eef3fb"),
             ("Distance limit", "slower the closer\nan obstacle is", "#eef3fb"), ("Wheels", "PI speed control", "#e8f6ec")]
    w = 14.2
    for i, (t, sub, fc) in enumerate(steps):
        x = 1 + i * (w + 2.3)
        ax.add_patch(patches.FancyBboxPatch((x, 8), w, 18, boxstyle="round,pad=0.3,rounding_size=1.5", fc=fc, ec=NAVY, lw=1.3))
        ax.text(x + w / 2, 21.5, t, ha="center", va="center", fontsize=7.8, weight="bold", color=NAVY)
        ax.text(x + w / 2, 14, sub, ha="center", va="center", fontsize=6.6, color="#2b3550")
        if i:
            ax.annotate("", xy=(x - 0.2, 17), xytext=(x - 2.1, 17), arrowprops=dict(arrowstyle="-|>", lw=1.4, color=ORANGE))
    ax.add_patch(patches.FancyBboxPatch((17.5, 2), 64, 3.2, boxstyle="round,pad=0.3", fc=NAVY, ec=NAVY))
    ax.text(49.5, 3.6, "runs on the rover, 100 times a second; the laptop cannot switch it off", ha="center", va="center",
            fontsize=8, color="white")
    fig.savefig(path, dpi=200, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def fig_sound(path):
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8), gridspec_kw=dict(width_ratios=[1.1, 1]))
    ax = axs[0]
    ax.set_xlim(-6, 6)
    ax.set_ylim(-1.5, 6.5)
    ax.set_aspect("equal")
    ax.axis("off")
    src = np.array([-3.6, 5.2])
    mics = {"L": np.array([-0.5, 0]), "R": np.array([0.5, 0])}
    for k, m in mics.items():
        ax.plot(*m, "o", ms=11, color=NAVY)
        ax.text(m[0], -0.9, "mic " + k, ha="center", fontsize=8)
        ax.plot([src[0], m[0]], [src[1], m[1]], color=ORANGE if k == "L" else "#8a93a3", lw=1.6)
    ax.plot(*src, "*", ms=18, color="#d62828")
    ax.text(src[0], src[1] + 0.6, "scream", ha="center", fontsize=8.5, weight="bold")
    ax.text(0, -1.45, "10 cm apart", ha="center", fontsize=7.5, color="#5b6476")
    ax.text(-5.8, 2.6, "the left mic hears it a\nfraction of a millisecond\nearlier than the right one", fontsize=8)
    ax.set_title("1. A small time difference", fontsize=9.5, color=NAVY, weight="bold")
    ax = axs[1]
    t = np.linspace(0, 1, 400)
    sig = np.sin(2 * np.pi * 9 * t) * np.exp(-((t - 0.45) / 0.18) ** 2)
    ax.plot(t, sig + 1.4, color=ORANGE, lw=1.4)
    ax.plot(t + 0.08, sig - 0.6, color="#8a93a3", lw=1.4)
    ax.text(-0.02, 1.4, "left", ha="right", va="center", fontsize=8)
    ax.text(-0.02, -0.6, "right", ha="right", va="center", fontsize=8)
    ax.annotate("", xy=(0.53, -1.6), xytext=(0.45, -1.6), arrowprops=dict(arrowstyle="<->", lw=1.2))
    ax.text(0.49, -2.1, "delay -> angle", ha="center", fontsize=8)
    ax.set_xlim(-0.12, 1.1)
    ax.set_ylim(-2.4, 2.6)
    ax.axis("off")
    ax.set_title("2. GCC-PHAT finds the delay, maths gives the angle", fontsize=9.5, color=NAVY, weight="bold")
    fig.tight_layout()
    fig.savefig(path, dpi=200, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def fig_states(path):
    fig, ax = plt.subplots(figsize=(10, 3.6))
    ax.axis("off")
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 36)
    S = [("EXPLORE", "search the table", 8), ("PRE-GRASP", "go 30 cm in front", 25), ("APPROACH", "door open, drive in", 42),
         ("CLOSE", "door shut, check", 59), ("RETURN", "safe route home", 76), ("DONE", "token home", 92)]
    for name, sub, x in S:
        ax.add_patch(patches.FancyBboxPatch((x - 7, 16), 14, 12, boxstyle="round,pad=0.3,rounding_size=2",
                                            fc="#e8f6ec" if name == "DONE" else "#eef3fb", ec=NAVY, lw=1.3))
        ax.text(x, 24, name, ha="center", fontsize=8.6, weight="bold", color=NAVY)
        ax.text(x, 19.4, sub, ha="center", fontsize=6.3, color="#2b3550")
    for (_, _, a), (_, _, b) in zip(S, S[1:]):
        ax.annotate("", xy=(b - 7.4, 22), xytext=(a + 7.4, 22), arrowprops=dict(arrowstyle="-|>", lw=1.4, color=ORANGE))
    lab = ["token seen", "in position", "token in the bay", "still inside", "at the start"]
    for (_, _, a), (_, _, b), t in zip(S, S[1:], lab):
        ax.text((a + b) / 2, 29.5, t, ha="center", fontsize=6.8, color="#5b6476")
    ax.annotate("", xy=(8, 15.6), xytext=(42, 15.6), arrowprops=dict(arrowstyle="-|>", lw=1.2, color="#8a93a3",
                                                                     connectionstyle="arc3,rad=-0.35"))
    ax.text(25, 4.5, "missed it: back off, look again", ha="center", fontsize=7.2, color="#5b6476")
    fig.savefig(path, dpi=200, facecolor="white", bbox_inches="tight")
    plt.close(fig)


FIGS = {
    "decks": (fig_decks, "The two decks as LaserCAD will show them (lasercad_*.dxf)."),
    "plates": (fig_plates, "Print plates as laid out in the two 3MF files."),
    "system": (fig_system, "What runs where, and how the three boards talk."),
    "safety": (fig_safety, "Every laptop command passes these checks on the rover before the wheels turn."),
    "sound": (fig_sound, "Two microphones 10 cm apart: the arrival-time difference gives the direction."),
    "states": (fig_states, "The rescue as a state machine."),
}


# ------------------------------------------------------------------ build
def data_uri(path):
    ext = os.path.splitext(path)[1][1:].replace("jpg", "jpeg")
    with open(path, "rb") as f:
        return "data:image/%s;base64,%s" % (ext, base64.b64encode(f.read()).decode())


def build(g):
    src = open(g["src"]).read()
    for name, (fn, caption) in FIGS.items():
        marker = "<!-- fig:%s -->" % name
        if marker in src:
            p = os.path.join(ASSETS, name + ".png")
            fn(p)
            src = src.replace(marker, '<figure><img src="%s"><figcaption>%s</figcaption></figure>' %
                              (data_uri(p), html.escape(caption)))
    for m in re.finditer(r"<!-- img:([^|]+)\|(.*?) -->", src):
        src = src.replace(m.group(0), '<figure><img src="%s"><figcaption>%s</figcaption></figure>' %
                          (data_uri(os.path.join(TS, m.group(1))), html.escape(m.group(2))))
    body = markdown.markdown(src, extensions=["tables", "fenced_code", "sane_lists"])
    cover = ('<div class="cover"><div class="band"></div><div class="brand">A.E.G.I.S.  MK2</div>'
             '<h1 style="display:block">%s</h1><div class="sub">%s</div><div class="img"><img src="%s"></div>'
             '<div class="foot">Autonomous search-and-rescue rover<br>TL101 Tinkering Lab, IIT Ropar, AY 2026-27</div></div>'
             % (html.escape(g["title"]), html.escape(g["sub"]), data_uri(g["cover"])))
    doc = "<!doctype html><html><head><meta charset='utf-8'><style>%s</style></head><body>%s%s</body></html>" % (
        CSS % dict(title=g["title"], navy=NAVY, orange=ORANGE), cover, body)
    with tempfile.NamedTemporaryFile("w", suffix=".html", delete=False) as f:
        f.write(doc)
        tmp = f.name
    subprocess.run([CHROME, "--headless", "--disable-gpu", "--no-pdf-header-footer", "--print-to-pdf=" + g["out"],
                    "file://" + tmp], check=True, capture_output=True)
    os.remove(tmp)
    print(g["out"])


def main():
    os.makedirs(ASSETS, exist_ok=True)
    for g in GUIDES:
        build(g)


if __name__ == "__main__":
    main()
