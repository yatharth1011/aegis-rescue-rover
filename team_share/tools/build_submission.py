"""
Rebuilds the two submission documents from the current design and results:

  to_submit/ProjectIdea_AEGIS_Thursday_1.pptx   (2 slides, course template structure)
  to_submit/ApprovalForm_AEGIS_Thursday_1.docx (technical bullets refreshed)

Inputs (all optional; the deck shows a clear placeholder if one is missing):
  team_share/Build_AEGIS_Rover/output/render_iso_door_open.png   Fusion render
  team_share/simulation3d/results/summary.json                   3D simulation numbers

    ~/aegis_venv/bin/python team_share/tools/build_submission.py
"""
import json
import os

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Emu, Pt

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(PROJ, "to_submit")
RENDER = next((p for p in (os.path.join(PROJ, "team_share", "Build_AEGIS_Rover", d, "render_iso_door_open.png")
                for d in ("output_detailed", "output")) if os.path.exists(p)), "")
SIM = os.path.join(PROJ, "team_share", "simulation3d", "results", "summary.json")

INK = RGBColor(0x0F, 0x17, 0x2A)
MUTED = RGBColor(0x47, 0x55, 0x69)
ACCENT = RGBColor(0xEA, 0x6C, 0x1C)
PANEL = RGBColor(0xF1, 0xF5, 0xF9)
LINE = RGBColor(0xCB, 0xD5, 0xE1)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

FACULTY = [
    ("Prof. Shashi Shekhar Jha", "Computer Science, Room 215, Ramanujan Block", "shashi@iitrpr.ac.in"),
    ("Prof. Sudeepta Mishra", "Computer Science, Room 315, Ramanujan Block", "sudeepta@iitrpr.ac.in"),
    ("Prof. Dhiraj K. Mahajan", "Mechanical, Room 329, Satish Dhawan Block", "dhiraj.mahajan@iitrpr.ac.in"),
    ("Prof. Shrikant S. Padhee", "Mechanical, Room 323, Satish Dhawan Block", "sspadhee@iitrpr.ac.in"),
    ("Prof. Brijesh Kumbhani", "Electrical, Room 217, J C Bose Block", "brijesh@iitrpr.ac.in"),
    ("Prof. Arun Kumar Sharma", "School of AI & DE, Room 216, S. S. Bhatnagar Block", "arnksh@iitrpr.ac.in"),
    ("Prof. Rajendra Munian", "Mechanical, Room 217, Satish Dhawan Block", "rajendra.munian@iitrpr.ac.in"),
]

W, H = Emu(12192000), Emu(6858000)          # 16:9


def text(slide, x, y, w, h, runs, size=14, color=INK, bold=False, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP,
         spacing=1.1):
    """runs: str or list of paragraphs; a paragraph is str or list of (text, {bold, color, size})."""
    tb = slide.shapes.add_textbox(x, y, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Emu(0)
    tf.margin_top = tf.margin_bottom = Emu(0)
    paras = runs if isinstance(runs, list) else [runs]
    for i, para in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        p.line_spacing = spacing
        parts = para if isinstance(para, list) else [(para, {})]
        for t, st in parts:
            r = p.add_run()
            r.text = t
            r.font.size = Pt(st.get("size", size))
            r.font.bold = st.get("bold", bold)
            r.font.color.rgb = st.get("color", color)
            r.font.name = "Calibri"
        if st_space := (0 if i == len(paras) - 1 else 6):
            p.space_after = Pt(st_space)
    return tb


def rect(slide, x, y, w, h, fill=PANEL, line=None, shape=MSO_SHAPE.ROUNDED_RECTANGLE):
    s = slide.shapes.add_shape(shape, x, y, w, h)
    s.fill.solid()
    s.fill.fore_color.rgb = fill
    if line:
        s.line.color.rgb = line
        s.line.width = Pt(1)
    else:
        s.line.fill.background()
    if shape == MSO_SHAPE.ROUNDED_RECTANGLE:
        s.adjustments[0] = 0.08
    s.shadow.inherit = False
    return s


def header(slide, title, sub):
    rect(slide, 0, 0, W, Emu(80000), fill=ACCENT, shape=MSO_SHAPE.RECTANGLE)
    text(slide, Emu(457200), Emu(260000), Emu(11277600), Emu(520000), title, size=28, bold=True)
    text(slide, Emu(457200), Emu(800000), Emu(11277600), Emu(330000), sub, size=14, color=MUTED)


def load_sim():
    try:
        with open(SIM) as f:
            return json.load(f)
    except Exception:
        return None


def build_deck(sim):
    prs = Presentation()
    prs.slide_width, prs.slide_height = W, H
    blank = prs.slide_layouts[6]

    # ---------------- slide 1: title, advisor, faculty list -------------------------
    s = prs.slides.add_slide(blank)
    header(s, "A.E.G.I.S.: an autonomous search-and-rescue rover",
           "Hears a distress call, searches the area, recognises the victim, takes it inside and brings it back "
           "by the shortest known path")
    text(s, Emu(457200), Emu(1330000), Emu(11277600), Emu(380000),
         [[("Selected faculty advisor:  ", {"bold": True}), ("Prof. ____________________  (one, from the list below)", {"color": MUTED})]],
         size=16)
    tx, ty, tw = Emu(457200), Emu(1880000), Emu(11277600)
    rows = len(FACULTY) + 1
    tbl = s.shapes.add_table(rows, 3, tx, ty, tw, Emu(470000 * rows)).table
    widths = [Emu(3100000), Emu(5100000), Emu(3077600)]
    for i, wdt in enumerate(widths):
        tbl.columns[i].width = wdt
    for c, h in enumerate(("Faculty instructor", "Department and room", "Email")):
        cell = tbl.cell(0, c)
        cell.text = h
        cell.fill.solid()
        cell.fill.fore_color.rgb = INK
        p = cell.text_frame.paragraphs[0]
        p.runs[0].font.size, p.runs[0].font.bold, p.runs[0].font.color.rgb = Pt(14), True, WHITE
    for r, row in enumerate(FACULTY, start=1):
        for c, v in enumerate(row):
            cell = tbl.cell(r, c)
            cell.text = v
            cell.fill.solid()
            cell.fill.fore_color.rgb = PANEL if r % 2 else WHITE
            run = cell.text_frame.paragraphs[0].runs[0]
            run.font.size, run.font.color.rgb = Pt(14), INK
    text(s, Emu(457200), Emu(6200000), Emu(11277600), Emu(330000),
         "Team of 6, School of AI & Data Engineering.  Budget: Rs 11,085 of Rs 15,000 (itemised list attached).  "
         "* Approval by e-mail or the signed form.", size=11, color=MUTED)

    # ---------------- slide 2: experiment setup -------------------------------------------
    s = prs.slides.add_slide(blank)
    header(s, "Possible experiment setup", "CAD model in Autodesk Fusion, how the system works, and what simulation shows")
    # left: CAD
    lx, ly, lw, lh = Emu(457200), Emu(1300000), Emu(5600000), Emu(3550000)
    rect(s, lx, ly, lw, lh, fill=PANEL, line=LINE)
    if RENDER and os.path.exists(RENDER):
        from PIL import Image
        iw, ih = Image.open(RENDER).size
        scale = min((lw - Emu(120000)) / iw, (lh - Emu(420000)) / ih)
        pw, ph = int(iw * scale), int(ih * scale)
        s.shapes.add_picture(RENDER, lx + (lw - pw) // 2, ly + Emu(60000), pw, ph)
        cap = "Mk2 in Autodesk Fusion: all purchased parts, wiring, intake door open."
    else:
        text(s, lx, ly + lh // 2 - Emu(300000), lw, Emu(600000),
             ["Fusion render goes here", [("Run Build_AEGIS_Rover in Fusion, then rebuild this deck", {"size": 12, "color": MUTED})]],
             size=18, bold=True, color=ACCENT, align=PP_ALIGN.CENTER)
        cap = ""
    if cap:
        text(s, lx + Emu(120000), ly + lh - Emu(330000), lw - Emu(240000), Emu(300000), cap, size=11, color=MUTED)

    # right: how it works
    rx, rw = Emu(6300000), Emu(5434800)
    text(s, rx, Emu(1300000), rw, Emu(330000), "How it works", size=18, bold=True)
    steps = [
        ("Listen. ", "Two microphones on the rover estimate where a scream comes from (GCC-PHAT); turning resolves front/back."),
        ("Search. ", "The laptop builds a map from the laser distance sensors and steers the rover to look at every unseen part of the table."),
        ("Recognise. ", "A YOLO detector on the laptop finds the victim token in the ESP32-CAM video and locates it on the table."),
        ("Rescue. ", "The rover drives the token into its intake bay; an IR sensor confirms it; the servo door closes."),
        ("Return. ", "Shortest path home over ground it has already driven. Wheels, safety stop and table-edge sensors stay on the robot, so a Wi-Fi drop cannot crash it."),
    ]
    paras = [[("%d. " % (i + 1), {"bold": True, "color": ACCENT}), (a, {"bold": True}), (b, {})] for i, (a, b) in enumerate(steps)]
    text(s, rx, Emu(1700000), rw, Emu(3200000), paras, size=13, spacing=1.05)

    # bottom: architecture strip
    by = Emu(5000000)
    boxes = [("Robot  (ESP32-S3)", "motors, sensors, mics, safety"), ("Wi-Fi link", "video, audio, map data"),
             ("Laptop  (edge brain)", "vision, scream detection, planning")]
    bw, gap = Emu(2600000), Emu(450000)
    x = Emu(457200)
    for i, (t1, t2) in enumerate(boxes):
        rect(s, x, by, bw, Emu(620000), fill=INK if i != 1 else PANEL, line=None if i != 1 else LINE)
        text(s, x, by + Emu(90000), bw, Emu(500000),
             [[(t1, {"bold": True, "color": WHITE if i != 1 else INK})], [(t2, {"size": 11, "color": WHITE if i != 1 else MUTED})]],
             size=13, align=PP_ALIGN.CENTER)
        if i < 2:
            a = s.shapes.add_shape(MSO_SHAPE.LEFT_RIGHT_ARROW, x + bw + Emu(60000), by + Emu(220000), gap - Emu(120000), Emu(180000))
            a.fill.solid()
            a.fill.fore_color.rgb = ACCENT
            a.line.fill.background()
        x += bw + gap

    # bottom right: results
    rx2 = Emu(457200) + 3 * bw + 2 * gap + Emu(250000)
    rw2 = W - rx2 - Emu(457200)
    rect(s, rx2, by, rw2, Emu(620000), fill=PANEL, line=LINE)
    if sim:
        res = [[("3D simulation, %d tables each" % sim["n_per_demo"], {"bold": True})],
               [("Hidden: %d%% rescued  |  Scream: %d%%" % (sim["demo1_success_pct"], sim["demo2_success_pct"]), {"size": 11})]]
    else:
        res = [[("Simulation results", {"bold": True})], [("pending", {"size": 11, "color": MUTED})]]
    text(s, rx2 + Emu(100000), by + Emu(90000), rw2 - Emu(200000), Emu(500000), res, size=13, align=PP_ALIGN.CENTER)
    text(s, Emu(457200), Emu(5750000), Emu(11277600), Emu(700000),
         [[("Demo plan: ", {"bold": True}),
           ("run 1, the token is hidden behind books on the table and the rover must inspect the whole table; "
            "run 2, a speaker at the token plays a scream and the rover homes in on the sound.", {})]],
         size=12, color=MUTED)
    # ---------------- slides 3+: the full story -------------------------------------------------
    import csv
    import sys
    sys.path.insert(0, HERE)
    from deck_story import add_story
    TS = os.path.join(PROJ, "team_share")
    A = os.path.join(TS, "deck_assets")
    R = os.path.join(TS, "simulation3d", "results")

    def example(name, seed):
        with open(os.path.join(R, name + ".csv")) as f:
            r = {x["seed"]: x for x in csv.DictReader(f)}[seed]
        return {k: float(r[k]) for k in ("first_seen", "captured", "t_end")}
    if sim and os.path.exists(os.path.join(R, "example_demo1.gif")):
        paths = dict(render_open=os.path.join(A, "render_iso_door_open.png"), render_closed=os.path.join(A, "render_iso_door_closed.png"),
                     render_wiring=os.path.join(A, "render_wiring_top.png"), wiring_guide=os.path.join(A, "wiring_guide_crop.png"),
                     gif1=os.path.join(A, "demo1_poster.gif"), gif2=os.path.join(A, "demo2_poster.gif"),
                     ai_sees=os.path.join(A, "ai_sees.png"), summary=os.path.join(R, "summary.png"),
                     ex1=example("final_demo1", "3005"), ex2=example("final_demo2", "4036"))
        add_story(prs, paths, sim)

    path = os.path.join(OUT, "ProjectIdea_AEGIS_Thursday_1.pptx")
    prs.save(path)
    return path


def update_form(sim):
    from docx import Document
    f = os.path.join(OUT, "ApprovalForm_AEGIS_Thursday_1.docx")
    d = Document(f)
    sim_txt = ("In a 3D simulation of %d random table layouts per case (camera, sound, Wi-Fi loss included), the rover "
               "rescued the hidden token in %d%% of runs and homed in on the scream in %d%%."
               % (sim["n_per_demo"], sim["demo1_success_pct"], sim["demo2_success_pct"])) if sim else \
        "Simulation results to be added."
    new = {
        "Search & Rescue Paradigm": ("Search & Rescue Prototype: ",
                                     "A small autonomous rover that hears a distress call, searches a table-top arena, recognises a victim token, "
                                     "takes it into its intake bay and returns by the shortest path it has already driven. A low-cost prototype of the "
                                     "search-and-rescue loop, not a field robot."),
        "Acoustic Localization": ("Acoustic Localization: ",
                                  "Two INMP441 microphones at 48 kHz; GCC-PHAT gives the bearing on the robot, and readings taken at different "
                                  "headings resolve the front/back ambiguity. A scream detector runs on the laptop."),
        "Navigation Policy": ("Edge Brain and Validation: ",
                              "The laptop runs the YOLO victim detector, mapping, coverage search and path planning over Wi-Fi. " + sim_txt),
        "Deterministic Safety": ("On-board Safety: ",
                                 "A control-barrier-function speed limit from three VL53L0X laser sensors, IR table-edge sensors, stall escape and a "
                                 "stop-on-link-loss watchdog run on the ESP32-S3 itself."),
        "Mechanical CAD": ("Mechanical CAD: ",
                           "Parametric Mk2 assembly in Autodesk Fusion: raised 4WD chassis, intake bay with a servo-driven door, camera mast, "
                           "laser-cut acrylic decks and 3D-printed PLA+ parts, with joints and limits."),
        "Shortest Path": ("Return Path: ",
                          "Wheel encoders and an MPU-6050 gyro give odometry; A* plans the shortest route home over cells the rover has "
                          "already seen free."),
        "Budget Compliance": ("Budget Compliance: ",
                              "Itemised Bill of Materials: Rs 11,085 of the Rs 15,000 limit (18 of 19 items linked to Robu.in)."),
    }
    for p in d.paragraphs:
        if len(p.runs) >= 2:
            for key, (label, body) in new.items():
                if p.runs[0].text.startswith(key) or p.text.startswith(key):
                    p.runs[0].text = label
                    p.runs[1].text = body
                    for r in p.runs[2:]:
                        r.text = ""
    for p in d.paragraphs:
        for r in p.runs:
            if "Autonomous Edge-Guided Identification & Salvage Rover" in r.text:
                r.text = r.text.replace("A.E.G.I.S. — Autonomous Edge-Guided Identification & Salvage Rover",
                                        "A.E.G.I.S.: Autonomous Search-and-Rescue Rover")
    d.save(f)
    return f


if __name__ == "__main__":
    sim = load_sim()
    print(build_deck(sim))
    print(update_form(sim))
