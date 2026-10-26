"""
A.E.G.I.S. assembly guide: exploded-view steps rendered from the Fusion STEP model.

    ~/aegis_venv/bin/python team_share/tools/assembly_guide.py   ->  team_share/AEGIS_Assembly_Guide.pdf

Parts are rendered off-screen with MuJoCo (grey, the parts added in a step in blue), outlined from
the segmentation and depth buffers, then laid out with numbered callouts and a parts box per step.
Re-run after rebuilding the Fusion model; the geometry comes from Build_AEGIS_Rover/output_detailed.
"""
import math
import os
import tempfile
import textwrap

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import trimesh
from matplotlib.backends.backend_pdf import PdfPages
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
TS = os.path.join(HERE, "..")
STEP = os.path.join(TS, "Build_AEGIS_Rover", "output_detailed", "AEGIS_Rover_Detailed.step")
OUT = os.path.join(TS, "AEGIS_Assembly_Guide.pdf")
W, H = 1500, 1000
SKIP = ("Wiring_Harness", "Victim_Token_40mm")
GREY, NEW, DECK = (0.86, 0.86, 0.84), (0.42, 0.66, 0.93), (0.80, 0.86, 0.90)


def load_parts():
    s = trimesh.load(STEP)
    parts = {}
    for node in s.graph.nodes_geometry:
        T, g = s.graph[node]
        if node.split(":")[0] in SKIP:
            continue
        m = s.geometry[g].copy()
        m.apply_transform(T)
        m.fix_normals()                                  # STEP faces can come out inward; shade them right
        parts[node] = m
    return parts


def build_model(parts, tmp):
    assets = []
    for i, (name, m) in enumerate(parts.items()):
        f = os.path.join(tmp, "p%03d.stl" % i)
        m.export(f)
        assets.append((name, f))
    xml = ['<mujoco><compiler angle="radian"/><visual><global offwidth="%d" offheight="%d" fovy="30"/>' % (W, H),
           '<headlight ambient="0.45 0.45 0.45" diffuse="0.55 0.55 0.55" specular="0.05 0.05 0.05"/>',
           '<quality shadowsize="0"/></visual><asset>']
    xml += ['<mesh name="m%d" file="%s"/>' % (i, f) for i, (_, f) in enumerate(assets)]
    xml.append('</asset><worldbody><camera name="view" pos="0 0 1"/>')
    xml.append('<light directional="true" dir="-0.4 -0.3 -1" diffuse="0.35 0.35 0.35" castshadow="false"/>')
    xml += ['<geom name="g%d" type="mesh" mesh="m%d" contype="0" conaffinity="0" rgba="0.8 0.8 0.8 1"/>' % (i, i)
            for i in range(len(assets))]
    xml.append("</worldbody></mujoco>")
    model = mujoco.MjModel.from_xml_string("".join(xml))
    names = [n for n, _ in assets]
    return model, names


class Renderer:
    def __init__(self, parts):
        self.tmp = tempfile.mkdtemp()
        self.parts = parts
        self.model, self.names = build_model(parts, self.tmp)
        self.data = mujoco.MjData(self.model)
        self.base = self.model.geom_pos.copy()
        self.r = mujoco.Renderer(self.model, H, W)
        self.cent = {n: parts[n].bounds.mean(axis=0) for n in self.names}

    def set_view(self, az, el, dist, look, fovy=30.0):
        az, el = math.radians(az), math.radians(el)
        look = np.array(look, float)
        pos = look + dist * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        z = (pos - look) / np.linalg.norm(pos - look)
        x = np.cross([0, 0, 1.0], z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, np.column_stack([x, y, z]).ravel())
        self.model.cam_pos[0], self.model.cam_quat[0], self.model.cam_fovy[0] = pos, q, fovy
        self.cam = (pos, x, y, z, (H / 2) / math.tan(math.radians(fovy) / 2))

    def project(self, p):
        pos, x, y, z, f = self.cam
        d = np.asarray(p) - pos
        depth = -d @ z
        return W / 2 + f * (d @ x) / depth, H / 2 - f * (d @ y) / depth

    def render(self, show, new, explode):
        for i, n in enumerate(self.names):
            vis = any(n.startswith(s) for s in show)
            off = np.array(explode.get(n, explode.get(n.split(":")[0], (0, 0, 0))), float)
            self.model.geom_pos[i] = self.base[i] + off
            col = NEW if any(n.startswith(s) for s in new) else (DECK if "Deck" in n else GREY)
            self.model.geom_rgba[i] = (*col, 1.0 if vis else 0.0)
        mujoco.mj_forward(self.model, self.data)
        self.r.update_scene(self.data, camera="view")
        rgb = self.r.render().copy()
        self.r.enable_segmentation_rendering()
        seg = self.r.render()[:, :, 0].copy()
        self.r.disable_segmentation_rendering()
        self.r.enable_depth_rendering()
        dep = self.r.render().copy()
        self.r.disable_depth_rendering()
        hidden = [i for i, n in enumerate(self.names) if not any(n.startswith(s) for s in show)]
        seg[np.isin(seg, hidden)] = -1
        edge = (seg != np.roll(seg, 1, 0)) | (seg != np.roll(seg, 1, 1))                    # part outlines
        d2 = np.abs(ndimage.laplace(np.where(seg >= 0, dep, dep.max())))
        edge |= (d2 > 0.004) & (seg >= 0)                     # depth steps (a tilted flat face has none)
        lum = rgb.astype(float).mean(axis=2)
        crease = np.hypot(ndimage.sobel(lum, 0), ndimage.sobel(lum, 1)) > 130
        edge |= crease & (seg >= 0)
        edge = ndimage.binary_dilation(edge, iterations=1)
        out = rgb.astype(float) / 255
        out[seg < 0] = 1.0
        out = 0.25 + 0.75 * out                                                            # flat, light shading
        out[seg < 0] = 1.0
        out[edge] = 0.08
        return np.clip(out, 0, 1), {n: off for n, off in ((n, self.model.geom_pos[i] - self.base[i])
                                                           for i, n in enumerate(self.names))}


# ---------------------------------------------------------------- steps
M = "N20_Gearmotor_150to1"
HG = "Motor_Hanger_PLA"
WH = "Wheel_44mm"
CL = ("IR_Cliff_FL", "IR_Cliff_FR", "IR_Cliff_Rear", "IR_Cliff_RL", "IR_Cliff_RR")
LOW = ("Lower_Deck_Acrylic",)
BAY = ("Intake_Shroud_PLA", "Servo_MG946R", "Intake_Door_PLA", "IR_Capture_Sensor")
LOWER_EL = ("Battery_3S1P_18650", "TB6612FNG_A", "TB6612FNG_B", "XL4015_Logic_5V", "XL4015_Servo_5V",
            "PCF8574_IO_Expander", "Fuse_Holder_Board", "VL53L0X_Front")
UPPER = ("Upper_Deck_Acrylic", "Power_Switch_Rocker", "ESP32_Carrier_Perfboard", "ESP32_S3_DevKitC", "MPU6050_IMU",
         "INMP441_Mic_L", "INMP441_Mic_R", "VL53L0X_Left", "VL53L0X_Right")
MAST = ("Camera_Mast_PLA", "ESP32_CAM_OV2640")
PIL = ("M3_Pillar_30mm",)


def side_out(n, d):
    """explode a corner part outward along y, by its own side"""
    return lambda parts: {k: (0, d * np.sign(parts[k].bounds.mean(axis=0)[1]), 0) for k in parts if k.startswith(n)}


STEPS = [
    dict(title="Motors into their mounts",
         show=(M, HG), new=(M, HG), explode=lambda p: {**{k: (0, 0.025 * np.sign(p[k].bounds.mean(0)[1]), 0) for k in p if k.startswith(M)}},
         view=(-60, 25, 0.42, (-0.03, 0, 0.03)), callouts=[(HG + ":1", 1), (M + ":1", 2)],
         parts=[(1, "Motor mount (printed, black PLA+)", "x 4"), (2, "N20 gearmotor with encoder (#5)", "x 4"),
                ("", "Cable tie 100 mm (#36)", "x 4")],
         text=["Slide each motor into a mount, gearbox end first, encoder cable out of the back.",
               "Loop a cable tie through the two channels in the mount, under the motor, and pull it snug.",
               "Two mounts face left and two face right: the shaft must point outward."]),
    dict(title="Mounts under the lower deck",
         show=LOW + (M, HG), new=(M, HG), explode=lambda p: {k: (0, 0, -0.035) for k in p if k.startswith((M, HG))},
         view=(-55, -28, 0.55, (0.01, 0, 0.04)), callouts=[(LOW[0] + ":1", 3), (HG + ":2", 1)],
         parts=[(3, "Lower deck (laser-cut acrylic, engraved face up)", "x 1"), (1, "Motor mount with motor", "x 4"),
                ("", "M3 x 10 screw from the top (#15)", "x 8")],
         text=["Peel the deck film. Engraved marks face up.",
               "Each mount sits under its two holes; screw M3 x 10 down from the top into the 2.5 mm holes.",
               "Do not over-tighten: the screws cut their own thread in the PLA."]),
    dict(title="Wheels",
         show=LOW + (M, HG, WH), new=(WH,), explode=lambda p: {k: (0, 0.04 * np.sign(p[k].bounds.mean(0)[1]), 0) for k in p if k.startswith(WH)},
         view=(-60, 38, 0.62, (0.01, 0, 0.03)), callouts=[(WH + ":1", 4), (WH + ":3", 4)],
         parts=[(4, "44 mm N20 wheel (#13)", "x 4")],
         text=["Press each wheel onto its motor's D-shaft, flat side matching, until it stops.",
               "Spin each wheel by hand: nothing should rub on the mount or the deck."]),
    dict(title="Table-edge sensors under the deck",
         show=LOW + (M, HG, WH) + CL, new=CL, explode=lambda p: {k: (0, 0, -0.03) for k in p if k.startswith(CL)},
         view=(-125, -38, 0.6, (0.01, 0, 0.04)), callouts=[("IR_Cliff_FL:1", 5), ("IR_Cliff_RR:1", 5)],
         parts=[(5, "TCRT5000 edge sensor (#18)", "x 5"), ("", "Foam tape (#38), paper template", "")],
         text=["Tape the printed underside template to the deck and stick each sensor on its outline,",
               "lens pointing down: front-left, front-right, rear, and both rear corners.",
               "Lens 3-8 mm above the table. Run the wires up through the harness slot."]),
    dict(title="Intake bay: shroud, servo, door, bay sensor",
         show=LOW + (M, HG, WH) + CL + BAY, new=BAY,
         explode=lambda p: {"Intake_Shroud_PLA:1": (0.0, 0, -0.04), "Servo_MG946R:1": (0, 0, 0.05),
                            "Intake_Door_PLA:1": (0.05, 0.0, -0.02), "IR_Capture_Sensor:1": (-0.03, 0, -0.04)},
         view=(-35, 22, 0.55, (0.08, 0, 0.04)), callouts=[("Intake_Shroud_PLA:1", 6), ("Servo_MG946R:1", 7),
                                                           ("Intake_Door_PLA:1", 8), ("IR_Capture_Sensor:1", 9)],
         parts=[(6, "Intake shroud (printed)", "x 1"), (7, "MG946R servo (#9)", "x 1"), (8, "Intake door (printed)", "x 1"),
                (9, "IR obstacle sensor (#19)", "x 1"), ("", "M3 x 10 screw + nut", "x 6")],
         text=["Bolt the shroud flanges under the deck (4 x M3). Drop the servo through its pocket, spline down,",
               "and screw its ears to the deck (2 x M3). Warm the spline and press the door hub onto it",
               "with the servo at the CLOSED position. Tape the bay sensor to the boss on the back wall."]),
    dict(title="Lower-deck electronics",
         show=LOW + (M, HG, WH) + CL + BAY + LOWER_EL, new=LOWER_EL,
         explode=lambda p: {k: (0, 0, 0.045) for k in p if k.startswith(LOWER_EL)},
         view=(-60, 42, 0.62, (0.0, 0, 0.06)),
         callouts=[("Battery_3S1P_18650:1", 10), ("TB6612FNG_A:1", 11), ("XL4015_Logic_5V:1", 12),
                   ("PCF8574_IO_Expander:1", 13), ("Fuse_Holder_Board:1", 14), ("VL53L0X_Front:1", 15)],
         parts=[(10, "3S 18650 pack (#10), cable-tied down", "x 1"), (11, "TB6612FNG driver (#6)", "x 2"),
                (12, "XL4015 buck, set to 5.0 V first (#12)", "x 2"), (13, "PCF8574 I/O expander (#17)", "x 1"),
                (14, "Fuse board: holder + 6.3 A fuse (#22, #23)", "x 1"), (15, "VL53L0X front + L-bracket", "x 1")],
         text=["Stick each board on its engraved outline with foam tape; strap the battery with two cable ties.",
               "Set both XL4015 outputs to 5.0 V with a multimeter BEFORE connecting anything to them.",
               "Wire this deck now, following AEGIS_Wiring_Guide.pdf; leave the battery unplugged."]),
    dict(title="Pillars",
         show=LOW + (M, HG, WH) + CL + BAY + LOWER_EL + PIL, new=PIL,
         explode=lambda p: {k: (0, 0, 0.04) for k in p if k.startswith(PIL)},
         view=(-60, 30, 0.62, (0.0, 0, 0.06)), callouts=[("M3_Pillar_30mm:1", 16)],
         parts=[(16, "M3 pillar stack: 20 mm F-F + 10 mm M-F (#15)", "x 4")],
         text=["Screw a 10 mm M-F pillar up through each corner hole of the lower deck,",
               "then a 20 mm F-F pillar on top: 30 mm between the decks."]),
    dict(title="Upper deck",
         show=LOW + (M, HG, WH) + CL + BAY + LOWER_EL + PIL + UPPER, new=UPPER,
         explode=lambda p: {**{k: (0, 0, 0.06) for k in p if k.startswith(UPPER)},
                            **{k: (0, 0, 0.10) for k in p if k.startswith(UPPER[1:])}},
         view=(-55, 30, 0.75, (0.0, 0, 0.1)),
         callouts=[("Upper_Deck_Acrylic:1", 17), ("Power_Switch_Rocker:1", 18), ("ESP32_S3_DevKitC:1", 19),
                   ("MPU6050_IMU:1", 20), ("INMP441_Mic_L:1", 21), ("VL53L0X_Left:1", 22)],
         parts=[(17, "Upper deck (acrylic)", "x 1"), (18, "Power switch, snaps into its cut-out (#21)", "x 1"),
                (19, "ESP32-S3 on its soldered carrier board (#1, #31, #32)", "x 1"),
                (20, "MPU-6050 IMU (#8)", "x 1"), (21, "INMP441 microphone, 100 mm apart (#4)", "x 2"),
                (22, "VL53L0X side sensor + L-bracket (#7)", "x 2"), ("", "M3 x 6 screw", "x 4")],
         text=["Feed the harness through the slot, set the deck on the pillars, screw down (4 x M3 x 6).",
               "Press the switch in from the top. Tape the carrier, IMU, mics and side sensors on their outlines;",
               "the IMU must sit flat and square to the deck edge. Then wire the upper deck."]),
    dict(title="Camera mast",
         show=LOW + (M, HG, WH) + CL + BAY + LOWER_EL + PIL + UPPER + MAST, new=MAST,
         explode=lambda p: {"Camera_Mast_PLA:1": (0, 0, 0.05), "ESP32_CAM_OV2640:1": (0.05, 0, 0.07)},
         view=(-45, 22, 0.72, (0.03, 0, 0.11)), callouts=[("Camera_Mast_PLA:1", 23), ("ESP32_CAM_OV2640:1", 24)],
         parts=[(23, "Camera mast (printed)", "x 1"), (24, "ESP32-CAM with OV2640 (#2)", "x 1"),
                ("", "M3 x 10 screw + nut, foam tape", "x 2")],
         text=["Bolt the mast to the upper deck (2 x M3 up from below, nuts on the base).",
               "Stick the ESP32-CAM on the sloped pad with two layers of foam tape, lens forward and up,",
               "SD slot clear. It looks 30 degrees down at the table."]),
]


def crop(img, pad=40):
    """Trim the white margin so the drawing fills its frame; returns the offset for projected points."""
    ys, xs = np.where(img.min(axis=2) < 0.97)
    y0, y1 = max(ys.min() - pad, 0), min(ys.max() + pad, img.shape[0])
    x0, x1 = max(xs.min() - pad, 0), min(xs.max() + pad, img.shape[1])
    return img[y0:y1, x0:x1], (x0, y0)


def page(pdf, rend, i, st):
    parts = rend.parts
    ex = st["explode"](parts) if callable(st["explode"]) else st["explode"]
    rend.set_view(*st["view"])
    img, offs = rend.render(st["show"], st["new"], ex)
    img, (cx0, cy0) = crop(img)
    fig = plt.figure(figsize=(11.69, 8.27))
    fig.text(0.04, 0.93, "%d" % (i + 1), fontsize=40, weight="bold", va="center")
    fig.text(0.11, 0.935, st["title"], fontsize=20, weight="bold", va="center")
    fig.add_artist(plt.Line2D([0.04, 0.96], [0.895, 0.895], color="k", lw=1.2))
    ax = fig.add_axes([0.02, 0.17, 0.66, 0.71])
    ax.imshow(img)
    ax.axis("off")
    for n, c in rend.cent.items():                              # assembly lines for exploded parts
        if any(n.startswith(s) for s in st["show"]) and np.linalg.norm(offs[n]) > 1e-4:
            a, b = rend.project(c), rend.project(c + offs[n])
            ax.plot([a[0] - cx0, b[0] - cx0], [a[1] - cy0, b[1] - cy0], ls=(0, (4, 3)), color="0.35", lw=0.8)
    used = []
    for name, num in st["callouts"]:
        u, v = rend.project(rend.cent[name] + offs[name])
        u, v = u - cx0, v - cy0
        ih, iw = img.shape[:2]
        bu = u + (0.09 * iw if u > iw * 0.5 else -0.09 * iw)
        bv = v - 0.09 * ih
        for pu, pv in used:                                     # keep balloons apart
            if abs(pu - bu) < 0.06 * iw and abs(pv - bv) < 0.06 * iw:
                bv -= 0.07 * iw
        bu = min(max(bu, 0.03 * iw), 0.97 * iw)                 # keep balloons on the drawing
        if bv < 0.04 * ih:
            bv = v + 0.09 * ih
        while any(abs(pu - bu) < 0.05 * iw and abs(pv - bv) < 0.05 * iw for pu, pv in used):
            bu += 0.06 * iw if bu < 0.9 * iw else -0.06 * iw
        used.append((bu, bv))
        ax.plot([u, bu], [v, bv], color="k", lw=0.8)
        ax.plot(u, v, "o", color="k", ms=2.5)
        ax.add_patch(plt.Circle((bu, bv), 0.022 * iw, fc="white", ec="k", lw=1.2))
        ax.text(bu, bv, str(num), ha="center", va="center", fontsize=11, weight="bold")
    ax.set_xlim(0, img.shape[1])
    ax.set_ylim(img.shape[0], 0)
    box = fig.add_axes([0.70, 0.17, 0.27, 0.71])
    box.axis("off")
    box.add_patch(plt.Rectangle((0, 0), 1, 1, fc="0.97", ec="k", lw=1, transform=box.transAxes))
    box.text(0.05, 0.96, "PARTS", fontsize=11, weight="bold", va="top")
    y = 0.89
    for num, name, qty in st["parts"]:
        if num != "":
            box.add_patch(plt.Circle((0.09, y - 0.012), 0.035, fc="white", ec="k", lw=1, transform=box.transAxes))
            box.text(0.09, y - 0.012, str(num), ha="center", va="center", fontsize=8, weight="bold")
        lines = textwrap.wrap(name, 30)
        box.text(0.17, y, "\n".join(lines), fontsize=8.5, va="top", linespacing=1.3)
        box.text(0.95, y, qty, fontsize=9, va="top", ha="right", weight="bold")
        y -= 0.045 + 0.033 * len(lines)
    for j, line in enumerate(st["text"]):
        fig.text(0.04, 0.125 - 0.032 * j, line, fontsize=10.5)
    fig.text(0.96, 0.02, "A.E.G.I.S. Mk2 assembly  |  step %d of %d  |  blue = parts added in this step" %
             (i + 1, len(STEPS)), ha="right", fontsize=8, color="0.4")
    pdf.savefig(fig)
    plt.close(fig)


def cover(pdf, rend):
    rend.set_view(-50, 25, 0.68, (0.02, 0, 0.08))
    every = tuple(sorted({n.split(":")[0] for n in rend.names}))
    img, _ = rend.render(every, (), {})
    img, _ = crop(img)
    fig = plt.figure(figsize=(11.69, 8.27))
    fig.text(0.05, 0.9, "A.E.G.I.S. Mk2", fontsize=34, weight="bold")
    fig.text(0.05, 0.85, "Assembly guide", fontsize=18)
    ax = fig.add_axes([0.03, 0.08, 0.58, 0.72])
    ax.imshow(img)
    ax.axis("off")
    notes = ["BEFORE YOU START", "",
             "Tools: Phillips screwdriver, side cutters, wire strippers,",
             "soldering iron, multimeter, 2.5 mm hex key (optional).", "",
             "Print first (FABRICATION_GUIDE.md):",
             "  laser-cut decks, printed parts, carrier board soldered,",
             "  headers soldered on every module.", "",
             "Measure each board you received with calipers and check",
             "it fits its engraved outline.", "",
             "Battery: keep it unplugged until step %d is wired and" % 6,
             "both regulators read 5.0 V.", "",
             "Wiring: AEGIS_Wiring_Guide.pdf (pins, colours, gauges).",
             "First power-up and tests: edge/README.md."]
    for j, t in enumerate(notes):
        fig.text(0.63, 0.78 - 0.034 * j, t, fontsize=9.5, weight="bold" if j == 0 else "normal")
    pdf.savefig(fig)
    plt.close(fig)


def main():
    rend = Renderer(load_parts())
    with PdfPages(OUT) as pdf:
        cover(pdf, rend)
        for i, st in enumerate(STEPS):
            page(pdf, rend, i, st)
    print(OUT)


if __name__ == "__main__":
    main()
