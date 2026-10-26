"""
A.E.G.I.S. Mk2 assembly guide: one action per step, rendered from the Fusion STEP model.

    ~/aegis_venv/bin/python team_share/tools/assembly_guide.py   ->  team_share/AEGIS_Assembly_Guide.pdf

Every step shows the assembly so far in grey and the parts added in that step in blue, exploded along
their assembly direction with dashed lines. Screws, nuts and pillar studs are drawn at the real hole
positions (Build_AEGIS_Rover.py) and use only what the M3 kit (#15) contains: 20 x M3x6 screws, 20 nuts,
male-female and female-female pillars. Renders are MuJoCo off-screen at 2x and downsampled; outlines
come from the segmentation and depth buffers. Pages are HTML printed to PDF by headless Chrome.
Re-run after rebuilding the Fusion model.
"""
import base64
import html
import io
import math
import os
import subprocess
import sys
import tempfile

import mujoco
import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
TS = os.path.abspath(os.path.join(HERE, ".."))
STEP = os.path.join(TS, "Build_AEGIS_Rover", "output_detailed", "AEGIS_Rover_Detailed.step")
OUT = os.path.join(TS, "AEGIS_Assembly_Guide.pdf")
PREVIEW = os.path.join(TS, "deck_assets", "guides", "assembly")
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
FONT = "/System/Library/Fonts/Helvetica.ttc"
SS = 2                                   # supersampling factor
W, H = 1600, 1000                        # final image size
SKIP = ("Wiring_Harness", "Victim_Token_40mm")
NEW_C, OLD_C, DECK_C, HW_C = (0.34, 0.60, 0.92), (0.90, 0.90, 0.88), (0.86, 0.91, 0.95), (0.70, 0.72, 0.76)
NAVY, ORANGE = "#1E2A44", "#EA6C1C"
MM = 0.001
LZ0, LZ1, UZ0, UZ1 = 50.0, 53.0, 83.0, 86.0     # deck faces, mm


# ------------------------------------------------------------------ fasteners (real hole positions)
def screw(length, head_up=True):
    """M3 pan-head screw on z: head z 0..+3 with the shank below (head_up), or mirrored."""
    head = trimesh.creation.cylinder(radius=2.75 * MM, height=2.4 * MM, sections=28)
    head.apply_translation((0, 0, 1.2 * MM if head_up else -1.2 * MM))
    shank = trimesh.creation.cylinder(radius=1.5 * MM, height=length * MM, sections=18)
    shank.apply_translation((0, 0, -length / 2 * MM if head_up else length / 2 * MM))
    return trimesh.util.concatenate([head, shank])


def nut():
    return trimesh.creation.cylinder(radius=3.17 * MM, height=2.4 * MM, sections=6)


def fasteners():
    f = {}

    def put(name, mesh, x, y, z):
        m = mesh.copy()
        m.apply_translation((x * MM, y * MM, z * MM))
        f[name] = m
    k = 0
    for ax in (20.0, -80.0):                                    # motor mounts: 8 x M3x6 into the PLA bridge
        for sx in (-6.5, 6.5):
            for sy in (1, -1):
                k += 1
                put("Screw_Mount:%d" % k, screw(6), ax + sx, sy * 34.0, LZ1)
    k = 0
    for x in (55.0, 85.0):                                      # shroud flanges: 4 x M3x6 into 2.5 mm pilots
        for sy in (1, -1):
            k += 1
            put("Screw_Shroud:%d" % k, screw(6), x, sy * 38.5, LZ1)
    for k, (x, y) in enumerate(((-100.0, 52.0), (-100.0, -52.0), (85.0, 52.0), (85.0, -52.0)), 1):
        put("Nut_Pillar:%d" % k, nut(), x, y, LZ0 - 1.2)                    # under the lower deck, on the pillar stud
        put("Screw_PillarTop:%d" % k, screw(6), x, y, UZ1)                   # down into the pillar
    tie = []                                                    # cable tie round motor 1: bridge tunnel -> cheeks -> notches
    for (x0, x1, z0, z1) in ((-10.4, 10.4, 28.8, 30.2), (-10.4, -9.2, 15.4, 30.2), (9.2, 10.4, 15.4, 30.2),
                             (-10.4, 10.4, 15.4, 16.8)):
        b = trimesh.creation.box(extents=((x1 - x0) * MM, 2.5 * MM, (z1 - z0) * MM))
        b.apply_translation(((20.0 + (x0 + x1) / 2) * MM, 42.0 * MM, (z0 + z1) / 2 * MM))
        tie.append(b)
    f["Cable_Tie:1"] = trimesh.boolean.union(tie)              # one solid: MuJoCo misplaces self-overlapping meshes
    for k, sy in enumerate((1, -1), 1):                         # mast: 2 x M3x6 up through the deck into the base
        put("Screw_Mast:%d" % k, screw(6, head_up=False), 64.0, sy * 11.0, UZ0)
    return f


def load_parts():
    s = trimesh.load(STEP)
    parts = {}
    for node in s.graph.nodes_geometry:
        T, g = s.graph[node]
        if node.split(":")[0] in SKIP:
            continue
        m = s.geometry[g].copy()
        m.apply_transform(T)
        m.fix_normals()
        parts[node] = m
    parts.update(fasteners())
    return parts


# ------------------------------------------------------------------ renderer
class Renderer:
    def __init__(self, parts):
        self.tmp = tempfile.mkdtemp()
        self.parts = parts
        self.names = list(parts)
        xml = ['<mujoco><compiler angle="radian"/><visual><global offwidth="%d" offheight="%d"/>' % (W * SS, H * SS),
               '<headlight ambient="0.40 0.40 0.40" diffuse="0.32 0.32 0.32" specular="0.03 0.03 0.03"/>',
               '<quality shadowsize="0" offsamples="8"/></visual><asset>']
        for i, n in enumerate(self.names):
            p = os.path.join(self.tmp, "p%03d.stl" % i)
            parts[n].export(p)
            xml.append('<mesh name="m%d" file="%s"/>' % (i, p))
        xml.append('</asset><worldbody><camera name="view" pos="0 0 1"/>')
        xml.append('<light directional="true" dir="-0.5 -0.4 -1" diffuse="0.30 0.30 0.30" castshadow="false"/>')
        xml.append('<light directional="true" dir="0.6 0.3 0.6" diffuse="0.15 0.15 0.15" castshadow="false"/>')
        xml += ['<geom type="mesh" mesh="m%d" contype="0" conaffinity="0"/>' % i for i in range(len(self.names))]
        xml.append("</worldbody></mujoco>")
        self.model = mujoco.MjModel.from_xml_string("".join(xml))
        self.data = mujoco.MjData(self.model)
        self.base = self.model.geom_pos.copy()
        self.r = mujoco.Renderer(self.model, H * SS, W * SS)
        self.cent = {n: parts[n].bounds.mean(axis=0) for n in self.names}
        self.edges = {}                                             # sharp (feature) edges of each part, world coords
        for n in self.names:
            m = parts[n]
            sharp = m.face_adjacency_edges[m.face_adjacency_angles > np.radians(28)]
            # mesh borders (open edges) are outlines too
            e_sorted = np.sort(m.edges_sorted, axis=1)
            u, cnt = np.unique(e_sorted, axis=0, return_counts=True)
            border = u[cnt == 1]
            self.edges[n] = m.vertices[np.vstack([sharp, border]) if len(border) else sharp]

    @staticmethod
    def font(size):
        return ImageFont.truetype(FONT, size)

    def set_view(self, az, el, look, dist, fovy=26.0):
        az, el = math.radians(az), math.radians(el)
        look = np.asarray(look, float)
        pos = look + dist * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        z = (pos - look) / np.linalg.norm(pos - look)
        up = np.array([0, 0, 1.0]) if abs(z[2]) < 0.99 else np.array([1.0, 0, 0])
        x = np.cross(up, z)
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
        return np.array([W / 2 + f * (d @ x) / depth, H / 2 - f * (d @ y) / depth])

    def frame(self, az, el, pts, fill=0.80):
        """Look-at and distance so the points fill `fill` of the frame (and stay centred)."""
        pts = np.asarray(pts)
        look = (pts.min(0) + pts.max(0)) / 2
        lo, hi = 0.02, 6.0
        for _ in range(40):
            d = (lo + hi) / 2
            self.set_view(az, el, look, d)
            uv = np.array([self.project(p) for p in pts])
            span = max((uv[:, 0].max() - uv[:, 0].min()) / W, (uv[:, 1].max() - uv[:, 1].min()) / H)
            lo, hi = (d, hi) if span > fill else (lo, d)
        self.set_view(az, el, look, hi)
        uv = np.array([self.project(p) for p in pts])          # re-centre on the projected box
        c = (uv.min(0) + uv.max(0)) / 2 - np.array([W / 2, H / 2])
        pos, x, y, z, f = self.cam
        depth = hi
        look = look + x * c[0] * depth / f - y * c[1] * depth / f
        self.set_view(az, el, look, hi)

    def corners(self, n, off=0):
        b = self.parts[n].bounds + off
        return np.array([[x, y, z] for x in b[:, 0] for y in b[:, 1] for z in b[:, 2]])

    def render(self, show, new, offs):
        for i, n in enumerate(self.names):
            vis = n in show
            self.model.geom_pos[i] = self.base[i] + (offs.get(n, 0) if vis else np.array([0, 0, 50.0]))
            if n.startswith("Cable_Tie"):
                col = (0.92, 0.42, 0.11)
            elif n in new:
                col = NEW_C
            elif "Deck" in n:
                col = DECK_C
            elif n.startswith(("Screw_", "Nut_")):
                col = HW_C
            else:
                col = OLD_C
            self.model.geom_rgba[i] = (*col, 1.0)
        mujoco.mj_forward(self.model, self.data)
        self.r.update_scene(self.data, camera="view")
        rgb = self.r.render().astype(float) / 255
        self.r.enable_segmentation_rendering()
        seg = self.r.render()[:, :, 0].astype(int)
        self.r.disable_segmentation_rendering()
        self.r.enable_depth_rendering()
        dep = self.r.render().copy()
        self.r.disable_depth_rendering()
        bg = seg < 0
        img = 0.30 + 0.70 * rgb
        img[bg] = 1.0
        # silhouettes: where the part under the pixel changes (continuous by construction)
        sil = (seg != np.roll(seg, 1, 0)) | (seg != np.roll(seg, 1, 1))
        img[ndimage.binary_dilation(sil, iterations=1)] = 0.10
        im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
        self._draw_edges(im, show, offs, dep)
        return im.resize((W, H), Image.LANCZOS)

    def _draw_edges(self, im, show, offs, dep):
        """CAD-style lines: every sharp edge of the geometry, drawn only where the depth buffer says it is visible."""
        pos, x, y, z, f = self.cam
        f2, Hs, Ws = f * SS, H * SS, W * SS
        d = ImageDraw.Draw(im)
        for n in show:
            E = self.edges.get(n)
            if E is None or not len(E):
                continue
            E = E + offs.get(n, 0)
            k = 12                                                  # samples per edge for the visibility test
            t = np.linspace(0, 1, k)[None, :, None]
            P = E[:, :1, :] * (1 - t) + E[:, 1:, :] * t             # (edges, k, 3)
            rel = P - pos
            depth = -(rel @ z)
            ok = depth > 1e-4
            u = Ws / 2 + f2 * (rel @ x) / np.where(ok, depth, 1)
            v = Hs / 2 - f2 * (rel @ y) / np.where(ok, depth, 1)
            ui, vi = np.clip(u.astype(int), 0, Ws - 1), np.clip(v.astype(int), 0, Hs - 1)
            buf = np.minimum.reduce([dep[np.clip(vi + dv, 0, Hs - 1), np.clip(ui + du, 0, Ws - 1)]
                                     for du in (-1, 0, 1) for dv in (-1, 0, 1)])
            vis = ok & (u >= 0) & (u < Ws) & (v >= 0) & (v < Hs) & (depth <= buf + 0.0015 + 0.004 * buf)
            for i in np.where(vis.any(axis=1))[0]:
                run = []
                for j in range(k):
                    if vis[i, j]:
                        run.append((u[i, j], v[i, j]))
                    elif len(run) > 1:
                        d.line(run, fill=(26, 26, 26), width=2 * SS - 1)
                        run = []
                    else:
                        run = []
                if len(run) > 1:
                    d.line(run, fill=(26, 26, 26), width=2 * SS - 1)


# ------------------------------------------------------------------ drawing helpers
def dashed(d, a, b, fill=(80, 80, 80), width=3, dash=12, gap=8):
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = np.linalg.norm(b - a)
    if L < 1:
        return
    u = (b - a) / L
    t = 0.0
    while t < L:
        d.line([tuple(a + u * t), tuple(a + u * min(t + dash, L))], fill=fill, width=width)
        t += dash + gap


def arrow(d, a, b, color=(234, 108, 28), width=8, head=30):
    """Solid motion arrow from a toward b, stopping short of b so the target stays visible."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = np.linalg.norm(b - a)
    if L < 25:
        return
    u = (b - a) / L
    tip = a + u * (L - min(18, 0.15 * L))
    base = tip - u * head
    nrm = np.array([-u[1], u[0]])
    d.line([tuple(a + u * 6), tuple(base)], fill=color, width=width)
    d.polygon([tuple(tip), tuple(base + nrm * head * 0.55), tuple(base - nrm * head * 0.55)], fill=color)


def balloon(d, xy, txt, r=26, size=30):
    x, y = xy
    d.ellipse([x - r, y - r, x + r, y + r], fill="white", outline="black", width=3)
    d.text((x, y), txt, fill="black", font=Renderer.font(size), anchor="mm")


def crop(im, pad=30, keep=()):
    a = np.asarray(im.convert("L"))
    ys, xs = np.where(a < 250)
    pts = [(xs.min(), ys.min()), (xs.max(), ys.max())] + list(keep)
    x0 = max(min(p[0] for p in pts) - pad, 0)
    y0 = max(min(p[1] for p in pts) - pad, 0)
    x1 = min(max(p[0] for p in pts) + pad, im.width)
    y1 = min(max(p[1] for p in pts) + pad, im.height)
    return im.crop((int(x0), int(y0), int(x1), int(y1)))


def bbox(im, pad=30, keep=()):
    a = np.asarray(im.convert("L"))
    solid = ndimage.binary_opening((a < 250) & (a > 70), iterations=3)   # shaded geometry, not stray lines
    ys, xs = np.where(solid if solid.any() else a < 250)
    pts = [(xs.min(), ys.min()), (xs.max(), ys.max())] + list(keep)
    return (int(max(min(p[0] for p in pts) - pad, 0)), int(max(min(p[1] for p in pts) - pad, 0)),
            int(min(max(p[0] for p in pts) + pad, im.width)), int(min(max(p[1] for p in pts) + pad, im.height)))


def png_uri(im):
    b = io.BytesIO()
    im.save(b, "PNG", optimize=True)
    return "data:image/png;base64," + base64.b64encode(b.getvalue()).decode()


# ------------------------------------------------------------------ steps
def build_steps(P):
    G = lambda *p: sorted((n for n in P if n.startswith(p)), key=lambda n: (n.split(":")[0], int(n.split(":")[1])))  # noqa: E731
    up = lambda d: np.array([0, 0, d * MM])                    # noqa: E731
    outy = lambda n, d: np.array([0, np.sign(P[n].bounds.mean(0)[1]) * d * MM, 0])  # noqa: E731
    mot, hang, whl = G("N20_Gearmotor"), G("Motor_Hanger"), G("Wheel_44mm")
    deck = G("Lower_Deck")
    S = []
    S.append(dict(
        title="Motor into its printed mount", new=mot[:1] + hang[:1], only=True, view=(-35, 25), repeat="x 4",
        offs={mot[0]: outy(mot[0], 30)},
        need=[(hang[0], "Motor mount (printed)", 4, "PLA, plate 1"), (mot[0], "N20 gearmotor + encoder", 4, "#5")],
        steps=["Slide the motor into the mount, gearbox first, until the gearbox is under the bridge.",
               "The encoder cable leaves through the open back of the mount.",
               "Make all four. Two face left and two face right: shafts point outward."], zoom=True))
    S.append(dict(
        title="Lock the motor with a cable tie", new=["Cable_Tie:1"], only=True, ctx=mot[:1] + hang[:1], view=(-120, -28),
        zoom=True,
        repeat="x 4", offs={},
        need=[("Cable_Tie:1", "Cable tie, 100 mm", 4, "#36")],
        steps=["Push a cable tie through one channel in the bridge, down the outside of both cheeks,",
               "under the motor through the two notches, and back into its head.",
               "Pull snug and cut the tail. Use the second channel too if the motor still slides."],
        warn="Tight enough that the motor cannot slide, not so tight that it crushes the encoder board."))
    S.append(dict(
        title="Mounts under the lower deck", new=mot + hang + G("Screw_Mount"), base=deck, view=(-58, 18), wide=True,
        offs={**{n: up(-32) for n in mot + hang}, **{n: up(26) for n in G("Screw_Mount")}},
        need=[(deck[0], "Lower deck (laser-cut)", 1, "#16"), (hang[1], "Motor + mount", 4, "steps 1-2"),
              ("Screw_Mount:1", "M3 x 6 screw", 8, "#15")],
        steps=["Peel the film off the deck and lay it engraved side up.",
               "Hold each mount under its two holes, shaft outward, and drive an M3 x 6 down into each hole.",
               "The screw cuts its own thread in the PLA: stop as soon as the head is snug."],
        warn="Keep each mount square to the deck edge, or the rover will drive crooked."))
    S.append(dict(
        title="Wheels", new=whl, view=(-62, 26), offs={n: outy(n, 38) for n in whl},
        need=[(whl[0], "44 mm wheel", 4, "#13")],
        steps=["Line up the flat in the hub with the flat on the motor's D-shaft.",
               "Press the wheel on until it stops. Spin each wheel by hand: nothing may rub."]))
    cl = G("IR_Cliff")
    S.append(dict(
        title="Table-edge sensors (deck upside down)", new=cl, view=(-130, -52), offs={n: up(-30) for n in cl},
        need=[("IR_Cliff_FL:1", "TCRT5000 edge sensor", 5, "#18"), (None, "Foam tape + paper template", 1, "#38")],
        steps=["Turn the deck over. Tape the printed underside template on and stick each sensor on its outline,",
               "lens down: front-left, front-right, rear, and the two rear corners.",
               "Lens 3-8 mm above the table. Route each lead up through the harness slot."],
        warn="These sensors stop the rover at the table edge. Every one is checked in bring-up before any run."))
    sh = G("Intake_Shroud")
    S.append(dict(
        title="Intake shroud", new=sh + G("Screw_Shroud"), view=(-38, 14),
        offs={**{n: up(-34) for n in sh}, **{n: up(26) for n in G("Screw_Shroud")}},
        need=[(sh[0], "Intake shroud (printed)", 1, "PLA, plate 1"), ("Screw_Shroud:1", "M3 x 6 screw", 4, "#15")],
        steps=["Hold the shroud under the front of the deck, flanges flat against it, open side forward.",
               "Drive an M3 x 6 down through the deck into each flange's pilot hole; it cuts its own thread."]))
    sv = G("Servo_MG946R")
    S.append(dict(
        title="Door servo", new=sv, view=(-40, 26), offs={n: up(42) for n in sv},
        need=[(sv[0], "MG946R servo", 1, "#9"), (None, "Cable tie, 100 mm", 2, "#36")],
        steps=["Lower the servo, output spline DOWN, into the rectangular pocket at the front-left of the deck.",
               "Its ears rest on the deck. Tie each ear down with a cable tie through the ear hole and the deck hole.",
               "Power it once from bring-up and send it to CLOSED before fitting the door."]))
    dr = G("Intake_Door")
    S.append(dict(
        title="Intake door", new=dr, view=(-20, 12), offs={n: up(-28) for n in dr},
        need=[(dr[0], "Intake door (printed)", 1, "PLA, plate 1")],
        steps=["Warm the servo spline for a few seconds with a hairdryer.",
               "Press the door's hub straight up onto the spline, door across the bay mouth (CLOSED).",
               "If it ever slips, glue the servo's round horn to the hub instead."],
        detail=dict(center=(129.5, 45, 38), r=14, az=30, el=-20, label="A", note="A: door hub pressed onto the spline")))
    cap = G("IR_Capture")
    S.append(dict(
        title="Bay sensor", new=cap, view=(0, -8), offs={n: np.array([45 * MM, 0, 0]) for n in cap}, frame_on=sh + cap,
        need=[(cap[0], "IR obstacle sensor", 1, "#19"), (None, "Foam tape", 1, "#38")],
        steps=["Reach in through the bay mouth (door open) and stick the sensor on the boss on the back wall,",
               "lens facing forward, out of the bay.",
               "It must see the token only once the token is fully inside."]))
    bat = G("Battery")
    S.append(dict(
        title="Battery", new=bat, view=(-60, 36), offs={n: up(42) for n in bat},
        need=[(bat[0], "3S 18650 pack", 1, "#10"), (None, "Cable tie, 100 mm", 2, "#36")],
        steps=["Place the pack on its outline at the rear of the deck, leads toward the middle.",
               "Strap it down with two cable ties through the deck. Leave it UNPLUGGED."],
        warn="The battery stays unplugged until both regulators are set to 5.0 V (step 12)."))
    drv = G("TB6612")
    S.append(dict(
        title="Motor drivers", new=drv, view=(-55, 44), offs={n: up(36) for n in drv},
        need=[(drv[0], "TB6612FNG driver", 2, "#6"), (None, "Foam tape", 1, "#38")],
        steps=["Stick each driver on its outline (A left, B right), pins up."]))
    xl = G("XL4015")
    S.append(dict(
        title="Voltage regulators", new=xl, view=(-55, 44), offs={n: up(36) for n in xl},
        need=[(xl[0], "XL4015 regulator", 2, "#12"), (None, "Foam tape, 3 layers", 1, "#38")],
        steps=["Three screw heads sit under each regulator's outline. Build two 3-layer foam-tape pads",
               "per regulator, beside the heads, and stick the board on them: LOGIC left, SERVO right.",
               "Feed each input from the battery, turn its trimmer until a multimeter reads 5.0 V at the",
               "output, then unplug the battery again."],
        warn="5.0 V first. At its factory setting a regulator can put out 12 V and destroy the ESP32 and the servo."))
    pcf, fuse, tf = G("PCF8574"), G("Fuse_Holder"), G("VL53L0X_Front")
    S.append(dict(
        title="I/O expander", new=pcf, view=(-60, 46), offs={n: up(34) for n in pcf},
        need=[(pcf[0], "PCF8574 I/O expander", 1, "#17"), (None, "Foam tape, 3 layers", 1, "#38")],
        steps=["A shroud screw head sits under one corner of this outline: build a 3-layer foam pad beside it.",
               "Stick the PCF8574 on its outline, pins toward the middle of the deck."]))
    S.append(dict(
        title="Fuse board", new=fuse, view=(-100, 42), offs={n: up(34) for n in fuse},
        need=[(fuse[0], "Fuse board: holder + 6.3 A fuse", 1, "#22, #23"), (None, "Foam tape", 1, "#38")],
        steps=["Push the 6.3 A fuse into its holder.",
               "Stick the fuse board on its outline at the rear left, next to the battery."]))
    S.append(dict(
        title="Front distance sensor", new=tf, view=(-30, 22), offs={n: np.array([36 * MM, 0, 10 * MM]) for n in tf},
        need=[(tf[0], "VL53L0X + L-bracket", 1, "#7, plate 1"), (None, "Foam tape", 1, "#38")],
        steps=["Tape the sensor to the upright face of an L-bracket, sensor window facing forward.",
               "Tape the bracket to the front lip of the deck, centred, sensor looking straight ahead."]))
    S.append(dict(
        title="Wire the lower deck", new=[], view=(-90, 70), offs={}, wide=True,
        need=[(None, "Wiring guide, pages 2-3", 1, "PDF"), (None, "Silicone wire, solder, heat-shrink", 1, "#24-#35")],
        steps=["Follow AEGIS_Wiring_Guide.pdf, page 3, one row at a time; tick each row off as you go.",
               "Power first: battery lead - fuse board - switch lead (leave it loose for now) - both regulators and both drivers.",
               "Then motor leads to the drivers, encoder leads, edge-sensor leads to the PCF8574.",
               "Leave the wires for the upper deck 15 cm long and bundle them at the harness slot."],
        warn="Battery unplugged the whole time. Heat-shrink every power joint before you go on."))
    pil = G("M3_Pillar")
    S.append(dict(
        title="Pillars", new=pil + G("Nut_Pillar"), view=(-58, 16),
        offs={**{n: up(32) for n in pil}, **{n: up(-24) for n in G("Nut_Pillar")}},
        need=[(pil[0], "Pillar: 20 mm M-F + 10 mm M-F", 4, "#15"), ("Nut_Pillar:1", "M3 nut", 4, "#15")],
        steps=["For each corner: push the stud of a 20 mm male-female pillar down through the hole and fit a nut",
               "underneath. Screw a 10 mm male-female pillar into its top: 30 mm between the decks."]))
    ud = G("Upper_Deck")
    S.append(dict(
        title="Upper deck", new=ud + G("Screw_PillarTop"), view=(-58, 24),
        offs={**{n: up(42) for n in ud}, **{n: up(74) for n in G("Screw_PillarTop")}},
        need=[(ud[0], "Upper deck (laser-cut)", 1, "#16"), ("Screw_PillarTop:1", "M3 x 6 screw", 4, "#15")],
        steps=["Feed the harness wires up through the slot in the upper deck.",
               "Set the deck on the pillars, engraved side up, and fix it with four M3 x 6 screws."]))
    sw = G("Power_Switch")
    S.append(dict(
        title="Power switch", new=sw, view=(-125, 32), offs={n: up(34) for n in sw},
        need=[(sw[0], "Rocker switch", 1, "#21")],
        steps=["Solder the battery + lead and the fuse-board lead to the switch first (wiring guide).",
               "Push the switch down into its cut-out at the rear left until the clips snap. O = off."]))
    esp = G("ESP32_Carrier", "ESP32_S3")
    S.append(dict(
        title="ESP32-S3 on its carrier board", new=esp, view=(-60, 36),
        offs={"ESP32_Carrier_Perfboard:1": up(30), "ESP32_S3_DevKitC:1": up(62)},
        need=[("ESP32_Carrier_Perfboard:1", "Carrier board, soldered", 1, "#31, #32"), ("ESP32_S3_DevKitC:1", "ESP32-S3 DevKitC", 1, "#1")],
        steps=["Stick the soldered carrier board on its outline with foam tape, sockets up.",
               "Plug the ESP32-S3 into the sockets, USB ports toward the rear (the arrow on the deck)."]))
    imu, mic = G("MPU6050"), G("INMP441")
    S.append(dict(
        title="Gyro", new=imu, view=(-60, 46), offs={n: up(30) for n in imu},
        need=[(imu[0], "MPU-6050 gyro", 1, "#8"), (None, "Foam tape", 1, "#38")],
        steps=["Stick the MPU-6050 flat on its outline, edges square to the deck edge.",
               "It measures the rover's heading: a tilted or skewed gyro makes the rover drive crooked."]))
    S.append(dict(
        title="Microphones", new=mic, view=(-60, 42), offs={n: up(30) for n in mic},
        need=[(mic[0], "INMP441 microphone", 2, "#4"), (None, "Foam tape", 1, "#38")],
        steps=["Stick one microphone on each outline, sound hole facing up: 100 mm apart, left and right.",
               "Keep tape off the sound hole."]))
    tof = G("VL53L0X_Left", "VL53L0X_Right")
    S.append(dict(
        title="Side distance sensors", new=tof, view=(-40, 32), offs={n: outy(n, 32) for n in tof},
        need=[("VL53L0X_Left:1", "VL53L0X + L-bracket", 2, "#7, plate 1")],
        steps=["Tape each sensor to an L-bracket and the brackets on the deck edges, sensors facing out.",
               "Point both straight out to the sides."]))
    S.append(dict(
        title="Wire the upper deck", new=[], view=(-90, 72), offs={}, wide=True,
        need=[(None, "Wiring guide, pages 1-3", 1, "PDF")],
        steps=["Connect the upper deck with AEGIS_Wiring_Guide.pdf: I2C (gyro, side and front sensors, PCF8574),",
               "the two microphones, the motor-driver and encoder pins, the servo signal and the camera UART.",
               "Keep the microphone (I2S) wires under 15 cm. Bundle the rest with cable ties, clear of the wheels."]))
    mast = G("Camera_Mast")
    S.append(dict(
        title="Camera mast", new=mast + G("Screw_Mast"), view=(-45, 18),
        offs={**{n: up(36) for n in mast}, **{n: up(-26) for n in G("Screw_Mast")}},
        need=[(mast[0], "Camera mast (printed)", 1, "PLA, plate 1"), ("Screw_Mast:1", "M3 x 6 screw", 2, "#15")],
        steps=["Stand the mast on its outline, sloped pad facing forward.",
               "Drive two M3 x 6 screws up from under the upper deck into the pilot holes in its base."]))
    cam = G("ESP32_CAM")
    S.append(dict(
        title="Camera", new=cam, view=(-20, 14), offs={n: np.array([32 * MM, 0, 20 * MM]) for n in cam},
        need=[(cam[0], "ESP32-CAM, flashed", 1, "#2"), (None, "Foam tape, 2 layers", 1, "#38")],
        steps=["Put two layers of foam tape on the sloped pad, clear of the SD-card slot.",
               "Press the ESP32-CAM on, lens forward: it looks 30 degrees down at the table."]))
    S.append(dict(
        title="Finished: final checks", new=[], view=(-50, 24), offs={},
        need=[],
        steps=["Every screw tight, no wire near a wheel, the door swings freely, the bay is clear.",
               "Switch OFF, plug in the battery, switch ON.",
               "Follow AEGIS_Deployment_Guide.pdf: flash both boards, bring-up checks, and the table-edge test",
               "before the first autonomous run."]))
    return S


def thumbnail(rend, n):
    rend.frame(-50, 25, rend.corners(n), fill=0.85)
    im = crop(rend.render({n}, {n}, {}), pad=10)
    im.thumbnail((360, 260), Image.LANCZOS)
    return im


def render_step(rend, st, installed):
    new = list(st["new"])
    ctx = list(st.get("ctx", []))
    show = set(new) | set(ctx) | set(st.get("base", []))
    if not st.get("only"):
        show |= set(installed)
    offs = {n: np.asarray(st["offs"].get(n, np.zeros(3))) for n in show}
    if st.get("frame_on"):
        focus = list(st["frame_on"])
    elif new and not st.get("wide"):
        focus = new + ctx                                       # zoom on the work; the rest is context
    else:
        focus = list(show)
    pts = np.vstack([rend.corners(n, offs[n]) for n in focus] + [rend.corners(n) for n in focus if n in new])
    c, half = (pts.min(0) + pts.max(0)) / 2, np.maximum((pts.max(0) - pts.min(0)) / 2, 0.07)   # at least 14 cm of context
    pts = np.vstack([pts, c - half, c + half])
    rend.frame(*st["view"], pts, fill=st.get("fill", 0.62 if (new and not st.get("wide")) else 0.86))
    im = rend.render(show, set(new), offs)
    result = rend.render(show, set(new), {})                    # same camera, everything in place
    d = ImageDraw.Draw(im)
    for n in new:
        if np.linalg.norm(offs[n]) > 1e-4:
            arrow(d, rend.project(rend.cent[n] + offs[n]), rend.project(rend.cent[n]))
    keep, placed = [], []
    for idx, (nm, label, qty, item) in enumerate(st["need"], 1):
        if not nm or nm not in show:
            continue
        p = rend.project(rend.cent[nm] + offs[nm])
        q = p + np.array([120 if p[0] > W / 2 else -120, -100])
        q = np.clip(q, [40, 40], [W - 40, H - 40])
        while any(np.linalg.norm(q - o) < 72 for o in placed):
            q = q + np.array([0, 72 if q[1] < H / 2 else -72])
        placed.append(q)
        d.line([tuple(p), tuple(q)], fill="black", width=3)
        d.ellipse([p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6], fill="black")
        balloon(d, q, str(idx))
        keep += [tuple(q - 32), tuple(q + 32)]
    box = bbox(im, pad=26, keep=keep)
    im, result = im.crop(box), result.crop(box)
    detail = None
    if st.get("detail"):
        dt = st["detail"]
        c, r = np.array(dt["center"]) * MM, dt["r"] * MM
        cube = np.array([c + np.array([sx, sy, sz]) * r for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        rend.frame(dt["az"], dt["el"], cube, fill=0.95)
        dshow = set(dt["parts"]) if dt.get("parts") else show
        dim = rend.render(dshow, set(new), offs)
        side = min(dim.size)
        dim = dim.crop(((dim.width - side) // 2, (dim.height - side) // 2, (dim.width + side) // 2,
                        (dim.height + side) // 2)).resize((560, 560), Image.LANCZOS)
        mask = Image.new("L", dim.size, 0)
        ImageDraw.Draw(mask).ellipse([0, 0, 559, 559], fill=255)
        circ = Image.new("RGBA", (580, 580), (255, 255, 255, 0))
        circ.paste(dim, (10, 10), mask)
        dd = ImageDraw.Draw(circ)
        dd.ellipse([6, 6, 573, 573], outline=ORANGE, width=9)
        balloon(dd, (76, 76), dt["label"], r=36, size=40)
        detail = (circ, dt.get("note", ""))
    return im, detail, result


CSS = """
@page { size: A4 landscape; margin: 11mm 13mm 13mm 13mm;
  @bottom-left { content: "A.E.G.I.S. Mk2  |  Assembly guide"; font: 8pt 'Helvetica Neue', Arial; color: #8a93a3; }
  @bottom-right { content: "page " counter(page) " of " counter(pages); font: 8pt 'Helvetica Neue', Arial; color: #8a93a3; } }
@page :first { margin: 0; @bottom-left { content: none; } @bottom-right { content: none; } }
body { margin: 0; font: 10pt/1.45 'Helvetica Neue', Helvetica, Arial, sans-serif; color: #1d2433; }
.page { position: relative; width: 271mm; height: 184mm; page-break-after: always; overflow: hidden; }
.head { position: absolute; left: 0; top: 0; width: 271mm; height: 15mm; display: flex; align-items: center; gap: 5mm;
  border-bottom: 2.5px solid %(orange)s; }
.num { width: 12.5mm; height: 12.5mm; border-radius: 50%%; background: %(navy)s; color: white; font: 800 18pt 'Helvetica Neue';
  display: flex; align-items: center; justify-content: center; flex: none; }
.head h2 { margin: 0; font-size: 17pt; color: %(navy)s; }
.head .rep { margin-left: auto; font: 800 18pt 'Helvetica Neue'; color: %(orange)s; }
.fig { position: absolute; left: 0; top: 19mm; width: 196mm; height: 122mm; display: flex; align-items: center;
  justify-content: center; }
.fig > img { max-width: 196mm; max-height: 122mm; }
.detail { position: absolute; left: 0; bottom: 0; width: 44mm; text-align: center; }
.detail img { width: 44mm; }
.detail div { font-size: 7.8pt; color: #5b6476; line-height: 1.25; }
.side { position: absolute; left: 203mm; top: 19mm; width: 68mm; height: 165mm; background: #f4f6fa; border-radius: 4px;
  padding: 3mm; box-sizing: border-box; overflow: hidden; }
.result { position: relative; border: 1.6px solid %(navy)s; border-radius: 4px; background: white; padding: 1mm;
  margin-bottom: 3mm; }
.result img { width: 100%%; max-height: 44mm; object-fit: contain; display: block; }
.result span { position: absolute; left: 1.5mm; top: 1mm; font: 800 7.5pt 'Helvetica Neue'; color: white;
  background: %(navy)s; padding: 0.4mm 1.6mm; border-radius: 2px; letter-spacing: 1px; }
.side h3 { margin: 0 0 2mm; font-size: 9.5pt; letter-spacing: 1.5px; color: %(navy)s; }
.item { display: flex; gap: 2mm; align-items: center; margin-bottom: 2mm; background: white; border-radius: 3px; padding: 1.6mm; }
.item .b { width: 6.5mm; height: 6.5mm; border-radius: 50%%; border: 1.6px solid black; background: white; font-weight: 700;
  display: flex; align-items: center; justify-content: center; font-size: 9pt; flex: none; }
.item img { width: 17mm; max-height: 12mm; object-fit: contain; flex: none; }
.item .t { font-size: 8.4pt; line-height: 1.3; }
.item .q { font-weight: 800; color: %(orange)s; font-size: 10pt; }
.item .i { color: #8a93a3; font-size: 7.3pt; }
.text { position: absolute; left: 0; top: 143mm; width: 196mm; height: 41mm; overflow: hidden; }
.text ol { margin: 0; padding-left: 6mm; font-size: 10pt; }
.text li { margin: 0.6mm 0; }
.warn { margin-top: 1.6mm; padding: 1.6mm 3mm; background: #fff4ec; border-left: 4px solid %(orange)s; font-size: 9pt; }
.warn b { color: %(orange)s; }
.cover { height: 210mm; width: 297mm; background: %(navy)s; color: white; position: relative; page-break-after: always; overflow: hidden; }
.cover .band { position: absolute; left: 0; top: 0; width: 9mm; height: 100%%; background: %(orange)s; }
.cover .brand { position: absolute; left: 22mm; top: 18mm; letter-spacing: 3px; color: #c9d2e3; font-size: 12pt; }
.cover h1 { position: absolute; left: 22mm; top: 25mm; margin: 0; font-size: 34pt; }
.cover .sub { position: absolute; left: 22mm; top: 46mm; font-size: 12pt; color: #dfe5ef; }
.cover .img { position: absolute; left: 22mm; top: 60mm; width: 160mm; height: 132mm; background: white; border-radius: 6px;
  display: flex; align-items: center; justify-content: center; }
.cover .img img { max-width: 94%%; max-height: 94%%; }
.cover .box { position: absolute; left: 192mm; top: 60mm; width: 90mm; font-size: 9.4pt; line-height: 1.5; color: #e6ebf3; }
.cover .box h3 { color: white; margin: 0 0 2mm; font-size: 10.5pt; letter-spacing: 1.5px; }
.cover .box p { margin: 0 0 2.6mm; }
.sw { width: 8mm; height: 4.5mm; border: 1px solid #111; display: inline-block; vertical-align: middle; margin-right: 1.5mm; }
.hw { page-break-after: always; }
.hw h2 { color: %(navy)s; border-bottom: 2.5px solid %(orange)s; padding-bottom: 2mm; margin: 0 0 4mm; }
.hw table { border-collapse: collapse; width: 100%%; font-size: 9.6pt; }
.hw th { background: %(navy)s; color: white; text-align: left; padding: 5px 7px; }
.hw td { padding: 5px 7px; border-bottom: 1px solid #dde2ea; }
.hw tr:nth-child(even) td { background: #f4f6fa; }
.hw p { font-size: 9.6pt; }
"""

HARDWARE = """<div class="hw"><h2>Fasteners: everything comes from the M3 kit (item #15)</h2>
<table><tr><th>Where</th><th>What</th><th>Qty</th><th>Step</th></tr>
<tr><td>Motor mounts to the lower deck</td><td>M3 x 6 screw, self-tapping into the PLA bridge</td><td>8</td><td>3</td></tr>
<tr><td>Intake shroud flanges</td><td>M3 x 6 screw, down through the deck into the 2.5 mm pilot</td><td>4</td><td>6</td></tr>
<tr><td>Pillars, lower end</td><td>20 mm male-female pillar, stud through the deck + M3 nut</td><td>4 + 4</td><td>17</td></tr>
<tr><td>Pillars, upper part</td><td>10 mm male-female pillar on top of the 20 mm one</td><td>4</td><td>17</td></tr>
<tr><td>Upper deck to the pillars</td><td>M3 x 6 screw</td><td>4</td><td>18</td></tr>
<tr><td>Camera mast</td><td>M3 x 6 screw, up through the deck into the 2.5 mm pilot</td><td>2</td><td>25</td></tr>
<tr><td>Servo ears, battery, motors</td><td>Cable ties, 100 mm (item #36)</td><td>2 + 2 + 4</td><td>2, 7, 10</td></tr>
<tr><td>All boards, brackets, camera</td><td>3M foam tape (item #38)</td><td>about 1.5 m</td><td>5, 9-15, 20-23, 26</td></tr></table>
<p><b>From the kit:</b> 18 of the 20 M3 x 6 screws (2 spare), 4 of the 20 nuts, 4 of the 10 pillars of each of the
two male-female lengths. No other screws are needed. Self-tapping screws: turn them in slowly the first time and stop
as soon as the head is snug, or the PLA thread strips.</p>
<p><b>Printed parts</b> (plate 1, black PLA+): intake shroud, intake door, camera mast, four motor mounts, three
L-brackets for the distance sensors. Plate 2 (orange PLA) holds the demo props: token and two decoys.</p></div>"""


def main():
    only = [int(a) for a in sys.argv[1:]] or None              # e.g. "3 7" renders preview PNGs of steps 3 and 7
    P = load_parts()
    rend = Renderer(P)
    steps = build_steps(P)
    thumbs, pages, installed = {}, [], []
    os.makedirs(PREVIEW, exist_ok=True)
    for i, st in enumerate(steps, 1):
        if only is None or i in only:
            img, detail, result = render_step(rend, st, installed)
            if only:
                canvas = img.copy()
                sm = result.copy()
                sm.thumbnail((canvas.width // 3, canvas.height // 3))
                canvas.paste(sm, (canvas.width - sm.width, 0))
                ImageDraw.Draw(canvas).rectangle([canvas.width - sm.width, 0, canvas.width - 1, sm.height], outline=NAVY, width=3)
                if detail:
                    canvas.paste(detail[0].resize((300, 300)), (canvas.width - 300, canvas.height - 300), detail[0].resize((300, 300)))
                canvas.save(os.path.join(PREVIEW, "step%02d.png" % i))
            else:
                items = []
                for k, (nm, label, qty, item) in enumerate(st["need"], 1):
                    if nm and nm not in thumbs:
                        thumbs[nm] = png_uri(thumbnail(rend, nm))
                    items.append('<div class="item"><div class="b">%s</div>%s<div class="t">%s <span class="q">x %d</span>'
                                 '<br><span class="i">%s</span></div></div>' %
                                 (k if nm else "&bull;", '<img src="%s">' % thumbs[nm] if nm else "", html.escape(label), qty,
                                  html.escape(item if item.startswith(("PLA", "step")) else "item " + item)))
                det = ('<div class="detail"><img src="%s"><div>%s</div></div>' % (png_uri(detail[0]), html.escape(detail[1]))
                       if detail else "")
                res = ""
                if st["new"] and not st.get("no_result"):
                    small = result.copy()
                    small.thumbnail((700, 460), Image.LANCZOS)
                    res = '<div class="result"><span>RESULT</span><img src="%s"></div>' % png_uri(small)
                warn = '<div class="warn"><b>Important.</b> %s</div>' % html.escape(st["warn"]) if st.get("warn") else ""
                pages.append(
                    '<div class="page"><div class="head"><div class="num">%d</div><h2>%s</h2><div class="rep">%s</div></div>'
                    '<div class="fig"><img src="%s">%s</div><div class="side">%s<h3>YOU NEED</h3>%s</div>'
                    '<div class="text"><ol>%s</ol>%s</div></div>' %
                    (i, html.escape(st["title"]), st.get("repeat", ""), png_uri(img), det, res,
                     "".join(items) or "<div class='t'>Nothing new: checks only.</div>",
                     "".join("<li>%s</li>" % html.escape(s) for s in st["steps"]), warn))
            print("step %2d  %s" % (i, st["title"]), flush=True)
        installed += [n for n in st["new"] + list(st.get("base", [])) if n not in installed]
    if only:
        return
    rend.frame(-50, 24, np.vstack([rend.corners(n) for n in installed]), fill=0.88)
    hero = crop(rend.render(set(installed), set(), {}), pad=20)
    cover = ('<div class="cover"><div class="band"></div><div class="brand">A.E.G.I.S.  MK2</div><h1>Assembly guide</h1>'
             '<div class="sub">%d steps, one action each, from bare deck to finished rover</div>'
             '<div class="img"><img src="%s"></div><div class="box">'
             '<h3>BEFORE YOU START</h3>'
             '<p><b>Tools:</b> Phillips screwdriver, side cutters, wire strippers, soldering iron, multimeter, hairdryer.</p>'
             '<p><b>Made first</b> (Fabrication guide): both laser-cut decks, all printed parts, the soldered ESP32-S3 '
             'carrier board, headers soldered on every module.</p>'
             '<p><b>Check</b> every board against its engraved outline before sticking it down.</p>'
             '<p><b>Battery</b> stays unplugged until step 27.</p>'
             '<h3 style="margin-top:4mm">HOW TO READ A STEP</h3>'
             '<p><span class="sw" style="background:#57a0eb"></span>added in this step &nbsp; '
             '<span class="sw" style="background:#e6e6e0"></span>already fitted &nbsp; '
             '<span class="sw" style="background:#b3b8c2"></span>screws, nuts</p>'
             '<p>Orange arrows show where a part moves. RESULT shows the same view once the step is done. Numbered balloons match the YOU NEED list. '
             'An orange circle is a close-up. "x 4" next to the title: do it four times.</p>'
             '<p><b>Wiring</b> with AEGIS_Wiring_Guide.pdf at steps 16 and 24.</p></div></div>'
             % (len(steps), png_uri(hero)))
    doc = ("<!doctype html><html><head><meta charset='utf-8'><style>%s</style></head><body>%s%s%s</body></html>" %
           (CSS % dict(navy=NAVY, orange=ORANGE), cover, HARDWARE, "".join(pages)))
    with tempfile.NamedTemporaryFile("w", suffix=".html", delete=False) as f:
        f.write(doc)
        tmp = f.name
    subprocess.run([CHROME, "--headless", "--disable-gpu", "--no-pdf-header-footer", "--print-to-pdf=" + OUT,
                    "file://" + tmp], check=True, capture_output=True)
    os.remove(tmp)
    print(OUT)


if __name__ == "__main__":
    main()
