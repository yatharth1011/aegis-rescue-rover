"""
A.E.G.I.S. Mk2 wiring guide, step by step: what to wire when, one small job per page.

    ~/aegis_venv/bin/python team_share/tools/wiring_steps.py   ->  team_share/AEGIS_Wiring_Guide.pdf

Each step draws only the boards involved, every wire pin to pin with a number that matches the table,
and ends with a multimeter check. Pin numbers follow firmware/aegis_s3/main/aegis_config.h. The pin map
and board faces from wiring_guide.py follow as a reference appendix.
"""
import base64
import html
import io
import os
import subprocess
import tempfile

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches

HERE = os.path.dirname(os.path.abspath(__file__))
TS = os.path.abspath(os.path.join(HERE, ".."))
OUT = os.path.join(TS, "AEGIS_Wiring_Guide.pdf")
REF = os.path.join(TS, "deck_assets", "guides")
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
NAVY, ORANGE = "#1E2A44", "#EA6C1C"

COL = {"BAT": "#d62828", "GND": "#111111", "5V": "#f77f00", "SRV": "#e9a400", "3V3": "#c2255c", "SDA": "#c9a000",
       "SCL": "#2a9d8f", "I2S": "#1d4ed8", "PWM": "#7b2cbf", "ENC": "#8d5524", "IR": "#3a9bdc", "VBAT": "#0f9b8e",
       "XS": "#5c6bc0", "MOT": "#b5179e"}
NAME = {"BAT": "battery +", "GND": "ground", "5V": "5 V logic", "SRV": "5 V servo", "3V3": "3.3 V", "SDA": "I2C SDA",
        "SCL": "I2C SCL", "I2S": "I2S audio", "PWM": "motor control", "ENC": "encoder", "IR": "IR signal",
        "VBAT": "battery sense", "XS": "XSHUT", "MOT": "motor power"}
WIRE = {"BAT": "18 AWG red", "GND": "18 AWG black (power) / 26 AWG black (signal)", "5V": "22 AWG red (3-core)",
        "SRV": "22 AWG red (3-core)", "3V3": "26 AWG red", "SDA": "26 AWG white", "SCL": "26 AWG green",
        "I2S": "26 AWG white", "PWM": "26 AWG white", "ENC": "26 AWG green", "IR": "26 AWG white",
        "VBAT": "26 AWG white", "XS": "26 AWG green", "MOT": "22 AWG (3-core) red / black"}


# ------------------------------------------------------------------ tiny schematic engine
class Sheet:
    def __init__(self, w=270, h=150):
        self.fig = plt.figure(figsize=(w / 25.4, h / 25.4), dpi=220)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, w)
        self.ax.set_ylim(0, h)
        self.ax.axis("off")
        self.ax.set_aspect("equal")
        self.pins = {}
        self.tags = []
        self.titles = {}
        self.rows = []                                              # (n, from, to, kind) for the table

    def board(self, key, x, y, w, h, title, pins, fc="#1f6f43", tc="white", sub=None):
        """pins: list of (name, side, position 0..1); side in L R T B. Returns nothing; pins are addressable as key.name."""
        ax = self.ax
        self.titles[key] = title
        ax.add_patch(patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=2", fc=fc, ec="#0b0b0b", lw=1.2))
        ax.text(x + w / 2, y + h / 2 + (2.2 if sub else 0), title, ha="center", va="center", color=tc, fontsize=8.5,
                weight="bold")
        if sub:
            ax.text(x + w / 2, y + h / 2 - 3.2, sub, ha="center", va="center", color=tc, fontsize=6.5)
        for name, side, t in pins:
            if side == "L":
                px, py, dx, dy = x, y + h * (1 - t), -1, 0
            elif side == "R":
                px, py, dx, dy = x + w, y + h * (1 - t), 1, 0
            elif side == "T":
                px, py, dx, dy = x + w * t, y + h, 0, 1
            else:
                px, py, dx, dy = x + w * t, y, 0, -1
            ax.add_patch(patches.Rectangle((px - 1.1 + dx * 0.6, py - 1.1 + dy * 0.6), 2.2, 2.2, fc="#d9b44a", ec="#5a4a10", lw=0.5))
            ha = {"L": "left", "R": "right"}.get(side, "center")
            va = {"T": "top", "B": "bottom"}.get(side, "center")
            ax.text(px - dx * 2.2, py - dy * 2.2, name, ha=ha, va=va, fontsize=6.2, color=tc, family="monospace",
                    rotation=90 if side in "TB" else 0)
            self.pins["%s.%s" % (key, name)] = (px + dx * 1.7, py + dy * 1.7, dx, dy)

    def label(self, pin):
        k, name = pin.split(".", 1)
        return "%s  %s" % (self.titles[k], name.strip())

    def wire(self, a, b, kind, n=None, via=None, lw=2.6):
        if n is not None:
            self.rows.append((n, self.label(a), self.label(b), kind))
        ax0, ay0, adx, ady = self.pins[a]
        bx0, by0, bdx, bdy = self.pins[b]
        p0 = (ax0 + adx * 5, ay0 + ady * 5)
        p1 = (bx0 + bdx * 5, by0 + bdy * 5)
        if via is None:
            via = ("x", (p0[0] + p1[0]) / 2) if adx or bdx else ("y", (p0[1] + p1[1]) / 2)
        if via[0] == "x":
            pts = [(ax0, ay0), p0, (via[1], p0[1]), (via[1], p1[1]), p1, (bx0, by0)]
        else:
            pts = [(ax0, ay0), p0, (p0[0], via[1]), (p1[0], via[1]), p1, (bx0, by0)]
        xs, ys = zip(*pts)
        self.ax.plot(xs, ys, color="white", lw=lw + 2.2, solid_capstyle="round", solid_joinstyle="round", zorder=3)
        self.ax.plot(xs, ys, color=COL[kind], lw=lw, solid_capstyle="round", solid_joinstyle="round", zorder=4)
        if n is not None:
            seg = max(range(len(pts) - 1), key=lambda i: abs(pts[i + 1][0] - pts[i][0]) + abs(pts[i + 1][1] - pts[i][1]))
            mx, my = (pts[seg][0] + pts[seg + 1][0]) / 2, (pts[seg][1] + pts[seg + 1][1]) / 2
            for tx, ty in self.tags:                              # keep number tags apart
                if abs(tx - mx) < 5 and abs(ty - my) < 5:
                    if pts[seg][0] == pts[seg + 1][0]:
                        my += 6
                    else:
                        mx += 6
            self.tags.append((mx, my))
            self.ax.add_patch(patches.Circle((mx, my), 2.6, fc="white", ec=COL[kind], lw=1.4, zorder=5))
            self.ax.text(mx, my, str(n), ha="center", va="center", fontsize=6.5, weight="bold", zorder=6)

    def note(self, x, y, txt, **kw):
        self.ax.text(x, y, txt, fontsize=kw.pop("fs", 7.2), color=kw.pop("c", "#333"), **kw)

    def resistor(self, x, y, label, vertical=True):
        w, h = (3.2, 9) if vertical else (9, 3.2)
        self.ax.add_patch(patches.Rectangle((x - w / 2, y - h / 2), w, h, fc="#e8d7a6", ec="#222", lw=0.9, zorder=5))
        self.ax.text(x + (3.5 if vertical else 0), y + (0 if vertical else 3.6), label, fontsize=6.3, va="center",
                     ha="left" if vertical else "center", zorder=6)

    def png(self):
        b = io.BytesIO()
        self.fig.savefig(b, format="png", facecolor="white")
        plt.close(self.fig)
        return "data:image/png;base64," + base64.b64encode(b.getvalue()).decode()


TB_PINS = [("VM", "B", 0.08), ("VCC", "B", 0.22), ("GND", "B", 0.36), ("AO1", "B", 0.5), ("AO2", "B", 0.64),
           ("BO2", "B", 0.78), ("BO1", "B", 0.92), ("PWMA", "T", 0.08), ("AIN2", "T", 0.22), ("AIN1", "T", 0.36),
           ("STBY", "T", 0.5), ("BIN1", "T", 0.64), ("BIN2", "T", 0.78), ("PWMB", "T", 0.92)]
XL_PINS = [("IN+", "L", 0.3), ("IN-", "L", 0.7), ("OUT+", "R", 0.3), ("OUT-", "R", 0.7)]
MOT_PINS = [("M1", "R", 0.12), ("M2", "R", 0.28), ("VCC", "R", 0.44), ("GND", "R", 0.6), ("C1", "R", 0.76), ("C2", "R", 0.92)]
TOF_PINS = [("VIN", "R", 0.1), ("GND", "R", 0.26), ("SCL", "R", 0.42), ("SDA", "R", 0.58), ("XSHUT", "R", 0.9)]


# ------------------------------------------------------------------ steps
def s_headers():
    s = Sheet(270, 120)
    boards = [("VL53L0X (x3)", 6), ("MPU-6050", 8), ("INMP441 (x2)", 6), ("TB6612FNG (x2)", 16), ("PCF8574", "if loose")]
    for i, (n, k) in enumerate(boards):
        x = 8 + i * 52
        s.board("h%d" % i, x, 50, 42, 30, n.split(" (")[0], [], fc="#244b8f")
        for j in range(6):
            s.ax.add_patch(patches.Rectangle((x + 6 + j * 5.4, 41), 2.0, 9, fc="#d9b44a", ec="#5a4a10", lw=0.5))
        s.note(x + 21, 32, "%s pins" % k if isinstance(k, int) else k, ha="center", fs=8, weight="bold")
        s.note(x + 21, 25, n, ha="center", fs=7)
    s.note(8, 104, "Short pins down, long pins up. Push the header into a breadboard or a scrap of perfboard to hold it square,",
           fs=8)
    s.note(8, 97, "lay the module on top, solder one corner pin, check it is square, then solder the rest.", fs=8)
    s.note(8, 10, "Each joint: heat pin and pad together 2 s, feed solder until it flows round the pin, remove solder, then iron. "
                  "A good joint is a small shiny cone.", fs=7.5, c="#555")
    return s


def s_carrier():
    s = Sheet(270, 160)
    J1 = ["3V3", "3V3", "RST", "4", "5", "6", "7", "15", "16", "17", "18", "8", "3", "46", "9", "10", "11", "12", "13",
          "14", "5V", "GND"]
    J3 = ["GND", "TX", "RX", "1", "2", "42", "41", "40", "39", "38", "37", "36", "35", "0", "45", "48", "47", "21", "20",
          "19", "GND", "GND"]
    use = {"4": "I2S", "5": "I2S", "6": "I2S", "8": "SDA", "9": "SCL", "10": "XS", "11": "XS", "12": "PWM", "13": "PWM",
           "14": "PWM", "15": "PWM", "16": "PWM", "17": "PWM", "18": "PWM", "21": "PWM", "46": "PWM", "38": "ENC",
           "39": "ENC", "40": "ENC", "41": "ENC", "42": "ENC", "47": "ENC", "48": "ENC", "7": "ENC", "3": "SRV",
           "1": "VBAT", "5V": "5V", "GND": "GND", "3V3": "3V3"}
    ax = s.ax
    ax.add_patch(patches.FancyBboxPatch((70, 12), 130, 136, boxstyle="round,pad=0,rounding_size=3", fc="#c9a46a", ec="#5a4a10"))
    s.note(135, 152, "CARRIER BOARD (perfboard, 66 x 34 mm), seen from the top. USB end toward the rover's rear.", ha="center",
           fs=7.5, weight="bold", c=NAVY)
    for col, pins, x0 in (("L", J1, 105), ("R", J3, 165)):
        for i, p in enumerate(pins):
            y = 140 - i * 5.9
            k = use.get(p)
            ax.add_patch(patches.Rectangle((x0 - 2, y - 2), 4, 4, fc="#222", ec="#000"))
            ax.add_patch(patches.Circle((x0, y), 0.9, fc=COL[k] if k else "#777"))
            name = p if not p.isdigit() else "GPIO " + p
            if col == "L":
                ax.text(x0 + 4, y, name, fontsize=6, va="center", family="monospace")
                if k:
                    ax.plot([x0 - 3, x0 - 30], [y, y], color=COL[k], lw=2.2)
                    ax.text(x0 - 31, y, NAME[k], fontsize=5.8, va="center", ha="right")
            else:
                ax.text(x0 - 4, y, name, fontsize=6, va="center", ha="right", family="monospace")
                if k:
                    ax.plot([x0 + 3, x0 + 30], [y, y], color=COL[k], lw=2.2)
                    ax.text(x0 + 31, y, NAME[k], fontsize=5.8, va="center")
    ax.text(105, 144.5, "J1 socket (22)", fontsize=6.5, ha="center", weight="bold")
    ax.text(165, 144.5, "J3 socket (22)", fontsize=6.5, ha="center", weight="bold")
    ax.add_patch(patches.Rectangle((114, 20), 42, 18, fc="#2f5d9e", ec="#111"))
    ax.text(135, 29, "ESP32-S3 plugs in here\n(after soldering)", ha="center", va="center", fontsize=6.5, color="white")
    # pull-ups and divider, drawn beside the board
    s.note(208, 120, "On the carrier, solder:", fs=7.5, weight="bold")
    s.resistor(215, 104, "2.2 k: GPIO 8 (SDA) to 3V3")
    s.resistor(215, 88, "2.2 k: GPIO 9 (SCL) to 3V3")
    s.resistor(215, 66, "100 k: battery-sense lead to GPIO 1")
    s.resistor(215, 50, "20 k: GPIO 1 to GND")
    s.note(208, 34, "Then a 3V3 rail and a GND rail along the edge,", fs=6.8)
    s.note(208, 28, "and a solder pad for every coloured pin:", fs=6.8)
    s.note(208, 22, "that is where its wire lands later.", fs=6.8)
    s.note(4, 6, "Sockets: cut each 22-way row from one full 1x20 strip plus 2 sockets from a spare strip. The two rows are 22.86 mm "
                 "(9 holes) apart: check with the ESP32-S3 itself before soldering.", fs=7)
    return s


def s_fuse_switch():
    s = Sheet(270, 120)
    s.board("bat", 8, 40, 46, 40, "BATTERY", [("+", "R", 0.3), ("-", "R", 0.75)], fc="#3a3f4b", sub="T-connector lead")
    s.board("fuse", 92, 64, 40, 22, "FUSE BOARD", [("IN", "L", 0.5), ("OUT", "R", 0.5)], fc="#1f6f43", sub="6.3 A")
    s.board("sw", 160, 64, 30, 22, "SWITCH", [("1", "L", 0.5), ("2", "R", 0.5)], fc="#3a3f4b", sub="rocker")
    s.board("bus", 220, 30, 40, 60, "POWER BUS", [("+", "L", 0.25), ("-", "L", 0.8)], fc="#7a5c2e", sub="perfboard strip")
    s.wire("bat.+", "fuse.IN", "BAT", 1)
    s.wire("fuse.OUT", "sw.1", "BAT", 2)
    s.wire("sw.2", "bus.+", "BAT", 3)
    s.wire("bat.-", "bus.-", "GND", 4, via=("x", 200))
    s.note(8, 12, "Make the POWER BUS from a 2 x 8 strip of perfboard: one row for +, one row for -, each row bridged with solder. "
                  "Everything that needs battery power connects to it.", fs=7.2)
    return s


def s_power():
    s = Sheet(270, 150)
    s.board("bus", 8, 46, 34, 70, "POWER BUS", [("+", "R", 0.15), ("-", "R", 0.85)], fc="#7a5c2e")
    s.board("xl1", 92, 112, 54, 26, "XL4015 LOGIC", XL_PINS, fc="#244b8f")
    s.board("xl2", 92, 78, 54, 26, "XL4015 SERVO", XL_PINS, fc="#244b8f")
    s.board("tba", 190, 76, 66, 32, "TB6612 A", TB_PINS, fc="#7b2cbf")
    s.board("tbb", 190, 24, 66, 32, "TB6612 B", TB_PINS, fc="#7b2cbf")
    s.wire("bus.+", "xl1.IN+", "BAT", 1, via=("x", 62))
    s.wire("bus.+", "xl2.IN+", "BAT", 2, via=("x", 68))
    s.wire("bus.-", "xl1.IN-", "GND", 3, via=("x", 76))
    s.wire("bus.-", "xl2.IN-", "GND", 4, via=("x", 82))
    s.wire("bus.+", "tba.VM", "BAT", 5, via=("y", 66))
    s.wire("bus.-", "tba.GND", "GND", 6, via=("y", 62))
    s.wire("bus.+", "tbb.VM", "BAT", 7, via=("y", 14))
    s.wire("bus.-", "tbb.GND", "GND", 8, via=("y", 10))
    s.note(8, 140, "Leave the regulator OUTPUTS unconnected in this step.", fs=8, weight="bold", c=ORANGE)
    return s


def s_set5v():
    s = Sheet(270, 120)
    s.board("xl", 70, 40, 90, 46, "XL4015", XL_PINS, fc="#244b8f", sub="trimmer: small brass screw on the blue pot")
    s.ax.add_patch(patches.FancyBboxPatch((195, 30), 50, 70, boxstyle="round,pad=0,rounding_size=4", fc="#f2c94c", ec="#333"))
    s.ax.add_patch(patches.Rectangle((202, 70), 36, 20, fc="#c8e6c9", ec="#333"))
    s.ax.text(220, 80, "5.00 V", ha="center", va="center", fontsize=13, weight="bold", family="monospace")
    s.ax.text(220, 52, "MULTIMETER\nDC V", ha="center", va="center", fontsize=8)
    s.ax.plot([175, 200], [76, 60], color="#d62828", lw=2)
    s.ax.plot([175, 200], [57, 45], color="#111", lw=2)
    s.note(8, 106, "1. Battery plugged in, switch ON, nothing connected to either regulator's output.", fs=8.5)
    s.note(8, 98, "2. Meter on DC volts: red probe on OUT+, black on OUT-.", fs=8.5)
    s.note(8, 90, "3. Turn the trimmer (often 10+ turns) until it reads 5.0 V. Do the same for the second regulator.", fs=8.5)
    s.note(8, 16, "4. Switch OFF and unplug the battery before the next step.", fs=8.5, weight="bold")
    return s


def s_motors():
    s = Sheet(270, 150)
    s.board("tba", 105, 100, 60, 34, "TB6612 A (left)", TB_PINS, fc="#7b2cbf")
    s.board("tbb", 105, 18, 60, 34, "TB6612 B (right)", TB_PINS, fc="#7b2cbf")
    for k, (nm, y) in enumerate((("FL", 112), ("RL", 72), ("FR", 46), ("RR", 6))):
        s.board("m" + nm, 8, y, 40, 30, "MOTOR " + nm, MOT_PINS, fc="#555b66", sub={"FL": "front left", "RL": "rear left",
                                                                                   "FR": "front right", "RR": "rear right"}[nm])
    s.wire("mFL.M1", "tba.AO1", "MOT", 1, via=("x", 80))
    s.wire("mFL.M2", "tba.AO2", "MOT", 2, via=("x", 86))
    s.wire("mRL.M1", "tba.BO1", "MOT", 3, via=("x", 92))
    s.wire("mRL.M2", "tba.BO2", "MOT", 4, via=("x", 98))
    s.wire("mFR.M1", "tbb.AO1", "MOT", 5, via=("x", 80))
    s.wire("mFR.M2", "tbb.AO2", "MOT", 6, via=("x", 86))
    s.wire("mRR.M1", "tbb.BO1", "MOT", 7, via=("x", 92))
    s.wire("mRR.M2", "tbb.BO2", "MOT", 8, via=("x", 98))
    s.note(175, 128, "Each motor's cable has six wires.", fs=7.5, weight="bold")
    s.note(175, 121, "Read the labels on the encoder board:", fs=7.2)
    s.note(175, 114, "M1, M2 = motor power (this step)", fs=7.2)
    s.note(175, 107, "VCC, GND, C1, C2 = encoder (later)", fs=7.2)
    s.note(175, 96, "Extend each cable with 22 AWG", fs=7.2)
    s.note(175, 89, "(M1, M2) and 26 AWG (the rest).", fs=7.2)
    s.note(175, 78, "Tape-label the encoder wires", fs=7.2)
    s.note(175, 71, "\"FL C1\", \"FL C2\" ... and run them", fs=7.2)
    s.note(175, 64, "to the harness slot for step 24.", fs=7.2)
    return s


def s_servo():
    s = Sheet(270, 110)
    s.board("xl2", 140, 50, 56, 30, "XL4015 SERVO", XL_PINS, fc="#244b8f", sub="set to 5.0 V")
    s.board("sv", 10, 42, 54, 40, "MG946R SERVO", [("brown", "R", 0.2), ("red", "R", 0.5), ("orange", "R", 0.8)], fc="#3a3f4b")
    s.wire("sv.red", "xl2.OUT+", "SRV", 1, via=("x", 100))
    s.wire("sv.brown", "xl2.OUT-", "GND", 2, via=("x", 110))
    s.note(70, 22, "Orange = signal: extend it with 26 AWG, label it \"SERVO\" and leave it at the harness slot for step 24.", fs=7.5)
    s.note(10, 98, "Cut the servo plug off, strip 5 mm, solder to the SERVO regulator's output, heat-shrink each joint.", fs=7.5)
    return s


def s_ir():
    s = Sheet(270, 160)
    pcf = [("P0", "L", 0.12), ("P1", "L", 0.27), ("P2", "L", 0.42), ("P3", "L", 0.57), ("P4", "L", 0.72), ("P5", "L", 0.87),
           ("VCC", "R", 0.2), ("GND", "R", 0.4), ("SDA", "R", 0.6), ("SCL", "R", 0.8)]
    s.board("pcf", 150, 50, 50, 70, "PCF8574", pcf, fc="#244b8f", sub="address 0x20")
    names = [("FL", "edge front-left"), ("FR", "edge front-right"), ("R", "edge rear"), ("RL", "edge rear-left"),
             ("RR", "edge rear-right"), ("BAY", "bay sensor")]
    for i, (k, sub) in enumerate(names):
        y = 132 - i * 23
        s.board("ir" + k, 8, y, 42, 17, k, [("VCC", "R", 0.2), ("GND", "R", 0.5), ("DO", "R", 0.8)], fc="#555b66", sub=sub)
        s.wire("ir%s.DO" % k, "pcf.P%d" % i, "IR", i + 1, via=("x", 100 + i * 6))
    s.board("bus3", 225, 60, 38, 50, "3V3 / GND", [("3V3", "L", 0.3), ("GND", "L", 0.7)], fc="#7a5c2e", sub="small strip")
    s.wire("pcf.VCC", "bus3.3V3", "3V3", 7)
    s.wire("pcf.GND", "bus3.GND", "GND", 8)
    s.note(8, 6, "Daisy-chain VCC and GND of all six sensors to the 3V3 / GND strip (not drawn). PCF8574 SDA / SCL and the "
                 "strip's 3V3 lead go up to the carrier in step 24: label them.", fs=7)
    return s


def s_bundle():
    s = Sheet(270, 120)
    rows = [("5V", "logic regulator OUT+", "5V"), ("GND", "power bus -", "GND"), ("VBAT", "switch side of the power bus +, via nothing",
                                                                                    "VBAT"),
            ("3V3", "lower 3V3 / GND strip", "3V3"), ("SDA", "PCF8574 + front ToF", "SDA"), ("SCL", "PCF8574 + front ToF", "SCL"),
            ("XS-F", "front ToF XSHUT", "XS"), ("SERVO", "servo orange", "SRV"),
            ("A-IN1 A-IN2 A-BIN1 A-BIN2", "TB6612 A control", "PWM"), ("B-IN1 B-IN2 B-BIN1 B-BIN2", "TB6612 B control", "PWM"),
            ("STBY", "both TB6612 STBY, joined", "PWM"), ("FL/RL/FR/RR C1 C2", "8 encoder signals", "ENC")]
    for i, (tag, frm, k) in enumerate(rows):
        y = 108 - i * 8.2
        s.ax.add_patch(patches.FancyBboxPatch((10, y - 2.8), 42, 5.6, boxstyle="round,pad=0,rounding_size=1.2",
                                              fc="#fff3c4", ec="#8a6d00", lw=0.8))
        s.ax.text(31, y, tag, ha="center", va="center", fontsize=6.4, weight="bold", family="monospace")
        s.ax.plot([52, 120], [y, y], color=COL[k], lw=2.4)
        s.ax.text(124, y, frm, fontsize=7, va="center")
    s.ax.add_patch(patches.FancyBboxPatch((205, 20), 55, 90, boxstyle="round,pad=0,rounding_size=3", fc="#eef3fb", ec=NAVY))
    s.ax.text(232.5, 70, "HARNESS\nSLOT\n\nleave\n15 cm\nabove\nthe deck", ha="center", va="center", fontsize=8, color=NAVY,
              weight="bold")
    s.note(10, 6, "Masking-tape flag on every lead with the tag shown. Encoder VCC/GND and TB6612 VCC go to the 3V3 / GND strip; "
                  "tie TB6612 PWMA and PWMB to its own VCC with a short wire.", fs=7)
    return s


def s_up_power():
    s = Sheet(270, 120)
    s.board("car", 120, 30, 70, 60, "CARRIER", [("5V", "L", 0.2), ("GND", "L", 0.4), ("BATSENSE", "L", 0.6), ("3V3", "L", 0.8),
                                                ("5V ", "R", 0.3), ("GND ", "R", 0.6)], fc="#c9a46a", tc="#111", sub="ESP32-S3 not plugged in")
    s.board("har", 8, 30, 48, 60, "HARNESS", [("5V", "R", 0.2), ("GND", "R", 0.4), ("VBAT", "R", 0.6), ("3V3", "R", 0.8)],
            fc="#7a5c2e", sub="from the lower deck")
    s.board("cam", 220, 40, 40, 40, "ESP32-CAM", [("5V", "L", 0.3), ("GND", "L", 0.6)], fc="#3a3f4b", sub="on the mast")
    s.wire("har.5V", "car.5V", "5V", 1)
    s.wire("har.GND", "car.GND", "GND", 2)
    s.wire("har.VBAT", "car.BATSENSE", "VBAT", 3)
    s.wire("har.3V3", "car.3V3", "3V3", 4)
    s.wire("car.5V ", "cam.5V", "5V", 5)
    s.wire("car.GND ", "cam.GND", "GND", 6)
    s.note(8, 106, "BATSENSE is the top of the 100 k / 20 k divider on the carrier.", fs=7.5)
    return s


def s_i2c():
    s = Sheet(270, 160)
    s.board("car", 196, 50, 54, 70, "CARRIER", [("SDA", "L", 0.15), ("SCL", "L", 0.3), ("GPIO10", "L", 0.7),
                                               ("GPIO11", "L", 0.85)], fc="#c9a46a", tc="#111", sub="GPIO 8 = SDA, 9 = SCL")
    s.board("imu", 8, 124, 40, 24, "MPU-6050", [("SDA", "R", 0.3), ("SCL", "R", 0.7)], fc="#244b8f")
    s.board("tl", 8, 84, 40, 30, "ToF LEFT", [("SDA", "R", 0.2), ("SCL", "R", 0.45), ("XSHUT", "R", 0.8)], fc="#222")
    s.board("tr", 8, 44, 40, 30, "ToF RIGHT", [("SDA", "R", 0.2), ("SCL", "R", 0.45), ("XSHUT", "R", 0.75), ("VIN", "R", 0.92)],
            fc="#222")
    s.board("har", 8, 6, 40, 30, "HARNESS", [("SDA", "R", 0.2), ("SCL", "R", 0.45), ("XS-F", "R", 0.8)], fc="#7a5c2e",
            sub="PCF8574 + front ToF")
    s.wire("imu.SDA", "car.SDA", "SDA", 1, via=("x", 150))
    s.wire("imu.SCL", "car.SCL", "SCL", 2, via=("x", 156))
    s.wire("tl.SDA", "car.SDA", "SDA", 3, via=("x", 138))
    s.wire("tl.SCL", "car.SCL", "SCL", 4, via=("x", 144))
    s.wire("tr.SDA", "car.SDA", "SDA", 5, via=("x", 126))
    s.wire("tr.SCL", "car.SCL", "SCL", 6, via=("x", 132))
    s.wire("har.SDA", "car.SDA", "SDA", 7, via=("x", 114))
    s.wire("har.SCL", "car.SCL", "SCL", 8, via=("x", 120))
    s.wire("har.XS-F", "car.GPIO10", "XS", 9, via=("x", 168))
    s.wire("tl.XSHUT", "car.GPIO11", "XS", 10, via=("x", 174))
    s.wire("tr.XSHUT", "tr.VIN", "3V3", 11, via=("x", 62))
    s.note(60, 154, "All SDA wires meet at the carrier's GPIO 8 pad, all SCL at GPIO 9 (pull-ups already on the carrier).", fs=7)
    s.note(60, 148, "Every module's VIN / VCC to 3V3 and GND to GND as well (not drawn).", fs=7)
    return s


def s_mics():
    s = Sheet(270, 130)
    mic = [("SCK", "R", 0.12), ("WS", "R", 0.3), ("SD", "R", 0.48), ("L/R", "R", 0.66), ("VDD", "R", 0.82), ("GND", "R", 0.96)]
    s.board("ml", 8, 74, 40, 40, "MIC LEFT", mic, fc="#244b8f")
    s.board("mr", 8, 12, 40, 40, "MIC RIGHT", mic, fc="#244b8f")
    s.board("car", 170, 40, 60, 60, "CARRIER", [("GPIO4", "L", 0.15), ("GPIO5", "L", 0.35), ("GPIO6", "L", 0.55),
                                               ("3V3", "L", 0.75), ("GND", "L", 0.92)], fc="#c9a46a", tc="#111")
    for side, b in (("ml", 0), ("mr", 1)):
        s.wire(side + ".SCK", "car.GPIO4", "I2S", 1 + b * 6, via=("x", 100 + b * 30))
        s.wire(side + ".WS", "car.GPIO5", "I2S", 2 + b * 6, via=("x", 106 + b * 30))
        s.wire(side + ".SD", "car.GPIO6", "I2S", 3 + b * 6, via=("x", 112 + b * 30))
    s.wire("ml.L/R", "car.GND", "GND", 4, via=("x", 90))
    s.wire("mr.L/R", "car.3V3", "3V3", 10, via=("x", 160))
    s.note(60, 122, "LEFT mic: L/R to GND.  RIGHT mic: L/R to 3V3.  Both: VDD to 3V3, GND to GND.", fs=7.8, weight="bold")
    s.note(60, 4, "Keep these wires under 15 cm and away from the motor leads: audio is the most noise-sensitive signal.", fs=7)
    return s


def s_motor_ctrl():
    s = Sheet(270, 150)
    names = [("A-IN1", "12"), ("A-IN2", "13"), ("A-BIN1", "14"), ("A-BIN2", "15"), ("B-IN1", "16"), ("B-IN2", "17"),
             ("B-BIN1", "18"), ("B-BIN2", "21"), ("STBY", "46")]
    s.board("har", 8, 10, 50, 130, "HARNESS", [(n, "R", (i + 0.5) / len(names)) for i, (n, _) in enumerate(names)], fc="#7a5c2e",
            sub="tags from step 16")
    s.board("car", 180, 10, 50, 130, "CARRIER", [("GPIO" + g, "L", (i + 0.5) / len(names)) for i, (_, g) in enumerate(names)],
            fc="#c9a46a", tc="#111")
    for i, (n, g) in enumerate(names):
        s.wire("har." + n, "car.GPIO" + g, "PWM", i + 1)
    s.note(70, 145, "A = TB6612 A (left wheels: IN = front-left, BIN = rear-left).  B = TB6612 B (right wheels).", fs=7.5)
    return s


def s_enc():
    s = Sheet(270, 150)
    names = [("FL C1", "38"), ("FL C2", "39"), ("RL C1", "40"), ("RL C2", "41"), ("FR C1", "42"), ("FR C2", "47"),
             ("RR C1", "48"), ("RR C2", "7"), ("SERVO", "3")]
    s.board("har", 8, 10, 50, 130, "HARNESS", [(n, "R", (i + 0.5) / len(names)) for i, (n, _) in enumerate(names)], fc="#7a5c2e")
    s.board("car", 180, 10, 50, 130, "CARRIER", [("GPIO" + g, "L", (i + 0.5) / len(names)) for i, (_, g) in enumerate(names)],
            fc="#c9a46a", tc="#111")
    for i, (n, g) in enumerate(names):
        s.wire("har." + n, "car.GPIO" + g, "SRV" if n == "SERVO" else "ENC", i + 1)
    return s


def s_first_power():
    s = Sheet(270, 120)
    steps = ["ESP32-S3 OUT of its sockets. Switch OFF. Battery plugged in.",
             "Meter on continuity: carrier 5V to GND, 3V3 to GND, battery + to GND: no beep on any.",
             "Switch ON. Carrier 5V pad reads 4.9-5.1 V. Servo regulator output reads 4.9-5.1 V.",
             "Switch OFF. Plug the ESP32-S3 in, USB toward the rear. Switch ON: its red LED lights.",
             "Carrier 3V3 pad now reads 3.2-3.4 V (from the ESP32-S3's regulator).",
             "Flash and run bring-up: AEGIS_Deployment_Guide.pdf, sections 2-5."]
    for i, t in enumerate(steps):
        y = 104 - i * 16
        s.ax.add_patch(patches.Circle((16, y), 5, fc=NAVY))
        s.ax.text(16, y, str(i + 1), color="white", ha="center", va="center", fontsize=9, weight="bold")
        s.ax.text(28, y, t, va="center", fontsize=9)
    return s


STEPS = [
    ("A", "Before assembly: solder headers on the modules", s_headers,
     [("VL53L0X, MPU-6050, INMP441, TB6612 headers", "solder", "-")],
     "Each module sits flat and square on its header; no solder bridges between pins (check with a magnifier)."),
    ("A", "Before assembly: build the ESP32-S3 carrier board", s_carrier,
     [("2.2 k resistors x 2, 100 k x 1, 20 k x 1", "resistor kit", "#34"), ("1x20 female strips", "4", "#32")],
     "Meter on resistance: GPIO 8 to 3V3 reads 2.2 k, GPIO 9 to 3V3 reads 2.2 k, GPIO 1 to GND reads 20 k."),
    ("A", "Before assembly: battery lead, fuse, switch, power bus", s_fuse_switch,
     [("18 AWG red, 3 pieces x 15 cm", "", "#24"), ("18 AWG black, 1 x 25 cm", "", "#25"), ("Heat-shrink", "", "#35")],
     "Switch OFF: no continuity from battery + to the bus +. Switch ON: continuity. Never any continuity from + to -."),
    ("C", "Step 16 a: power to regulators and motor drivers", s_power,
     [("18 AWG red / black, about 15 cm each", "8 wires", "#24, #25")],
     "Switch OFF, battery unplugged: no continuity between + and - anywhere."),
    ("C", "Step 16 b: set both regulators to 5.0 V", s_set5v, [("Multimeter", "lab", "-")],
     "Both regulators read 4.95-5.05 V with nothing connected to their outputs."),
    ("C", "Step 16 c: motors to the motor drivers", s_motors,
     [("22 AWG 3-core, 4 x 20 cm", "", "#26"), ("26 AWG for encoder extensions", "", "#27-#30")],
     "Each motor: M1 to M2 reads a few ohms through the motor."),
    ("C", "Step 16 d: servo power", s_servo, [("22 AWG red / black, 2 x 15 cm", "", "#26")],
     "Servo red to brown: no short (more than 1 k)."),
    ("C", "Step 16 e: edge sensors and bay sensor to the PCF8574", s_ir,
     [("26 AWG white (signals), red / black (3V3, GND)", "about 4 m", "#27-#30")],
     "Each sensor's DO has continuity to its PCF8574 pin P0-P5 and to nothing else."),
    ("C", "Step 16 f: label and route the leads for the upper deck", s_bundle, [("Masking tape, pen", "your own", "-")],
     "Every lead has a tag and reaches 15 cm above the deck through the harness slot."),
    ("E", "Step 24 a: power to the carrier and the camera", s_up_power, [("22 AWG red / black", "", "#26")],
     "Carrier 5V to GND: no short. BATSENSE to GND reads about 20 k."),
    ("E", "Step 24 b: I2C sensors (gyro, distance sensors, PCF8574)", s_i2c,
     [("26 AWG white (SDA), green (SCL), red / black (3V3, GND)", "", "#27-#30")],
     "GPIO 8 has continuity to every SDA pin; GPIO 9 to every SCL pin; SDA and SCL are not connected to each other."),
    ("E", "Step 24 c: microphones", s_mics, [("26 AWG, under 15 cm each", "12 wires", "#27-#30")],
     "Left mic L/R to GND: continuity. Right mic L/R to 3V3: continuity."),
    ("E", "Step 24 d: motor-driver control", s_motor_ctrl, [("Harness leads from step 16", "9", "-")],
     "Each tagged lead has continuity to its GPIO pad and to no neighbour."),
    ("E", "Step 24 e: encoders and servo signal", s_enc, [("Harness leads from step 16", "9", "-")],
     "Each tagged lead has continuity to its GPIO pad and to no neighbour."),
    ("E", "First power-up", s_first_power, [("Multimeter", "lab", "-")],
     "If any reading is wrong: switch OFF and find it before plugging the ESP32-S3 in."),
]

PHASES = {"A": "Bench, before assembly", "C": "Assembly step 16: lower deck", "E": "Assembly step 24: upper deck"}

CSS = """
@page { size: A4 landscape; margin: 11mm 13mm 13mm 13mm;
  @bottom-left { content: "A.E.G.I.S. Mk2  |  Wiring guide"; font: 8pt 'Helvetica Neue', Arial; color: #8a93a3; }
  @bottom-right { content: "page " counter(page) " of " counter(pages); font: 8pt 'Helvetica Neue', Arial; color: #8a93a3; } }
@page :first { margin: 0; @bottom-left { content: none; } @bottom-right { content: none; } }
body { margin: 0; font: 10pt/1.45 'Helvetica Neue', Helvetica, Arial, sans-serif; color: #1d2433; }
.page { position: relative; width: 271mm; height: 184mm; page-break-after: always; overflow: hidden; }
.head { position: absolute; left: 0; top: 0; width: 271mm; height: 15mm; display: flex; align-items: center; gap: 4mm;
  border-bottom: 2.5px solid %(orange)s; }
.num { min-width: 12.5mm; height: 12.5mm; border-radius: 6mm; background: %(navy)s; color: white; font: 800 14pt 'Helvetica Neue';
  display: flex; align-items: center; justify-content: center; padding: 0 2mm; box-sizing: border-box; }
.head h2 { margin: 0; font-size: 16pt; color: %(navy)s; }
.phase { margin-left: auto; font: 700 9pt 'Helvetica Neue'; color: white; background: %(orange)s; padding: 1.2mm 3mm; border-radius: 3mm; }
.fig { position: absolute; left: 0; top: 18mm; width: 200mm; height: 118mm; display: flex; align-items: center; justify-content: center; }
.fig img { max-width: 200mm; max-height: 118mm; }
.fig.short { height: 84mm; }
.fig.short img { max-height: 84mm; }
.tables { position: absolute; left: 0; top: 104mm; width: 200mm; height: 35mm; display: flex; gap: 4mm; overflow: hidden; }
.wt { border-collapse: collapse; font-size: 7.4pt; flex: 1; align-self: flex-start; }
.wt th { background: %(navy)s; color: white; text-align: left; padding: 0.8mm 1.6mm; font-weight: 600; }
.wt td { padding: 0.6mm 1.6mm; border-bottom: 1px solid #e2e6ee; }
.wt .n { display: inline-block; width: 4.2mm; height: 4.2mm; line-height: 4.2mm; border-radius: 50%%; border: 1.6px solid;
  text-align: center; font-weight: 700; font-size: 6.6pt; }
.side { position: absolute; left: 206mm; top: 18mm; width: 65mm; height: 166mm; background: #f4f6fa; border-radius: 4px;
  padding: 3mm; box-sizing: border-box; }
.side h3 { margin: 0 0 2mm; font-size: 9.5pt; letter-spacing: 1.5px; color: %(navy)s; }
.need { background: white; border-radius: 3px; padding: 1.6mm 2mm; margin-bottom: 1.6mm; font-size: 8.4pt; }
.need i { color: #8a93a3; font-style: normal; font-size: 7.4pt; }
.legend { margin-top: 3mm; font-size: 7.6pt; }
.legend div { display: flex; align-items: center; gap: 2mm; margin: 0.6mm 0; }
.legend span { width: 9mm; height: 2.4mm; border-radius: 1mm; display: inline-block; flex: none; }
.check { position: absolute; left: 0; top: 140mm; width: 200mm; padding: 2.4mm 3.5mm; box-sizing: border-box; background: #eaf7ee;
  border-left: 4px solid #2e9e57; border-radius: 3px; font-size: 9.6pt; }
.check b { color: #1f7a43; letter-spacing: 1px; }
.cover { height: 210mm; width: 297mm; background: %(navy)s; color: white; position: relative; page-break-after: always; overflow: hidden; }
.cover .band { position: absolute; left: 0; top: 0; width: 9mm; height: 100%%; background: %(orange)s; }
.cover .brand { position: absolute; left: 22mm; top: 18mm; letter-spacing: 3px; color: #c9d2e3; font-size: 12pt; }
.cover h1 { position: absolute; left: 22mm; top: 25mm; margin: 0; font-size: 34pt; }
.cover .sub { position: absolute; left: 22mm; top: 46mm; font-size: 12pt; color: #dfe5ef; }
.order { position: absolute; left: 22mm; top: 62mm; width: 255mm; }
.order .row { display: flex; align-items: stretch; margin-bottom: 3mm; }
.order .tag { width: 42mm; background: %(orange)s; color: white; font-weight: 800; display: flex; align-items: center;
  justify-content: center; border-radius: 3px 0 0 3px; font-size: 10.5pt; }
.order .tag.assy { background: #4a5878; }
.order .txt { flex: 1; background: rgba(255,255,255,0.08); padding: 2.6mm 4mm; font-size: 10pt; color: #e6ebf3; border-radius: 0 3px 3px 0; }
.rules { page-break-after: always; }
.rules h2 { color: %(navy)s; border-bottom: 2.5px solid %(orange)s; padding-bottom: 2mm; margin: 0 0 4mm; }
.rules .cols { display: flex; gap: 8mm; }
.rules .cols > div { flex: 1; }
.rules li { margin: 1.2mm 0; font-size: 9.8pt; }
.rules table { border-collapse: collapse; font-size: 9pt; width: 100%%; }
.rules td { padding: 1.4mm 2mm; border-bottom: 1px solid #dde2ea; }
.appx img { width: 271mm; max-height: 184mm; object-fit: contain; page-break-after: always; display: block; }
"""


def main():
    pages = []
    for i, (ph, title, fn, need, check) in enumerate(STEPS, 1):
        kinds = set()
        sheet = fn()
        img = sheet.png()
        needs = "".join('<div class="need">%s<br><i>%s %s</i></div>' %
                        (html.escape(a), html.escape(b), ("item " + html.escape(c)) if c not in ("-", "") else "")
                        for a, b, c in need)
        used = []
        for r in sheet.rows:
            if r[3] not in used:
                used.append(r[3])
        legend = "".join('<div><span style="background:%s"></span>%s: %s</div>' % (COL[k], NAME[k], html.escape(WIRE[k]))
                         for k in used)
        table = ""
        if sheet.rows:
            rows = sorted(sheet.rows)
            half = (len(rows) + 1) // 2
            def tab(rs):
                return ('<table class="wt"><tr><th>#</th><th>From</th><th>To</th><th>Wire</th></tr>%s</table>' %
                        "".join('<tr><td><span class="n" style="border-color:%s">%d</span></td><td>%s</td><td>%s</td>'
                                '<td>%s</td></tr>' % (COL[k], n, html.escape(f), html.escape(t),
                                                      html.escape(WIRE[k].split(" (")[0].split(" / ")[0]))
                                for n, f, t, k in rs))
            table = '<div class="tables">%s%s</div>' % (tab(rows[:half]), tab(rows[half:]) if rows[half:] else "")
        del kinds
        pages.append('<div class="page"><div class="head"><div class="num">W%d</div><h2>%s</h2><div class="phase">%s</div></div>'
                     '<div class="fig%s"><img src="%s"></div>%s<div class="side"><h3>YOU NEED</h3>%s'
                     '%s</div>'
                     '<div class="check"><b>CHECK BEFORE YOU GO ON.</b> %s</div></div>' %
                     (i, html.escape(title), PHASES[ph], " short" if table else "", img, table, needs,
                      ('<h3 style="margin-top:3mm">WIRE COLOURS</h3><div class="legend">%s</div>' % legend) if legend else "",
                      html.escape(check)))
    order = [("BENCH", "Solder headers on the modules, build the ESP32-S3 carrier, the fuse board, the switch leads and the power bus "
                       "(W1-W3). Easier on the bench than inside the rover.", ""),
             ("ASSEMBLY 1-15", "Motors, wheels, edge sensors, intake bay, battery, lower-deck boards (Assembly guide).", "assy"),
             ("STEP 16: WIRE", "Lower deck: power, set 5.0 V, motors, servo power, IR sensors, then label the leads for the "
                               "upper deck (W4-W9).", ""),
             ("ASSEMBLY 17-23", "Pillars, upper deck, switch, carrier with the ESP32-S3 removed, gyro, mics, side sensors.", "assy"),
             ("STEP 24: WIRE", "Upper deck: power, I2C, microphones, motor control, encoders and servo signal (W10-W14).", ""),
             ("POWER-UP", "First power-up checks (W15), then assembly 25-27 and the Deployment guide.", "")]
    cover = ('<div class="cover"><div class="band"></div><div class="brand">A.E.G.I.S.  MK2</div><h1>Wiring guide</h1>'
             '<div class="sub">%d steps, each with its own drawing and a check. Wire as you assemble, in this order:</div>'
             '<div class="order">%s</div></div>' %
             (len(STEPS), "".join('<div class="row"><div class="tag %s">%s</div><div class="txt">%s</div></div>' % (c, t, html.escape(x))
                                  for t, x, c in order)))
    rules = ('<div class="rules"><h2>Rules for every wire</h2><div class="cols"><div><ol>'
             '<li><b>Battery unplugged</b> whenever you solder or move a wire.</li>'
             '<li>Cut each wire 3 cm longer than the route, strip 5 mm, twist the strands, tin them.</li>'
             '<li>Slide heat-shrink on <b>before</b> soldering; shrink it over every power joint.</li>'
             '<li>Tug test: pull each finished joint gently. It must not move.</li>'
             '<li>Red = plus, black = ground, always. Signal wires: white or green, flagged with masking tape.</li>'
             '<li>One row of the table at a time; tick it off on a printed copy.</li>'
             '<li>Never connect a regulator output before it reads 5.0 V (W5).</li></ol></div>'
             '<div><table><tr><td><b>Wire</b></td><td><b>Use</b></td></tr>'
             '<tr><td>18 AWG red / black</td><td>battery, fuse, switch, power bus, regulator and driver inputs</td></tr>'
             '<tr><td>22 AWG 3-core</td><td>5 V outputs, servo power, motor M1 / M2</td></tr>'
             '<tr><td>26 AWG red / black</td><td>3.3 V and ground to sensors</td></tr>'
             '<tr><td>26 AWG white / green</td><td>all signals</td></tr></table>'
             '<p style="font-size:9pt;margin-top:4mm">Pin numbers match the firmware (aegis_config.h). The reference pages at the '
             'end show every board face and the full pin map. Clone boards sometimes order their pins differently: '
             '<b>always read the label printed on the board</b>.</p></div></div></div>')
    appx = "".join('<div class="appx"><img src="data:image/png;base64,%s"></div>' %
                   base64.b64encode(open(os.path.join(REF, "wiring_reference_p%d.png" % k), "rb").read()).decode()
                   for k in (1, 2, 3))
    doc = ("<!doctype html><html><head><meta charset='utf-8'><style>%s</style></head><body>%s%s%s%s</body></html>" %
           (CSS % dict(navy=NAVY, orange=ORANGE), cover, rules, "".join(pages), appx))
    with tempfile.NamedTemporaryFile("w", suffix=".html", delete=False) as f:
        f.write(doc)
        tmp = f.name
    subprocess.run([CHROME, "--headless", "--disable-gpu", "--no-pdf-header-footer", "--print-to-pdf=" + OUT, "file://" + tmp],
                   check=True, capture_output=True)
    os.remove(tmp)
    print(OUT)


if __name__ == "__main__":
    main()
