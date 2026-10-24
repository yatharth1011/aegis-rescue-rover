"""
A.E.G.I.S. wiring guide: to-scale board faces with every used pin labelled,
plus the full wire list. Pin assignments match firmware/aegis_s3/main/aegis_config.h.
Colours match the Fusion wiring harness (Build_AEGIS_Rover_Detailed).

    python3 team_share/tools/wiring_guide.py   ->  team_share/AEGIS_Wiring_Guide.pdf (+ PNG per page)

Header pin orders follow the reference boards (ESP32-S3-DevKitC-1 v1.1, SparkFun TB6612FNG,
GY-521, GY-530). Clone boards sometimes reorder pins: check the silkscreen of the boards you
receive before soldering, and fix the table here if they differ.
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.backends.backend_pdf import PdfPages

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "AEGIS_Wiring_Guide.pdf")

C = {"12V": "#d01c1c", "GND": "#111111", "5V": "#e8590c", "5V_servo": "#f59f00", "3V3": "#c2255c", "SDA": "#f2c500",
     "SCL": "#2b9348", "I2S": "#1c64d8", "UART": "#9aa0a6", "PWM": "#7b2cbf", "ENC": "#8d5524", "XSHUT": "#5c7cfa",
     "IR": "#4dabf7", "VBAT": "#15aabf", "MOTOR": "#d01c1c"}

# ESP32-S3-DevKitC-1 v1.1: J1 (left) and J3 (right), antenna end first
J1 = ["3V3", "3V3", "RST", "4", "5", "6", "7", "15", "16", "17", "18", "8", "3", "46", "9", "10", "11", "12", "13", "14", "5V", "GND"]
J3 = ["GND", "TX", "RX", "1", "2", "42", "41", "40", "39", "38", "37", "36", "35", "0", "45", "48", "47", "21", "20", "19", "GND", "GND"]
USE = {  # GPIO -> (function, colour key)
    "4": ("I2S BCLK (both mics)", "I2S"), "5": ("I2S WS", "I2S"), "6": ("I2S SD (data)", "I2S"),
    "8": ("I2C SDA", "SDA"), "9": ("I2C SCL", "SCL"), "10": ("XSHUT front ToF", "XSHUT"), "11": ("XSHUT left ToF", "XSHUT"),
    "12": ("FL motor IN1", "PWM"), "13": ("FL motor IN2", "PWM"), "14": ("RL motor IN1", "PWM"), "15": ("RL motor IN2", "PWM"),
    "16": ("FR motor IN1", "PWM"), "17": ("FR motor IN2", "PWM"), "18": ("RR motor IN1", "PWM"), "21": ("RR motor IN2", "PWM"),
    "46": ("STBY both TB6612", "PWM"), "38": ("FL enc A", "ENC"), "39": ("FL enc B", "ENC"), "40": ("RL enc A", "ENC"),
    "41": ("RL enc B", "ENC"), "42": ("FR enc A", "ENC"), "47": ("FR enc B", "ENC"), "48": ("RR enc A", "ENC"), "7": ("RR enc B", "ENC"),
    "3": ("Door servo signal", "5V_servo"), "2": ("ESP32-CAM TX -> RX", "UART"), "1": ("Battery sense (divider)", "VBAT"),
    "0": ("BOOT = start/abort", "UART"), "5V": ("5 V from logic buck", "5V"), "GND": ("Ground", "GND"), "3V3": ("3V3 to sensors", "3V3"),
}

BOARDS = [  # name, w, h (mm), list of (side, [(pin, function, colour)])
    ("TB6612FNG (x2: A = left wheels, B = right)", 20.3, 20.3, [
        ("bottom", [("VM", "Battery + (10.5-12.6 V)", "12V"), ("VCC", "3V3", "3V3"), ("GND", "GND", "GND"), ("AO1", "front motor +", "MOTOR"),
                    ("AO2", "front motor -", "GND"), ("BO2", "rear motor -", "GND"), ("BO1", "rear motor +", "MOTOR"), ("GND", "GND", "GND")]),
        ("top", [("PWMA", "tie to 3V3", "3V3"), ("AIN2", "IN2 (GPIO 13/17)", "PWM"), ("AIN1", "IN1 (GPIO 12/16)", "PWM"),
                 ("STBY", "GPIO 46", "PWM"), ("BIN1", "IN1 (GPIO 14/18)", "PWM"), ("BIN2", "IN2 (GPIO 15/21)", "PWM"),
                 ("PWMB", "tie to 3V3", "3V3"), ("GND", "GND", "GND")])]),
    ("XL4015 buck (x2: logic 5.0 V, servo 5.0 V)", 51, 26, [
        ("left", [("IN+", "Battery +", "12V"), ("IN-", "Battery -", "GND")]),
        ("right", [("OUT+", "5.0 V out (set BEFORE load)", "5V"), ("OUT-", "GND", "GND")])]),
    ("GY-530 VL53L0X (x3: front, left, right)", 25, 12, [
        ("left", [("VIN", "3V3", "3V3"), ("GND", "GND", "GND"), ("SCL", "I2C SCL", "SCL"), ("SDA", "I2C SDA", "SDA"),
                  ("GPIO1", "not used", "UART"), ("XSHUT", "GPIO 10 / 11 / tie high (right)", "XSHUT")])]),
    ("GY-521 MPU-6050", 21, 16, [
        ("bottom", [("VCC", "3V3", "3V3"), ("GND", "GND", "GND"), ("SCL", "I2C SCL", "SCL"), ("SDA", "I2C SDA", "SDA"),
                    ("XDA", "-", "UART"), ("XCL", "-", "UART"), ("AD0", "GND (addr 0x68)", "GND"), ("INT", "-", "UART")])]),
    ("INMP441 mic (x2)", 14, 14, [
        ("bottom", [("L/R", "L: GND / R: 3V3", "GND"), ("WS", "GPIO 5", "I2S"), ("SCK", "GPIO 4", "I2S"), ("SD", "GPIO 6", "I2S"),
                    ("VDD", "3V3", "3V3"), ("GND", "GND", "GND")])]),
    ("PCF8574T I/O expander", 40, 16, [
        ("left", [("GND", "GND", "GND"), ("VCC", "3V3", "3V3"), ("SDA", "I2C SDA", "SDA"), ("SCL", "I2C SCL", "SCL")]),
        ("bottom", [("P0", "cliff FL", "IR"), ("P1", "cliff FR", "IR"), ("P2", "cliff rear", "IR"), ("P3", "bay capture", "IR")])]),
    ("TCRT5000 / IR obstacle module (x4)", 32, 14, [
        ("left", [("VCC", "3V3", "3V3"), ("GND", "GND", "GND"), ("DO", "PCF8574 P0-P3", "IR")])]),
    ("MG946R servo  |  N20 encoder motor (x4)", 40.7, 19.7, [
        ("left", [("brown", "GND", "GND"), ("red", "servo buck 5.0 V", "5V_servo"), ("orange", "GPIO 3", "5V_servo")]),
        ("right", [("red", "M+ to AO1/BO1", "MOTOR"), ("white", "M- to AO2/BO2", "GND"), ("black", "encoder VCC 3V3", "3V3"),
                   ("blue", "encoder GND", "GND"), ("yellow", "enc A", "ENC"), ("green", "enc B", "ENC")])]),
]

WIRES = [  # from, to, signal, colour key, gauge
    ("Battery + (DC jack lead)", "Both XL4015 IN+, both TB6612 VM", "10.5-12.6 V", "12V", "20 AWG"),
    ("Battery -", "Both XL4015 IN-, all GND", "0 V", "GND", "20 AWG"),
    ("XL4015 logic OUT+", "ESP32-S3 5V, ESP32-CAM 5V", "5.0 V", "5V", "22 AWG"),
    ("XL4015 servo OUT+", "MG946R red", "5.0 V", "5V_servo", "22 AWG"),
    ("ESP32-S3 3V3", "sensors VCC, TB6612 VCC + PWMA/PWMB, encoders", "3.3 V", "3V3", "26 AWG"),
    ("ESP32 GPIO 8 / 9", "VL53L0X x3, MPU-6050, PCF8574 (SDA/SCL), 2.2 k pull-ups", "I2C 400 kHz", "SDA", "26 AWG"),
    ("ESP32 GPIO 4 / 5 / 6", "both INMP441 SCK / WS / SD", "I2S 48 kHz", "I2S", "26 AWG, short"),
    ("ESP32 GPIO 10 / 11", "front / left VL53L0X XSHUT (right: tie to VIN)", "address select", "XSHUT", "26 AWG"),
    ("ESP32 GPIO 12-18, 21", "TB6612 AIN1/AIN2/BIN1/BIN2", "20 kHz PWM", "PWM", "26 AWG"),
    ("ESP32 GPIO 46", "STBY on both TB6612", "enable", "PWM", "26 AWG"),
    ("TB6612 AO1/AO2/BO1/BO2", "motor red / white", "motor power", "MOTOR", "24 AWG"),
    ("Encoders yellow / green", "GPIO 38/39, 40/41, 42/47, 48/7", "quadrature", "ENC", "26 AWG"),
    ("ESP32 GPIO 3", "MG946R orange", "50 Hz servo pulse", "5V_servo", "26 AWG"),
    ("ESP32-CAM U0T", "ESP32 GPIO 2", "UART 115200", "UART", "26 AWG"),
    ("Battery + via 100k / 22k divider", "ESP32 GPIO 1", "battery voltage", "VBAT", "26 AWG"),
    ("IR modules DO", "PCF8574 P0-P3", "cliff / capture", "IR", "26 AWG"),
]


def draw_esp32(ax):
    w, h = 25.4, 63.0
    ax.add_patch(patches.FancyBboxPatch((0, 0), w, h, boxstyle="round,pad=0,rounding_size=1.5", fc="#1b1b1f", ec="black"))
    ax.add_patch(patches.Rectangle((3.6, 37.5), 18.0, 25.5, fc="#2f5d9e", ec="#555"))
    ax.add_patch(patches.Rectangle((4.8, 40.0), 15.8, 17.6, fc="#c9ccd1", ec="#777"))
    ax.text(w / 2, 48.5, "ESP32-S3\nWROOM-1", ha="center", va="center", fontsize=7, weight="bold")
    for x in (6.5, 18.9):
        ax.add_patch(patches.Rectangle((x - 4.5, -0.8), 9.0, 7.5, fc="#c9ccd1", ec="#666"))
    ax.text(w / 2, 9, "USB-C  x2", ha="center", fontsize=6, color="white")
    pitch = 2.54
    for side, pins, x in (("L", J1, 1.27), ("R", J3, w - 1.27)):
        for i, p in enumerate(pins):
            y = h - 3.0 - i * pitch * (57.0 / (21 * pitch))
            name = p if p in ("3V3", "5V", "GND", "RST", "TX", "RX") else "IO" + p
            fn = USE.get(p)
            col = C[fn[1]] if fn else "#555"
            ax.add_patch(patches.Circle((x, y), 0.7, fc=col if fn else "#888", ec="white", lw=0.4))
            if side == "L":
                ax.text(x - 2.0, y, name, ha="right", va="center", fontsize=6.5, family="monospace")
                if fn and p not in ("GND", "3V3"):
                    ax.plot([x - 9.5, x - 30], [y, y], color=col, lw=2.2, solid_capstyle="round")
                    ax.text(x - 31, y, fn[0], ha="right", va="center", fontsize=6.5)
            else:
                ax.text(x + 2.0, y, name, ha="left", va="center", fontsize=6.5, family="monospace")
                if fn and p not in ("GND", "3V3"):
                    ax.plot([x + 9.5, x + 30], [y, y], color=col, lw=2.2, solid_capstyle="round")
                    ax.text(x + 31, y, fn[0], ha="left", va="center", fontsize=6.5)
    ax.set_xlim(-75, w + 75)
    ax.set_ylim(-5, h + 5)
    ax.set_aspect("equal")
    ax.axis("off")


def draw_board(ax, name, w, h, sides):
    ax.add_patch(patches.FancyBboxPatch((0, 0), w, h, boxstyle="round,pad=0,rounding_size=1.0", fc="#24539c", ec="black"))
    ax.text(w / 2, h / 2, "%g x %g mm" % (w, h), ha="center", va="center", color="white", fontsize=7)
    for side, pins in sides:
        n = len(pins)
        for i, (pin, fn, ck) in enumerate(pins):
            col = C[ck]
            if side in ("left", "right"):
                y = h / 2 + ((n - 1) / 2 - i) * min(2.54, (h - 2) / max(n - 1, 1)) * (1 if n > 1 else 0)
                x = 1.3 if side == "left" else w - 1.3
                dx = -1 if side == "left" else 1
                ax.add_patch(patches.Circle((x, y), 0.65, fc=col, ec="white", lw=0.4))
                ax.plot([x + dx * 1.2, x + dx * 9], [y, y], color=col, lw=2.0)
                ax.text(x + dx * 10, y, "%s  %s" % (pin, fn) if dx > 0 else "%s  %s" % (fn, pin), ha="left" if dx > 0 else "right",
                        va="center", fontsize=6)
            else:
                x = w / 2 + (i - (n - 1) / 2) * min(2.54, (w - 2) / max(n - 1, 1))
                y = 1.3 if side == "bottom" else h - 1.3
                dy = -1 if side == "bottom" else 1
                ax.add_patch(patches.Circle((x, y), 0.65, fc=col, ec="white", lw=0.4))
                ax.plot([x, x], [y + dy * 1.2, y + dy * 7], color=col, lw=2.0)
                ax.text(x, y + dy * 8, "%s  %s" % (pin, fn) if dy > 0 else "%s  %s" % (fn, pin), ha="center",
                        va="bottom" if dy > 0 else "top", fontsize=6, rotation=90, rotation_mode="anchor" if False else "default")
    ax.set_title(name, fontsize=9, weight="bold")
    ax.set_xlim(-42, w + 42)
    ax.set_ylim(-40, h + 40)
    ax.set_aspect("equal")
    ax.axis("off")


def legend(fig):
    names = [("Battery 12 V", "12V"), ("Ground", "GND"), ("5 V logic", "5V"), ("5 V servo", "5V_servo"), ("3.3 V", "3V3"),
             ("I2C SDA", "SDA"), ("I2C SCL", "SCL"), ("I2S audio", "I2S"), ("Motor PWM / STBY", "PWM"), ("Encoders", "ENC"),
             ("ToF XSHUT", "XSHUT"), ("IR sensors", "IR"), ("UART / misc", "UART"), ("Battery sense", "VBAT")]
    hs = [patches.Patch(color=C[k], label=n) for n, k in names]
    fig.legend(handles=hs, loc="lower center", ncol=7, fontsize=7, frameon=False)


def main():
    pages = []
    fig = plt.figure(figsize=(11.7, 8.3))
    fig.suptitle("A.E.G.I.S. wiring guide 1/3: ESP32-S3-DevKitC-1 pin assignment (top view, antenna up)", fontsize=12, weight="bold")
    ax = fig.add_axes([0.03, 0.08, 0.94, 0.86])
    draw_esp32(ax)
    legend(fig)
    pages.append(fig)

    fig, axs = plt.subplots(2, 4, figsize=(16.5, 8.3))
    fig.suptitle("A.E.G.I.S. wiring guide 2/3: module faces, pins and where each wire lands (top views, to scale)", fontsize=12, weight="bold")
    for ax, (name, w, h, sides) in zip(axs.flat, BOARDS):
        draw_board(ax, name, w, h, sides)
    legend(fig)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.08, wspace=0.05, hspace=0.25)
    pages.append(fig)

    fig = plt.figure(figsize=(11.7, 8.3))
    fig.suptitle("A.E.G.I.S. wiring guide 3/3: wire list", fontsize=12, weight="bold")
    ax = fig.add_axes([0.03, 0.05, 0.94, 0.86])
    ax.axis("off")
    rows = [[a, b, c, g] for a, b, c, k, g in WIRES]
    t = ax.table(cellText=rows, colLabels=["From", "To", "Signal", "Wire"], loc="upper center", cellLoc="left",
                 colWidths=[0.25, 0.47, 0.15, 0.1])
    t.auto_set_font_size(False)
    t.set_fontsize(8)
    t.scale(1, 1.55)
    for i, (a, b, c, k, g) in enumerate(WIRES, start=1):
        t[i, 0].set_facecolor(C[k] + "33")
        t[i, 0].get_text().set_weight("bold")
    ax.text(0.0, 0.02, "Common ground star point at the battery negative. Set both XL4015 outputs to 5.0 V with a meter before connecting "
            "any board. Twist each motor's two leads together. Keep the I2S wires short (< 15 cm).\n"
            "Header orders follow the reference boards; clone modules sometimes differ: check the silkscreen before soldering.",
            fontsize=8, va="bottom", wrap=True)
    pages.append(fig)

    with PdfPages(OUT) as pdf:
        for i, f in enumerate(pages, 1):
            pdf.savefig(f)
            f.savefig(OUT.replace(".pdf", "_p%d.png" % i), dpi=110)
    print(os.path.abspath(OUT))


if __name__ == "__main__":
    main()
