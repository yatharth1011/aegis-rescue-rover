"""Top-down plots and an animated GIF of a recorded mission (truth vs. the brain's beliefs)."""
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation, patches

from brain import RES
from world import TABLE

STATE_COL = {"EXPLORE": "#2563eb", "PREGRASP": "#f59e0b", "APPROACH": "#ea580c", "BACKOFF": "#a855f7",
             "CLOSE": "#dc2626", "RETURN": "#16a34a", "DONE": "#111827"}


def rover_poly(x, y, th):
    pts = [(-0.115, -0.07), (0.13, -0.07), (0.148, -0.045), (0.148, 0.045), (0.13, 0.07), (-0.115, 0.07)]
    c, s = math.cos(th), math.sin(th)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in pts]


def draw_table(ax, info):
    ax.add_patch(patches.Rectangle((0, 0), TABLE["lx"], TABLE["ly"], fc="#f1f5f9", ec="#334155", lw=2))
    for cx, cy, sx, sy, h, yaw in info["boxes"]:
        c, s = math.cos(yaw), math.sin(yaw)
        poly = [(cx + c * px - s * py, cy + s * px + c * py) for px, py in ((-sx, -sy), (sx, -sy), (sx, sy), (-sx, sy))]
        ax.add_patch(patches.Polygon(poly, fc="#94a3b8", ec="#475569"))
    for x, y, kind in info["decoys"]:
        ax.plot(x, y, ["s", "o", "D"][kind], color="#fb923c", ms=6, mec="#7c2d12")
    ax.plot(*info["token"], "o", color="#ea580c", ms=9, mec="black", label="victim token (start)")
    sx, sy, sth = info["start"]
    ax.add_patch(patches.Polygon(rover_poly(sx, sy, sth), fill=False, ec="black", ls="--"))
    ax.set_xlim(-0.05, TABLE["lx"] + 0.05)
    ax.set_ylim(-0.05, TABLE["ly"] + 0.05)
    ax.set_aspect("equal")


def plot_run(rec, info, title, out):
    fig, ax = plt.subplots(figsize=(9, 6.2))
    draw_table(ax, info)
    last_insp = [r["inspected"] for r in rec if r["inspected"] is not None]
    if last_insp:
        ins = last_insp[-1].T.astype(float)
        ax.imshow(np.ma.masked_where(ins < 0.5, ins), origin="lower", extent=(0, TABLE["lx"], 0, TABLE["ly"]),
                  cmap="Greens", alpha=0.25, vmin=0, vmax=1.5)
    for st, col in STATE_COL.items():
        pts = np.array([r["pose"][:2] for r in rec if r["state"] == st])
        if len(pts):
            ax.plot(pts[:, 0], pts[:, 1], ".", ms=3, color=col, label=st)
    est = np.array([r["est"][:2] for r in rec])
    ax.plot(est[:, 0], est[:, 1], "-", lw=0.7, color="#64748b", label="brain's estimate")
    tok = np.array([r["token"] for r in rec])
    ax.plot(tok[:, 0], tok[:, 1], "-", color="#ea580c", lw=1.5, label="token (moved)")
    ax.add_patch(patches.Polygon(rover_poly(*rec[-1]["pose"]), fc="#0f172a", alpha=0.4))
    ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.01, 1))
    ax.set_title(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def animate_run(rec, info, title, out, fps=12):
    frames = [r for r in rec]
    fig = plt.figure(figsize=(11, 4.8))
    ax = fig.add_axes([0.03, 0.08, 0.55, 0.84])
    axc = fig.add_axes([0.61, 0.25, 0.37, 0.55])
    axc.axis("off")
    draw_table(ax, info)
    path_ln, = ax.plot([], [], "-", color="#2563eb", lw=1.2)
    plan_ln, = ax.plot([], [], ":", color="#16a34a", lw=1.5)
    tok_pt, = ax.plot([], [], "o", color="#ea580c", ms=8, mec="black")
    rover = [ax.add_patch(patches.Polygon(rover_poly(*frames[0]["pose"]), fc="#0f172a", alpha=0.5))]
    arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#dc2626", lw=2))
    txt = ax.set_title("")
    ins_img = [None]
    cam_img = axc.imshow(np.zeros((240, 320, 3), np.uint8))
    axc.set_title("ESP32-CAM view (streamed to the laptop)", fontsize=9)
    last_frame = [np.zeros((240, 320, 3), np.uint8)]

    def upd(i):
        r = frames[i]
        pts = np.array([q["pose"][:2] for q in frames[: i + 1]])
        path_ln.set_data(pts[:, 0], pts[:, 1])
        plan_ln.set_data([p[0] for p in r["path"]], [p[1] for p in r["path"]])
        tok_pt.set_data([r["token"][0]], [r["token"][1]])
        rover[0].remove()
        rover[0] = ax.add_patch(patches.Polygon(rover_poly(*r["pose"]), fc="#0f172a", alpha=0.5))
        if r["inspected"] is not None:
            if ins_img[0] is not None:
                ins_img[0].remove()
            ins = r["inspected"].T.astype(float)
            ins_img[0] = ax.imshow(np.ma.masked_where(ins < 0.5, ins), origin="lower",
                                   extent=(0, TABLE["lx"], 0, TABLE["ly"]), cmap="Greens", alpha=0.25, vmin=0, vmax=1.5)
        if r["bearing"] is not None:
            x, y = r["pose"][:2]
            arrow.xy = (x + 0.35 * math.cos(r["bearing"]), y + 0.35 * math.sin(r["bearing"]))
            arrow.set_position((x, y))
            arrow.set_visible(True)
        else:
            arrow.set_visible(False)
        if r["frame"] is not None:
            last_frame[0] = r["frame"]
        cam_img.set_data(last_frame[0])
        txt.set_text("%s   t = %5.1f s   %s   (green: inspected, red arrow: heard scream)" % (title, r["t"], r["state"]))
        txt.set_fontsize(9)
        return []

    ani = animation.FuncAnimation(fig, upd, frames=len(frames), interval=1000 / fps)
    ani.save(out, writer=animation.PillowWriter(fps=fps))
    plt.close(fig)
