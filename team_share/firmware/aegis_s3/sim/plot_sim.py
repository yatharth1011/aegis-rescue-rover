"""
Plots for the A.E.G.I.S. software-in-the-loop simulator.

  python3 plot_sim.py trace <prefix> <out.png>        one run: map, truth vs odometry, states
  python3 plot_sim.py anim  <prefix> <out.gif>        animated version of one run
  python3 plot_sim.py summary <results_dir> <out.png> outcome / timing / DOA charts per scenario
"""
import csv
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation, patches

STATES = ["IDLE", "CALIB", "SEARCH", "APPROACH", "GRASP", "RETURN", "RELEASE", "DONE", "FAULT"]
LABEL = {"nominal": "nominal", "harsh": "harsh room", "no_audio": "beacon off",
         "qtable_nominal": "Q-table policy\n(nominal room)"}
STATE_COLOR = {2: "#2563eb", 3: "#f59e0b", 4: "#dc2626", 5: "#16a34a", 6: "#7c3aed", 7: "#111827"}


def load_world(prefix):
    world = {"boxes": []}
    with open(prefix + "_world.csv") as f:
        for row in csv.DictReader(f):
            vals = [float(row[k]) for k in "abcd"]
            if row["kind"] == "box":
                world["boxes"].append(vals)
            else:
                world[row["kind"]] = vals
    return world


def load_trace(prefix):
    with open(prefix + "_trace.csv") as f:
        rows = list(csv.DictReader(f))
    return [{k: float(v) for k, v in r.items()} for r in rows]


def draw_world(ax, world):
    w, h = world["room"][:2]
    ax.add_patch(patches.Rectangle((0, 0), w, h, fill=False, lw=2, ec="#334155"))
    for x0, y0, x1, y1 in world["boxes"]:
        ax.add_patch(patches.Rectangle((x0, y0), x1 - x0, y1 - y0, fc="#94a3b8", ec="#475569"))
    sx, sy, sth = world["start"][:3]
    ax.plot(sx, sy, "s", ms=10, mfc="none", mec="#111827", mew=2, label="start / home")
    ax.plot(*world["target"][:2], "*", ms=12, color="#dc2626", label="target + sound beacon")
    ax.set_xlim(-0.1, w + 0.1)
    ax.set_ylim(-0.1, h + 0.1)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def draw_rover(ax, x, y, th, color="#0f172a"):
    c, s = math.cos(th), math.sin(th)
    pts = [(-0.085, -0.075), (0.085, -0.075), (0.085, -0.035), (0.175, -0.035), (0.175, 0.035),
           (0.085, 0.035), (0.085, 0.075), (-0.085, 0.075)]
    poly = [(x + c * px - s * py, y + s * px + c * py) for px, py in pts]
    return ax.add_patch(patches.Polygon(poly, closed=True, fc=color, alpha=0.35, ec=color))


def plot_trace(prefix, out):
    world, tr = load_world(prefix), load_trace(prefix)
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 6.2), gridspec_kw={"width_ratios": [1.35, 1]})
    draw_world(ax, world)
    for st, col in STATE_COLOR.items():
        seg = [r for r in tr if int(r["state"]) == st]
        if seg:
            ax.plot([r["x"] for r in seg], [r["y"] for r in seg], "o", ms=1.6, color=col, label=STATES[st])
    ax.plot([r["ex"] for r in tr], [r["ey"] for r in tr], "-", lw=0.8, color="#64748b", alpha=0.8,
            label="rover's own estimate (odometry)")
    draw_rover(ax, tr[-1]["x"], tr[-1]["y"], tr[-1]["th"])
    ax.legend(loc="upper center", fontsize=7, framealpha=0.9, markerscale=1.0, bbox_to_anchor=(0.5, -0.12), ncol=4)
    ax.set_title("Ground truth path, coloured by mission state")

    t = [r["t"] for r in tr]
    ax2.plot(t, [r["tof_f"] for r in tr], lw=0.8, label="front ToF (m)")
    ax2.plot(t, [r["v"] for r in tr], lw=0.8, label="v command after CBF (m/s)")
    ax2.step(t, [r["state"] / 4 for r in tr], lw=1.2, color="#111827", label="mission state (/4)")
    err = [math.hypot(r["ex"] - r["x"], r["ey"] - r["y"]) for r in tr]
    ax2.plot(t, err, lw=1.0, color="#dc2626", label="odometry error (m)")
    ax2.set_xlabel("time (s)")
    ax2.set_ylim(-0.05, 2.1)
    ax2.legend(fontsize=7)
    ax2.set_title("Signals over time")
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")


def anim_trace(prefix, out):
    world, tr = load_world(prefix), load_trace(prefix)
    tr = tr[::3]
    fig, ax = plt.subplots(figsize=(7, 5.6))
    draw_world(ax, world)
    path, = ax.plot([], [], "-", color="#2563eb", lw=1.2)
    est, = ax.plot([], [], "-", color="#64748b", lw=0.8, alpha=0.7)
    obj, = ax.plot([], [], "o", color="#dc2626", ms=5)
    doa = ax.annotate("", xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#f59e0b", lw=2))
    label = ax.text(0.02, 1.02, "", transform=ax.transAxes, fontsize=10)
    rover = [draw_rover(ax, tr[0]["x"], tr[0]["y"], tr[0]["th"])]

    def frame(i):
        r = tr[i]
        path.set_data([q["x"] for q in tr[: i + 1]], [q["y"] for q in tr[: i + 1]])
        est.set_data([q["ex"] for q in tr[: i + 1]], [q["ey"] for q in tr[: i + 1]])
        obj.set_data([r["obj_x"]], [r["obj_y"]])
        rover[0].remove()
        rover[0] = draw_rover(ax, r["x"], r["y"], r["th"])
        if r["doa_ok"]:
            doa.xy = (r["x"] + 0.5 * math.cos(r["doa_world"]), r["y"] + 0.5 * math.sin(r["doa_world"]))
            doa.set_position((r["x"], r["y"]))
            doa.set_visible(True)
        else:
            doa.set_visible(False)
        label.set_text(f"t = {r['t']:5.1f} s   {STATES[int(r['state'])]}   (orange arrow: heard bearing)")
        return path, est, obj, doa, label, rover[0]

    ani = animation.FuncAnimation(fig, frame, frames=len(tr), interval=60, blit=False)
    ani.save(out, writer=animation.PillowWriter(fps=16))


def load_results(d):
    out = {}
    for name in ["nominal", "harsh", "no_audio", "qtable_nominal"]:
        fn = os.path.join(d, name + ".csv")
        if os.path.exists(fn):
            with open(fn) as f:
                out[name] = list(csv.DictReader(f))
    return out


def plot_summary(d, out):
    res = load_results(d)
    names = list(res)
    outcomes = ["success", "return_fail", "grasp_miss", "timeout", "fault"]
    colors = ["#16a34a", "#f59e0b", "#a855f7", "#94a3b8", "#dc2626"]
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.6))

    ax = axs[0]
    bottom = [0] * len(names)
    for o, col in zip(outcomes, colors):
        vals = [100 * sum(r["outcome"] == o for r in res[n]) / len(res[n]) for n in names]
        ax.bar([LABEL.get(n, n) for n in names], vals, bottom=bottom, color=col, label=o.replace("_", " "))
        for i, v in enumerate(vals):
            if v >= 6:
                ax.text(i, bottom[i] + v / 2, f"{v:.0f}%", ha="center", va="center", fontsize=8, color="white")
        bottom = [b + v for b, v in zip(bottom, vals)]
    ax.set_ylabel("% of runs")
    ax.set_title(f"Mission outcome ({len(res[names[0]])} random rooms each)")
    ax.legend(fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=5)

    ax = axs[1]
    for n in names:
        ts = sorted(float(r["t_end"]) for r in res[n] if r["outcome"] == "success")
        if ts:
            ax.plot(ts, [100 * (i + 1) / len(res[n]) for i in range(len(ts))], label=LABEL.get(n, n))
    ax.set_xlabel("mission time (s)")
    ax.set_ylabel("% of runs completed")
    ax.set_title("Time to rescue and return home")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axs[2]
    for n in names:
        if n in ("no_audio", "qtable_nominal"):
            continue
        raw = [float(r["doa_err_deg"]) for r in res[n] if float(r["doa_err_deg"]) >= 0]
        fold = [float(r["doa_err_folded_deg"]) for r in res[n] if float(r["doa_err_folded_deg"]) >= 0]
        ax.hist(fold, bins=20, range=(0, 60), alpha=0.6, label=f"{n}: after front/back folding")
        ax.hist(raw, bins=20, range=(0, 90), alpha=0.5, histtype="step", lw=2, label=f"{n}: raw two-mic reading")
    ax.set_xlabel("mean bearing error per run (deg)")
    ax.set_ylabel("runs")
    ax.set_title("Acoustic bearing accuracy while stopped")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")


def plot_deck(d, out):
    """One slide-sized figure (aspect ~1.76): an example mission and the headline numbers."""
    res = load_results(d)
    world, tr = load_world(os.path.join(d, "example_nominal")), load_trace(os.path.join(d, "example_nominal"))
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(10.5, 6.0), gridspec_kw={"width_ratios": [1.25, 1]})
    draw_world(ax, world)
    for st, col in STATE_COLOR.items():
        seg = [r for r in tr if int(r["state"]) == st]
        if seg:
            ax.plot([r["x"] for r in seg], [r["y"] for r in seg], "o", ms=1.8, color=col, label=STATES[st])
    draw_rover(ax, tr[-1]["x"], tr[-1]["y"], tr[-1]["th"])
    ax.legend(loc="upper right", fontsize=7, ncol=2, framealpha=0.9)
    t_grip = next(r["t"] for r in tr if int(r["state"]) == 4)
    home_cm = math.hypot(tr[-1]["x"] - world["start"][0], tr[-1]["y"] - world["start"][1]) * 100
    ax.set_title(f"Example run: target gripped at {t_grip:.0f} s, home at {tr[-1]['t']:.0f} s, "
                 f"{home_cm:.0f} cm from start", fontsize=10)
    names = [n for n in ["nominal", "harsh", "no_audio", "qtable_nominal"] if n in res]
    succ = [100 * sum(r["outcome"] == "success" for r in res[n]) / len(res[n]) for n in names]
    cols = ["#16a34a", "#f59e0b", "#94a3b8", "#dc2626"]
    bars = ax2.bar([LABEL[n] for n in names], succ, color=cols[: len(names)])
    for b, v in zip(bars, succ):
        ax2.text(b.get_x() + b.get_width() / 2, v + 1.5, f"{v:.0f}%", ha="center", fontsize=11, fontweight="bold")
    ax2.set_ylim(0, 100)
    ax2.set_ylabel("missions completed (%)")
    ax2.set_title(f"Software-in-the-loop: {len(res[names[0]])} random rooms per bar", fontsize=10)
    ax2.tick_params(axis="x", labelsize=8)
    fig.suptitle("A.E.G.I.S. firmware decision code running in simulated rooms (search, grasp, return home)",
                 fontsize=11, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out, dpi=160, bbox_inches="tight")


if __name__ == "__main__":
    cmd, a, b = sys.argv[1:4]
    {"trace": plot_trace, "anim": anim_trace, "summary": plot_summary, "deck": plot_deck}[cmd](a, b)
