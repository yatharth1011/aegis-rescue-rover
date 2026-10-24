"""
Summarise the final evaluation and record example missions.

    ~/aegis_venv/bin/python make_report.py
writes results/summary.json, results/summary.png and, for one successful
mission of each demo, results/example_demo{1,2}.png and .gif
"""
import csv
import json
import os
import statistics as st

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, "results")
SETS = [("final_demo1", "Hidden token\n(no sound)"), ("final_demo2", "Scream\nhoming"),
        ("final_demo1_badlink", "Hidden token\nbad Wi-Fi"), ("final_demo2_badlink", "Scream\nbad Wi-Fi")]


def load(name):
    p = os.path.join(R, name + ".csv")
    if not os.path.exists(p):
        return None
    with open(p) as f:
        return list(csv.DictReader(f))


def fnum(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return None


def stats(rows):
    n = len(rows)
    ok = [r for r in rows if r["outcome"] == "success"]
    out = {"n": n, "success_pct": round(100 * len(ok) / n, 1),
           "median_time_s": round(st.median(fnum(r, "t_end") for r in ok), 1) if ok else None,
           "found_pct": round(100 * sum(fnum(r, "first_seen") is not None for r in rows) / n, 1),
           "captured_pct": round(100 * sum(fnum(r, "captured") is not None for r in rows) / n, 1),
           "contact_free_pct": round(100 * sum(int(float(r.get("collisions") or 0)) == 0 for r in rows) / n, 1),
           "fell_pct": round(100 * sum(r["outcome"] == "fell" for r in rows) / n, 1),
           "link_loss_pct": round(st.mean(fnum(r, "link_loss_pct") or 0 for r in rows), 1),
           "watchdog_stops_mean": round(st.mean(fnum(r, "watchdog_stops") or 0 for r in rows), 1),
           "outcomes": {}}
    for r in rows:
        out["outcomes"][r["outcome"]] = out["outcomes"].get(r["outcome"], 0) + 1
    return out


def main():
    res = {}
    for name, _ in SETS:
        rows = load(name)
        if rows:
            res[name] = stats(rows)
    summary = {"sets": res}
    if "final_demo1" in res and "final_demo2" in res:
        summary.update(n_per_demo=res["final_demo1"]["n"], demo1_success_pct=round(res["final_demo1"]["success_pct"]),
                       demo2_success_pct=round(res["final_demo2"]["success_pct"]))
    with open(os.path.join(R, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))

    names = [n for n, _ in SETS if n in res]
    labels = [l for n, l in SETS if n in res]
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.4))
    order = ["success", "home_off", "lost_token", "timeout", "fell", "error"]
    cols = {"success": "#16a34a", "home_off": "#f59e0b", "lost_token": "#a855f7", "timeout": "#94a3b8", "fell": "#dc2626"}
    bottom = [0.0] * len(names)
    for o in order:
        vals = [100 * res[n]["outcomes"].get(o, 0) / res[n]["n"] for n in names]
        if not any(vals):
            continue
        axs[0].bar(labels, vals, bottom=bottom, color=cols.get(o, "#444"), label=o.replace("_", " "))
        for i, v in enumerate(vals):
            if v >= 7:
                axs[0].text(i, bottom[i] + v / 2, "%.0f%%" % v, ha="center", va="center", color="white", fontsize=9)
        bottom = [b + v for b, v in zip(bottom, vals)]
    axs[0].set_ylabel("% of missions")
    axs[0].set_title("Outcome (3D sim: camera + YOLO, room acoustics, Wi-Fi loss)")
    axs[0].legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=5)
    for n, l in zip(names, labels):
        ts = sorted(float(r["t_end"]) for r in load(n) if r["outcome"] == "success")
        if ts:
            axs[1].plot(ts, [100 * (i + 1) / res[n]["n"] for i in range(len(ts))], label=l.replace("\n", " "))
    axs[1].set_xlabel("mission time (s)")
    axs[1].set_ylabel("% completed")
    axs[1].set_title("Time to find, rescue and return home")
    axs[1].grid(alpha=0.3)
    axs[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(R, "summary.png"), dpi=140, bbox_inches="tight")

    # record one successful mission per demo
    import run_mission
    import viz
    from world import World
    det = run_mission.YoloDetector(os.path.join(HERE, "victim_token_yolov8n.pt"), device="mps")
    for demo, name in ((1, "final_demo1"), (2, "final_demo2")):
        rows = load(name) or []
        good = sorted((r for r in rows if r["outcome"] == "success"), key=lambda r: float(r["t_end"]))
        if not good:
            continue
        seed = int(good[len(good) // 2]["seed"])                  # a typical (median-time) success
        rec = []
        r = run_mission.run(seed, demo, "typical", det, max_t=240.0, record=rec)
        info = World(seed, token_hidden=(demo == 1)).info
        title = ("Demo 1, hidden token" if demo == 1 else "Demo 2, scream homing") + " (seed %d, %s)" % (seed, r["outcome"])
        viz.plot_run(rec, info, title, os.path.join(R, "example_demo%d.png" % demo))
        viz.animate_run(rec, info, title.split(" (")[0], os.path.join(R, "example_demo%d.gif" % demo))
        print("recorded", demo, seed, r["outcome"])


if __name__ == "__main__":
    main()
