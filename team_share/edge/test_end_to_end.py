"""
End-to-end test of the deployable laptop software: for each seed, start fake_rover.py (simulator
behind the real wire protocol) and run aegis_edge.py against it in real time, then read the true
outcome from the fake rover.

    ~/aegis_venv/bin/python team_share/edge/test_end_to_end.py --demo 1 --seeds 3009 3010 3011
"""
import argparse
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable


def run(demo, seed, max_time, table):
    fake = subprocess.Popen([PY, os.path.join(HERE, "fake_rover.py"), "--demo", str(demo), "--seed", str(seed),
                             "--max-time", str(max_time + 30)], stdout=subprocess.PIPE, text=True)
    start = None
    for line in fake.stdout:
        m = re.search(r"--start ([-\d.]+) ([-\d.]+) ([-\d.]+)", line)
        if m:
            start = m.groups()
            break
    edge = subprocess.run([PY, os.path.join(HERE, "aegis_edge.py"), "--demo", str(demo), "--rover-ip", "127.0.0.1",
                           "--cam-url", "http://127.0.0.1:8081/stream", "--start", *start, "--table", *table, "--no-window",
                           "--max-time", str(max_time), "--cam-latency", "0.03"],
                          capture_output=True, text=True)
    fake.terminate()
    out = fake.communicate(timeout=20)[0]
    res = re.search(r"RESULT fell=(\w+) token_in_bay=(\w+) home_err=([\d.]+)", out)
    fin = re.search(r"finished: (\w+) after ([\d.]+) s", edge.stdout)
    gaps = re.search(r"max (\d+) ms", edge.stdout)
    fell, inside, home = res.group(1) == "True", res.group(2) == "True", float(res.group(3))
    ok = inside and home < 0.15 and not fell and fin and fin.group(1) == "DONE"
    return dict(seed=seed, ok=ok, fell=fell, token_in_bay=inside, home_err=home,
                state=fin.group(1) if fin else "?", t=float(fin.group(2)) if fin else None,
                max_gap_ms=int(gaps.group(1)) if gaps else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", type=int, default=1)
    ap.add_argument("--seeds", type=int, nargs="+", default=[3009])
    ap.add_argument("--max-time", type=float, default=200)
    ap.add_argument("--table", nargs="+", default=["unknown"], help="'unknown' or the true size: 1.8 1.2")
    a = ap.parse_args()
    results = []
    for s in a.seeds:
        r = run(a.demo, s, a.max_time, a.table)
        results.append(r)
        print("demo %d seed %d: %s  (%s, %s s, token in bay %s, home %.2f m, fell %s, max command gap %s ms)" %
              (a.demo, s, "SUCCESS" if r["ok"] else "fail", r["state"], r["t"], r["token_in_bay"], r["home_err"],
               r["fell"], r["max_gap_ms"]), flush=True)
        time.sleep(1)
    n = sum(r["ok"] for r in results)
    print("demo %d: %d / %d succeeded, %d fell" % (a.demo, n, len(results), sum(r["fell"] for r in results)))


if __name__ == "__main__":
    main()
