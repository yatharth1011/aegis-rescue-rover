"""
Batch evaluation in parallel worker processes (YOLO on CPU per worker).
    python eval_batch.py <out.csv> <demo> <link> <n> <first_seed> <workers>
"""
import csv
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_det = None


def _init():
    """AEGIS_DETECTOR=oracle uses ground truth (upper bound for planning/control);
    otherwise the trained YOLO model runs on the CPU in each worker."""
    global _det
    if os.environ.get("AEGIS_DETECTOR") == "oracle":
        import world as W
        from oracle import Oracle
        _det = Oracle()
        orig = W.World.__init__

        def patched(self, *a, **k):
            orig(self, *a, **k)
            _det.world = self
        W.World.__init__ = patched
        return
    import torch
    torch.set_num_threads(1)
    from run_mission import YoloDetector
    _det = YoloDetector(os.path.join(HERE, "victim_token_yolov8n.pt"), device="cpu")


def _one(args):
    seed, demo, link = args
    from run_mission import run
    try:
        r = run(seed, demo, link, _det, max_t=240.0)
    except Exception as e:
        r = dict(seed=seed, demo=demo, link=link, outcome="error:" + type(e).__name__ + ":" + str(e)[:80])
    return {k: (float(v) if hasattr(v, "item") else v) for k, v in r.items()}


if __name__ == "__main__":
    out, demo, link, n, first, workers = sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6])
    jobs = [(first + i, demo, link) for i in range(n)]
    rows = []
    with mp.get_context("spawn").Pool(workers, initializer=_init) as pool:
        for r in pool.imap_unordered(_one, jobs):
            rows.append(r)
            print("%3d/%d seed %d %s" % (len(rows), n, r["seed"], r["outcome"]), flush=True)
    keys = sorted(set().union(*rows))
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    ok = sum(r["outcome"] == "success" for r in rows)
    print("SUCCESS %d/%d" % (ok, n))
