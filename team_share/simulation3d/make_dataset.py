"""
Render a YOLO training set from the rover's camera: random tables, random rover
poses (60 % aimed near the token), ESP32-CAM style degradation (JPEG,
noise, exposure, motion blur). Labels come from MuJoCo segmentation, so they
are exact. Class 0 = victim token. Decoys (orange cube, red ball, orange
disc) are present but unlabelled: the detector must learn the shape.

    python make_dataset.py <out_dir> <n_train> <n_val> <seed>
"""
import io
import math
import os
import sys

import numpy as np
from PIL import Image, ImageFilter

from world import TABLE, World


def degrade(img, rng):
    im = Image.fromarray(img)
    if rng.random() < 0.3:
        im = im.filter(ImageFilter.BoxBlur(rng.uniform(0.6, 1.6)))
    a = np.asarray(im).astype(np.float32)
    a = a * rng.uniform(0.7, 1.25) + rng.uniform(-20, 20)                  # exposure / white level
    a += rng.normal(0, rng.uniform(2, 7), a.shape)                          # sensor noise
    im = Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))
    buf = io.BytesIO()
    im.save(buf, "JPEG", quality=int(rng.integers(35, 80)))                 # ESP32-CAM JPEG
    return Image.open(io.BytesIO(buf.getvalue())).convert("RGB")


def bbox_from_seg(seg, gids):
    mask = np.isin(seg[..., 0], list(gids)) & (seg[..., 1] == 5)           # objtype 5 = geom
    if mask.sum() < 12:
        return None
    ys, xs = np.nonzero(mask)
    return xs.min(), ys.min(), xs.max(), ys.max()


def main(out, n_train, n_val, seed):
    rng = np.random.default_rng(seed)
    for split, n in (("train", n_train), ("val", n_val)):
        os.makedirs(os.path.join(out, "images", split), exist_ok=True)
        os.makedirs(os.path.join(out, "labels", split), exist_ok=True)
        k = 0
        while k < n:
            w = World(int(rng.integers(1e9)), token_hidden=False, n_decoys=int(rng.integers(1, 4)))
            gids = {w.m.geom("token_g").id, w.m.geom("token_band").id}
            for _ in range(12):
                if k >= n:
                    break
                tx, ty = w.info["token"]
                if rng.random() < 0.75:
                    d, b = rng.uniform(0.15, 0.95), rng.uniform(-math.pi, math.pi)
                    x, y = tx + d * math.cos(b), ty + d * math.sin(b)
                    th = math.atan2(ty - y, tx - x) + rng.uniform(-0.42, 0.42)
                else:
                    x, y, th = rng.uniform(0.12, TABLE["lx"] - 0.12), rng.uniform(0.12, TABLE["ly"] - 0.12), rng.uniform(-math.pi, math.pi)
                if not (0.1 < x < TABLE["lx"] - 0.1 and 0.1 < y < TABLE["ly"] - 0.1):
                    continue
                if any(abs(x - bx) < sx + 0.12 and abs(y - by) < sy + 0.12 for bx, by, sx, sy, _, _ in w.info["boxes"]):
                    continue
                x0, y0, th0 = w.info["start"]
                w.d.qpos[w.jx], w.d.qpos[w.jy], w.d.qpos[w.jyaw] = x - x0, y - y0, th
                import mujoco
                mujoco.mj_forward(w.m, w.d)
                img = w.render()
                bb = bbox_from_seg(w.render(segmentation=True), gids)
                name = "%s_%06d" % (split, k)
                degrade(img, rng).save(os.path.join(out, "images", split, name + ".jpg"), quality=95)
                with open(os.path.join(out, "labels", split, name + ".txt"), "w") as f:
                    if bb:
                        x1, y1, x2, y2 = bb
                        H, W = img.shape[:2]
                        f.write("0 %.6f %.6f %.6f %.6f\n" % ((x1 + x2) / 2 / W, (y1 + y2) / 2 / H, (x2 - x1 + 1) / W, (y2 - y1 + 1) / H))
                k += 1
            w.close()
    with open(os.path.join(out, "data.yaml"), "w") as f:
        f.write("path: %s\ntrain: images/train\nval: images/val\nnames:\n  0: victim_token\n" % os.path.abspath(out))


if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))
