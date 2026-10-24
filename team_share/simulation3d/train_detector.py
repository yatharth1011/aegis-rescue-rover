"""Merge the per-worker datasets and fine-tune YOLOv8n on the M5 GPU (MPS)."""
import glob
import os
import shutil
import sys

from ultralytics import YOLO

root = sys.argv[1]
merged = os.path.join(root, "merged")
for split in ("train", "val"):
    for kind in ("images", "labels"):
        os.makedirs(os.path.join(merged, kind, split), exist_ok=True)
for wdir in sorted(glob.glob(os.path.join(root, "w*"))):
    tag = os.path.basename(wdir)
    for split in ("train", "val"):
        for kind, ext in (("images", ".jpg"), ("labels", ".txt")):
            for f in glob.glob(os.path.join(wdir, kind, split, "*" + ext)):
                shutil.copy(f, os.path.join(merged, kind, split, tag + "_" + os.path.basename(f)))
with open(os.path.join(merged, "data.yaml"), "w") as f:
    f.write("path: %s\ntrain: images/train\nval: images/val\nnames:\n  0: victim_token\n" % os.path.abspath(merged))

model = YOLO("yolov8n.pt")
model.train(data=os.path.join(merged, "data.yaml"), imgsz=320, epochs=45, batch=64, device="mps", workers=4,
            project=os.path.join(root, "runs"), name="token", exist_ok=True, patience=12, verbose=False,
            hsv_h=0.01, mosaic=0.5, plots=True)
best = os.path.join(root, "runs", "token", "weights", "best.pt")
shutil.copy(best, os.path.join(os.path.dirname(os.path.abspath(__file__)), "victim_token_yolov8n.pt"))
print("best weights:", best)
