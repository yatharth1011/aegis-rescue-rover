"""
Collect real ESP32-CAM photos for retraining the victim-token detector.

    ~/aegis_venv/bin/python team_share/edge/capture_frames.py [--cam-url URL]

Window keys: space = save this frame   a = auto-save every 0.5 s on/off   q = quit
Frames go to team_share/edge/dataset/w_real/images/{train,val}/ (9 in 10 to train).

What to capture (aim for 300+ frames): the token at 15-90 cm, from all sides, partly behind books,
under the lab's lights and near a window; the decoys (orange cube, red ball, orange disc) with and
without the token; and 50+ frames with no token at all. Drive the rover with bringup.py meanwhile.

Then label: open https://www.makesense.ai, load the images, one class "victim_token", draw a box
around every token (not the decoys), export "YOLO" format, and put the .txt files in
dataset/w_real/labels/{train,val}/ next to their images (same file names). Images with no token need
no .txt file. Retrain on rendered + real images:

    cp -r team_share/edge/dataset/w_real team_share/simulation3d/build/ds/
    ~/aegis_venv/bin/python team_share/simulation3d/train_detector.py team_share/simulation3d/build/ds
"""
import argparse
import os
import random
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from aegis_link import CameraStream, RoverLink, find_camera  # noqa: E402

OUT = os.path.join(HERE, "dataset", "w_real", "images")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam-url")
    a = ap.parse_args()
    url = a.cam_url or find_camera(RoverLink(), 15)
    if not url:
        sys.exit("no camera beacon: ESP32-CAM powered and on the same Wi-Fi? Or pass --cam-url")
    for split in ("train", "val"):
        os.makedirs(os.path.join(OUT, split), exist_ok=True)
    cam = CameraStream(url)
    auto, last_auto, n = False, 0.0, 0
    print("camera", url, "| space = save, a = auto, q = quit")
    while True:
        jpeg, _ = cam.latest()
        if jpeg is None:
            time.sleep(0.05)
            continue
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        view = cv2.resize(img, (640, 480))
        cv2.putText(view, "saved %d   auto %s" % (n, "ON" if auto else "off"), (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("AEGIS dataset capture", view)
        k = cv2.waitKey(30) & 0xFF
        now = time.time()
        if k == ord("q"):
            break
        if k == ord("a"):
            auto = not auto
        if k == ord(" ") or (auto and now - last_auto > 0.5):
            last_auto = now
            split = "val" if random.random() < 0.1 else "train"
            with open(os.path.join(OUT, split, time.strftime("real_%Y%m%d_%H%M%S_") + "%03d.jpg" % n), "wb") as f:
                f.write(jpeg)
            n += 1
    cam.close()
    print("saved %d frames in %s" % (n, OUT))


if __name__ == "__main__":
    main()
