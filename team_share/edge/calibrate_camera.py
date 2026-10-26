"""
Optional: calibrate the ESP32-CAM lens for a more accurate start-pad fix (aegis_edge.py --calib).

    ~/aegis_venv/bin/python team_share/edge/calibrate_camera.py [--cam-url URL]

Print a 9 x 6 inner-corner chessboard with 25 mm squares (any online generator, print at 100 %),
tape it to a flat board and show it to the camera from 15+ different angles and distances.
Keys: space = use this view (only when the corners are found)   c = calibrate and save   q = quit
Writes team_share/edge/camera_calib.npz (K, dist).
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from aegis_link import CameraStream, RoverLink, find_camera  # noqa: E402

PATTERN, SQUARE = (9, 6), 0.025


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam-url")
    a = ap.parse_args()
    url = a.cam_url or find_camera(RoverLink(), 15)
    if not url:
        sys.exit("no camera beacon: pass --cam-url")
    cam = CameraStream(url)
    obj = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE
    views, size = [], None
    while True:
        jpeg, _ = cam.latest()
        if jpeg is None:
            time.sleep(0.05)
            continue
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, PATTERN)
        view = img.copy()
        if found:
            cv2.drawChessboardCorners(view, PATTERN, corners, found)
        cv2.putText(view, "views %d" % len(views), (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imshow("AEGIS camera calibration", cv2.resize(view, (640, 480)))
        k = cv2.waitKey(30) & 0xFF
        if k == ord("q"):
            break
        if k == ord(" ") and found:
            views.append(cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)))
        if k == ord("c") and len(views) >= 10:
            rms, K, dist, _, _ = cv2.calibrateCamera([obj] * len(views), views, size, None, None)
            out = os.path.join(HERE, "camera_calib.npz")
            np.savez(out, K=K, dist=dist)
            print("RMS reprojection error %.2f px (under 0.5 is good). Saved %s" % (rms, out))
            print("vertical FOV %.1f deg (simulator assumed 42)" % np.degrees(2 * np.arctan(size[1] / 2 / K[1, 1])))
            break
    cam.close()


if __name__ == "__main__":
    main()
