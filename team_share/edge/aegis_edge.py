"""
A.E.G.I.S. edge brain on the real rover: the same brain.py the 3D simulation was evaluated with,
fed by the rover's Wi-Fi link instead of the simulator.

    ~/aegis_venv/bin/python team_share/edge/aegis_edge.py --demo 2 --table 1.8 1.2 --start 0.20 0.20 0

Table frame: origin at the table corner nearest the start pad, x along the long edge, y along the
short edge, metres. --start is the rover's pose on the start pad (x, y, heading in degrees).

Window keys:  g = go   space = pause / resume   o = open/close door (paused only)   q = quit
The rover only moves after "g". Closing the program or losing Wi-Fi stops it (300 ms watchdog).
"""
import argparse
import csv
import math
import os
import sys
import time
from collections import deque

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "simulation3d")
sys.path.insert(0, SIM)
sys.path.insert(0, HERE)

import world  # noqa: E402  (table and camera geometry live here)
from aegis_link import CameraStream, KeepAlive, RoverLink, find_camera  # noqa: E402

MARKER_ID = 0
MARKER_SIZE = 0.100            # m, black square edge on the printed start pad (make_start_pad.py)


class YoloDetector:
    def __init__(self, weights):
        import torch
        from ultralytics import YOLO
        self.m = YOLO(weights)
        self.device = "mps" if torch.backends.mps.is_available() else "cpu"
        self(np.zeros((240, 320, 3), np.uint8))          # warm-up: the first call takes ~1.5 s

    def __call__(self, img):
        r = self.m.predict(img, imgsz=320, conf=0.25, device=self.device, verbose=False)[0]
        if r.boxes is None or len(r.boxes) == 0:
            return []
        return [(*b, float(c)) for b, c in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy())]


class PoseHistory:
    """Odometry over the last 2 s, so each camera frame gets the pose from when it was taken.
    The ESP32-CAM is a separate board: its frames carry no odometry, and pairing a frame with the
    newest telemetry is off by the frame latency (a 14 deg heading error while turning at 1.6 rad/s)."""

    def __init__(self):
        self.h = deque(maxlen=60)

    def add(self, t, est):
        self.h.append((t, np.array(est, float)))

    def at(self, t):
        if not self.h:
            return None, 0.0
        pts = list(self.h)
        if t <= pts[0][0]:
            return pts[0][1].copy(), 0.0
        for (t0, e0), (t1, e1) in zip(pts, pts[1:]):
            if t0 <= t <= t1:
                a = (t - t0) / max(t1 - t0, 1e-6)
                dth = (e1[2] - e0[2] + math.pi) % (2 * math.pi) - math.pi
                e = e0 + a * (e1 - e0)
                e[2] = e0[2] + a * dth
                return e, dth / max(t1 - t0, 1e-6)
        return pts[-1][1].copy(), 0.0


class StartPad:
    """Rover pose from the ArUco marker on the start pad (laptop side, as in the simulator)."""

    def __init__(self, start, calib=None):
        self.start = start
        cam = world.CAM
        if calib:
            d = np.load(calib)
            self.K, self.dist = d["K"], d["dist"]
        else:                                   # nominal OV2640 at QVGA, 42 deg vertical FOV
            f = (cam["h"] / 2) / math.tan(math.radians(cam["fovy"]) / 2)
            self.K = np.array([[f, 0, cam["w"] / 2], [0, f, cam["h"] / 2], [0, 0, 1]], float)
            self.dist = np.zeros(5)
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                           cv2.aruco.DetectorParameters())
        s = MARKER_SIZE / 2
        self.obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)

    def pose(self, rgb):
        corners, ids, _ = self.det.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        if ids is None or MARKER_ID not in ids.ravel():
            return None
        c = corners[list(ids.ravel()).index(MARKER_ID)].reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(self.obj, c, self.K, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        C = -R.T @ tvec.ravel()                 # camera centre in the marker frame
        axis = R.T @ np.array([0.0, 0.0, 1.0])  # optical axis in the marker frame
        cam = world.CAM
        dist = float(np.hypot(C[0], C[1]))
        if not (0.15 < dist < 0.9) or abs(C[2] - cam["pos"][2]) > 0.05:
            return None                         # outside the range the simulator assumed, or a bad solve
        h = math.atan2(axis[1], axis[0])
        lx, ly = C[0] - cam["pos"][0] * math.cos(h), C[1] - cam["pos"][0] * math.sin(h)
        sx, sy, sth = self.start
        return np.array([sx + math.cos(sth) * lx - math.sin(sth) * ly,
                         sy + math.sin(sth) * lx + math.cos(sth) * ly, sth + h])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", type=int, choices=(1, 2), default=2, help="1 = hidden token, 2 = scream homing")
    ap.add_argument("--table", type=float, nargs=2, default=(1.8, 1.2), metavar=("LX", "LY"))
    ap.add_argument("--start", type=float, nargs=3, default=(0.20, 0.20, 0.0), metavar=("X", "Y", "DEG"))
    ap.add_argument("--rover-ip", help="skip discovery")
    ap.add_argument("--cam-url", help="skip discovery, e.g. http://192.168.4.3/stream")
    ap.add_argument("--model", default=os.path.join(SIM, "victim_token_yolov8n.pt"))
    ap.add_argument("--calib", help="camera_calib.npz from calibrate_camera.py")
    ap.add_argument("--max-time", type=float, default=240.0)
    ap.add_argument("--cam-latency", type=float, default=0.08,
                    help="s from exposure to frame arrival (ESP32-CAM QVGA over Wi-Fi: ~0.06-0.12)")
    ap.add_argument("--max-turn", type=float, default=1.0,
                    help="rad/s; frames taken while turning faster are not used for detection")
    ap.add_argument("--no-window", action="store_true", help="start at once, no video window")
    a = ap.parse_args()

    world.TABLE.update(lx=a.table[0], ly=a.table[1])          # the brain reads these at construction
    from brain import Brain
    start = np.array([a.start[0], a.start[1], math.radians(a.start[2])])

    link = RoverLink(a.rover_ip)
    print("waiting for the rover ...")
    if not link.wait_for_rover(30):
        sys.exit("no rover telemetry: same Wi-Fi? rover powered? (see team_share/edge/README.md)")
    print("rover at %s, mode %s, battery %.2f V, faults 0x%02x" %
          (link.rover[0], link.telem["mode"], link.telem["vbat"], link.telem["fault"]))
    url = a.cam_url or find_camera(link, 10)
    if not url:
        sys.exit("no camera beacon: is the ESP32-CAM powered and on the same Wi-Fi?")
    cam = CameraStream(url)
    print("camera", url)
    print("loading detector", os.path.basename(a.model))
    brain = Brain(start, YoloDetector(a.model), scream_mode=(a.demo == 2), rng=np.random.default_rng())
    pad = StartPad(start, a.calib)
    hist = PoseHistory()
    keep = KeepAlive(link)

    log_path = os.path.join(HERE, "logs", time.strftime("run_%Y%m%d_%H%M%S.csv"))
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    logf = open(log_path, "w", newline="")
    log = csv.writer(logf)
    log.writerow(["t", "state", "x", "y", "th", "v", "w", "door", "tof_f", "cliff", "capture", "vbat", "fault",
                  "token_x", "token_y"])

    running = a.no_window
    door_manual = False
    t0 = time.time()
    t_go = t0 if running else None
    last_frame_t = next_step = next_frame = 0.0
    cmd = dict(v=0.0, w=0.0, door=0)
    print("press g in the video window to start" if not running else "running")
    try:
        while True:
            now = time.time()
            t = now - t0
            for m in link.poll():
                if m["type"] == "telem":
                    hist.add(now, m["est"])
                    brain.on_telem(m)
                elif m["type"] == "audio" and running:
                    brain.on_audio(m, t)
            jpeg, ft = cam.latest()
            frame = None
            if jpeg is not None and ft > last_frame_t and now >= next_frame and link.telem is not None:
                last_frame_t, next_frame = ft, now + 0.125            # 8 fps, as in the simulator
                frame = cv2.cvtColor(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
                est, turn_rate = hist.at(ft - a.cam_latency)
                if running and est is not None and abs(turn_rate) <= a.max_turn:
                    brain.on_frame(dict(type="frame", t=t, est=est, jpeg=jpeg), t)
                p = pad.pose(frame)
                if p is not None and running and est is not None:
                    brain.on_marker(dict(type="marker", t=t, est=est, pose=p))

            if now >= next_step:
                next_step = now + 0.1
                if running:
                    c, listen = brain.step(t)
                    if c:
                        cmd = c
                    if listen:
                        link.listen(listen.get("dur", 0.8))
                    keep.set(cmd["v"], cmd["w"], cmd["door"])
                else:
                    keep.set(0.0, 0.0, door_manual)                   # link alive, rover still
                tm = link.telem or {}
                log.writerow([round(t, 2), brain.state, *np.round(brain.pose, 3), cmd["v"], cmd["w"], cmd["door"],
                              tm.get("tof", {}).get("front"), "".join(k for k, v in tm.get("cliff", {}).items() if v),
                              tm.get("capture"), tm.get("vbat"), tm.get("fault"),
                              *(np.round(brain.token_est, 3) if brain.token_est is not None else ("", ""))])

            if brain.state == "DONE" or (t_go and now - t_go > a.max_time):
                keep.set(0.0, 0.0, cmd["door"])
                print("finished: %s after %.1f s" % (brain.state, now - t_go))
                break
            if not link.connected(1.0):
                print("telemetry lost for 1 s (the rover has stopped itself)")

            if not a.no_window and frame is not None:
                view = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                tm = link.telem
                txt = ["%s  %s" % ("RUN" if running else "PAUSED (g = go)", brain.state),
                       "pose %.2f %.2f %3.0f deg" % (brain.pose[0], brain.pose[1], math.degrees(brain.pose[2])),
                       "tof %.2f  vbat %.1f  %s" % (tm["tof"]["front"], tm["vbat"],
                                                   "EDGE" if any(tm["cliff"].values()) else "")]
                for i, s in enumerate(txt):
                    cv2.putText(view, s, (4, 14 + 14 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                cv2.imshow("AEGIS", cv2.resize(view, (640, 480)))
            if not a.no_window:
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    keep.set(0.0, 0.0, cmd["door"])
                    break
                if k == ord("g") and not running:
                    running, t_go = True, now
                    print("GO")
                elif k == ord(" "):
                    running = not running
                    print("resumed" if running else "paused")
                elif k == ord("o") and not running:
                    door_manual = not door_manual
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        keep.close()
        link.stop()
        logf.close()
        if keep.gaps:
            g = np.array(keep.gaps[1:]) * 1000
            print("command gaps: median %.0f ms, max %.0f ms" % (np.median(g), g.max()))
        cam.close()
        print("log:", log_path)


if __name__ == "__main__":
    main()
