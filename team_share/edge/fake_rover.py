"""
Stand-in for the real rover and ESP32-CAM, for testing the laptop side without hardware.
Speaks the firmware's wire protocol (UDP JSON + audio chunks, MJPEG over HTTP) on localhost and is
driven by the 3D simulator (physics, camera, room acoustics, the robot.py reflexes).

    ~/aegis_venv/bin/python team_share/edge/fake_rover.py --demo 2 --seed 4036
    ~/aegis_venv/bin/python team_share/edge/aegis_edge.py --demo 2 --rover-ip 127.0.0.1 \
        --cam-url http://127.0.0.1:8081/stream --start <printed start pose>
"""
import argparse
import http.server
import json
import math
import os
import signal
import socket
import struct
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "simulation3d"))

from audio import FS as AFS, Room, synth_scream  # noqa: E402
from robot import Robot  # noqa: E402
from world import World  # noqa: E402

DT = 0.01
_jpeg = {"data": None}


class MJPEG(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace;boundary=aegisframe")
        self.end_headers()
        last = None
        try:
            while True:
                d = _jpeg["data"]
                if d is not None and d is not last:
                    self.wfile.write(b"\r\n--aegisframe\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(d))
                    self.wfile.write(d)
                    last = d
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *a):
        pass


class WireLink:
    """Adapter: Robot talks to .down/.up like the simulated link; this puts it on the real wire format."""

    def __init__(self, port_in=4210, laptop=("127.0.0.1", 4211)):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port_in))
        self.sock.setblocking(False)
        self.laptop = laptop
        self.down, self.up = self, self
        self.clip_id = 0

    def receive(self, t):                       # robot.link.down.receive(t)
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                return out
            m = json.loads(data)
            if m.get("type") == "cmd":
                out.append(dict(type="cmd", v=m["v"], w=m["w"], door=m["door"]))
            elif m.get("type") == "listen":
                out.append(dict(type="listen", dur=m.get("dur", 0.8)))

    def send(self, t, msg, size):              # robot.link.up.send(t, msg, size)
        k = msg["type"]
        if k == "telem":
            e = msg["est"]
            j = dict(type="telem", t=t, est=[float(e[0]), float(e[1]), float(e[2])],
                     tof={n: float(v) for n, v in msg["tof"].items()}, tof_ok=7,
                     cliff={n: int(bool(v)) for n, v in msg["cliff"].items()}, capture=int(bool(msg["capture"])),
                     stalled=int(bool(msg["stalled"])), door=int(msg["door"] > 0.5), vbat=11.4, fault=0, mode="edge")
            self.sock.sendto(json.dumps(j).encode(), self.laptop)
        elif k == "frame":
            _jpeg["data"] = msg["jpeg"]
        elif k == "audio":
            self.clip_id = (self.clip_id + 1) & 0xFFFF
            pcm = np.clip(msg["pcm"] * 32767, -32768, 32767).astype("<i2")
            total = (len(pcm) + 699) // 700
            for i in range(total):
                part = pcm[i * 700:(i + 1) * 700]
                self.sock.sendto(b"AUD1" + struct.pack("<HHHH", self.clip_id, i, total, len(part)) + part.tobytes(),
                                 self.laptop)
            e = msg["est"]
            self.sock.sendto(json.dumps(dict(type="audio_meta", id=self.clip_id, n=len(pcm), fs=16000,
                                             est=[float(e[0]), float(e[1]), float(e[2])],
                                             doas=[float(d) for d in msg["doas"]])).encode(), self.laptop)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", type=int, choices=(1, 2), default=2)
    ap.add_argument("--seed", type=int, default=4036)
    ap.add_argument("--max-time", type=float, default=260.0)
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed + 7919 * a.demo)
    w = World(a.seed, token_hidden=(a.demo == 1), n_decoys=2)
    link = WireLink()
    robot = Robot(w, link, rng)
    robot.scream_clip = synth_scream(0.6, AFS, rng)
    room = Room(rng)
    sched = (lambda tt: (tt % 2.0) < 0.6) if a.demo == 2 else (lambda tt: np.zeros_like(tt, bool))
    src = w.info["token"] if a.demo == 2 else None
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 8081), MJPEG)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    sx, sy, sth = w.info["start"]
    print("fake rover up. start pose: --start %.3f %.3f %.1f   (table 1.8 x 1.2)" % (sx, sy, math.degrees(sth)), flush=True)
    print("true token at %.3f %.3f" % tuple(w.info["token"]), flush=True)

    t, k, t0 = 0.0, 0, time.time()
    fell = False
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        loop(a, w, robot, room, src, sched, sx, sy, sth)
    except KeyboardInterrupt:
        pass
    fell = w.fell()
    inside, _ = w.token_in_bay()
    home = float(np.hypot(*(w.pose()[:2] - np.array(w.info["start"][:2]))))
    print("RESULT fell=%s token_in_bay=%s home_err=%.3f m events=%s" % (fell, bool(inside), home, robot.events), flush=True)


def loop(a, w, robot, room, src, sched, sx, sy, sth):
    t, k, t0 = 0.0, 0, time.time()
    while t < a.max_time:
        robot.control_step(t, DT)
        w.step(5)
        if k % 5 == 0:
            robot.send_telemetry(t)
        if k % 12 == 0:
            robot.send_frame(t)
        if robot.pending_listen is not None:
            robot.listen(t, room, src, sched, dur=robot.pending_listen.get("dur", 0.8))
            robot.pending_listen = None
        if w.fell():
            return
        k += 1
        t += DT
        lag = t - (time.time() - t0)
        if lag > 0:
            time.sleep(lag)                     # real time, like the hardware
        if k % 100 == 0:
            e = robot.est
            ox = sx + math.cos(sth) * e[0] - math.sin(sth) * e[1]
            oy = sy + math.sin(sth) * e[0] + math.cos(sth) * e[1]
            p = w.pose()
            print("wall %.2f t %5.1f true %.3f %.3f %4.0f  odom %.3f %.3f %4.0f  err %.3f m" %
                  (time.time(), t, p[0], p[1], math.degrees(p[2]), ox, oy, math.degrees(sth + e[2]),
                   math.hypot(p[0] - ox, p[1] - oy)), flush=True)


if __name__ == "__main__":
    main()
