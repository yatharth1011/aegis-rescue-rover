"""
Laptop side of the rover link (protocol in firmware/aegis_s3/main/task_net.c).

    link = RoverLink()          # listens on UDP 4211; finds the rover and the camera by their broadcasts
    link.poll()                 # call often: returns telemetry / audio / camera-announce messages
    link.cmd(v, w, door)        # 10 Hz or faster, or the rover's 300 ms watchdog stops it
    link.listen(0.8)            # ask for an audio clip; it arrives later as an "audio" message

    cam = CameraStream(url)     # background MJPEG reader; cam.latest() -> (jpeg bytes, laptop time)

Messages are converted to exactly the dicts simulation3d/brain.py consumes.
"""
import json
import socket
import struct
import threading
import time
import urllib.request

import numpy as np

ROVER_PORT = 4210
LAPTOP_PORT = 4211


class RoverLink:
    def __init__(self, rover_ip=None, laptop_port=LAPTOP_PORT, rover_port=ROVER_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("", laptop_port))
        self.sock.setblocking(False)
        self.rover = (rover_ip, rover_port) if rover_ip else None
        self.rover_port = rover_port
        self.cam_url = None
        self.telem = None
        self.telem_t = 0.0
        self.rx = self.tx = 0
        self._chunks = {}
        self._tx_lock = threading.Lock()

    # ---------------------------------------------------------------- receive
    def poll(self):
        out = []
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError):
                return out
            self.rx += 1
            if data[:4] == b"AUD1":
                cid, idx, total, n = struct.unpack_from("<HHHH", data, 4)
                buf = self._chunks.setdefault(cid, {"total": total, "parts": {}})
                buf["parts"][idx] = np.frombuffer(data, "<i2", n, 12)
                continue
            try:
                m = json.loads(data)
            except ValueError:
                continue
            kind = m.get("type")
            if kind == "telem":
                if self.rover is None or self.rover[0] != addr[0]:
                    self.rover = (addr[0], self.rover_port)
                m["est"] = np.array(m["est"], float)
                m["cliff"] = {k: bool(v) for k, v in m["cliff"].items()}
                for k in ("capture", "stalled", "door"):
                    m[k] = bool(m[k])
                self.telem, self.telem_t = m, time.time()
                out.append(m)
            elif kind == "audio_meta":
                buf = self._chunks.pop(m["id"], None)
                if not buf or len(buf["parts"]) != buf["total"]:
                    continue                                  # a chunk was lost: drop the clip
                pcm = np.concatenate([buf["parts"][i] for i in range(buf["total"])]).astype(np.float32) / 32767.0
                out.append(dict(type="audio", est=np.array(m["est"], float), doas=list(m["doas"]), pcm=pcm,
                                fs=m.get("fs", 16000)))
            elif kind == "cam":
                if self.cam_url != m["url"]:
                    self.cam_url = m["url"]
                    out.append(m)
        return out

    # ---------------------------------------------------------------- send
    def _send(self, msg):
        if self.rover:
            with self._tx_lock:
                self.sock.sendto(json.dumps(msg).encode(), self.rover)
                self.tx += 1

    def cmd(self, v, w, door=False):
        self._send(dict(type="cmd", v=round(float(v), 4), w=round(float(w), 4), door=1 if door else 0))

    def stop(self, door=None):
        d = self.telem["door"] if (door is None and self.telem) else bool(door)
        for _ in range(3):
            self.cmd(0.0, 0.0, d)

    def listen(self, dur=0.8):
        self._send(dict(type="listen", dur=float(dur)))

    def connected(self, timeout=1.0):
        return self.telem is not None and time.time() - self.telem_t < timeout

    def wait_for_rover(self, timeout=30.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.poll()
            if self.telem is not None:
                return True
            time.sleep(0.05)
        return False


class KeepAlive:
    """Resends the newest command at 20 Hz from a thread, so slow brain work (detector warm-up,
    planning) never trips the rover's 300 ms watchdog. If the brain itself stops updating for
    max_age s, it sends zero instead: a frozen laptop still stops the rover."""

    def __init__(self, link, max_age=0.5, period=0.05):
        self.link, self.max_age, self.period = link, max_age, period
        self.v = self.w = 0.0
        self.door = False
        self.t = 0.0
        self.gaps = []
        self._lock = threading.Lock()
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def set(self, v, w, door):
        with self._lock:
            self.v, self.w, self.door, self.t = v, w, door, time.time()

    def _run(self):
        last = time.time()
        while not self._stop:
            with self._lock:
                fresh = time.time() - self.t < self.max_age
                v, w, d = (self.v, self.w, self.door) if fresh else (0.0, 0.0, self.door)
            self.link.cmd(v, w, d)
            now = time.time()
            self.gaps.append(now - last)
            last = now
            time.sleep(self.period)

    def close(self):
        self._stop = True


class CameraStream:
    """Reads the ESP32-CAM MJPEG stream in a thread and keeps the newest frame."""

    def __init__(self, url):
        self.url = url
        self._jpeg, self._t, self.frames = None, 0.0, 0
        self._lock = threading.Lock()
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop:
            try:
                with urllib.request.urlopen(self.url, timeout=5) as r:
                    buf = b""
                    while not self._stop:
                        buf += r.read1(8192)                     # whatever has arrived: read() would wait for the next frame
                        a = buf.find(b"\xff\xd8")
                        b = buf.find(b"\xff\xd9", a + 2)
                        if a >= 0 and b > a:
                            with self._lock:
                                self._jpeg, self._t = buf[a:b + 2], time.time()
                                self.frames += 1
                            buf = buf[b + 2:]
                        elif len(buf) > 400000:
                            buf = b""
            except OSError:
                time.sleep(1.0)                                # camera rebooting or out of range: retry

    def latest(self):
        with self._lock:
            return self._jpeg, self._t

    def close(self):
        self._stop = True


def find_camera(link, timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout and link.cam_url is None:
        link.poll()
        time.sleep(0.05)
    return link.cam_url
