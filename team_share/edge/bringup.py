"""
Hardware bring-up: live view of every sensor, keyboard driving, door and microphone test.
Run it with the rover on a stand (wheels off the table) first, then on the table.

    ~/aegis_venv/bin/python team_share/edge/bringup.py [--rover-ip IP]

Keys:  w / s  forward / back      a / d  turn left / right      space  stop
       o      door open / close   l      listen 0.8 s (mic test)  r      record a scream (2 x 0.75 s)
       +/-    drive speed         q      quit
"""
import argparse
import curses
import math
import os
import sys
import time
import wave

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from aegis_link import KeepAlive, RoverLink  # noqa: E402

SCREAMS = os.path.join(HERE, "..", "simulation3d", "screams")
FAULTS = {1: "TILT", 2: "BATTERY LOW", 4: "IMU", 8: "FRONT TOF", 16: "IR/PCF8574"}
FAULT_HELP = {1: "rover tilted > 60 deg or upside down", 2: "pack below 9.9 V: charge it",
              4: "MPU-6050 not answering: check SDA/SCL, 3V3", 8: "front VL53L0X missing: check XSHUT GPIO10 wiring",
              16: "PCF8574 not at 0x20: check address pins, SDA/SCL; driving is disabled"}


def save_wav(pcm, fs, path):
    with wave.open(path, "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(fs)
        f.writeframes((np.clip(pcm, -1, 1) * 32767).astype("<i2").tobytes())


def ui(scr, a):
    curses.curs_set(0)
    scr.nodelay(True)
    link = RoverLink(a.rover_ip)
    keep = KeepAlive(link, max_age=0.35)
    v = w = 0.0
    speed = 0.10
    door = False
    held = 0.0
    notes = []
    recording = []                                   # clips for the current scream recording
    while True:
        for m in link.poll():
            if m["type"] == "audio":
                pcm, doas = m["pcm"], m["doas"]
                rms = 20 * math.log10(max(float(np.sqrt(np.mean(pcm ** 2))), 1e-6))
                bear = np.degrees(np.mean(doas)) if doas else float("nan")
                notes.append("audio: %.2f s, level %.0f dBFS, %d bearings, mean %+.0f deg" %
                             (len(pcm) / m["fs"], rms, len(doas), bear))
                if recording is not None and recording and recording[0] == "rec":
                    recording.append(pcm)
                    if len(recording) == 3:
                        os.makedirs(SCREAMS, exist_ok=True)
                        path = os.path.join(SCREAMS, time.strftime("scream_%Y%m%d_%H%M%S.wav"))
                        save_wav(np.concatenate(recording[1:]), m["fs"], path)
                        notes.append("saved " + os.path.relpath(path, os.path.join(HERE, "..")))
                        recording = []
                    else:
                        link.listen(0.75)
        k = scr.getch()
        now = time.time()
        if k == ord("q"):
            break
        if k in (ord("w"), ord("s"), ord("a"), ord("d")):
            v = {ord("w"): speed, ord("s"): -min(speed, 0.10)}.get(k, 0.0)
            w = {ord("a"): 0.8, ord("d"): -0.8}.get(k, 0.0)
            held = now
        elif k == ord(" "):
            v = w = 0.0
        elif k == ord("o"):
            door = not door
        elif k == ord("l"):
            link.listen(0.8)
        elif k == ord("r"):
            recording = ["rec"]
            notes.append("recording: scream now, about 1.5 s ...")
            link.listen(0.75)
        elif k in (ord("+"), ord("=")):
            speed = min(0.30, speed + 0.05)
        elif k == ord("-"):
            speed = max(0.05, speed - 0.05)
        if now - held > 0.3:                         # keys auto-repeat while held; stop when released
            v = w = 0.0
        keep.set(v, w, door)

        scr.erase()
        t = link.telem
        scr.addstr(0, 0, "A.E.G.I.S. bring-up   q quit | w/s/a/d drive | space stop | o door | l listen | r record scream")
        if t is None:
            scr.addstr(2, 0, "waiting for rover telemetry on UDP 4211 ... (same Wi-Fi? rover powered? see edge/README.md)")
        else:
            age = now - link.telem_t
            scr.addstr(2, 0, "rover %s   link %s   mode %-10s   battery %.2f V   loop max %s us" %
                       (link.rover[0], "OK" if age < 0.5 else "LOST %.1f s" % age, t["mode"], t["vbat"],
                        t.get("loop_max_us")))
            f = int(t["fault"])
            if f:
                for i, (bit, name) in enumerate((b, n) for b, n in FAULTS.items() if f & b):
                    scr.addstr(3 + i, 0, "FAULT %-12s %s" % (name, FAULT_HELP[bit]), curses.A_BOLD)
            else:
                scr.addstr(3, 0, "no faults")
            ok = int(t.get("tof_ok", 7))
            scr.addstr(8, 0, "distance  front %s  left %s  right %s" % tuple(
                ("%.3f m" % t["tof"][n]) if ok & (1 << i) else "  ----  " for i, n in enumerate(("front", "left", "right"))))
            c = t["cliff"]
            scr.addstr(9, 0, "edge      FL %s  FR %s  R %s  RL %s  RR %s     (EDGE = no table under that sensor)" %
                       tuple("EDGE" if c[k2] else "ok  " for k2 in ("FL", "FR", "R", "RL", "RR")))
            scr.addstr(10, 0, "bay       %s     door %s     stalled %s     PCF8574 pins %s" %
                       ("TOKEN INSIDE" if t["capture"] else "empty", "open" if t["door"] else "closed",
                        t["stalled"], format(int(t.get("ir_raw", 255)), "08b")[::-1]))
            e = t["est"]
            scr.addstr(11, 0, "odometry  x %.3f  y %.3f  heading %.0f deg     edge reflexes %s  stall escapes %s" %
                       (e[0], e[1], math.degrees(e[2]), t.get("edge_events"), t.get("stall_events")))
            scr.addstr(13, 0, "drive cmd v %+.2f m/s  w %+.2f rad/s   speed setting %.2f m/s" % (v, w, speed))
        for i, n in enumerate(notes[-8:]):
            scr.addstr(15 + i, 0, n[:150])
        scr.refresh()
        time.sleep(0.03)
    keep.set(0.0, 0.0, door)
    time.sleep(0.2)
    keep.close()
    link.stop(door)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--rover-ip")
    curses.wrapper(ui, ap.parse_args())
