"""
Rover side (what runs on the ESP32-S3 / ESP32-CAM in the edge architecture).

Keeps everything that must not depend on Wi-Fi:
  - wheel PI loops and stall detection        (firmware C code via core_ffi)
  - CBF forward-speed safety filter            (firmware C code via core_ffi)
  - table-edge reflex from the IR cliff sensors: stop and back off
  - link watchdog: no command for 300 ms -> stop and hold
  - odometry from encoders + gyro (with realistic drift)
  - GCC-PHAT bearing at 48 kHz on the robot (firmware C code); a 16 kHz mono
    stream goes to the laptop for scream classification
Sends telemetry (20 Hz), JPEG camera frames (8 fps) and audio over the link.
"""
import io
import math

import numpy as np
from PIL import Image

import core_ffi
from audio import FS as AFS
from audio import to16k
from world import TOKEN

D_SAFE = 0.06              # front ToF now sits 13 mm behind the funnel tip (Mk2 geometry)
CBF_ALPHA = 2.0
V_REV_MAX = 0.10
WATCHDOG_S = 0.30
VBAT = 11.4
WHEEL_R = 0.022
TRACK = 0.132
SKID_CAL = 1.45            # firmware's calibrated skid factor
DOOR_OPEN = math.radians(110)


class Robot:
    def __init__(self, world, link, rng, skid_true=None):
        self.w, self.link, self.rng = world, link, rng
        self.drive = core_ffi.Drive()
        self.doa = core_ffi.DOA()
        self.skid_true = skid_true or SKID_CAL * (1 + rng.uniform(-0.12, 0.12))
        self.gyro_bias = 0.0008 * rng.standard_normal()     # residual after the boot calibration (~0.05 deg/s)
        self.est = np.zeros(3)                         # odometry in the start frame
        self.cmd = (0.0, 0.0)
        self.door = 0.0
        self.last_cmd_t = -1.0
        self.wheel_w = np.zeros(4)
        self.tof = {"front": 2.0, "left": 2.0, "right": 2.0}
        self.tof_t = -1.0
        self.backoff = 0.0
        self.fwd_off = 0.0
        self.cliff_t = -9.0
        self.cliff_front = False
        self.stalled = False
        self.esc = 0.0
        self.events = dict(cbf_ticks=0, cliff_reflex=0, watchdog_stops=0, stalls=0)
        self.cam_seq = 0
        self.pending_listen = None
        self.scream_clip = None

    # ------------------------------------------------------------------ 100 Hz
    def control_step(self, t, dt):
        for msg in self.link.down.receive(t):
            if msg["type"] == "cmd":
                self.cmd = (msg["v"], msg["w"])
                self.door = msg["door"]
                self.last_cmd_t = t
            elif msg["type"] == "listen":
                self.pending_listen = msg

        v, w = self.cmd
        if t - self.last_cmd_t > WATCHDOG_S:
            if v or w:
                self.events["watchdog_stops"] += 1
            v, w = 0.0, 0.0

        if t - self.tof_t >= 0.033:
            for k in self.tof:
                self.tof[k] = self.w.tof(k, self.rng)
            self.tof_t = t
        cliff = self.w.cliff()
        # Table-edge reflex: whichever sensor sees the edge, stop turning and move away from it
        # (back off for a front sensor, drive forward for the rear one). Overrides the brain.
        if (cliff["FL"] or cliff["FR"]) and self.backoff <= 0 and self.fwd_off <= 0:
            self.backoff = 0.06
            self.events["cliff_reflex"] += 1
        elif (cliff["R"] or cliff.get("RL") or cliff.get("RR")) and self.fwd_off <= 0 and self.backoff <= 0:
            self.fwd_off = 0.06
            self.events["cliff_reflex"] += 1
        if any(cliff.values()):
            self.cliff_t = t
            self.cliff_front = cliff["FL"] or cliff["FR"]
        if t - self.cliff_t < 1.0:                      # right after an edge: no turning, no move toward it
            w = 0.0
            v = min(v, 0.0) if self.cliff_front else max(v, 0.0)
        if self.backoff > 0:
            v, w = -0.08, 0.0
            self.backoff -= 0.08 * dt
        elif self.fwd_off > 0:
            v, w = 0.08, 0.0
            self.fwd_off -= 0.08 * dt
        if self.esc > 0:                                        # stall escape: back off
            v, w = -0.06, 0.0
            self.esc -= 0.06 * dt

        v, w, lim = core_ffi.safety_filter(v, w, self.tof["front"], True, D_SAFE, CBF_ALPHA, V_REV_MAX)
        if lim and self.cmd[0] > 0:
            self.events["cbf_ticks"] += 1

        duty, stalled = self.drive.step(v, w, self.wheel_w.astype(np.float32), VBAT, dt)
        if stalled and not self.stalled:
            self.events["stalls"] += 1
            self.esc = 0.06
        self.stalled = stalled

        # motors -> wheel speeds (first order), true skid factor and slip -> body velocity
        w_ss = duty * 20.94 * VBAT / 12.0                      # 200 rpm no-load at 12 V
        self.wheel_w += (w_ss - self.wheel_w) * min(1.0, dt / 0.05)
        wl, wr = 0.5 * (self.wheel_w[0] + self.wheel_w[1]), 0.5 * (self.wheel_w[2] + self.wheel_w[3])
        v_cmd_body = WHEEL_R * 0.5 * (wl + wr) * 0.97
        w_cmd_body = WHEEL_R * (wr - wl) / (TRACK * self.skid_true)
        self.w.set_body_velocity(v_cmd_body, w_cmd_body, lateral_slip=0.004 * self.rng.standard_normal())
        self.w.set_door(DOOR_OPEN if self.door > 0.5 else 0.0)

        # what the encoders really see: the body moved (or not, if blocked)
        bv, _, bw = self.w.body_vel()
        blocked = abs(v_cmd_body) > 0.03 and abs(bv) < 0.3 * abs(v_cmd_body)
        if blocked:
            self.wheel_w *= 0.6                                 # motors stall against the obstacle
        # odometry. Encoders: the wheels roll on the table with ~3 % slip noise; while the
        # rover is pushing against something they slip much more (wheel spin).
        slip = 0.03 * self.rng.standard_normal() + (0.25 * self.rng.random() if blocked else 0.0)
        v_enc = bv * (1.0 + slip) + (0.25 * v_cmd_body if blocked else 0.0)
        w_enc = bw * self.skid_true / SKID_CAL
        gz = bw + self.gyro_bias + 0.003 * self.rng.standard_normal()
        wf = 0.95 * gz + 0.05 * w_enc
        th_mid = self.est[2] + 0.5 * wf * dt
        self.est += np.array([v_enc * math.cos(th_mid) * dt, v_enc * math.sin(th_mid) * dt, wf * dt])
        self.est[2] = (self.est[2] + math.pi) % (2 * math.pi) - math.pi
        return dict(cliff=cliff)

    # ------------------------------------------------------------------ telemetry
    def send_telemetry(self, t):
        msg = dict(type="telem", t=t, est=self.est.copy(), tof=dict(self.tof), cliff=self.w.cliff(),
                   capture=self.w.capture_ir(), stalled=self.stalled, door=self.door)
        self.link.up.send(t, msg, 64)

    def send_frame(self, t):
        img = self.w.render()
        a = img.astype(np.float32) + self.rng.normal(0, 4, img.shape)
        im = Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))
        buf = io.BytesIO()
        im.save(buf, "JPEG", quality=55)                           # ESP32-CAM QVGA JPEG
        data = buf.getvalue()
        self.cam_seq += 1
        self.link.up.send(t, dict(type="frame", t=t, est=self.est.copy(), jpeg=data, seq=self.cam_seq), len(data))
        return len(data)

    def listen(self, t, room, src_xy, scream_on_fn, dur=0.8):
        """Stationary capture: GCC-PHAT on the robot, 16 kHz mono to the laptop."""
        n = int(dur * AFS)
        if src_xy is not None:
            sig = np.zeros(n, np.float32)
            tt = t + np.arange(n) / AFS
            on = scream_on_fn(tt)
            sig[on] = self.scream_clip[(np.arange(n)[on]) % len(self.scream_clip)]
        else:
            sig = None
        L, R = room.capture(src_xy, sig, tuple(self.w.pose()), scream_gain=1.0 if sig is not None else 0.0,
                            dur=dur)
        doas = []
        for i in range(0, n - 1024, 1024):
            r = self.doa.frame(L[i:i + 1024], R[i:i + 1024])
            if r["valid"]:
                doas.append(r["bearing"])
        mono = to16k(0.5 * (L + R))
        self.link.up.send(t + dur, dict(type="audio", t=t + dur, est=self.est.copy(), doas=doas,
                                        pcm=mono), 2 * len(mono) + 64)
        return len(doas)
