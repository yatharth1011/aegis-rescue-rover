"""
Wi-Fi link between the rover (ESP32-S3 / ESP32-CAM) and the laptop brain.

Per message: base latency + exponential jitter + serialisation time at the
link rate; Gilbert-Elliott burst loss; and random outages where everything is
lost for 0.5-3 s (someone walks between laptop and robot, AP channel change).
Profiles are rough figures for a crowded 2.4 GHz lab.
"""
import heapq

import numpy as np

PROFILES = {
    "good":    dict(base=0.008, jitter=0.004, bw=6e6, p_loss=0.005, p_gb=0.002, p_bg=0.30, bad_loss=0.5, outage_per_min=0.0, outage=(0.5, 1.0)),
    "typical": dict(base=0.015, jitter=0.015, bw=3e6, p_loss=0.02, p_gb=0.010, p_bg=0.20, bad_loss=0.7, outage_per_min=1.0, outage=(0.5, 1.5)),
    "bad":     dict(base=0.030, jitter=0.040, bw=1.5e6, p_loss=0.05, p_gb=0.030, p_bg=0.15, bad_loss=0.8, outage_per_min=3.0, outage=(1.0, 3.0)),
}


class Channel:
    """One direction of the link."""

    def __init__(self, profile, rng):
        self.p = PROFILES[profile]
        self.rng = rng
        self.q = []
        self.bad = False
        self.busy_until = 0.0
        self.sent = self.lost = 0
        self.outage_until = -1.0
        self._seq = 0

    def _in_outage(self, t):
        if t < self.outage_until:
            return True
        rate = self.p["outage_per_min"] / 60.0
        if rate > 0 and self.rng.random() < rate * 0.01:          # called at <= 100 Hz
            self.outage_until = t + self.rng.uniform(*self.p["outage"])
            return True
        return False

    def send(self, t, msg, size_bytes):
        self.sent += 1
        if self.bad:
            self.bad = self.rng.random() > self.p["p_bg"]
        else:
            self.bad = self.rng.random() < self.p["p_gb"]
        loss = self.p["bad_loss"] if self.bad else self.p["p_loss"]
        if self._in_outage(t) or self.rng.random() < loss:
            self.lost += 1
            return
        start = max(t, self.busy_until)
        self.busy_until = start + size_bytes * 8 / self.p["bw"]
        arrive = self.busy_until + self.p["base"] + self.rng.exponential(self.p["jitter"])
        self._seq += 1
        heapq.heappush(self.q, (arrive, self._seq, msg))

    def receive(self, t):
        out = []
        while self.q and self.q[0][0] <= t:
            out.append(heapq.heappop(self.q)[2])
        return out


class Link:
    def __init__(self, profile, rng):
        self.up = Channel(profile, rng)      # robot -> laptop
        self.down = Channel(profile, rng)    # laptop -> robot

    def stats(self):
        s = self.up.sent + self.down.sent
        lost = self.up.lost + self.down.lost
        return dict(sent=s, lost=lost, loss_pct=100.0 * lost / max(1, s))
