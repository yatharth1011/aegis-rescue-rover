"""
One complete mission in the 3D simulator: robot (firmware safety + control),
lossy Wi-Fi link, laptop brain (YOLO, scream detector, mapping, planning).

    demo 1: token hidden behind obstacles, no sound -> systematic inspection
    demo 2: token somewhere, a speaker at the token screams 0.6 s every 2 s
"""
import math
import time

import numpy as np

from audio import FS as AFS
from audio import Room, synth_scream
from brain import Brain
from link import Link
from robot import Robot
from world import World

DT = 0.01


class YoloDetector:
    def __init__(self, weights, device="cpu"):
        from ultralytics import YOLO
        self.m = YOLO(weights)
        self.device = device

    def __call__(self, img):
        r = self.m.predict(img, imgsz=320, conf=0.25, device=self.device, verbose=False)[0]
        if r.boxes is None or len(r.boxes) == 0:
            return []
        xyxy = r.boxes.xyxy.cpu().numpy()
        conf = r.boxes.conf.cpu().numpy()
        return [(*b, float(c)) for b, c in zip(xyxy, conf)]


def run(seed, demo, link_profile="typical", detector=None, max_t=240.0, record=None):
    rng = np.random.default_rng(seed + 7919 * demo)
    world = World(seed, token_hidden=(demo == 1), n_decoys=2)
    link = Link(link_profile, rng)
    robot = Robot(world, link, rng)
    robot.scream_clip = synth_scream(0.6, AFS, rng)
    room = Room(rng)
    brain = Brain(world.info["start"], detector, scream_mode=(demo == 2), rng=rng)
    token_xy = world.info["token"]
    sched = (lambda tt: (tt % 2.0) < 0.6) if demo == 2 else (lambda tt: np.zeros_like(tt, bool))

    out = dict(seed=seed, demo=demo, link=link_profile, outcome="timeout", t_end=max_t, collisions=0, fell=False)
    contact_prev = decoy_prev = False
    last_contact_t = -9.0
    out["decoy_bumps"] = 0
    path_len = 0.0
    prev_xy = world.pose()[:2]
    frame_bytes = []
    t0 = time.time()
    t = 0.0
    while t < max_t:
        robot.control_step(t, DT)
        world.step(5)
        k = int(round(t / DT))
        if k % 5 == 0:
            robot.send_telemetry(t)
        if k % 12 == 0:
            frame_bytes.append(robot.send_frame(t))
            # ArUco start pad (A5 print at the start pose): the laptop measures the rover's pose
            # from it when it is in the camera view, 0.15-0.9 m away, with ~2 cm / 2 deg noise
            x, y, th = world.pose()
            sx0, sy0 = world.info["start"][:2]
            cx, cy = x + 0.072 * math.cos(th), y + 0.072 * math.sin(th)
            d = math.hypot(sx0 - cx, sy0 - cy)
            bearing = (math.atan2(sy0 - cy, sx0 - cx) - th + math.pi) % (2 * math.pi) - math.pi
            if 0.15 < d < 0.9 and abs(bearing) < 0.42 and rng.random() < 0.8:
                meas = (x + 0.02 * rng.standard_normal(), y + 0.02 * rng.standard_normal(), th + 0.035 * rng.standard_normal())
                link.up.send(t, dict(type="marker", t=t, est=robot.est.copy(), pose=meas), 48)
        if robot.pending_listen is not None:
            robot.listen(t, room, token_xy if demo == 2 else None, sched, dur=robot.pending_listen.get("dur", 0.8))
            robot.pending_listen = None
        for m in link.up.receive(t):
            if m["type"] == "telem":
                brain.on_telem(m)
            elif m["type"] == "frame":
                brain.on_frame(m, t)
            elif m["type"] == "audio":
                brain.on_audio(m, t)
            elif m["type"] == "marker":
                brain.on_marker(m)
        if k % 10 == 0:
            cmd, listen = brain.step(t)
            if cmd:
                link.down.send(t, cmd, 16)
            if listen:
                link.down.send(t, listen, 16)
        # robot handles 'listen' messages in control_step (stores pending_listen)
        c = world.contacts_with_obstacles() > 0
        if c and not contact_prev and t - last_contact_t > 1.0:     # one episode per contact, 1 s apart
            out["collisions"] += 1
        if c:
            last_contact_t = t
        contact_prev = c
        cd = world.contacts_with_decoys() > 0
        if cd and not decoy_prev:
            out["decoy_bumps"] += 1
        decoy_prev = cd
        xy = world.pose()[:2]
        path_len += float(np.hypot(*(xy - prev_xy)))
        prev_xy = xy
        if record is not None and k % 25 == 0:
            record.append(dict(t=t, pose=world.pose().copy(), est=brain.pose.copy(), state=brain.state,
                               token=world.token_pos()[:2].copy(), bearing=brain.bearing,
                               path=list(brain.path), inspected=brain.inspected.copy() if k % 100 == 0 else None,
                               occ=(brain.occ > 1.0).copy() if k % 100 == 0 else None,
                               frame=world.render() if k % 50 == 0 else None))
        if world.fell():
            out.update(outcome="fell", t_end=t, fell=True)
            break
        if brain.state == "DONE":
            inside, _ = world.token_in_bay()
            home = float(np.hypot(*(world.pose()[:2] - world.info["start"][:2])))
            out.update(outcome="success" if inside and home < 0.15 else
                       ("lost_token" if not inside else "home_off"), t_end=t, home_err=home)
            break
        t += DT
    inside, _ = world.token_in_bay()
    out.update(dict(token_in_bay=bool(inside), path_len=path_len, wall_s=time.time() - t0,
                    first_seen=brain.log["first_seen"], captured=brain.log["captured"], home=brain.log["home"],
                    scream_heard=brain.log["scream_heard"], detections=brain.log["detections"],
                    approach_attempts=brain.log["approach_attempts"], marker_fixes=brain.marker_fixes,
                    inspected_pct=100.0 * brain.inspected[brain.on_table].mean(),
                    odom_err=float(np.hypot(*(brain.pose[:2] - world.pose()[:2]))),
                    frame_kb=float(np.mean(frame_bytes)) / 1024 if frame_bytes else 0.0,
                    **{"link_" + k2: v for k2, v in link.stats().items()}, **robot.events))
    world.close()
    return out
