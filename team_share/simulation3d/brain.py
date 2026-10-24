"""
Laptop "edge brain". Receives telemetry, JPEG frames and audio over the
(lossy) link and sends velocity / door commands back.

  Mapping      occupancy grid (ToF cones) + "inspected" grid (camera ground
               footprint, occlusion-checked), in the table frame. The demo
               starts the rover at a marked pose on a table of known size.
  Perception   YOLO victim-token detector on each frame; the bottom-centre of
               the box is projected onto the table plane -> token position.
               Screams: ScreamDetector on the 16 kHz stream; bearings from the
               robot's GCC-PHAT are fused in a world-frame vote histogram that
               resolves the two-mic front/back ambiguity.
  Planning     coverage: pick the reachable viewpoint that would inspect the
               most unseen table area per metre travelled (biased toward the
               scream bearing when there is one); A* on an inflated grid;
               pure-pursuit following.
  Rescue       pre-grasp pose 0.30 m in front of the token, door open, slow
               straight approach until the bay IR sensor sees it, door shut,
               verify, then the shortest path home over cells the rover has
               already seen free ("shortest of the tested paths").
"""
import heapq
import io
import math

import numpy as np
from PIL import Image

from audio import ScreamDetector
from world import CAM, TABLE, TOKEN

RES = 0.02
INFLATE = 0.14            # m: obstacle clearance for the rover centre (turning sweep ~0.16 m)
EDGE_KEEP = 0.20          # m: keep the centre this far from table edges (0.17 let a rear wheel slip over)
V_MAX = 0.16
W_MAX = 1.6


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Brain:
    def __init__(self, start_pose, detector, scream_mode, rng):
        self.x0, self.y0, self.th0 = start_pose
        self.det = detector
        self.scream_mode = scream_mode
        self.rng = rng
        self.nx, self.ny = int(TABLE["lx"] / RES), int(TABLE["ly"] / RES)
        self.occ = np.zeros((self.nx, self.ny), np.float32)        # log-odds
        self.seen_free = np.zeros((self.nx, self.ny), bool)
        self.inspected = np.zeros((self.nx, self.ny), bool)
        gx, gy = np.meshgrid((np.arange(self.nx) + 0.5) * RES, (np.arange(self.ny) + 0.5) * RES, indexing="ij")
        self.gx, self.gy = gx, gy
        edge = np.minimum.reduce([gx, TABLE["lx"] - gx, gy, TABLE["ly"] - gy])
        self.edge_bad = edge < EDGE_KEEP
        self.on_table = edge > 0.03
        self.pose = np.array(start_pose, float)
        self.telem = None
        self.state = "EXPLORE"
        self.t_state = 0.0
        self.path = []
        self.goal = None
        self.goal_heading = None
        self.replan_t = -1.0
        self.token_obs = []
        self.token_est = None
        self.scream = ScreamDetector()
        self.votes = np.zeros(36)
        self.bearing = None
        self.last_listen_t = -10.0
        self.listening_until = -1.0
        self.door = 0.0
        self.attempts = 0
        self.log = dict(first_seen=None, captured=None, home=None, scream_heard=None, detections=0,
                        false_detections=0, frames=0, approach_attempts=0)
        self.home_path_len = None
        self.tabu = []                 # viewpoints already used: (x, y, heading)
        self.corr = np.zeros(3)        # pose correction from the start-pad marker (table frame)
        self.marker_fixes = 0

    # ------------------------------------------------------------ inputs
    def to_table(self, est):
        c, s = math.cos(self.th0), math.sin(self.th0)
        p = np.array([self.x0 + c * est[0] - s * est[1], self.y0 + s * est[0] + c * est[1], wrap(self.th0 + est[2])])
        p += self.corr
        p[2] = wrap(p[2])
        return p

    def on_marker(self, m):
        """Pose of the rover measured from the ArUco start-pad marker (camera on the laptop side).
        Blend it into the odometry correction; the marker is only visible near home."""
        raw = self.to_table(m["est"]) - self.corr
        target = np.array(m["pose"]) - raw
        target[2] = wrap(target[2])
        d = target - self.corr
        d[2] = wrap(d[2])
        self.corr += 0.6 * d
        self.marker_fixes += 1

    def cell(self, x, y):
        return int(np.clip(x / RES, 0, self.nx - 1)), int(np.clip(y / RES, 0, self.ny - 1))

    CLIFF_POS = {"FL": (0.076, 0.052), "FR": (0.140, -0.050), "R": (-0.098, 0.0),
                 "RL": (-0.108, 0.055), "RR": (-0.108, -0.055)}

    def on_telem(self, m):
        self.telem = m
        self.pose = self.to_table(m["est"])
        self._edge_relocalise(m.get("cliff") or {})
        x, y, th = self.pose
        cx, cy = self.cell(x, y)
        r = int(0.09 / RES)
        self.seen_free[max(0, cx - r):cx + r, max(0, cy - r):cy + r] = True     # the rover's own footprint is free
        for name, ((px, py, _), yaw) in (("front", ((0.135, 0.0, 0), 0.0)), ("left", ((0.04, 0.057, 0), math.pi / 2)),
                                          ("right", ((0.04, -0.057, 0), -math.pi / 2))):
            d = m["tof"][name]
            sx, sy = x + math.cos(th) * px - math.sin(th) * py, y + math.sin(th) * px + math.cos(th) * py
            a = th + yaw
            reach = min(d, 1.2)
            for k in np.arange(0.0, reach, RES):
                i, j = self.cell(sx + math.cos(a) * k, sy + math.sin(a) * k)
                self.occ[i, j] = max(-4.0, self.occ[i, j] - 0.4)
                self.seen_free[i, j] = True
            if d < 1.9:
                i, j = self.cell(sx + math.cos(a) * d, sy + math.sin(a) * d)
                self.occ[i, j] = min(6.0, self.occ[i, j] + 2.0)

    def _edge_relocalise(self, cliff):
        """The table size is known: a cliff sensor firing means that sensor is at an edge.
        Snap the pose estimate perpendicular to the nearest edge (odometry drift fix)."""
        x, y, th = self.pose
        for k, on in cliff.items():
            if not on or k not in self.CLIFF_POS:
                continue
            px, py = self.CLIFF_POS[k]
            sx, sy = x + math.cos(th) * px - math.sin(th) * py, y + math.sin(th) * px + math.cos(th) * py
            d = {"x0": sx, "x1": TABLE["lx"] - sx, "y0": sy, "y1": TABLE["ly"] - sy}
            edge = min(d, key=lambda e: abs(d[e]))
            if abs(d[edge]) > 0.30:
                continue                                      # too far off to trust the match
            if edge == "x0":
                self.corr[0] -= sx
            elif edge == "x1":
                self.corr[0] += TABLE["lx"] - sx
            elif edge == "y0":
                self.corr[1] -= sy
            else:
                self.corr[1] += TABLE["ly"] - sy
            self.pose = self.to_table(self.telem["est"])
            self.edge_fixes = getattr(self, "edge_fixes", 0) + 1
            self.path = []                                    # replan from the corrected pose
            break

    def blocked(self):
        return self.occ > 1.0

    def on_frame(self, m, t):
        self.log["frames"] += 1
        pose = self.to_table(m["est"])
        self._mark_inspected(pose)
        img = np.asarray(Image.open(io.BytesIO(m["jpeg"])).convert("RGB"))
        boxes = self.det(img)
        for (x1, y1, x2, y2, conf) in boxes:
            if conf < 0.5:
                continue
            g = self._pixel_to_table((x1 + x2) / 2, y2, pose)
            if g is None:
                continue
            self.log["detections"] += 1
            self.token_obs.append((t, g))
            if self.log["first_seen"] is None:
                self.log["first_seen"] = t
        recent = [g for (tt, g) in self.token_obs if t - tt < 3.0]
        if len(recent) >= 2:
            est = np.median(np.array(recent), axis=0)
            # bottom edge of the box is the token's near side: push to its centre
            d = est - pose[:2]
            est = est + d / (np.linalg.norm(d) + 1e-9) * TOKEN["r"]
            self.token_est = est

    def _pixel_to_table(self, u, v, pose):
        H, W = CAM["h"], CAM["w"]
        f = (H / 2) / math.tan(math.radians(CAM["fovy"]) / 2)
        dx, dy, dz = 1.0, -(u - W / 2) / f, -(v - H / 2) / f            # camera ray, body axes before pitch
        p = math.radians(CAM["pitch_deg"])
        rx, rz = dx * math.cos(p) + dz * math.sin(p), -dx * math.sin(p) + dz * math.cos(p)
        if rz >= -1e-3:
            return None
        cam_h = CAM["pos"][2]
        s = cam_h / -rz
        bx, by = CAM["pos"][0] + rx * s, dy * s
        if math.hypot(bx, by) > 1.4:
            return None
        x, y, th = pose
        return np.array([x + math.cos(th) * bx - math.sin(th) * by, y + math.sin(th) * bx + math.cos(th) * by])

    def _mark_inspected(self, pose):
        x, y, th = pose
        cx, cy = x + math.cos(th) * CAM["pos"][0], y + math.sin(th) * CAM["pos"][0]
        p = math.radians(CAM["pitch_deg"])
        half_v = math.radians(CAM["fovy"]) / 2
        near = CAM["pos"][2] / math.tan(p + half_v)
        far = min(0.85, CAM["pos"][2] / max(math.tan(p - half_v), 0.05))
        half_h = math.atan(math.tan(half_v) * CAM["w"] / CAM["h"])
        i0, i1 = self.cell(cx - far, cy - far), self.cell(cx + far, cy + far)
        sub_x = self.gx[i0[0]:i1[0] + 1, i0[1]:i1[1] + 1]
        sub_y = self.gy[i0[0]:i1[0] + 1, i0[1]:i1[1] + 1]
        dxs, dys = sub_x - cx, sub_y - cy
        r = np.hypot(dxs, dys)
        ang = np.abs(np.vectorize(wrap)(np.arctan2(dys, dxs) - th)) if r.size < 50 else np.abs(((np.arctan2(dys, dxs) - th + np.pi) % (2 * np.pi)) - np.pi)
        vis = (r > near) & (r < far) & (ang < half_h * 0.95)
        if not vis.any():
            return
        blk = self.blocked()
        ii, jj = np.nonzero(vis)
        for k in range(len(ii)):                                         # occlusion check along each ray
            tx, ty = sub_x[ii[k], jj[k]], sub_y[ii[k], jj[k]]
            n = int(r[ii[k], jj[k]] / RES)
            ok = True
            for s in range(2, n):
                a, b = self.cell(cx + dxs[ii[k], jj[k]] * s / n, cy + dys[ii[k], jj[k]] * s / n)
                if blk[a, b]:
                    ok = False
                    break
            if ok:
                self.inspected[i0[0] + ii[k], i0[1] + jj[k]] = True

    def on_audio(self, m, t):
        heard = self.scream.detect(m["pcm"])
        if heard and self.log["scream_heard"] is None:
            self.log["scream_heard"] = t
        if heard and m["doas"]:
            th = self.to_table(m["est"])[2]
            self.votes *= 0.85
            for b in m["doas"]:
                for cand in (wrap(th + b), wrap(th + math.pi - b)):           # front / back mirror
                    k = int((cand + math.pi) / (2 * math.pi) * 36) % 36
                    self.votes[k] += 1.0
                    self.votes[(k + 1) % 36] += 0.5
                    self.votes[(k - 1) % 36] += 0.5
            best = int(np.argmax(self.votes))
            angs = -math.pi + (np.arange(36) + 0.5) * 2 * math.pi / 36
            idx = [(best + d) % 36 for d in (-2, -1, 0, 1, 2)]
            self.bearing = math.atan2(np.sum(self.votes[idx] * np.sin(angs[idx])), np.sum(self.votes[idx] * np.cos(angs[idx])))

    # ------------------------------------------------------------ planning
    def _cost_grid(self, known_free_only=False):
        blk = self.blocked()
        r = int(INFLATE / RES)
        inflated = np.zeros_like(blk)
        ii, jj = np.nonzero(blk)
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    a, b = np.clip(ii + di, 0, self.nx - 1), np.clip(jj + dj, 0, self.ny - 1)
                    inflated[a, b] = True
        bad = inflated | self.edge_bad
        if known_free_only:
            bad = bad | ~self.seen_free
        return bad

    def _astar(self, start, goal, bad, step=2):
        sx, sy = start
        gx, gy = goal
        sx, sy, gx, gy = sx // step, sy // step, gx // step, gy // step
        W, H = (self.nx + step - 1) // step, (self.ny + step - 1) // step
        badc = bad[::step, ::step]
        if badc[gx, gy]:
            return None
        badc = badc.copy()
        # the rover may already be inside an obstacle's safety margin (after a push or a
        # turn): let it leave through a small disk around its own position
        r = 3
        badc[max(0, sx - r):sx + r + 1, max(0, sy - r):sy + r + 1] &= ~((np.add.outer(
            (np.arange(max(0, sx - r), min(W, sx + r + 1)) - sx) ** 2,
            (np.arange(max(0, sy - r), min(H, sy + r + 1)) - sy) ** 2)) <= r * r)
        openq = [(0.0, (sx, sy))]
        g = {(sx, sy): 0.0}
        parent = {}
        nbr = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        while openq:
            f, cur = heapq.heappop(openq)
            if cur == (gx, gy):
                path = [cur]
                while cur in parent:
                    cur = parent[cur]
                    path.append(cur)
                return [((c[0] * step + 0.5 * step) * RES, (c[1] * step + 0.5 * step) * RES) for c in reversed(path)]
            for dx, dy, c in nbr:
                nb = (cur[0] + dx, cur[1] + dy)
                if not (0 <= nb[0] < W and 0 <= nb[1] < H) or badc[nb]:
                    continue
                ng = g[cur] + c
                if ng < g.get(nb, 1e9):
                    g[nb] = ng
                    parent[nb] = cur
                    heapq.heappush(openq, (ng + math.hypot(gx - nb[0], gy - nb[1]), nb))
        return None

    def _dijkstra_costs(self, bad, step=2):
        W, H = (self.nx + step - 1) // step, (self.ny + step - 1) // step
        badc = bad[::step, ::step].copy()
        sx, sy = self.cell(*self.pose[:2])
        sx, sy = sx // step, sy // step
        badc[sx, sy] = False
        dist = np.full((W, H), np.inf)
        dist[sx, sy] = 0
        q = [(0.0, sx, sy)]
        while q:
            d, i, j = heapq.heappop(q)
            if d > dist[i, j]:
                continue
            for di, dj, c in ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)):
                a, b = i + di, j + dj
                if 0 <= a < W and 0 <= b < H and not badc[a, b] and d + c < dist[a, b]:
                    dist[a, b] = d + c
                    heapq.heappush(q, (d + c, a, b))
        return dist * step * RES

    def _choose_viewpoint(self):
        bad = self._cost_grid()
        dist = self._dijkstra_costs(bad)
        unseen = self.on_table & ~self.inspected & ~self.blocked()
        if unseen.sum() < 25:
            return None
        ux, uy = self.gx[unseen], self.gy[unseen]
        best, best_score = None, -1e9
        step = 2
        cand = np.argwhere(np.isfinite(dist))
        if len(cand) > 400:
            cand = cand[self.rng.choice(len(cand), 400, replace=False)]
        for i, j in cand:
            px, py = (i * step + 1) * RES, (j * step + 1) * RES
            if any(math.hypot(px - tx, py - ty) < 0.15 for tx, ty, _ in self.tabu[-12:]):
                continue                                   # already looked from here
            dxs, dys = ux - px, uy - py
            r = np.hypot(dxs, dys)
            near = (r > 0.2) & (r < 0.8)
            if not near.any():
                continue
            heading = math.atan2(dys[near].mean(), dxs[near].mean())
            ang = np.abs(((np.arctan2(dys, dxs) - heading + np.pi) % (2 * np.pi)) - np.pi)
            gain = np.count_nonzero(near & (ang < 0.45))
            score = gain / (1.0 + 2.5 * dist[i, j])
            if self.bearing is not None:
                to = math.atan2(py - self.pose[1], px - self.pose[0])
                score *= 1.0 + 1.5 * max(0.0, math.cos(wrap(to - self.bearing))) if dist[i, j] > 0.15 else 1.0
            if score > best_score:
                best, best_score = (px, py, heading), score
        return best

    def _plan_to(self, x, y, heading=None, known_free_only=False):
        """A* with the preferred clearance first; narrower clearances only if that is the
        only way through (gaps between books can be ~22 cm)."""
        global INFLATE
        keep = INFLATE
        p = None
        try:
            for infl in (keep, 0.11, 0.09):
                INFLATE = infl
                for kfo in ((True, False) if known_free_only else (False,)):
                    p = self._astar(self.cell(*self.pose[:2]), self.cell(x, y), self._cost_grid(kfo))
                    if p is not None:
                        break
                if p is not None:
                    break
        finally:
            INFLATE = keep
        self.path = p or []
        self.goal = (x, y)
        self.goal_heading = heading
        return p is not None

    def _follow(self):
        x, y, th = self.pose
        while len(self.path) > 1 and math.hypot(self.path[0][0] - x, self.path[0][1] - y) < 0.06:
            self.path.pop(0)
        if not self.path:
            return None
        look = self.path[0]
        for p in self.path:
            if math.hypot(p[0] - x, p[1] - y) > 0.12:
                look = p
                break
        else:
            look = self.path[-1]
        e = wrap(math.atan2(look[1] - y, look[0] - x) - th)
        if math.hypot(self.path[-1][0] - x, self.path[-1][1] - y) < 0.05:
            self.path = []
            return None
        w = max(-W_MAX, min(W_MAX, 3.0 * e))
        v = 0.0 if abs(e) > 0.7 else V_MAX * math.cos(e)
        return v, w

    def _turn_to(self, heading):
        e = wrap(heading - self.pose[2])
        if abs(e) < 0.06:
            return None
        w = max(-W_MAX, min(W_MAX, 3.0 * e))
        if abs(w) < 0.5:
            w = math.copysign(0.5, e)
        return 0.0, w

    # ------------------------------------------------------------ main step (10 Hz)
    def step(self, t):
        """Returns (cmd dict or None, listen request or None)."""
        if self.telem is None:
            return None, None
        x, y, th = self.pose
        listen = None
        v = w = 0.0

        if t < self.listening_until:                                   # stay still while the robot listens
            return dict(type="cmd", v=0.0, w=0.0, door=self.door), None

        if self.state in ("EXPLORE",):
            if self.token_est is not None:
                self.state, self.t_state = "PREGRASP", t
                self.path = []
            elif self.scream_mode and t - self.last_listen_t > 4.0:
                self.last_listen_t = t
                self.listening_until = t + 1.0
                return dict(type="cmd", v=0.0, w=0.0, door=self.door), dict(type="listen", dur=0.8)
            else:
                if not self.path and (self.goal_heading is None or self._turn_to(self.goal_heading) is None) \
                        or t - self.replan_t > 6.0:
                    self.replan_t = t
                    vp = self._choose_viewpoint()
                    if vp is None:
                        self.inspected[:] = False                       # everything seen: sweep again
                        vp = self._choose_viewpoint()
                    if vp:
                        self._plan_to(vp[0], vp[1], vp[2])
                cmd = self._follow()
                if cmd is None and self.goal_heading is not None:
                    cmd = self._turn_to(self.goal_heading)
                    if cmd is None:
                        if self.goal:
                            self.tabu.append((self.goal[0], self.goal[1], self.goal_heading))
                        self.goal_heading = None
                if cmd:
                    v, w = cmd

        elif self.state == "PREGRASP":
            tx, ty = self.token_est
            if not self.path or t - self.replan_t > 3.0:
                self.replan_t = t
                ok = False
                d0 = math.atan2(y - ty, x - tx)
                # stand-off points around the token: nearest side first, then wider
                # angles, then shorter/longer stand-offs, then with relaxed clearance
                for rad in (0.30, 0.24, 0.38):
                    for off in (0.0, 0.5, -0.5, 1.0, -1.0, 1.6, -1.6, 2.2, -2.2, math.pi):
                        a = d0 + off
                        px, py = tx + rad * math.cos(a), ty + rad * math.sin(a)
                        ok = self._plan_to(px, py, math.atan2(ty - py, tx - px))
                        if ok:
                            break
                    if ok:
                        break
                if not ok:
                    self.pregrasp_fail = getattr(self, "pregrasp_fail", 0) + 1
                    if self.pregrasp_fail >= 3:                  # give up on this sighting
                        self.pregrasp_fail = 0
                        self.token_est = None
                        self.token_obs = []
                        self.state = "EXPLORE"
                    else:
                        self.state, self.t_state = "UNSTICK", t
                    return dict(type="cmd", v=0.0, w=0.0, door=self.door), None
            cmd = self._follow()
            if cmd is None:
                cmd = self._turn_to(math.atan2(ty - y, tx - x))
                if cmd is None:
                    self.state, self.t_state = "APPROACH", t
                    self.attempts += 1
                    self.log["approach_attempts"] += 1
                    self.approach_start = np.array([x, y])
            if cmd:
                v, w = cmd
            self.door = 1.0

        elif self.state == "APPROACH":
            self.door = 1.0
            tx, ty = self.token_est
            if self.telem["capture"]:
                self.state, self.t_state = "CLOSE", t
            else:
                e = wrap(math.atan2(ty - y, tx - x) - th)
                dist = math.hypot(tx - x, ty - y)
                travelled = np.linalg.norm(np.array([x, y]) - self.approach_start)
                if travelled > 0.45 or (dist < 0.03 and t - self.t_state > 2) or t - self.t_state > 12:
                    self.state, self.t_state = "BACKOFF", t                # missed: back off and retry
                else:
                    v, w = 0.07, max(-0.8, min(0.8, 2.5 * e)) if dist > 0.09 else 0.0

        elif self.state == "BACKOFF":
            v = -0.06
            if t - self.t_state > 2.0:
                self.token_est = None
                self.token_obs = []
                self.state = "EXPLORE" if self.attempts < 4 else "EXPLORE"
                self.path = []

        elif self.state == "CLOSE":
            self.door = 0.0
            if t - self.t_state > 1.0:
                if self.telem["capture"]:
                    self.state, self.t_state = "RETURN", t
                    self.log["captured"] = t
                    self._plan_to(self.x0, self.y0, known_free_only=True)
                    self.home_path_len = sum(math.dist(self.path[i], self.path[i + 1]) for i in range(len(self.path) - 1)) if self.path else None
                else:
                    self.state, self.t_state = "BACKOFF", t

        elif self.state == "RETURN":
            self.door = 0.0
            if math.hypot(x - self.x0, y - self.y0) < 0.07:
                self.state = "DONE"
                self.log["home"] = t
            else:
                if not self.path or t - self.replan_t > 4.0:
                    self.replan_t = t
                    if not self._plan_to(self.x0, self.y0, known_free_only=True):
                        self.state, self.t_state = "UNSTICK", t
                cmd = self._follow()
                if cmd:
                    v, w = cmd

        elif self.state == "UNSTICK":                    # no path from here: reverse a little, turn, retry
            v, w = (-0.05, 0.0) if t - self.t_state < 1.5 else (0.0, 1.0)
            if t - self.t_state > 3.0:
                self.state = "RETURN" if self.log["captured"] else ("PREGRASP" if self.token_est is not None else "EXPLORE")
                self.path = []

        if self.telem.get("stalled"):
            self.path = []
        return dict(type="cmd", v=float(v), w=float(w), door=self.door), listen
