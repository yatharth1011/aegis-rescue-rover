"""
MuJoCo world for the A.E.G.I.S. table-top demo.

Table (1.8 x 1.2 m, top at 0.75 m) in a lab room. Random "book" obstacles,
the victim token (orange cylinder, 40 mm, white band), look-alike decoys
(orange cube, red ball, orange flat disc) so the detector must learn shape.

The rover matches the Fusion Mk2 model: covered intake bay at the front,
right funnel fin, hinged left door driven by a position servo, camera on a
mast tilted 30 deg down, front/left/right ToF, three IR cliff sensors and an
IR capture sensor in the bay. It drives on planar joints (x, y, yaw) with
velocity actuators whose force is limited, so it stalls realistically
against obstacles and pushes free objects. The token is a free body: it is
pushed, funnelled, captured and dragged by real contact physics.
"""
import math

import mujoco
import numpy as np

TABLE = dict(lx=1.8, ly=1.2, top=0.75)
ROVER_Z = TABLE["top"]

# Rover geometry (m), body frame: +x forward, origin at ground under the chassis centre.
BAY = dict(x0=0.040, x1=0.128, hw=0.032)
CAM = dict(pos=(0.072, 0.0, 0.142), pitch_deg=30.0, fovy=42.0, w=320, h=240)
TOF = {"front": ((0.135, 0.0, 0.066), 0.0), "left": ((0.040, 0.057, 0.099), math.pi / 2),
       "right": ((0.040, -0.057, 0.099), -math.pi / 2)}
CLIFF = {"FL": (0.076, 0.052), "FR": (0.140, -0.050), "R": (-0.098, 0.0)}
TOKEN = dict(r=0.020, h=0.040)


def _box(name, pos, size, rgba, extra=""):
    return ('<geom name="%s" type="box" pos="%.4f %.4f %.4f" size="%.4f %.4f %.4f" rgba="%s" %s/>'
            % (name, *pos, *size, " ".join("%.3f" % c for c in rgba), extra))


def make_scene(rng, n_obstacles=(4, 7), n_decoys=2, token_hidden=False):
    """Random arena. Returns (xml, info). info has obstacle boxes, token and start poses (world frame)."""
    lx, ly, top = TABLE["lx"], TABLE["ly"], TABLE["top"]
    start = np.array([0.20, 0.20, rng.uniform(-0.3, 0.6)])       # near one corner, roughly facing in
    boxes = []
    n = rng.integers(n_obstacles[0], n_obstacles[1] + 1)
    tries = 0
    while len(boxes) < n and tries < 500:
        tries += 1
        sx, sy = rng.uniform(0.06, 0.16), rng.uniform(0.04, 0.12)
        cx, cy = rng.uniform(0.15, lx - 0.15), rng.uniform(0.15, ly - 0.15)
        if np.hypot(cx - start[0], cy - start[1]) < 0.40:
            continue
        if any(abs(cx - b[0]) < sx + b[2] + 0.22 and abs(cy - b[1]) < sy + b[3] + 0.22 for b in boxes):
            continue                                              # keep corridors the rover fits through
        boxes.append((cx, cy, sx, sy, rng.uniform(0.08, 0.20), rng.uniform(0, math.pi)))

    def clear(x, y, margin):
        if not (0.15 < x < lx - 0.15 and 0.15 < y < ly - 0.15):
            return False
        return all(np.hypot(x - b[0], y - b[1]) > max(b[2], b[3]) + margin for b in boxes)

    for _ in range(2000):
        tx, ty = rng.uniform(0.2, lx - 0.2), rng.uniform(0.2, ly - 0.2)
        if np.hypot(tx - start[0], ty - start[1]) < 0.8 or not clear(tx, ty, 0.12):
            continue
        if token_hidden:
            # hidden: no straight line of sight from the start point
            if not any(_seg_hits_box(start[:2], (tx, ty), b) for b in boxes):
                continue
        break
    if not _reachable(boxes, start, (tx, ty)):
        return make_scene(rng, n_obstacles, n_decoys, token_hidden)   # redraw: the arena must be solvable

    decoys = []
    for _ in range(n_decoys):
        for _ in range(500):
            x, y = rng.uniform(0.2, lx - 0.2), rng.uniform(0.2, ly - 0.2)
            if clear(x, y, 0.12) and np.hypot(x - tx, y - ty) > 0.25 and np.hypot(x - start[0], y - start[1]) > 0.4:
                decoys.append((x, y, int(rng.integers(0, 3))))
                break

    tbl_rgba = (*(0.55 + 0.25 * rng.random(3)), 1.0)
    light_dir = rng.uniform([-0.6, -0.6, -1.0], [0.6, 0.6, -0.6])
    amb = rng.uniform(0.25, 0.45)

    g = []
    g.append(_box("table", (lx / 2, ly / 2, top - 0.015), (lx / 2, ly / 2, 0.015), tbl_rgba,
                  'contype="2" conaffinity="2"'))   # free objects rest on it; the rover (planar) does not
    for i, (x, y) in enumerate(((0.05, 0.05), (lx - 0.05, 0.05), (0.05, ly - 0.05), (lx - 0.05, ly - 0.05))):
        g.append(_box("leg%d" % i, (x, y, (top - 0.03) / 2), (0.025, 0.025, (top - 0.03) / 2), (0.3, 0.3, 0.32, 1)))
    for i, (cx, cy, sx, sy, h, yaw) in enumerate(boxes):
        col = rng.choice([(0.15, 0.3, 0.6, 1), (0.6, 0.15, 0.15, 1), (0.2, 0.45, 0.25, 1), (0.35, 0.35, 0.38, 1),
                          (0.75, 0.7, 0.55, 1), (0.1, 0.1, 0.12, 1)])
        g.append('<body name="obs%d" pos="%.4f %.4f %.4f" euler="0 0 %.4f"><geom name="obsg%d" type="box" '
                 'size="%.4f %.4f %.4f" rgba="%s" friction="0.8 0.01 0.001"/></body>'
                 % (i, cx, cy, top + h / 2, yaw, i, sx, sy, h / 2, " ".join("%.3f" % c for c in col)))
    for i, (x, y, kind) in enumerate(decoys):
        if kind == 0:
            gg = '<geom name="decoy%d" type="box" size="0.02 0.02 0.02" rgba="0.95 0.45 0.08 1" contype="3" conaffinity="3"/>' % i
            z = 0.02
        elif kind == 1:
            gg = '<geom name="decoy%d" type="sphere" size="0.022" rgba="0.85 0.12 0.10 1" contype="3" conaffinity="3"/>' % i
            z = 0.022
        else:
            gg = '<geom name="decoy%d" type="cylinder" size="0.03 0.006" rgba="0.95 0.45 0.08 1" contype="3" conaffinity="3"/>' % i
            z = 0.006
        g.append('<body name="decoyb%d" pos="%.4f %.4f %.4f"><freejoint/>%s<inertial pos="0 0 0" mass="0.03" '
                 'diaginertia="1e-5 1e-5 1e-5"/></body>' % (i, x, y, top + z + 0.001, gg))

    # victim token: free body, orange cylinder with a white band
    g.append('<body name="token" pos="%.4f %.4f %.4f"><freejoint name="token_free"/>'
             '<geom name="token_g" type="cylinder" size="%.3f %.3f" rgba="0.95 0.45 0.08 1" friction="0.5 0.005 0.0001" contype="3" conaffinity="3"/>'
             '<geom name="token_band" type="cylinder" size="%.4f 0.005" rgba="0.97 0.97 0.97 1" contype="0" conaffinity="0"/>'
             '<inertial pos="0 0 0" mass="0.03" diaginertia="1e-5 1e-5 1e-5"/></body>'
             % (tx, ty, top + TOKEN["h"] / 2 + 0.001, TOKEN["r"], TOKEN["h"] / 2, TOKEN["r"] + 0.0005))

    rover = _rover_xml(start)
    xml = """
<mujoco model="aegis_table">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81" cone="elliptic" impratio="5"/>
  <visual><global offwidth="640" offheight="480"/><headlight ambient="%.3f %.3f %.3f" diffuse="0.5 0.5 0.5"/>
    <quality shadowsize="2048"/></visual>
  <asset>
    <texture name="floor" type="2d" builtin="checker" rgb1="0.42 0.42 0.44" rgb2="0.36 0.36 0.38" width="256" height="256"/>
    <material name="floor" texture="floor" texrepeat="12 12"/>
  </asset>
  <worldbody>
    <light directional="true" dir="%.3f %.3f %.3f" diffuse="0.7 0.7 0.68" castshadow="true"/>
    <geom name="floor" type="plane" size="6 6 0.1" material="floor" contype="2" conaffinity="2"/>
    <geom name="wall_n" type="box" pos="0.9 3.2 1.5" size="4 0.05 1.5" rgba="0.82 0.82 0.8 1" contype="0" conaffinity="0"/>
    <geom name="wall_e" type="box" pos="3.6 0.6 1.5" size="0.05 4 1.5" rgba="0.78 0.8 0.82 1" contype="0" conaffinity="0"/>
    %s
    %s
  </worldbody>
  <actuator>
    <velocity name="vx" joint="rx" kv="40" forcerange="-6 6"/>
    <velocity name="vy" joint="ry" kv="40" forcerange="-6 6"/>
    <velocity name="wz" joint="ryaw" kv="0.8" forcerange="-0.6 0.6"/>
    <position name="door" joint="door" kp="0.6" forcerange="-0.25 0.25"/>
  </actuator>
</mujoco>""" % (amb, amb, amb, *light_dir, "\n    ".join(g), rover)
    info = dict(boxes=boxes, token=(tx, ty), start=start, decoys=decoys)
    return xml, info


def _reachable(boxes, start, token, r=0.13, cell=0.02):
    """Flood fill for a rover of radius r from the start to any point 0.2-0.35 m from the token."""
    lx, ly = TABLE["lx"], TABLE["ly"]
    nx, ny = int(lx / cell), int(ly / cell)
    gx, gy = np.meshgrid((np.arange(nx) + 0.5) * cell, (np.arange(ny) + 0.5) * cell, indexing="ij")
    free = (gx > 0.15) & (gx < lx - 0.15) & (gy > 0.15) & (gy < ly - 0.15)
    for cx, cy, sx, sy, _, yaw in boxes:
        c, s_ = math.cos(-yaw), math.sin(-yaw)
        qx, qy = c * (gx - cx) - s_ * (gy - cy), s_ * (gx - cx) + c * (gy - cy)
        dx, dy = np.maximum(np.abs(qx) - sx, 0), np.maximum(np.abs(qy) - sy, 0)
        free &= np.hypot(dx, dy) > r
    goal = free & (np.hypot(gx - token[0], gy - token[1]) > 0.2) & (np.hypot(gx - token[0], gy - token[1]) < 0.35)
    si, sj = int(start[0] / cell), int(start[1] / cell)
    if not free[si, sj]:
        return False
    seen = np.zeros_like(free)
    seen[si, sj] = True
    stack = [(si, sj)]
    while stack:
        i, j = stack.pop()
        if goal[i, j]:
            return True
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < nx and 0 <= b < ny and free[a, b] and not seen[a, b]:
                seen[a, b] = True
                stack.append((a, b))
    return False


def _seg_hits_box(a, b, box):
    cx, cy, sx, sy, _, yaw = box
    c, s = math.cos(-yaw), math.sin(-yaw)
    for t in np.linspace(0, 1, 60):
        px, py = a[0] + (b[0] - a[0]) * t - cx, a[1] + (b[1] - a[1]) * t - cy
        qx, qy = c * px - s * py, s * px + c * py
        if abs(qx) < sx + 0.01 and abs(qy) < sy + 0.01:
            return True
    return False


def _rover_xml(start):
    x, y, yaw = start
    # image right = body -Y; image up = (sin p, 0, cos p): the view axis is forward, p below the horizon
    p = math.radians(CAM["pitch_deg"])
    cam_axes = "0 -1 0 %.6f 0 %.6f" % (math.sin(p), math.cos(p))
    rov = """
    <body name="rover" pos="%.4f %.4f %.4f">   <!-- unrotated: slide joints move along world x/y; heading is the yaw joint -->
      <joint name="rx" type="slide" axis="1 0 0" damping="2"/>
      <joint name="ry" type="slide" axis="0 1 0" damping="2"/>
      <joint name="ryaw" type="hinge" axis="0 0 1" damping="0.05"/>
      <inertial pos="-0.02 0 0.05" mass="0.97" diaginertia="0.004 0.006 0.008"/>
      <!-- decks and electronics (above the token, no collision with it below 0.05 m) -->
      <geom name="deck_lo" type="box" pos="0.0125 0 0.0515" size="0.1275 0.060 0.0015" rgba="0.14 0.15 0.18 0.95"/>
      <geom name="deck_up" type="box" pos="-0.010 0 0.0845" size="0.105 0.060 0.0015" rgba="0.14 0.15 0.18 0.95"/>
      <geom name="battery" type="box" pos="-0.075 0 0.0625" size="0.0325 0.028 0.0095" rgba="0.16 0.35 0.67 1"/>
      <geom name="mast" type="box" pos="0.054 0 0.115" size="0.003 0.017 0.028" rgba="0.92 0.42 0.11 1"/>
      <geom name="camboard" type="box" pos="0.068 0 0.136" size="0.007 0.020 0.013" rgba="0.08 0.08 0.09 1"/>
      <geom name="servo" type="box" pos="0.1192 0.045 0.0618" size="0.0204 0.0099 0.0181" rgba="0.09 0.09 0.1 1"/>
      <!-- running gear -->
      <geom name="w_fl" type="cylinder" pos="0.020 0.061 0.022" euler="1.5708 0 0" size="0.022 0.009" rgba="0.1 0.1 0.11 1"/>
      <geom name="w_fr" type="cylinder" pos="0.020 -0.061 0.022" euler="1.5708 0 0" size="0.022 0.009" rgba="0.1 0.1 0.11 1"/>
      <geom name="w_rl" type="cylinder" pos="-0.080 0.061 0.022" euler="1.5708 0 0" size="0.022 0.009" rgba="0.1 0.1 0.11 1"/>
      <geom name="w_rr" type="cylinder" pos="-0.080 -0.061 0.022" euler="1.5708 0 0" size="0.022 0.009" rgba="0.1 0.1 0.11 1"/>
      <geom name="hangers" type="box" pos="-0.030 0 0.033" size="0.059 0.047 0.017" rgba="0.22 0.24 0.27 1"/>
      <!-- intake bay: side walls, back wall, right fin (collide with the token) -->
      <geom name="bay_l" type="box" pos="0.084 0.0335 0.027" size="0.044 0.0015 0.023" rgba="0.92 0.42 0.11 1"/>
      <geom name="bay_r" type="box" pos="0.084 -0.0335 0.027" size="0.044 0.0015 0.023" rgba="0.92 0.42 0.11 1"/>
      <geom name="bay_b" type="box" pos="0.0385 0 0.027" size="0.0015 0.035 0.023" rgba="0.92 0.42 0.11 1"/>
      <geom name="fin_r" type="box" pos="0.139 -0.0400 0.0255" euler="0 0 -0.66" size="0.0110 0.0015 0.0215" rgba="0.92 0.42 0.11 1"/>
      <body name="door" pos="0.1295 0.045 0.004">
        <joint name="door" type="hinge" axis="0 0 1" range="0 2.0944" damping="0.02"/>
        <geom name="door_leaf" type="box" pos="0 -0.040 0.019" size="0.0015 0.040 0.019" rgba="0.92 0.42 0.11 1"/>
        <inertial pos="0 -0.04 0.02" mass="0.01" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
      <camera name="cam" pos="%.4f %.4f %.4f" xyaxes="%s" fovy="%.1f"/>
      <site name="cam_site" pos="%.4f %.4f %.4f"/>
    </body>""" % (x, y, ROVER_Z, *CAM["pos"], cam_axes, CAM["fovy"], *CAM["pos"])
    return rov


def _quat_cam(pitch_deg):
    # MuJoCo cameras look along their local -Z with +Y up. Rotate so the view
    # points along body +X, tilted pitch_deg below the horizon.
    p = math.radians(pitch_deg)
    # yaw -90 about z then about local x by (90 - pitch)
    q1 = np.array([math.cos(-math.pi / 4), 0, 0, math.sin(-math.pi / 4)])           # z by -90
    a = math.pi / 2 - p
    q2 = np.array([math.cos(a / 2), math.sin(a / 2), 0, 0])                           # x by (90 - pitch)
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    q = np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                  w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
    return q / np.linalg.norm(q)


class World:
    def __init__(self, seed, token_hidden=False, n_decoys=2):
        self.rng = np.random.default_rng(seed)
        self.xml, self.info = make_scene(self.rng, token_hidden=token_hidden, n_decoys=n_decoys)
        self.m = mujoco.MjModel.from_xml_string(self.xml)
        self.d = mujoco.MjData(self.m)
        self.renderer = None
        self.seg = None
        self.jx, self.jy, self.jyaw = (self.m.joint(n).qposadr[0] for n in ("rx", "ry", "ryaw"))
        self.dx, self.dy, self.dyaw = (self.m.joint(n).dofadr[0] for n in ("rx", "ry", "ryaw"))
        self.token_adr = self.m.joint("token_free").qposadr[0]
        self.rover_bid = self.m.body("rover").id
        self.token_gid = self.m.geom("token_g").id
        self.obstacle_gids = set(i for i in range(self.m.ngeom) if self.m.geom(i).name.startswith("obsg"))
        self.decoy_gids = set(i for i in range(self.m.ngeom) if self.m.geom(i).name.startswith("decoy"))
        self.d.qpos[self.jyaw] = self.info["start"][2]           # initial heading
        mujoco.mj_forward(self.m, self.d)
        self.rover_gids = set(i for i in range(self.m.ngeom) if self.m.geom_bodyid[i] in
                              (self.rover_bid, self.m.body("door").id))

    # ---- state -------------------------------------------------------------------
    def pose(self):
        x0, y0, th0 = self.info["start"]
        q = self.d.qpos
        # slide joints are along world axes but measured from the body's initial position
        return np.array([x0 + q[self.jx], y0 + q[self.jy], q[self.jyaw]])

    def body_vel(self):
        x, y, th = self.pose()
        vx, vy, w = self.d.qvel[self.dx], self.d.qvel[self.dy], self.d.qvel[self.dyaw]
        return vx * math.cos(th) + vy * math.sin(th), -vx * math.sin(th) + vy * math.cos(th), w

    def token_pos(self):
        return self.d.qpos[self.token_adr:self.token_adr + 3].copy()

    def set_body_velocity(self, v, w, lateral_slip=0.0):
        th = self.pose()[2]
        self.d.ctrl[0] = v * math.cos(th) - lateral_slip * math.sin(th)
        self.d.ctrl[1] = v * math.sin(th) + lateral_slip * math.cos(th)
        self.d.ctrl[2] = w

    def set_door(self, angle_rad):
        self.d.ctrl[3] = angle_rad

    def step(self, n=1):
        mujoco.mj_step(self.m, self.d, n)

    # ---- sensors -------------------------------------------------------------------
    def tof(self, which, rng):
        (px, py, pz), yaw = TOF[which]
        x, y, th = self.pose()
        best = 2.0
        for k in range(-2, 3):                                   # ~25 deg cone, 5 rays
            a = th + yaw + k * 0.109
            o = np.array([x + math.cos(th) * px - math.sin(th) * py, y + math.sin(th) * px + math.cos(th) * py,
                          ROVER_Z + pz])
            dirv = np.array([math.cos(a), math.sin(a), 0.0])
            gid = np.array([-1], np.int32)
            dist = mujoco.mj_ray(self.m, self.d, o, dirv, None, 1, self.rover_bid, gid)
            if 0 <= dist < best:
                best = dist
        if best >= 2.0:
            return 2.0
        return max(0.0, best * (1 + 0.02 * rng.standard_normal()) + 0.003 * rng.standard_normal())

    def cliff(self):
        x, y, th = self.pose()
        out = {}
        for k, (px, py) in CLIFF.items():
            wx, wy = x + math.cos(th) * px - math.sin(th) * py, y + math.sin(th) * px + math.cos(th) * py
            out[k] = not (0.005 < wx < TABLE["lx"] - 0.005 and 0.005 < wy < TABLE["ly"] - 0.005)
        return out

    def fell(self):
        """True if the centre of mass or two wheels have left the table top."""
        x, y, th = self.pose()
        off = 0
        for px, py in ((0.02, 0.061), (0.02, -0.061), (-0.08, 0.061), (-0.08, -0.061)):
            wx, wy = x + math.cos(th) * px - math.sin(th) * py, y + math.sin(th) * px + math.cos(th) * py
            off += not (0 < wx < TABLE["lx"] and 0 < wy < TABLE["ly"])
        cx, cy = x - 0.02 * math.cos(th), y - 0.02 * math.sin(th)
        return off >= 2 or not (0 < cx < TABLE["lx"] and 0 < cy < TABLE["ly"])

    def token_in_bay(self):
        x, y, th = self.pose()
        t = self.token_pos()
        dx, dy = t[0] - x, t[1] - y
        bx, by = dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)
        return BAY["x0"] < bx < BAY["x1"] and abs(by) < BAY["hw"], (bx, by)

    def capture_ir(self):
        """IR proximity sensor on the bay back wall: sees an object 0..60 mm in front of it."""
        inside, (bx, by) = self.token_in_bay()
        return inside and bx < BAY["x0"] + 0.060 and abs(by) < 0.02

    def contacts_with_decoys(self):
        n = 0
        for i in range(self.d.ncon):
            a, b = self.d.contact[i].geom1, self.d.contact[i].geom2
            if (a in self.rover_gids and b in self.decoy_gids) or (b in self.rover_gids and a in self.decoy_gids):
                n += 1
        return n

    def contacts_with_obstacles(self):
        n = 0
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            a, b = c.geom1, c.geom2
            if (a in self.rover_gids and b in self.obstacle_gids) or (b in self.rover_gids and a in self.obstacle_gids):
                n += 1
        return n

    # ---- camera ----------------------------------------------------------------------
    def render(self, segmentation=False):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.m, CAM["h"], CAM["w"])
        self.renderer.update_scene(self.d, camera="cam")
        if segmentation:
            self.renderer.enable_segmentation_rendering()
            seg = self.renderer.render().copy()
            self.renderer.disable_segmentation_rendering()
            return seg
        return self.renderer.render().copy()

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
