"""
3D terrain capability test for the Mk2 chassis (full rigid-body physics).

The rover is a free body with four hinge-jointed 44 mm rubber wheels driven by
torque-limited velocity servos (N20 150:1: stall torque as listed, 7 kg-cm,
and derated to 40 %, 2.8 kg-cm). Collision geometry follows the Fusion model:
intake bay walls and door 4 mm above the ground, motor hangers 15 mm, decks 50 mm.

Tests (each: drive straight at 0.15 m/s for 15 s, success = 0.6 m of progress):
  step   a single kerb of height h across the path
  slope  a ramp of angle a
  rough  random heightfield of amplitude A (rubble / gravel-like)

    ~/aegis_venv/bin/python terrain_test.py
"""
import json
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
KGCM = 9.81 / 100.0


def rover_xml(stall_kgcm):
    tau = stall_kgcm * KGCM
    wheels = ""
    for name, x, y in (("fl", 0.020, 0.061), ("fr", 0.020, -0.061), ("rl", -0.080, 0.061), ("rr", -0.080, -0.061)):
        wheels += """
      <body name="w_%s" pos="%.3f %.3f 0.022">
        <joint name="j_%s" type="hinge" axis="0 1 0" damping="0.0005" armature="0.0002"/>   <!-- N20 rotor x 150^2 -->
        <geom type="cylinder" size="0.022 0.009" euler="1.5708 0 0" friction="0.9 0.01 0.001" rgba="0.1 0.1 0.1 1" mass="0.02"/>
      </body>""" % (name, x, y, name)
    acts = "".join('<velocity joint="j_%s" kv="0.05" forcerange="-%.4f %.4f"/>' % (n, tau, tau) for n in ("fl", "fr", "rl", "rr"))
    return wheels, acts


def scene(kind, value, stall_kgcm, seed=0):
    rng = np.random.default_rng(seed)
    wheels, acts = rover_xml(stall_kgcm)
    terrain, asset = "", ""
    if kind == "flat":
        terrain = ""
    elif kind == "step":
        terrain = '<geom type="box" pos="0.45 0 %.4f" size="0.6 0.5 %.4f" rgba="0.6 0.55 0.5 1"/>' % (value / 2, value / 2)
    elif kind == "slope":
        a = math.radians(value)
        L = 1.2
        terrain = '<geom type="box" pos="%.4f 0 %.4f" euler="0 %.5f 0" size="%.3f 0.5 0.01" rgba="0.6 0.55 0.5 1"/>' % (
            0.25 + L / 2 * math.cos(a), L / 2 * math.sin(a) - 0.01 * math.cos(a), -a, L / 2)
    elif kind == "rough":
        n = 120
        h = rng.random((n, n))
        from scipy.ndimage import gaussian_filter
        h = gaussian_filter(h, 2.0)
        h = (h - h.min()) / (h.max() - h.min() + 1e-9)
        asset = '<hfield name="rough" nrow="%d" ncol="%d" size="1.5 1.5 %.4f 0.01" elevation="%s"/>' % (
            n, n, max(value, 1e-4), " ".join("%.4f" % v for v in h.flatten()))
        terrain = '<geom type="hfield" hfield="rough" pos="0.9 0 0" rgba="0.6 0.55 0.5 1"/>'
    z0 = 0.024 + (value if kind == "rough" else 0.0)
    xml = """
<mujoco>
  <compiler angle="radian"/>
  <option timestep="0.001" cone="elliptic" impratio="10" integrator="implicitfast"/>
  <asset>%s</asset>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.75 0.75 0.75 1" friction="0.9 0.01 0.001"/>
    %s
    <body name="rover" pos="-0.3 0 %.4f">
      <freejoint/>
      <inertial pos="-0.02 0 0.055" mass="0.92" diaginertia="0.004 0.006 0.008"/>
      <geom name="deck" type="box" pos="0.0125 0 0.0515" size="0.1275 0.060 0.0015" contype="1" conaffinity="1"/>
      <geom name="hangers" type="box" pos="-0.030 0 0.033" size="0.059 0.047 0.017"/>
      <geom name="bay_l" type="box" pos="0.084 0.0335 0.027" size="0.044 0.0015 0.023"/>
      <geom name="bay_r" type="box" pos="0.084 -0.0335 0.027" size="0.044 0.0015 0.023"/>
      <geom name="bay_b" type="box" pos="0.0385 0 0.027" size="0.0015 0.035 0.023"/>
      <geom name="door" type="box" pos="0.1295 0 0.023" size="0.0015 0.040 0.019"/>
      <geom name="battery" type="box" pos="-0.075 0 0.0625" size="0.0325 0.028 0.0095"/>
      %s
    </body>
  </worldbody>
  <actuator>%s</actuator>
</mujoco>""" % (asset, terrain, z0, wheels, acts)
    return xml


def run(kind, value, stall_kgcm, seed=0, v=0.15, T=15.0):
    m = mujoco.MjModel.from_xml_string(scene(kind, value, stall_kgcm, seed))
    d = mujoco.MjData(m)
    w = v / 0.022                          # wheel axis +Y: positive spin rolls the rover toward +X
    d.ctrl[:] = w
    bid = m.body("rover").id
    low = {m.geom(n).id for n in ("bay_l", "bay_r", "bay_b", "door", "hangers")}
    belly_s = 0.0
    x0 = d.xpos[bid][0]
    steps = int(T / m.opt.timestep)
    tipped = False
    for i in range(steps):
        mujoco.mj_step(m, d)
        if i % 50 == 0:
            R = d.xmat[bid].reshape(3, 3)
            if R[2, 2] < 0.5:                                         # more than 60 deg roll/pitch
                tipped = True
                break
            touching = any((d.contact[k].geom1 in low or d.contact[k].geom2 in low) for k in range(d.ncon))
            belly_s += 0.05 if touching else 0.0
    prog = d.xpos[bid][0] - x0
    status = "tipped" if tipped else ("crossed" if prog > 0.6 else ("high-centred / snagged" if belly_s > 1.0 else "stalled"))
    return dict(kind=kind, value=value, stall_kgcm=stall_kgcm, progress_m=round(float(prog), 3),
                low_parts_dragging_s=round(belly_s, 1), status=status)


def main():
    out = []
    for stall in (7.0, 2.8):
        for h in (0.002, 0.004, 0.006, 0.008, 0.010, 0.015, 0.020):
            out.append(run("step", h, stall))
        for a in (5, 10, 15, 20, 25, 30):
            out.append(run("slope", a, stall))
        for A in (0.003, 0.006, 0.010, 0.020):
            r = [run("rough", A, stall, seed=s) for s in range(3)]
            crossed = sum(x["status"] == "crossed" for x in r)
            out.append(dict(kind="rough", value=A, stall_kgcm=stall, crossed_of_3=crossed,
                            statuses=[x["status"] for x in r]))
    for r in out:
        print(r)
    with open(os.path.join(HERE, "results", "terrain_test.json"), "w") as f:
        json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
