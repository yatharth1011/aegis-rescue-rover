"""
Reward for retraining the A.E.G.I.S. navigation policy in the 3D simulator.

Safety terms dominate, so the policy learns that losing balance or leaving the
table is far worse than being slow. Values are per control step (10 Hz) unless
marked "terminal" (the episode ends).
"""
import math

REWARD = {
    # ---- terminal failures ------------------------------------------------------
    "fell_off_table": -200.0,      # centre of mass or two wheels past the table edge (terminal)
    "upside_down": -200.0,         # body z-axis below horizontal (terminal)
    # ---- balance / stability ------------------------------------------------------
    "tilt_per_deg_over_15": -0.5,  # per degree of roll/pitch beyond 15 deg, every step
    "tilt_over_35": -50.0,         # one-off when roll/pitch first exceeds 35 deg (about to tip)
    "wheel_lift": -2.0,            # per step with any wheel off the ground
    "high_centred": -1.0,          # per step with the bay walls / door dragging on the ground
    # ---- safety layer had to intervene ------------------------------------------
    "cliff_reflex": -10.0,         # the on-board table-edge reflex fired (the policy got too close)
    "collision": -5.0,             # new contact episode with an obstacle
    "cbf_limited": -0.05,          # per step the speed filter clipped the command
    "stall_escape": -3.0,          # on-board stall escape triggered
    # ---- task progress ----------------------------------------------------------------
    "new_area_inspected_per_m2": 40.0,
    "scream_bearing_progress": 2.0,  # per metre moved toward the resolved scream bearing
    "token_first_seen": 20.0,
    "token_captured": 100.0,
    "home_with_token": 200.0,       # terminal success
    "step": -0.02,                  # time cost
}


def step_reward(ev):
    """ev: dict of per-step events from the simulator (booleans, counts, metres, degrees)."""
    r = REWARD["step"]
    tilt = max(abs(ev.get("roll_deg", 0.0)), abs(ev.get("pitch_deg", 0.0)))
    if tilt > 15.0:
        r += REWARD["tilt_per_deg_over_15"] * (tilt - 15.0)
    if ev.get("tilt_over_35_first"):
        r += REWARD["tilt_over_35"]
    r += REWARD["wheel_lift"] * ev.get("wheels_off_ground", 0)
    r += REWARD["high_centred"] * bool(ev.get("belly_contact"))
    r += REWARD["cliff_reflex"] * ev.get("cliff_reflex", 0)
    r += REWARD["collision"] * ev.get("new_collisions", 0)
    r += REWARD["cbf_limited"] * bool(ev.get("cbf_limited"))
    r += REWARD["stall_escape"] * ev.get("stall_escapes", 0)
    r += REWARD["new_area_inspected_per_m2"] * ev.get("new_inspected_m2", 0.0)
    r += REWARD["scream_bearing_progress"] * ev.get("toward_scream_m", 0.0)
    r += REWARD["token_first_seen"] * bool(ev.get("token_first_seen"))
    r += REWARD["token_captured"] * bool(ev.get("token_captured"))
    done = False
    if ev.get("fell"):
        r += REWARD["fell_off_table"]
        done = True
    if ev.get("upside_down"):
        r += REWARD["upside_down"]
        done = True
    if ev.get("home_with_token"):
        r += REWARD["home_with_token"]
        done = True
    return r, done


def roll_pitch_deg(R):
    """Roll and pitch (deg) from a 3x3 body rotation matrix (MuJoCo xmat reshaped)."""
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2][0]))))
    roll = math.degrees(math.atan2(R[2][1], R[2][2]))
    return roll, pitch
