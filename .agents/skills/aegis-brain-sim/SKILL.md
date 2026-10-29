---
name: aegis-brain-sim
description: Change or evaluate the A.E.G.I.S. laptop brain (simulation3d/brain.py), the edge program (edge/aegis_edge.py, aegis_link.py), the MuJoCo simulator, YOLO detection, scream homing, mapping, planning or the start pad. Use whenever behaviour of the autonomous mission could change, and to run before/after comparisons.
---

# Brain and simulator

## What is where
- `simulation3d/brain.py`: the **one** brain. It is used by both the simulator and the real edge program, so never fork it. It does:
  - mapping: the occupancy grid from the ToF sensors, plus the "inspected" grid from the camera footprint;
  - viewpoint selection;
  - A\* planning on the inflated grid;
  - the state machine EXPLORE → PREGRASP → APPROACH → CLOSE → RETURN → DONE, with BACKOFF and UNSTICK as recovery states.
  Tunables are at the top: `RES`, `INFLATE`, `EDGE_KEEP`, `SEARCH_SIZE`, `CLIFF_KEEP`, `V_MAX`, `W_MAX`.
- Table size: `Brain(..., table_known=False)` is the default on the real rover (`aegis_edge.py --table unknown`). Edges are learned from the cliff sensors. `--table LX LY` uses a known size.
- `simulation3d/world.py`: arena generator, geometry, camera intrinsics (`CAM`), cliff sensors, fall detection. `robot.py`: firmware behaviour plus the C core through `core_ffi.py`. `link.py`: the lossy Wi-Fi model. `run_mission.py`: one mission. `eval_batch.py`: parallel batches.
- `edge/aegis_edge.py`: the real deployment.
  - `PoseHistory` pairs each frame with the pose at exposure time.
  - `StartPad` gives the ArUco DICT_4X4_50 id 0 fix: a 100 mm marker, solved with solvePnP IPPE_SQUARE.
  - `KeepAlive` sends commands at 20 Hz.
- `edge/fake_rover.py`: the simulator behind the real UDP/MJPEG protocol. `edge/test_end_to_end.py` runs real-time missions through `aegis_edge.py`.

## Evaluation protocol (mandatory for any behaviour change)
Results are noisy: one mission proves nothing. Always compare **before and after on the same seeds**, with the same detector and the same link profile.

```bash
cd team_share/simulation3d
# oracle detector = perfect detection, isolates planning/control changes; fast
AEGIS_DETECTOR=oracle ~/aegis_venv/bin/python eval_batch.py /tmp/before_d1.csv 1 typical 16 200 9            # known table
AEGIS_DETECTOR=oracle ~/aegis_venv/bin/python eval_batch.py /tmp/before_d1u.csv 1 typical 16 200 9 unknown   # unknown table
# repeat for demo 2, then make the change, then rerun into after_*.csv
# YOLO (slower, realistic): leave AEGIS_DETECTOR unset
# real-time through the deployable program:
cd .. && ~/aegis_venv/bin/python edge/test_end_to_end.py --demo 2 --seeds 4004 4008 --table unknown
```
Use `git stash` for the "before" run, or run the batch before you edit.

Report a table with these columns: demo, mode, success/n, outcomes (timeout, lost_token, home_off, fell), mean time of the successes. **Falls must stay 0.** A change that adds a fall is rejected, whatever else it improves.
The latest reference (oracle, typical link, seeds 200-215):
- demo 1: known table 9/16, unknown 8/16.
- demo 2: known table 11/16, unknown 9/16.
- Falls: 0 in every run.

The real-time end-to-end runs score lower than the batch simulation. That's expected, because of the timing, and it doesn't mean a regression.

## Rules
- Don't retune constants to fit a few seeds. If you tune anything, check it on a **different** seed range afterwards (e.g. 300-315).
- Keep `run_mission.py` and `aegis_edge.py` feeding the brain the same messages (`telem`, `frame`, `audio`, `marker`) with the same fields.
- Don't commit `edge/logs/`, `edge/dataset/` or `camera_calib.npz` (all git-ignored).
