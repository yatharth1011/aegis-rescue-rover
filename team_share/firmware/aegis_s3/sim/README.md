# A.E.G.I.S. software-in-the-loop (SIL) simulator

The rover's **actual firmware decision code**, compiled unchanged from `../components/aegis_core`, runs inside simulated rooms. The simulated parts stand in for the hardware: motors, wheels, sensors, the sound field and the target.

| Runs the real firmware code | Simulated around it |
|---|---|
| Band-pass filter and GCC-PHAT bearing (`aegis_dsp`) | 48 kHz stereo mic signals: pulsed beacon, first-order wall echoes, obstacle shadowing, motor noise while driving, room noise |
| DOA gate and noise-floor tracker | Four wheel motors with battery scaling; true skid factor ±10–20 % off the calibrated value; 3 % slip; lateral noise |
| Policy (rule or Q-table), CBF safety filter | Quantised quadrature encoders (1800 counts/rev), gyro bias and noise |
| Wheel PI loops, skid-steer kinematics, odometry | Three VL53L0X ToF sensors as 25° cones (5 rays), 33 ms update, 2 m max |
| Stall detector, escape manoeuvre | Camera: 60° FOV, 1 m range, line of sight, 5 % dropouts |
| Mission state machine, front/back voting, return path | An 18 mm target that the gear hubs push, and that counts as grasped only if it sits between the jaws when they close |

Each room is 4 × 3 m, with 3–6 box obstacles (6–9 in "harsh"). The target sits at least 2 m from the start. A room is only used if the rover could physically reach the target. A mission counts as a **success** only if the rover grasps the target and its true position at the end is within 30 cm of the start.

## Results (200 random rooms per scenario, seed 7)

| Scenario | Success | Target found | Median time | Contact-free runs |
|---|---|---|---|---|
| **Nominal** (rule policy, the firmware default) | **76 %** | 87.5 % | 38 s | 82.5 % |
| Harsh: more obstacles, stronger echoes, louder motors, ±20 % skid | 52.5 % | 71.5 % | 44 s | 66 % |
| Beacon off (no acoustic homing) | 37.5 % | 54.5 % | 83 s | 65.5 % |
| Q-table policy, nominal rooms | 22 % | 47 % | 49 s | 51.5 % |

What the numbers show:

- **Acoustic homing doubles the success rate** (37.5 % → 76 %) and **more than halves the mission time** (83 s → 38 s).
- **Bearing accuracy while stopped:** the median error is 8° once front/back is resolved. The raw two-mic reading has a median error of 26°, because a mic pair alone confuses front and back.
- **Return home:** in successful runs, the rover ends a median of 14.5 cm from its start, and at most 28 cm.
- **The Q-table hurts.** It was trained on one fixed grid map, and in new rooms it misuses the audio input. The firmware therefore defaults to the rule (`POLICY_USE_TABLE false`).

Files in `results/`:

- `summary.png`: outcome chart for all four scenarios
- `deck_figure.png`: the figure on slide 2 of the deck
- `example_nominal.png` and `example_nominal.gif`: one full mission, as a still and as an animation
- one CSV per scenario, one row per room

## Bugs the simulator found, now fixed in the firmware

1. **The grasp could never trigger.** `DOCK_AREA_FRAC` was 0.18, but the gear hubs stop an 18 mm object where it fills only 4.7 % of the camera frame. The rover pushed the target into walls forever. The threshold is now 0.040, derived from the CAD.
2. **The jaws hit things.** The safety distance (0.10 m) was measured from the ToF sensor, which sits 92 mm behind the jaw tips. It is now 0.15 m.
3. **Front/back confusion.** A source behind the rover read as "ahead", and the rover drove away from it. It is now resolved by a bearing vote across headings: each reading votes for both possible directions, and only the true one stays put as the rover turns.
4. **Zig-zag search.** Fixed 90° turns toward a source 30° off overshoot every time. Turns toward a fresh bearing are now sized by that bearing.
5. **Pinned forever.** A box corner caught the chassis side, out of the front sensor's view, and nothing noticed. A stall detector (encoders plus PI saturation) now triggers an escape: back off 8 cm, then turn 40° toward the freer side.

## Limits: what the simulator does *not* prove

- **The target is assumed visible to the camera at 1 m.** No vision code exists yet. The ESP32-CAM detector is unwritten, and the camera's downward view of a low object has to be checked on the real build.
- **Rooms are 2D boxes.** There are no ramps, no rubble and no soft ground. All obstacles are tall enough for the ToF to see; low debris would be missed, exactly as on the real rover.
- **Motor, slip and sound models are first-order approximations.** Treat the percentages as a comparison between designs, not a promise for the lab floor.
- **About a quarter of nominal missions still time out**, mostly searches that never get the target in camera view.

## Run it

```bash
make -C sim batch RUNS=200
```

Or a single scenario:

```bash
./sim nominal 50 1 out.csv 0 results/example
```

Add `AEGIS_POLICY=table` in front of the command to test the Q-table policy instead of the rule.

Plots need matplotlib:

```bash
python3 sim/plot_sim.py summary sim/results sim/results/summary.png
```
