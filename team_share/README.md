# Project A.E.G.I.S. (Autonomous Edge-Guided Identification & Salvage Rover)
### Course: TL101 Tinkering Lab | Indian Institute of Technology Ropar
**Lead Architect:** Yatharth Shrivastava (Roll No: ``, AI & Data Engineering)  
**Co-Engineered with:** JAExokauber (Elite Project Partner & Worker Mode)

---

## 📑 1. Project Genesis & What Yatharth Asked

### The Original User Vision:
Yatharth approached with an ambitious first-year search-and-rescue (SAR) autonomous rover concept for disaster response (forest fires, collapsed infrastructure):
1. **Compute Core:** Use NVIDIA Jetson Nano.
2. **Perception:** Computer vision (human face/shape detection) + Acoustic sensing (localizing direction of human scream/distress calls).
3. **Intelligence:** Reinforcement Learning (RL) agent rewarded for obstacle avoidance, space inspection, target discovery, and shortest-path extraction.
4. **Mechanical Action:** Ingestion mechanism where the robot puts the victim/payload inside its chassis and extracts via the tested shortest path.
5. **Course Constraints (TL101 at IIT Ropar):**
   - Strictly capped budget: **₹15,000 INR max**.
   - Preferred procurement: **Robu.in** direct component links.
   - Team size: 4 to 6 members.
   - Evaluated deliverable: Faculty Instructor (FI) consent/approval carries **15 Marks**.
   - Deadline: Urgent (immediate submission window).
   - Strict CAD Mandate: **Zero AI-generated images** allowed. All CAD models, blueprints, and drawings must be native to **Autodesk Fusion**.

---

## 🛠️ 2. What We Engineered & Shipped (The Execution Log)

To prevent budget exhaustion (Jetson Nano alone is ₹15k–₹22k) and physical hardware destruction from online real-world RL, we pivoted to a sim-to-real edge robotics architecture. Section 4 lists the Revision 2 changes.

### A. Core Architecture & Hardware
- **Dual-compute:** ESP32-S3 DevKitC-1 (dual-core LX7, 240 MHz) plus an ESP32-CAM (OV2640) for vision and Wi-Fi video.
- **Acoustic homing:** two INMP441 I2S MEMS microphones on a 100 mm baseline. TDoA from GCC-PHAT gives the bearing:
  $$\theta = \arcsin\left(\frac{c \cdot \Delta t}{d}\right)$$
- **Sim-to-real RL policy with safety shield:**
  - Tabular Q-learning over a discretised state (front/left/right blocked, audio bearing in 3 bins, target visible), 5 actions, trained in `train_rl_policy.py` for 2,000 episodes and exported to `aegis_q_table.h`.
  - The earlier claims of a "98.4 % success rate" and "< 8.4 ms" were never measured. The script now measures it:
    - 71.8 % of training episodes succeed (with exploration).
    - Greedy rollouts succeed 100 % on the **single fixed training map**.
    - Only 23 of 48 states were ever visited.
    The table encodes one map, not a general skill.
  - The exported `aegis_q_table.h` previously **did not compile** (an extra opening brace). The exporter is fixed and the table regenerated.
  - Safety shield: any ToF reading < 10 cm overrides the policy action.
- **Gripper:** geared twin-jaw gripper printed in PLA+. TowerPro MG946R servo at 5.0 V drives one jaw directly; the second jaw follows through a 1:1 module-2, 15-tooth gear pair.

### B. File & Deliverables Manifest

```
project/
├── README.md                       # Start here
├── AEGIS Body.f3d, AEGIS Detailed.f3d   # Fusion designs saved by the team
├── to_submit/                      # Goes to the faculty instructor: deck, BOM, approval form
├── course_templates/               # Blank course templates, as issued
└── team_share/
    ├── README.md                   # This file: full project log and to-do list
    ├── AEGIS_Proposal_Deck.pdf     # The deck as a PDF (the .pptx is in to_submit/)
    ├── AEGIS_Wiring_Guide.pdf      # Board faces, pin map, wire list
    ├── Build_AEGIS_Rover/          # Fusion API script (printable + detailed); output*/ = renders, STEP, DXF, STL
    ├── engineering/                # Torque, current, battery and strength calculations
    ├── firmware/aegis_s3/          # ESP-IDF firmware, host tests, 2D SIL simulator (sim/)
    ├── simulation3d/               # MuJoCo + YOLO + room acoustics + Wi-Fi: the 3D simulator and results
    ├── deck_assets/                # Images the deck is built from
    ├── tools/                      # build_submission.py (deck + form), deck_story.py, bom_excel.py, wiring_guide.py
    ├── train_rl_policy.py, aegis_q_table.h, aegis_rl_training_curve.png   # first Q-learning policy (experimental)
```

---

## ⚡ 3. HANDOFF TO CLAUDE: "Stop Playing in the Sandbox, Make This Production-Ready"

> ### 🚨 Attention Claude:
> **Let's be brutally honest: What has been built so far is solid 1st-year academic scaffolding, but from a Tier-1 senior systems engineer perspective, it is still too high-level, naive, and childish.**
>
> You are inheriting this codebase. Yatharth expects **ruthless, full production-level, industrial-grade depth** with zero hand-waving, zero placeholders, and zero theoretical fluff. 
> 
> Here is the exact hit-list you need to execute next to take this from an undergraduate lab prototype to a DARPA/RoboCup publication-grade autonomous system:

### 🎯 Production Engineering Directives for Claude:

1. **FreeRTOS Real-Time Firmware (C++ / ESP-IDF):**
   - Do NOT give generic Arduino `.ino` sketches.
   - Write fully pinned FreeRTOS tasks:
     - `Task_AudioDSP` pinned strictly to **Core 0** (I2S DMA buffer capture @ 44.1 kHz, FFT / GCC-PHAT cross-correlation windowing, vocal bandpass IIR filter $300\text{ Hz} - 3.4\text{ kHz}$).
     - `Task_NavControl` pinned strictly to **Core 1** (ToF reading via I2C DMA, Q-table action lookup from `aegis_q_table.h`, PID motor speed synchronization via hardware PWM MCPWM peripheral, and odometry calculation).
   - Implement inter-task communication using FreeRTOS queues and thread-safe mutexes.

2. **Rigorous Mechanical Dynamics & Actuator Math:**
   - Derive the mathematical traction and motor torque equations:
     $$\tau_{motor} = \frac{r_{wheel}}{4 \cdot \eta} \left( \mu \cdot m \cdot g + m \cdot a + m \cdot g \cdot \sin(\theta_{incline}) \right)$$
   - Calculate battery runtime curves under full continuous stall load:
     $$I_{total} = 4 \cdot I_{N20\_stall} + I_{servo} + I_{ESP32} + I_{sensors} \approx 4(0.45\text{A}) + 1.2\text{A} + 0.25\text{A} + 0.1\text{A} = 3.35\text{A}$$
   - Verify battery discharge C-rating for the 2200mAh 3S pack ($3.35\text{A} / 2.2\text{Ah} = 1.52\text{C} \ll 20\text{C}$ safe continuous limit).

3. **Advanced Autodesk Fusion 360 CAD & Stress Analysis:**
   - Expand the parametric script into a fully constrained kinematic assembly with joint limits (Revolute joints on wheels, Slider/Pin-Slot joints on compliant gripper jaws).
   - Define Finite Element Analysis (FEA) simulation parameters: Apply a $10\text{ N}$ pinching force at the gripper fingertip, set Tough PLA material yield strength ($45\text{ MPa}$), and verify that von Mises stress maintains a Safety Factor ($SF \ge 2.5$).

4. **SLAM & Micro-ROS Hardware Integration:**
   - Implement micro-ROS client on the ESP32-S3 over Wi-Fi/XRCE-DDS communicating with ROS2 Humble on the base laptop.
   - Publish `/scan` (from sweeping ToF/servo), `/odom` (from quadrature encoders), and `/audio_doa` topics.
   - Subscribe to `/cmd_vel` for manual teleoperation override.

---
## ✅ 4. Revision 2 (Claude, 23-Oct-2026): submission risks fixed

### What was wrong and what changed
| Area | Problem found | Fix |
|---|---|---|
| CAD compliance | `aegis_cad_technical_drawing.png`, the `.obj` and the `.dxf` were generated by matplotlib/Python, not Fusion, and the drawing was on slide 2 of the deck | Moved to `_superseded_not_fusion/`. The deck now has a marked slot for the Fusion export. `Build_AEGIS_Rover.py` was rewritten into a full parametric assembly (29 user parameters, joints with limits, materials) |
| BOM links | **All 15 Robu.in links returned "404 – Page Not Found"** | Every line re-sourced from live Robu.in listings, with price and stock recorded on 23-Oct-2026 |
| BOM content | One TB6612FNG for four motors (2 A on a 1.2 A channel); two ToF sensors for a three-direction state; no programmer for the ESP32-CAM; reserve misstated (₹3,750 vs actual ₹5,522) | Two drivers, three VL53L0X, ESP32-CAM-MB added, reserve computed by formula |
| Power | 2C battery pack (4.4 A max) against a 5.8 A worst case; LM2596 too small for servo stall | 10C ISR pack; XL4015 5 A bucks on two separate 5 V rails |
| Current math | 0.45 A motor stall (listing says 1 A); 5 V/6 V rail currents added straight to battery current; "20C" pack | Redone per rail with buck conversion: 5.84 A worst case = 2.66 C |
| Torque math | μ used as a drag term, η applied twice, skid-steer turning ignored | C_rr drag, μ only as slip limit, pivot-turn case added. Worst case is 27 % of rated torque |
| Gripper | No strength check; 12-tooth gear fails (SF 2.18) | 15-tooth module-2 gear, servo at 5.0 V: gear SF 3.25, arm SF 4.6 at servo stall |

The complete to-do list is in section 7.

## 🔧 5. Firmware (Claude, 23-Oct-2026)

`team_share/firmware/aegis_s3/` holds ESP-IDF v5.3 firmware. Its README covers the wiring table, bring-up checklist and design rationale.

**Tasks:**

- `Task_NavControl` runs alone on core 1 at 100 Hz:
  - encoders on PCNT, MPU-6050 yaw, 3×VL53L0X;
  - odometry, Q-table policy with safety overrides, CBF speed filter;
  - 4× wheel PI driving MCPWM;
  - mission FSM with loop-pruned return path.
- `Task_AudioDSP` runs on core 0: I2S DMA → Butterworth band-pass → GCC-PHAT bearing.
- Inter-task links: mailbox queues (`xQueueOverwrite`) and mutex-guarded telemetry.
- The mission state machine lives in the portable core (`aegis_mission.c`), so the firmware and the simulator run the same decision code.

**Verified:**

- The portable core (DSP, control, policy, path) passes **32/32 host unit tests**. GCC-PHAT recovers delays to within 0.07 samples, and the CBF closed loop never goes below 10 cm.
- The full firmware **builds cleanly with ESP-IDF v5.3.2** for the ESP32-S3 (zero project warnings, 354 KB image). It has not yet run on hardware.

**Not done:**

- micro-ROS (directive 4)
- ESP32-CAM firmware
- retraining the policy on randomised maps

## 🧪 6. Software-in-the-loop simulation (Claude, 24-Oct-2026)

`firmware/aegis_s3/sim/` runs the firmware's decision code, unchanged, in 200 random rooms per scenario. Around it, the simulator models:

- motors, wheel slip, encoders, gyro;
- three ToF cones and the camera;
- 48 kHz microphone audio with wall echoes and motor noise;
- the target object, which can be pushed and must be gripped.

| Scenario | Search → grasp → home success | Median time |
|---|---|---|
| **Nominal** (rule policy, the default) | **76 %** | 38 s |
| Harsh room | 52.5 % | 44 s |
| Beacon off | 37.5 % | 83 s |
| Q-table policy | 22 % | 49 s |

**Acoustic homing doubles the success rate and more than halves the mission time.**

The simulator found **five firmware bugs**, all now fixed:

- a docking threshold that could never trigger;
- jaws that hit obstacles;
- front/back confusion of the two-mic array;
- zig-zag search from fixed 90° turns;
- no recovery when pinned against an obstacle.

It also showed that the **Q-table hurts** in new rooms, so the firmware now defaults to the rule.

`sim/README.md` covers the method, the results and the limits; most importantly, no vision code exists yet. Slide 2 of the deck shows `sim/results/deck_figure.png`. `sim/results/example_nominal.gif` is a one-mission animation to show the faculty member.

## 🤖 7. Mk2 design, 3D simulation, and the computing decision (24-Oct-2026)

**Mk2 in Autodesk Fusion** (`Build_AEGIS_Rover/`; run both scripts from Fusion's Scripts list):

- **`Build_AEGIS_Rover`: printable assembly.** 41 components, 34 parameters, joints with limits, zero clashes.
  - A covered **intake bay**: the token rolls in between a funnel fin and a **servo-driven door**, which then closes. The rover really "takes it inside".
  - 4WD on N20 motors in printed hangers.
  - Two laser-cut decks.
  - The camera mast is tilted 30° down.
- **`Build_AEGIS_Rover_Detailed`: illustration model.**
  - Every purchased part is modelled to its published size in its own material.
  - A colour-coded **wiring harness**.
  - A wiring view with the upper deck hidden.
- **Engraved placement marks.** Both decks get component outlines and labels, so parts go exactly where the model says. Engraving has separate DXF layers from cutting.
- **Exports** (in `output*/`): STEP, `.f3d`, laser-cut DXFs, engrave DXFs, STL files for the four printed parts, and renders.
- **`AEGIS_Wiring_Guide.pdf`** (3 pages): the ESP32-S3 pin map, every module's face with where each wire lands, and the wire list. Colours match the Fusion harness; pins match the firmware.

**3D simulation** (`simulation3d/`; see its README).

What's simulated:

- the table and room in MuJoCo;
- the rover's camera feeding a **YOLO detector trained on the M5 GPU** (98.9% recall, 98.7% precision);
- room acoustics with synthetic screams (scream detector: 80/80 caught, 0/160 false alarms);
- the firmware's own GCC-PHAT, safety filter and PI loops;
- a lossy Wi-Fi link;
- the laptop "edge brain" (mapping, coverage search, A*, rescue sequence, return home).

Results:

| Demo (random table layouts) | Missions completed: found, captured, home within 15 cm |
|---|---|
| 1. Hidden token, no sound (40) | **70%**, median 35 s |
| 2. Scream homing (40) | **72%**, median 31 s |
| 1. Bad Wi-Fi, about 20% packet loss (20) | 70% |
| 2. Bad Wi-Fi (20) | 50% |

The token was found in 37 of 40 runs in both demos.

**Known gaps:**

- **Falls:** the first version fell off the table in 2 of 40 scream runs. Two rear-corner edge sensors, a 20 cm planner edge margin and a "no turning right after an edge" reflex brought it to **1 fall in 120 missions** (on the bad Wi-Fi link). The price was hidden-token success dropping from 80% to 70%: the rover is now more cautious near edges. The RL reward (`rl_reward.py`) punishes falling, tipping and going upside down.
- **Light obstacle contacts are common.**
- **Low decoys get nudged,** because the ToF sensors can't see objects that low.

**Terrain** (full 3D rigid-body test):

- crosses steps of 4 mm or less and rolling bumps up to about 1 cm;
- climbs slopes up to 25°;
- **gets stuck on any sharp step of 6 mm or more**, because the bay walls sit 4 mm off the ground.

Mk2 is a table and indoor robot; rugged terrain needs bigger wheels, suspension and clearance.

**Computing: use the Mac, not a Raspberry Pi or a Jetson.**

- The Mac trains the RL policy and the detector, and runs the "edge brain" over Wi-Fi in the demo.
- Safety stays on the ESP32.
- A Pi only makes sense for a laptop-free field version. It would probably blow the ₹3,817 budget reserve (approximate) and the power budget.

## 📋 8. What is left to make the project complete

**Before the approval meeting**

1. Run both Fusion scripts once more, which picks up the engraved decks.
2. Fill in the team names and roll numbers (6 rows, all AI & DE) in the form. Fill the advisor blank on slide 1 with **one** name.
3. Add an acrylic vendor link (BOM line 16) and re-check stock on the out-of-stock lines.
4. Submit `to_submit/`. Show the faculty member `simulation3d/results/example_demo1.gif` and `example_demo2.gif`.

**After approval**

5. Order parts. **Measure every board with calipers** and correct the footprints in `Build_AEGIS_Rover.py` before laser cutting.
6. Laser-cut and engrave the decks, print the four PLA parts, and wire to `AEGIS_Wiring_Guide.pdf`. Check each clone board's pin order against its silkscreen.
7. Bring-up with the firmware README checklist.
8. Write the laptop brain's real-hardware side:
   - Wi-Fi bridge on the ESP32 (UDP telemetry, MJPEG camera stream);
   - retrain YOLO on real photos of your token;
   - record real screams into `simulation3d/screams/`.
9. Retrain the RL navigation policy in the simulator with `rl_reward.py`. Keep it only if it beats the rule-based brain (70% / 72%). Tune the edge margin to win back hidden-token success without new falls.
10. Lab trials: 10+ runs of each demo; compare with the simulation.

**Status:** Mk2 designed in Fusion (printable and detailed versions), firmware builds, and the full system has been tested in a 3D simulation: 70% hidden-token and 72% scream-homing success, 1 fall in 120 missions. Submission files are ready apart from team names and the advisor.
