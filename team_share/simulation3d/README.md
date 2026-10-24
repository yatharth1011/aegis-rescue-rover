# A.E.G.I.S. 3D simulation: robot + Wi-Fi + laptop "edge brain"

This is a realistic, all-virtual test of the whole system on one Mac. Nothing is played through the speakers: the sound exists only as numbers for the two simulated microphones.

## What is simulated, and with what

| Part | How | Realism notes |
|---|---|---|
| **Table and room** | MuJoCo. A 1.8 × 1.2 m table, 0.75 m high, in a lab room; random "book" obstacles; random lighting and table colour | The table has real edges (the rover can fall). Every arena is checked to be solvable |
| **Rover body** | MuJoCo planar body. Force-limited drive stalls against obstacles; the door is a hinged, servo-driven body | Geometry from the Fusion Mk2 model |
| **Victim token and decoys** | Free physical objects: pushed, funnelled and dragged by real contact physics. Decoys are orange cube, red ball and orange disc | The detector has to tell shape apart, not just colour |
| **Wheels, motors, encoders, gyro** | The firmware's own PI loops and stall detector (C code, `core_ffi`). Skid factor ±12 % off calibration; slip noise | Gyro residual bias after boot calibration is about 0.05°/s |
| **Distance sensors** | 3 × VL53L0X as 25° cones (5 rays); 33 ms update; noise; 2 m range | Low objects are invisible to them, as on the real rover |
| **Table-edge and capture IR** | Geometric checks at the real sensor positions: 5 edge sensors (front-left, front-right, rear, both rear corners) and 1 capture sensor | Rear corners were added after the simulation showed falls |
| **Camera** | MuJoCo render from the mast, tilted 30° down; 320 × 240; JPEG; noise | |
| **Detector** | **YOLOv8n trained on 4,800 rendered frames on the M5 GPU**. Validation: 98.9 % recall, 98.7 % precision | Trained on renders only: retrain on real photos of the real token before the demo |
| **Sound** | pyroomacoustics 6 × 5 × 3 m room (image-source echoes, air absorption), a fan in a corner, mic self-noise. Synthetic screams with formants | Scream detector: 80/80 screams caught, 0/160 false alarms on talking and fan noise. All synthetic: record real screams into `screams/` |
| **Bearing** | The firmware's GCC-PHAT (C code) on the robot at 48 kHz. Front/back resolved by voting across headings | |
| **Wi-Fi** | Latency, jitter, bandwidth, burst loss and 0.5–3 s outages (`link.py`; "typical" and "bad" profiles) | The robot's watchdog stops it when commands stop arriving |
| **Laptop brain** | `brain.py`: occupancy and "inspected" maps; coverage search with viewpoint memory; A*; token localisation from the image; rescue sequence; A* home over known-free cells; start-pad ArUco re-localisation | |

## Terrain capability (full 3D rigid-body physics, `terrain_test.py`)

| Terrain | Result |
|---|---|
| Sharp step ≤ 4 mm | crosses |
| Sharp step ≥ 6 mm | **stuck**: the bay walls and door are 4 mm above the ground |
| Slopes up to 25° | climbs. Runs marked "tipped" in the log drove off the far end of the test ramp; that is an artefact of the test |
| Random rough ground, ±1 cm | 3 of 3 crossed |
| Random rough ground, ±2 cm | 2 of 3 crossed |

Mk2 is a **table and indoor-floor robot**. Rugged terrain needs larger wheels, suspension and more ground clearance.

## Run

All commands use the `~/aegis_venv` Python 3.12 environment.

Build the training images and train the detector (about 40 minutes on the M5):

```bash
~/aegis_venv/bin/python make_dataset.py build/ds/w0 600 100 200
```

```bash
~/aegis_venv/bin/python train_detector.py build/ds
```

Run 40 missions on 7 cores (`AEGIS_DETECTOR=oracle` uses ground truth as an upper bound):

```bash
~/aegis_venv/bin/python eval_batch.py results/x.csv 1 typical 40 3000 7
```

Make the summary, the charts and the example mission GIFs:

```bash
~/aegis_venv/bin/python make_report.py
```

Run the 3D terrain capability test:

```bash
~/aegis_venv/bin/python terrain_test.py
```

## Known limits

- The detector was trained on synthetic renders; real lighting, motion blur and the real token will differ.
- Rooms are tidy (boxes on a flat table). There is no cable clutter and no people walking past.
- Odometry relies on the start-pad marker to stay accurate. Without it, return-home accuracy degrades to 10–40 cm.
