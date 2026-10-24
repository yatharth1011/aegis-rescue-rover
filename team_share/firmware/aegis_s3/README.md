# A.E.G.I.S. firmware: ESP32-S3, ESP-IDF v5.3+, FreeRTOS

## Task layout

| Task | Core | Prio | Rate | Job |
|---|---|---|---|---|
| `Task_NavControl` | **1** (alone) | 20 | 100 Hz, `xTaskDelayUntil` | Encoders (PCNT), IMU, 3×ToF (non-blocking), odometry, mission FSM, Q-table policy, CBF safety filter, 4× wheel PI → MCPWM, servo, battery |
| `Task_AudioDSP` | 0 | 10 | 47 frames/s | I2S DMA 48 kHz stereo → 4th-order Butterworth 300–3400 Hz → band-limited GCC-PHAT → bearing |
| `Task_Vision` | 0 | 8 | per camera frame | UART1 line protocol from the ESP32-CAM (checksummed) |
| `Task_Telemetry` | 0 | 3 | 2 Hz | Status line; later the micro-ROS publisher |

The hand-off note asked for audio on core 0 and navigation on core 1, and that is the layout here. It works because:

- ESP-IDF pins the Wi-Fi and lwIP tasks to core 0. Keeping core 1 for the control loop means radio interrupts never add jitter to it.
- Audio tolerates being preempted. The I2S DMA ring holds 8 × 512 frames (85 ms), so Wi-Fi bursts delay an estimate but never drop samples.

**Inter-task communication.**

- `q_doa` and `q_vision` are length-1 mailboxes written with `xQueueOverwrite`. Producers never block and the consumer always gets the newest sample.
- `g_telem` is guarded by `g_telem_mutex`. The nav task writes it at 10 Hz with a zero-timeout take, so the control loop never waits on the logger.

**Corrections to the hand-off spec:**

- **No I2C DMA.** The ESP32-S3 I2C peripheral has no DMA. Instead, ToF reads are non-blocking: each sensor ranges back-to-back and the loop only reads a result when the interrupt-status bit says one is ready.
- **Sample rate 48 kHz, not 44.1 kHz.** With a 100 mm mic baseline, the maximum delay grows from 13 to 14 samples, which improves angular resolution at no cost.

## Safety

- **CBF on forward speed:** `v ≤ α·(d_front − d_safe)` with α = 2 s⁻¹ and d_safe = 0.15 m, measured from the front ToF. The jaw tips are 92 mm ahead of the sensor, so this leaves about 6 cm at the tips. A missing or stale front reading (> 150 ms) blocks forward motion.
- **Stall escape:** two or more wheels near full duty but barely turning for 0.6 s means the rover is pinned on something the front sensor can't see. It backs off 8 cm, then turns 40° toward the freer side.
- **No autonomous reversing.** The rover has no rear sensor, so the policy's REVERSE action is replaced by a turn. Commanded reverse speed is capped at 0.10 m/s.
- **Latched faults stop the motors** (TB6612 STBY low) and need a reset:
  - tilt over 60°
  - IMU failure
  - front ToF missing
  - battery below 9.9 V
- **Low battery:** below 10.2 V the rover abandons SEARCH and drives home.
- **Boot safety:** STBY sits on GPIO46, which has a strap pull-down, so the motors are off from power-on until firmware enables them.
- **Watchdog:** the nav task subscribes to the task watchdog (1 s, panic). A hung control loop resets the chip.
- **Start / abort:** the BOOT button starts a mission, and pressing it again aborts.

## Policy

The default is a deterministic rule:

- step forward one cell when the front is clear;
- turn toward the sound, by the heard bearing itself (between 20° and 90°);
- when the front is blocked, turn 90° toward the open side;
- never reverse autonomously, because there is no rear sensor.

The Q-table from `train_rl_policy.py` is still compiled in, behind `POLICY_USE_TABLE`. In the SIL simulator over 200 random rooms, the rule completes **76 %** of missions and the Q-table **22 %**. The table was trained on a single fixed grid map and misuses the audio input in new rooms. Retrain it on randomised maps before turning it back on.

**Front/back resolution.** Two microphones can't tell a source in front from its mirror image behind. Each bearing read while stopped votes for both world directions in a 36-bin histogram, with old votes decaying. As the rover turns, only the true direction stays fixed, so the peak converges on it. In simulation, the median error is 8°.

## Mission

`IDLE → (BOOT) → SEARCH ⇄ APPROACH → GRASP → RETURN → RELEASE → DONE`

- **SEARCH** chains primitives:
  - a 0.3 s listening pause (motors stopped, so the bearing estimate is clean);
  - a policy decision;
  - FORWARD one 0.25 m cell or TURN 90° (gyro-closed).
- **Bearings are stored in the world frame.** A turn therefore doesn't make the last estimate wrong.
- **APPROACH:** visual servo on the camera centroid error. GRASP starts when the 18 mm target covers ≥ 4 % of the frame and is centred, which puts it between the jaws.
- **RETURN** retraces breadcrumbs recorded every 0.15 m. A loop is cut when the path comes back within 0.20 m of an older crumb, so the route home is loop-free and only crosses ground the rover has already driven.

## Wiring (ESP32-S3-DevKitC-1)

| Function | GPIO | Notes |
|---|---|---|
| I2S BCLK / WS / SD | 4 / 5 / 6 | Both INMP441 share the bus. Left mic L/R → GND, right mic L/R → 3V3 |
| I2C SDA / SCL | 8 / 9 | 400 kHz, 2.2 kΩ pull-ups to 3V3 |
| ToF XSHUT front / left | 10 / 11 | Right ToF XSHUT tied to its VIN (always on, moved to 0x32) |
| ToF addresses | 0x30 F, 0x31 L, 0x32 R | MPU-6050 at 0x68 |
| FL motor IN1 / IN2 | 12 / 13 | TB6612 #1 channel A |
| RL motor IN1 / IN2 | 14 / 15 | TB6612 #1 channel B |
| FR motor IN1 / IN2 | 16 / 17 | TB6612 #2 channel A |
| RR motor IN1 / IN2 | 18 / 21 | TB6612 #2 channel B |
| TB6612 PWMA, PWMB | 3V3 | Speed is PWM'd on IN1/IN2 (slow decay) |
| TB6612 STBY (both) | 46 | Strap pull-down: drivers stay off through boot |
| Encoders A / B: FL, RL, FR, RR | 38/39, 40/41, 42/47, 48/7 | Encoder VCC 3V3 |
| Gripper servo signal | 3 | Servo power from XL4015 rail B (5.0 V), common ground |
| ESP32-CAM TX → S3 RX | 2 | UART1, 115200 baud |
| Battery sense | 1 | 100 kΩ / 22 kΩ divider from pack + |
| Start / abort | 0 | On-board BOOT button |

Pins left unused on purpose:

- 19/20: USB
- 43/44: console
- 26–37: flash / PSRAM
- 45: VDD_SPI strap

GPIO 38 or 48 drives the RGB LED on some DevKitC revisions. As encoder inputs, they may make the LED flicker; this is harmless.

## Build and test

Host tests for the portable core need only a C compiler:

```bash
make -C test/host
```

Firmware. On this Mac, ESP-IDF v5.3.2 is installed at `~/esp/esp-idf-v5.3` (with Homebrew Python 3.12, because v5.3 does not support Python 3.14). `~/esp/idf_env.sh` activates it:

```bash
source ~/esp/idf_env.sh && idf.py build
```

```bash
idf.py -p /dev/cu.usbmodem* flash monitor
```

**Status of verification:**

| Part | Status |
|---|---|
| `components/aegis_core` (DSP, PI, kinematics, odometry, CBF, policy, return path) | Built with `-Wall -Wextra -Werror` and **32/32 host tests pass** |
| Closed-loop behaviour | SIL simulator, `sim/`: the core runs in 200 random rooms per scenario. **76 % of nominal missions succeed.** See `sim/README.md` |
| Full firmware (`main/` + core) | `idf.py build` with **ESP-IDF v5.3.2**, target esp32s3: **builds with zero warnings in project code**. The image is 0x58520 bytes (354 KB), leaving 65 % of the 1 MB app partition free. Not yet run on hardware. |

## Bring-up checklist

1. **Encoders and motors.** Lift the wheels and run with `V_MAX_MPS` low. Check that each `wheel_w` in the telemetry is positive when moving forward. If one isn't, flip that wheel in `MOTOR_INVERT`.
2. **Servo.** Calibrate `SERVO_US_OPEN` and `SERVO_US_GRIP` on the assembled gripper. GRIP must be the contact angle for the target object, not past it.
3. **Turn calibration.** Do a 10-turn spin test. Set `TRACK_SKID_GAIN` so that encoder yaw matches gyro yaw.
4. **Audio.** Play a 1 kHz tone from 1 m at ±45°. The telemetry bearing should read ±45° within about 5°. One sample of lag is 4.1° at broadside.
5. **CBF.** Drive at a wall. The rover must stop with ≥ 10 cm clearance. The ToF sits on the upper deck, 63 mm above the ground, so **objects lower than that are not seen**. Test with tall obstacles only.

## Not done yet

- micro-ROS client (`/scan`, `/odom`, `/audio_doa`, `/cmd_vel`). It would replace `Task_Telemetry` on core 0.
- ESP32-CAM firmware producing the `$T,...` line protocol (`task_vision.c` documents it).
- Retraining the policy on randomised maps.
