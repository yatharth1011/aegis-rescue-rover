/*
 * Tunable constants with no hardware dependency. Shared by the firmware
 * (via aegis_config.h) and the software-in-the-loop simulator (sim/).
 */
#pragma once

/* ---- Encoders (A, B) ---------------------------------------------------- */
#define ENC_PPR             3        /* Hall pulses per motor rev (Robu listing) */
#define GEAR_RATIO          150.0f
#define ENC_COUNTS_PER_WHEEL_REV (ENC_PPR * 4 * GEAR_RATIO)   /* x4 quadrature = 1800 */

/* ---- Intake-door servo (MG946R on the 5.0 V servo rail, Mk2) ------------- */
#define SERVO_US_DOOR_OPEN   1950    /* door swung 110 deg clear of the bay mouth: calibrate */
#define SERVO_US_DOOR_CLOSED 1050    /* door across the bay mouth: calibrate, stop short of the wall */
#define SERVO_US_OPEN       SERVO_US_DOOR_OPEN     /* names used by the stand-alone mission */
#define SERVO_US_GRIP       SERVO_US_DOOR_CLOSED
#define SERVO_US_MIN        900
#define SERVO_US_MAX        2100

/* ---- Battery sense: 100k / 20k divider on GPIO1 (ADC1_CH0) ------------- */
#define VBAT_LOW_V          10.2f    /* 3.4 V/cell under load -> return home */
#define VBAT_CUTOFF_V       9.9f     /* 3.3 V/cell -> FAULT, motors off */

/* ---- Geometry (Fusion model parameters) --------------------------------- */
#define WHEEL_RADIUS_M      0.022f
#define TRACK_M             0.132f
#define TRACK_SKID_GAIN     1.45f    /* effective/geometric track; calibrate with a
                                      * 10-turn spin test, heading comes from the gyro */
#define MIC_SPACING_M       0.100f

/* ---- Drive limits --------------------------------------------------------- */
#define WHEEL_W_NOLOAD      20.94f   /* 200 rpm in rad/s at 12 V */
#define V_MAX_MPS           0.30f
#define W_MAX_RPS           1.5f
#define V_REVERSE_MAX_MPS   0.10f

/* ---- Control loop --------------------------------------------------------- */
#define NAV_PERIOD_MS       10       /* 100 Hz */
#define PI_KP               0.02f
#define PI_KI               0.40f
#define CBF_D_SAFE_M        0.06f    /* Mk2: front ToF sits 13 mm behind the funnel tip (as in
                                      * simulation3d/robot.py). Mk1 with jaws needed 0.15. */
#define CBF_ALPHA           2.0f
#define ODOM_GYRO_WEIGHT    0.95f

/* ---- Reflexes (aegis_reflex.h), same values as simulation3d/robot.py ------ */
#define REFLEX_DIST_M        0.06f   /* travel away from a table edge */
#define REFLEX_V_MPS         0.08f
#define REFLEX_NO_TURN_S     1.0f    /* no turning / no motion toward the edge afterwards */
#define ESC_DIST_M           0.06f   /* stall escape, reverse */
#define ESC_V_MPS            0.06f
#define LINK_WATCHDOG_MS     300     /* edge mode: no command this long -> stop */

/* ---- IR sensors on the PCF8574 (P0 FL, P1 FR, P2 rear, P3 RL, P4 RR, P5 bay) */
#define PCF8574_ADDR         0x20    /* PCF8574T, A0-A2 low (0x38 on a PCF8574AT) */
#define CLIFF_ACTIVE_HIGH    1       /* TCRT5000 module DO goes high with no surface below */
#define CAPTURE_ACTIVE_LOW   1       /* obstacle module DO goes low with the token in the bay */

/* ---- Policy / primitives ------------------------------------------------ */
#define POLICY_BLOCK_FRONT_M 0.30f   /* "next cell occupied" = one rover length */
#define POLICY_BLOCK_SIDE_M  0.20f
#define POLICY_AUDIO_AHEAD   0.30f   /* rad, same as the simulator */
#define POLICY_USE_TABLE     false   /* SIL sim, 200 random rooms: rule 76 % success, Q-table
                                      * 22 % (it was trained on one fixed map). Keep the rule
                                      * until the table is retrained on randomised maps. */
#define STALL_HOLD_S         0.6f    /* pinned this long -> escape manoeuvre */
#define CELL_M               0.25f   /* FORWARD primitive length */
#define TURN_RAD             1.5708f /* TURN primitive: 90 deg */
#define TURN_TOL_RAD         0.05f

/* ---- Audio ----------------------------------------------------------------- */
#define AUDIO_FS            48000
#define AUDIO_FRAME         1024     /* 21.3 ms per DOA estimate */
#define AUDIO_F_LO          300.0f
#define AUDIO_F_HI          3400.0f
#define AUDIO_TEMP_C        25.0f
#define AUDIO_PSR_GATE      2.0f     /* host test: correlated 6 dB SNR ~4.4, noise ~0.2 */
#define AUDIO_SNR_GATE_DB   6.0f     /* band energy over tracked noise floor */

/* ---- Mission ---------------------------------------------------------------- */
#define DOA_STALE_MS        3000
#define VISION_STALE_MS     300
#define DOCK_AREA_FRAC      0.040f   /* target bbox area / frame area at which to grip.
                                      * 18 mm dowel, 60 deg HFOV: area = (0.018 / (2 d tan30))^2.
                                      * The gear hubs stop the object 72 mm from the camera
                                      * (area 0.047), so the old 0.18 could never trigger.
                                      * 0.040 = object 78 mm from camera, between the jaws. */
#define DOCK_EX_TOL         0.10f    /* normalised horizontal centroid error */
#define PATH_SPACING_M      0.15f
#define PATH_LOOP_RADIUS_M  0.20f
#define HOME_TOL_M          0.15f

/* ---- Task layout ------------------------------------------------------------
 * Core 1 runs only the control loop, so Wi-Fi interrupts (core 0) never add
 * jitter to it. Audio runs on core 0. The 8 x 512-frame I2S DMA ring holds
 * 85 ms, so Wi-Fi preemption delays an estimate but never drops samples. */
#define CORE_NAV            1
#define CORE_AUDIO          0
#define CORE_AUX            0
#define PRIO_NAV            20
#define PRIO_AUDIO          10
#define PRIO_VISION         8
#define PRIO_NET            6
#define PRIO_TELEMETRY      3
