/*
 * Portable control primitives: wheel PI controller, skid-steer kinematics and
 * odometry, and the control-barrier-function safety filter.
 * No ESP-IDF dependencies: unit-tested on the host (test/host).
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Wheel velocity PI with feed-forward and anti-windup ---------------- */
typedef struct {
    float kp, ki;          /* duty per (rad/s), duty per (rad)            */
    float kff;             /* duty per (rad/s) of reference (1 / no-load speed) */
    float integ;
    float out_min, out_max;
} pi_ctrl_t;

void pi_init(pi_ctrl_t *c, float kp, float ki, float kff, float out_min, float out_max);
void pi_reset(pi_ctrl_t *c);
/* Returns duty in [out_min, out_max]. Integrator is frozen while saturated
 * in the direction of the error (conditional integration). */
float pi_update(pi_ctrl_t *c, float ref, float meas, float dt);

/* ---- Skid-steer kinematics ---------------------------------------------- */
typedef struct {
    float wheel_radius;    /* m */
    float track;           /* m, left-right wheel distance */
    float track_eff_gain;  /* effective track / geometric track (skid factor, >= 1) */
} kinematics_t;

/* Body (v, w) -> left/right wheel angular speed (rad/s) */
void kin_inverse(const kinematics_t *k, float v, float w, float *wl, float *wr);
/* Wheel speeds (rad/s) -> body v (m/s), w (rad/s) */
void kin_forward(const kinematics_t *k, float wl, float wr, float *v, float *w);

/* ---- Odometry: encoder distance + gyro heading (complementary) --------- */
typedef struct {
    float x, y, theta;     /* m, m, rad (wrapped to [-pi, pi]) */
    float gyro_weight;     /* 0..1, share of heading rate taken from the gyro */
} odom_t;

void odom_reset(odom_t *o, float gyro_weight);
/* v, w_enc from encoders; w_gyro from the IMU (bias removed) */
void odom_update(odom_t *o, float v, float w_enc, float w_gyro, float dt);
float wrap_pi(float a);

/* ---- Control barrier function safety filter ----------------------------
 * Barrier h = d_front - d_safe. For the forward channel modelled as a single
 * integrator (d_dot = -v) the CBF condition h_dot >= -alpha * h gives
 *     v <= alpha * (d_front - d_safe)
 * Reverse and turning commands pass through unchanged (no rear sensor:
 * reverse speed is capped separately). When d_front is invalid the filter
 * fails safe and forbids forward motion.
 */
typedef struct {
    float d_safe;          /* m */
    float alpha;           /* 1/s */
    float v_rev_max;       /* m/s, cap on blind reversing */
} cbf_t;

typedef struct {
    float v, w;
    bool limited;          /* true if the filter changed the command */
} cbf_out_t;

cbf_out_t cbf_filter(const cbf_t *c, float v_cmd, float w_cmd, float d_front, bool d_front_valid);

/* ---- Stall detector -------------------------------------------------------
 * A wheel counts as stalled while its PI output is near saturation but it
 * turns at under a quarter of the requested speed. Two or more wheels stalled
 * for longer than hold_s means the rover is pinned (typically a corner the
 * front sensor cannot see). Uses only the encoders and the PI output. */
typedef struct {
    float t_stalled;
    float hold_s;
} stall_det_t;

bool stall_update(stall_det_t *s, const float duty[4], const float w_meas[4], const float w_ref[4], float dt);

#ifdef __cplusplus
}
#endif
