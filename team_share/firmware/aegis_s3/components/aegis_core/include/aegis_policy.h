/*
 * Discrete navigation policy (tabular Q-learning, trained in train_rl_policy.py)
 * plus the return-path recorder. Portable C, unit-tested on the host.
 *
 * State encoding must match RescueRoverSim._get_state():
 *   (front_blocked, left_blocked, right_blocked, audio_bin, target_visible)
 *   audio_bin: 0 = target to the LEFT, 1 = ahead, 2 = RIGHT  (|angle| <= 0.3 rad is "ahead")
 * Actions: 0 FORWARD, 1 TURN_LEFT, 2 TURN_RIGHT, 3 REVERSE, 4 ENGAGE_CLAMP
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ACT_FORWARD = 0,
    ACT_TURN_LEFT = 1,
    ACT_TURN_RIGHT = 2,
    ACT_REVERSE = 3,
    ACT_CLAMP = 4,
    ACT_COUNT = 5,
} action_t;

typedef struct {
    uint8_t front_blocked, left_blocked, right_blocked;
    uint8_t audio_bin;
    uint8_t target_visible;
} policy_state_t;

typedef struct {
    float block_front_m;   /* front blocked if d < this ("next cell occupied") */
    float block_side_m;
    float audio_ahead_rad; /* |bearing| <= this -> bin 1 (sim uses 0.3) */
    bool use_table;        /* false: deterministic rule only (A/B baseline) */
} policy_cfg_t;

typedef enum {
    POLICY_SRC_TABLE = 0,      /* greedy action from the Q-table            */
    POLICY_SRC_UNVISITED,      /* table row never trained -> fallback rule  */
    POLICY_SRC_UNSAFE,         /* table chose FORWARD into a blocked cell   */
    POLICY_SRC_CLAMP_REMAP,    /* CLAMP is handled by the mission FSM       */
    POLICY_SRC_RULE,           /* table disabled in the configuration       */
} policy_src_t;

/* Invalid range readings count as blocked (fail safe). bearing: + = left. */
policy_state_t policy_encode(const policy_cfg_t *cfg,
                             float d_front, bool v_front,
                             float d_left, bool v_left,
                             float d_right, bool v_right,
                             float bearing, bool bearing_valid, int last_audio_bin,
                             bool target_visible);

action_t policy_select(const policy_cfg_t *cfg, const policy_state_t *s, policy_src_t *src);

/* ---- Return path: breadcrumbs with loop pruning ------------------------ */
#define PATH_MAX_POINTS 256

typedef struct {
    float x[PATH_MAX_POINTS], y[PATH_MAX_POINTS];
    int count;
    float spacing;         /* m between recorded points */
    float loop_radius;     /* m: revisiting an older point closer than this cuts the loop */
    bool overflow;
} path_t;

void path_init(path_t *p, float spacing, float loop_radius);
/* Record the current pose. Keeps the path loop-free, so retracing it backwards
 * is the shortest route home along ground already proven traversable. */
void path_record(path_t *p, float x, float y);
/* Pop the next waypoint for the return trip (newest first). False when empty. */
bool path_pop(path_t *p, float *x, float *y);

/* Go-to-point steering: turn in place if heading error > turn_in_place_rad */
void goto_point(float x, float y, float theta, float tx, float ty,
                float v_max, float w_max, float k_w, float turn_in_place_rad,
                float *v, float *w);

#ifdef __cplusplus
}
#endif
