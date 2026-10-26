/*
 * On-robot reflexes that must never depend on Wi-Fi (same rules as
 * simulation3d/robot.py, which the 3D evaluation ran with):
 *
 *   table edge  - a front cliff sensor backs the rover off, a rear one drives
 *                 it forward, for REFLEX_DIST_M; for NO_TURN_S after any edge
 *                 event turning is forbidden and so is motion toward the edge
 *   stall       - when the stall detector trips, back off ESC_DIST_M
 *
 * The output replaces the commanded (v, w) while a reflex is active. The CBF
 * filter is applied after this, as in the simulator.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* cliff sensor bit mask (PCF8574 P0..P4 after polarity correction) */
#define CLIFF_FL (1u << 0)
#define CLIFF_FR (1u << 1)
#define CLIFF_R  (1u << 2)
#define CLIFF_RL (1u << 3)
#define CLIFF_RR (1u << 4)
#define CLIFF_FRONT_MASK (CLIFF_FL | CLIFF_FR)
#define CLIFF_REAR_MASK  (CLIFF_R | CLIFF_RL | CLIFF_RR)

typedef struct {
    float dist_m;          /* edge reflex travel */
    float v_mps;           /* edge reflex speed */
    float no_turn_s;       /* window after an edge event */
    float esc_dist_m;      /* stall escape travel (reverse) */
    float esc_v_mps;
} reflex_cfg_t;

typedef struct {
    reflex_cfg_t cfg;
    float backoff_m, fwd_m, esc_m;   /* remaining travel of each manoeuvre */
    float since_cliff_s;
    bool cliff_front;
    bool stalled_prev;
    uint32_t edge_events, stall_events;
} reflex_t;

typedef struct {
    float v, w;
    bool active;           /* a reflex changed the command */
} reflex_out_t;

void reflex_init(reflex_t *r, const reflex_cfg_t *cfg);
reflex_out_t reflex_step(reflex_t *r, float v, float w, uint8_t cliff_mask, bool stalled, float dt);

#ifdef __cplusplus
}
#endif
