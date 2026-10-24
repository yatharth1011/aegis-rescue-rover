/*
 * Mission state machine, shared by the firmware (Task_NavControl) and the
 * software-in-the-loop simulator (sim/). Portable C: everything it needs
 * arrives in mission_in_t, everything it decides leaves in mission_out_t.
 *
 *   IDLE -(start)-> SEARCH <-> APPROACH -> GRASP -> RETURN -> RELEASE -> DONE
 *   any fault -> FAULT (latched)        start pressed while running -> IDLE
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "aegis_control.h"
#include "aegis_policy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MISSION_IDLE = 0,
    MISSION_CALIBRATE,
    MISSION_SEARCH,
    MISSION_APPROACH,
    MISSION_GRASP,
    MISSION_RETURN,
    MISSION_RELEASE,
    MISSION_DONE,
    MISSION_FAULT,
} mission_t;

enum { TOF_FRONT = 0, TOF_LEFT = 1, TOF_RIGHT = 2 };

typedef struct {
    float v_max, w_max;
    float cell_m, turn_rad, turn_tol_rad;
    int64_t listen_us;
    int64_t doa_stale_us, vision_stale_us;
    float dock_area_frac, dock_ex_tol;
    float path_spacing_m, path_loop_radius_m, home_tol_m;
    float vbat_low_v;
    uint32_t servo_open_us, servo_grip_us;
    int64_t grip_dwell_us;
    cbf_t cbf;
    policy_cfg_t policy;
} mission_cfg_t;

typedef struct {
    int64_t now_us;
    float x, y, theta;             /* odometry estimate */
    float tof_m[3];
    bool tof_valid[3];
    bool doa_new;                  /* a gated DOA estimate arrived this tick */
    float doa_bearing;             /* rad, + = left, relative to the body at measurement */
    bool vis_new;
    bool vis_visible;
    float vis_ex, vis_area;
    float vbat;
    bool start_pressed;            /* rising edge */
    bool stalled;                  /* from stall_update(): pinned against something */
    uint32_t fault;                /* non-zero latches FAULT */
} mission_in_t;

typedef struct {
    float v, w;                    /* body command after the CBF */
    bool drive;                    /* enable motor drivers */
    bool cbf_limited;
    uint32_t servo_us;             /* 0 = leave unchanged */
    bool reset_odom;               /* caller must zero its odometry this tick */
} mission_out_t;

typedef enum { PRIM_NONE, PRIM_LISTEN, PRIM_FORWARD, PRIM_TURN } prim_kind_t;

#define DOA_BINS 36                /* 10 deg world-bearing histogram */

typedef struct {
    mission_cfg_t cfg;
    mission_t state;
    int64_t t_state;
    prim_kind_t prim;
    float prim_x0, prim_y0, prim_th;
    int64_t prim_t0, prim_t_blocked;
    action_t last_action;
    policy_src_t last_src;
    int last_audio_bin;
    float doa_world;               /* resolved world-frame bearing (rad) */
    int64_t doa_t;
    float doa_votes[DOA_BINS];     /* front/back resolution, see aegis_mission.c */
    bool vis_visible;
    float vis_ex, vis_area;
    int64_t vis_t;
    path_t path;
    float wp_x, wp_y;
    bool wp_valid;
    uint32_t decisions;
    int escape_stage;              /* 0 none, 1 back off, 2 turn away */
    float esc_x0, esc_y0, esc_th;
    int64_t esc_t0;
    uint32_t escapes;
} mission_ctx_t;

void mission_init(mission_ctx_t *m, const mission_cfg_t *cfg);
void mission_step(mission_ctx_t *m, const mission_in_t *in, mission_out_t *out);

#ifdef __cplusplus
}
#endif
