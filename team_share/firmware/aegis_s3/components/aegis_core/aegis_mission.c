#include "aegis_mission.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void mission_init(mission_ctx_t *m, const mission_cfg_t *cfg)
{
    memset(m, 0, sizeof(*m));
    m->cfg = *cfg;
    m->state = MISSION_IDLE;
    m->last_action = (action_t)-1;
    m->last_audio_bin = 1;
    m->doa_t = INT64_MIN / 2;
    m->vis_t = INT64_MIN / 2;
    path_init(&m->path, cfg->path_spacing_m, cfg->path_loop_radius_m);
}

static void set_state(mission_ctx_t *m, mission_t s, int64_t now)
{
    if (s == m->state) return;
    m->state = s;
    m->t_state = now;
    m->escape_stage = 0;
    m->prim = PRIM_NONE;
    m->wp_valid = false;
}

/* turn_rad: magnitude for TURN actions (the full 90 deg unless the turn is
 * toward a fresh sound bearing, in which case it is the bearing itself) */
static void start_primitive(mission_ctx_t *m, const mission_in_t *in, action_t a, float turn_rad)
{
    m->prim_t0 = in->now_us;
    m->prim_t_blocked = 0;
    m->prim_x0 = in->x;
    m->prim_y0 = in->y;
    m->prim_th = in->theta;
    switch (a) {
    case ACT_FORWARD:
        m->prim = PRIM_FORWARD;
        break;
    case ACT_TURN_LEFT:
        m->prim = PRIM_TURN;
        m->prim_th = wrap_pi(in->theta + turn_rad);
        break;
    case ACT_TURN_RIGHT:
        m->prim = PRIM_TURN;
        m->prim_th = wrap_pi(in->theta - turn_rad);
        break;
    default:
        m->prim = PRIM_NONE;
        break;
    }
}

/*
 * Front/back resolution for a two-microphone array.
 * A reading beta (relative to the body) is consistent with two world bearings:
 * heading + beta (source in front) and heading + pi - beta (its mirror behind).
 * Both get a vote. When the rover turns, the true bearing stays fixed in the
 * world while the mirror moves, so the histogram peak converges on the truth.
 * Old votes decay so the estimate follows the rover as it moves.
 */
static void doa_vote(mission_ctx_t *m, float heading, float beta, int64_t now)
{
    const float bin_w = 2.0f * (float)M_PI / DOA_BINS;
    for (int i = 0; i < DOA_BINS; i++) m->doa_votes[i] *= 0.97f;
    float cand[2] = {wrap_pi(heading + beta), wrap_pi(heading + (float)M_PI - beta)};
    for (int k = 0; k < 2; k++) {
        int b = (int)floorf((cand[k] + (float)M_PI) / bin_w) % DOA_BINS;
        m->doa_votes[b] += 1.0f;
        m->doa_votes[(b + 1) % DOA_BINS] += 0.5f;
        m->doa_votes[(b + DOA_BINS - 1) % DOA_BINS] += 0.5f;
    }
    /* peak bin, refined by a circular weighted mean over +-2 bins */
    int best = 0;
    for (int i = 1; i < DOA_BINS; i++)
        if (m->doa_votes[i] > m->doa_votes[best]) best = i;
    float sx = 0, sy = 0;
    for (int d = -2; d <= 2; d++) {
        int i = (best + d + DOA_BINS) % DOA_BINS;
        float a = -(float)M_PI + (i + 0.5f) * bin_w;
        sx += m->doa_votes[i] * cosf(a);
        sy += m->doa_votes[i] * sinf(a);
    }
    m->doa_world = atan2f(sy, sx);
    m->doa_t = now;
}

/* Runs the current primitive. Returns true when it has finished. */
static bool run_primitive(mission_ctx_t *m, const mission_in_t *in, float *v, float *w, bool blocking)
{
    const mission_cfg_t *c = &m->cfg;
    int64_t now = in->now_us;
    *v = 0;
    *w = 0;
    switch (m->prim) {
    case PRIM_LISTEN:
        return now - m->prim_t0 > c->listen_us;     /* stopped: clean DOA frames */
    case PRIM_FORWARD: {
        *v = c->v_max;
        *w = 2.0f * wrap_pi(m->prim_th - in->theta);  /* heading hold */
        if (blocking) {
            if (!m->prim_t_blocked) m->prim_t_blocked = now;
        } else {
            m->prim_t_blocked = 0;
        }
        bool blocked = m->prim_t_blocked && now - m->prim_t_blocked > 500000;
        return hypotf(in->x - m->prim_x0, in->y - m->prim_y0) >= c->cell_m || blocked ||
               now - m->prim_t0 > 4000000;
    }
    case PRIM_TURN: {
        float e = wrap_pi(m->prim_th - in->theta);
        float wc = 3.0f * e;
        if (wc > c->w_max) wc = c->w_max;
        if (wc < -c->w_max) wc = -c->w_max;
        if (fabsf(wc) < 0.4f) wc = copysignf(0.4f, e);   /* overcome static friction */
        *w = wc;
        return fabsf(e) < c->turn_tol_rad || now - m->prim_t0 > 4000000;
    }
    default:
        return true;
    }
}

void mission_step(mission_ctx_t *m, const mission_in_t *in, mission_out_t *out)
{
    const mission_cfg_t *c = &m->cfg;
    const int64_t now = in->now_us;
    float v = 0, w = 0;
    memset(out, 0, sizeof(*out));

    /* latch the newest sensor messages */
    /* only readings taken while stopped vote: motor noise reaches both mics
     * with equal delay and would pull every estimate towards "ahead" */
    bool still = m->state == MISSION_IDLE || (m->state == MISSION_SEARCH && m->prim == PRIM_LISTEN);
    if (in->doa_new && still) doa_vote(m, in->theta, in->doa_bearing, now);
    if (in->vis_new) {
        m->vis_visible = in->vis_visible;
        m->vis_ex = in->vis_ex;
        m->vis_area = in->vis_area;
        m->vis_t = now;
    }
    bool vis_fresh = m->vis_visible && (now - m->vis_t) < c->vision_stale_us;

    if (in->fault) {
        set_state(m, MISSION_FAULT, now);
    } else if (in->start_pressed && m->state != MISSION_IDLE && m->state != MISSION_FAULT) {
        set_state(m, MISSION_IDLE, now);
        return;
    }

    /* Escape when pinned: back off 8 cm, turn 40 deg toward the freer side */
    bool moving_state = m->state == MISSION_SEARCH || m->state == MISSION_APPROACH || m->state == MISSION_RETURN;
    if (moving_state && m->escape_stage == 0 && in->stalled) {
        m->escape_stage = 1;
        m->esc_x0 = in->x;
        m->esc_y0 = in->y;
        m->esc_t0 = now;
        m->escapes++;
        bool left_freer = (in->tof_valid[TOF_LEFT] ? in->tof_m[TOF_LEFT] : 0.0f) >
                          (in->tof_valid[TOF_RIGHT] ? in->tof_m[TOF_RIGHT] : 0.0f);
        m->esc_th = wrap_pi(in->theta + (left_freer ? 0.7f : -0.7f));
    }
    if (moving_state && m->escape_stage == 1) {
        v = -0.08f;
        if (hypotf(in->x - m->esc_x0, in->y - m->esc_y0) > 0.08f || now - m->esc_t0 > 1500000) {
            m->escape_stage = 2;
            m->esc_t0 = now;
        }
    } else if (moving_state && m->escape_stage == 2) {
        float e = wrap_pi(m->esc_th - in->theta);
        w = fmaxf(-c->w_max, fminf(c->w_max, 3.0f * e));
        if (fabsf(w) < 0.4f) w = copysignf(0.4f, e);
        if (fabsf(e) < c->turn_tol_rad || now - m->esc_t0 > 3000000) {
            m->escape_stage = 0;
            m->prim = PRIM_NONE;          /* SEARCH: listen and decide afresh */
            m->wp_valid = false;          /* RETURN: re-aim at the next crumb */
        }
    } else
    switch (m->state) {
    case MISSION_IDLE:
        if (in->start_pressed) {
            out->reset_odom = true;
            path_init(&m->path, c->path_spacing_m, c->path_loop_radius_m);
            out->servo_us = c->servo_open_us;
            set_state(m, MISSION_SEARCH, now);
        }
        break;

    case MISSION_SEARCH: {
        if (in->vbat < c->vbat_low_v) {
            set_state(m, MISSION_RETURN, now);
            break;
        }
        if (vis_fresh) {
            set_state(m, MISSION_APPROACH, now);
            break;
        }
        /* a FORWARD cell ends early if the safety filter has held it near 0 for 0.5 s */
        bool blocking = false;
        if (m->prim == PRIM_FORWARD) {
            cbf_out_t cb = cbf_filter(&c->cbf, c->v_max, 0, in->tof_m[TOF_FRONT], in->tof_valid[TOF_FRONT]);
            blocking = cb.v < 0.02f;
        }
        if (!run_primitive(m, in, &v, &w, blocking)) break;
        if (m->prim != PRIM_LISTEN) {
            m->prim = PRIM_LISTEN;
            m->prim_t0 = now;
            v = w = 0;
            break;
        }
        bool doa_ok = (now - m->doa_t) < c->doa_stale_us;
        float rel = wrap_pi(m->doa_world - in->theta);
        policy_state_t st = policy_encode(&c->policy,
                                          in->tof_m[TOF_FRONT], in->tof_valid[TOF_FRONT],
                                          in->tof_m[TOF_LEFT], in->tof_valid[TOF_LEFT],
                                          in->tof_m[TOF_RIGHT], in->tof_valid[TOF_RIGHT],
                                          rel, doa_ok, m->last_audio_bin, vis_fresh);
        m->last_audio_bin = st.audio_bin;
        m->last_action = policy_select(&c->policy, &st, &m->last_src);
        m->decisions++;
        /* A fixed 90 deg turn toward a source 30 deg off overshoots to 60 deg
         * the other way and the rover zig-zags without closing in (seen in
         * the SIL sim). Turn by the heard bearing when the turn goes that way. */
        float turn = c->turn_rad;
        bool toward_sound = doa_ok && ((m->last_action == ACT_TURN_LEFT && rel > 0) ||
                                       (m->last_action == ACT_TURN_RIGHT && rel < 0));
        if (toward_sound && !st.front_blocked) turn = fminf(c->turn_rad, fmaxf(0.35f, fabsf(rel)));
        start_primitive(m, in, m->last_action, turn);
        break;
    }

    case MISSION_APPROACH:
        if (!vis_fresh) {
            if (now - m->vis_t > 1000000) set_state(m, MISSION_SEARCH, now);
            break;   /* hold still briefly while the camera re-acquires */
        }
        if (m->vis_area >= c->dock_area_frac && fabsf(m->vis_ex) < c->dock_ex_tol) {
            set_state(m, MISSION_GRASP, now);
            break;
        }
        w = -1.5f * m->vis_ex;                           /* +ex = target right -> turn right */
        v = 0.15f * (1.0f - fminf(1.0f, 2.0f * fabsf(m->vis_ex)));
        break;

    case MISSION_GRASP:
        out->servo_us = c->servo_grip_us;
        if (now - m->t_state > c->grip_dwell_us) set_state(m, MISSION_RETURN, now);
        break;

    case MISSION_RETURN: {
        if (!m->wp_valid) {
            m->wp_valid = path_pop(&m->path, &m->wp_x, &m->wp_y);
            if (!m->wp_valid) {
                m->wp_x = m->wp_y = 0.0f;                /* path exhausted: home */
                m->wp_valid = true;
            }
        }
        if (hypotf(in->x, in->y) < c->home_tol_m && m->path.count == 0) {
            set_state(m, MISSION_RELEASE, now);
            break;
        }
        if (hypotf(m->wp_x - in->x, m->wp_y - in->y) < 0.08f) {
            m->wp_valid = false;
            break;
        }
        goto_point(in->x, in->y, in->theta, m->wp_x, m->wp_y, c->v_max, c->w_max, 2.5f, 0.6f, &v, &w);
        break;
    }

    case MISSION_RELEASE:
        out->servo_us = c->servo_open_us;
        if (now - m->t_state > c->grip_dwell_us) set_state(m, MISSION_DONE, now);
        break;

    case MISSION_DONE:
    case MISSION_FAULT:
    default:
        break;
    }

    if (m->state == MISSION_SEARCH || m->state == MISSION_APPROACH) {
        path_record(&m->path, in->x, in->y);
    }

    out->drive = m->state == MISSION_SEARCH || m->state == MISSION_APPROACH || m->state == MISSION_RETURN;
    cbf_out_t safe = cbf_filter(&c->cbf, v, w, in->tof_m[TOF_FRONT], in->tof_valid[TOF_FRONT]);
    out->v = out->drive ? safe.v : 0.0f;
    out->w = out->drive ? safe.w : 0.0f;
    out->cbf_limited = safe.limited;
}
