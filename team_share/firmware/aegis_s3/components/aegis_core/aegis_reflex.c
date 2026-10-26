#include "aegis_reflex.h"

#include <math.h>

void reflex_init(reflex_t *r, const reflex_cfg_t *cfg)
{
    *r = (reflex_t){.cfg = *cfg, .since_cliff_s = 1e3f};
}

reflex_out_t reflex_step(reflex_t *r, float v, float w, uint8_t cliff_mask, bool stalled, float dt)
{
    const reflex_cfg_t *c = &r->cfg;
    reflex_out_t o = {v, w, false};
    bool front = (cliff_mask & CLIFF_FRONT_MASK) != 0;
    bool rear = (cliff_mask & CLIFF_REAR_MASK) != 0;

    /* start a manoeuvre only when none is running (as in robot.py) */
    if (front && r->backoff_m <= 0 && r->fwd_m <= 0) {
        r->backoff_m = c->dist_m;
        r->edge_events++;
    } else if (rear && r->fwd_m <= 0 && r->backoff_m <= 0) {
        r->fwd_m = c->dist_m;
        r->edge_events++;
    }
    if (front || rear) {
        r->since_cliff_s = 0;
        r->cliff_front = front;
    } else {
        r->since_cliff_s += dt;
    }

    if (r->since_cliff_s < c->no_turn_s) {      /* no turning, no motion toward the edge */
        if (o.w != 0) o.active = true;
        o.w = 0;
        float lim = r->cliff_front ? fminf(o.v, 0.0f) : fmaxf(o.v, 0.0f);
        if (lim != o.v) o.active = true;
        o.v = lim;
    }
    if (r->backoff_m > 0) {
        o.v = -c->v_mps;
        o.w = 0;
        o.active = true;
        r->backoff_m -= c->v_mps * dt;
    } else if (r->fwd_m > 0) {
        o.v = c->v_mps;
        o.w = 0;
        o.active = true;
        r->fwd_m -= c->v_mps * dt;
    }

    if (stalled && !r->stalled_prev) {
        r->esc_m = c->esc_dist_m;
        r->stall_events++;
    }
    r->stalled_prev = stalled;
    if (r->esc_m > 0) {
        o.v = -c->esc_v_mps;
        o.w = 0;
        o.active = true;
        r->esc_m -= c->esc_v_mps * dt;
    }
    return o;
}
