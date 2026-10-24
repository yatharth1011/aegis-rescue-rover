#include "aegis_policy.h"
#include "aegis_control.h"

#include <math.h>
#include <string.h>

/* Single source of truth: project/aegis_q_table.h, exported by train_rl_policy.py */
#include "aegis_q_table.h"

policy_state_t policy_encode(const policy_cfg_t *cfg,
                             float d_front, bool v_front,
                             float d_left, bool v_left,
                             float d_right, bool v_right,
                             float bearing, bool bearing_valid, int last_audio_bin,
                             bool target_visible)
{
    policy_state_t s;
    s.front_blocked = (!v_front || d_front < cfg->block_front_m) ? 1 : 0;
    s.left_blocked = (!v_left || d_left < cfg->block_side_m) ? 1 : 0;
    s.right_blocked = (!v_right || d_right < cfg->block_side_m) ? 1 : 0;
    if (bearing_valid) {
        s.audio_bin = bearing > cfg->audio_ahead_rad ? 0 : (bearing < -cfg->audio_ahead_rad ? 2 : 1);
    } else {
        s.audio_bin = (last_audio_bin >= 0 && last_audio_bin <= 2) ? (uint8_t)last_audio_bin : 1;
    }
    s.target_visible = target_visible ? 1 : 0;
    return s;
}

/* Deterministic rule used where the table has no trained answer. */
static action_t fallback(const policy_state_t *s)
{
    if (!s->front_blocked) {
        if (s->audio_bin == 0 && !s->left_blocked) return ACT_TURN_LEFT;
        if (s->audio_bin == 2 && !s->right_blocked) return ACT_TURN_RIGHT;
        return ACT_FORWARD;
    }
    /* front blocked: turn toward the sound if that side is open, else any open side */
    if (s->audio_bin == 0 && !s->left_blocked) return ACT_TURN_LEFT;
    if (s->audio_bin == 2 && !s->right_blocked) return ACT_TURN_RIGHT;
    if (!s->left_blocked) return ACT_TURN_LEFT;
    if (!s->right_blocked) return ACT_TURN_RIGHT;
    return ACT_TURN_LEFT;   /* boxed in: rotate in place, never reverse blind */
}

action_t policy_select(const policy_cfg_t *cfg, const policy_state_t *s, policy_src_t *src)
{
    if (!cfg->use_table) {
        *src = POLICY_SRC_RULE;
        return fallback(s);
    }
    const float *q = Q_TABLE[s->front_blocked][s->left_blocked][s->right_blocked]
                            [s->audio_bin][s->target_visible];
    int best = 0;
    bool visited = false;
    for (int a = 0; a < ACT_COUNT; a++) {
        if (fabsf(q[a]) > 1e-6f) visited = true;
        if (q[a] > q[best]) best = a;
    }
    if (!visited) {
        *src = POLICY_SRC_UNVISITED;
        return fallback(s);
    }
    if ((best == ACT_FORWARD && s->front_blocked) || best == ACT_REVERSE) {
        /* REVERSE is never executed autonomously: there is no rear sensor */
        *src = POLICY_SRC_UNSAFE;
        return fallback(s);
    }
    if (best == ACT_CLAMP) {
        *src = POLICY_SRC_CLAMP_REMAP;
        return fallback(s);
    }
    *src = POLICY_SRC_TABLE;
    return (action_t)best;
}

/* ---- Return path ---------------------------------------------------------- */
void path_init(path_t *p, float spacing, float loop_radius)
{
    memset(p, 0, sizeof(*p));
    p->spacing = spacing;
    p->loop_radius = loop_radius;
}

static float dist2(float ax, float ay, float bx, float by)
{
    float dx = ax - bx, dy = ay - by;
    return dx * dx + dy * dy;
}

void path_record(path_t *p, float x, float y)
{
    if (p->count > 0 && dist2(x, y, p->x[p->count - 1], p->y[p->count - 1]) < p->spacing * p->spacing) {
        return;
    }
    /* Loop pruning: if we are back near an older crumb (excluding the last
     * two, which are always close), drop everything recorded after it. */
    float r2 = p->loop_radius * p->loop_radius;
    for (int i = 0; i < p->count - 2; i++) {
        if (dist2(x, y, p->x[i], p->y[i]) < r2) {
            p->count = i + 1;
            return;
        }
    }
    if (p->count == PATH_MAX_POINTS) {
        /* keep the start (home) and thin the rest by half */
        int j = 1;
        for (int i = 2; i < p->count; i += 2) {
            p->x[j] = p->x[i];
            p->y[j] = p->y[i];
            j++;
        }
        p->count = j;
        p->spacing *= 2.0f;
        p->overflow = true;
    }
    p->x[p->count] = x;
    p->y[p->count] = y;
    p->count++;
}

bool path_pop(path_t *p, float *x, float *y)
{
    if (p->count == 0) return false;
    p->count--;
    *x = p->x[p->count];
    *y = p->y[p->count];
    return true;
}

void goto_point(float x, float y, float theta, float tx, float ty,
                float v_max, float w_max, float k_w, float turn_in_place_rad,
                float *v, float *w)
{
    float e = wrap_pi(atan2f(ty - y, tx - x) - theta);
    float wc = k_w * e;
    if (wc > w_max) wc = w_max;
    if (wc < -w_max) wc = -w_max;
    *w = wc;
    *v = fabsf(e) > turn_in_place_rad ? 0.0f : v_max * cosf(e);
}
