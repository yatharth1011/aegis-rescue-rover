#include "aegis_control.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

void pi_init(pi_ctrl_t *c, float kp, float ki, float kff, float out_min, float out_max)
{
    c->kp = kp;
    c->ki = ki;
    c->kff = kff;
    c->out_min = out_min;
    c->out_max = out_max;
    c->integ = 0.0f;
}

void pi_reset(pi_ctrl_t *c)
{
    c->integ = 0.0f;
}

float pi_update(pi_ctrl_t *c, float ref, float meas, float dt)
{
    float e = ref - meas;
    float u_noint = c->kff * ref + c->kp * e;
    float u = u_noint + c->ki * c->integ;
    bool sat_hi = u > c->out_max, sat_lo = u < c->out_min;
    /* integrate only if it does not drive further into saturation */
    if (!((sat_hi && e > 0.0f) || (sat_lo && e < 0.0f))) {
        c->integ += e * dt;
    }
    return clampf(u_noint + c->ki * c->integ, c->out_min, c->out_max);
}

void kin_inverse(const kinematics_t *k, float v, float w, float *wl, float *wr)
{
    float b = k->track * k->track_eff_gain;
    *wl = (v - 0.5f * b * w) / k->wheel_radius;
    *wr = (v + 0.5f * b * w) / k->wheel_radius;
}

void kin_forward(const kinematics_t *k, float wl, float wr, float *v, float *w)
{
    float b = k->track * k->track_eff_gain;
    *v = 0.5f * k->wheel_radius * (wl + wr);
    *w = k->wheel_radius * (wr - wl) / b;
}

float wrap_pi(float a)
{
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void odom_reset(odom_t *o, float gyro_weight)
{
    o->x = o->y = o->theta = 0.0f;
    o->gyro_weight = clampf(gyro_weight, 0.0f, 1.0f);
}

void odom_update(odom_t *o, float v, float w_enc, float w_gyro, float dt)
{
    float w = o->gyro_weight * w_gyro + (1.0f - o->gyro_weight) * w_enc;
    /* midpoint integration */
    float th_mid = o->theta + 0.5f * w * dt;
    o->x += v * cosf(th_mid) * dt;
    o->y += v * sinf(th_mid) * dt;
    o->theta = wrap_pi(o->theta + w * dt);
}

cbf_out_t cbf_filter(const cbf_t *c, float v_cmd, float w_cmd, float d_front, bool d_front_valid)
{
    cbf_out_t out = {v_cmd, w_cmd, false};
    if (v_cmd > 0.0f) {
        float v_max = d_front_valid ? c->alpha * (d_front - c->d_safe) : 0.0f;
        if (v_max < 0.0f) v_max = 0.0f;
        if (v_cmd > v_max) {
            out.v = v_max;
            out.limited = true;
        }
    } else if (v_cmd < -c->v_rev_max) {
        out.v = -c->v_rev_max;
        out.limited = true;
    }
    return out;
}

bool stall_update(stall_det_t *s, const float duty[4], const float w_meas[4], const float w_ref[4], float dt)
{
    int n = 0;
    for (int i = 0; i < 4; i++) {
        if (fabsf(duty[i]) > 0.7f && fabsf(w_ref[i]) > 2.0f && fabsf(w_meas[i]) < 0.25f * fabsf(w_ref[i])) n++;
    }
    s->t_stalled = n >= 2 ? s->t_stalled + dt : 0.0f;
    return s->t_stalled > s->hold_s;
}
