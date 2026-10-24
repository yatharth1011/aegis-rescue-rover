/*
 * Thin C shim so the Python simulator can call the firmware's portable core
 * (components/aegis_core) with plain pointers. Built as libaegis_core.dylib by
 * core_ffi.py; the algorithms are the firmware's own, unchanged.
 */
#include <stdlib.h>

#include "aegis_control.h"
#include "aegis_dsp.h"
#include "aegis_params.h"

typedef struct {
    gcc_phat_t g;
    bandpass_t bl, br;
    noise_floor_t nf;
    float c;
} doa_t;

void *doa_new(void)
{
    doa_t *d = calloc(1, sizeof(doa_t));
    d->c = speed_of_sound(AUDIO_TEMP_C);
    gcc_phat_init(&d->g, AUDIO_FS, AUDIO_FRAME, MIC_SPACING_M / d->c, AUDIO_F_LO, AUDIO_F_HI);
    bandpass_init(&d->bl, AUDIO_FS, AUDIO_F_LO, AUDIO_F_HI);
    bandpass_init(&d->br, AUDIO_FS, AUDIO_F_LO, AUDIO_F_HI);
    d->nf.floor = -1.0f;
    return d;
}

void doa_free(void *p) { free(p); }

/* One AUDIO_FRAME of stereo audio (modified in place: band-passed).
 * out: bearing_rad, lag_samples, psr, band_db, valid */
void doa_frame(void *p, float *l, float *r, float *out)
{
    doa_t *d = p;
    bandpass_process(&d->bl, l, AUDIO_FRAME);
    bandpass_process(&d->br, r, AUDIO_FRAME);
    float e = 0;
    for (int i = 0; i < AUDIO_FRAME; i++) e += 0.5f * (l[i] * l[i] + r[i] * r[i]);
    float band_db = noise_floor_update(&d->nf, e / AUDIO_FRAME);
    gcc_result_t g = gcc_phat_run(&d->g, l, r);
    out[0] = doa_bearing_from_lag(g.lag_samples, AUDIO_FS, MIC_SPACING_M, d->c);
    out[1] = g.lag_samples;
    out[2] = g.psr;
    out[3] = band_db;
    out[4] = doa_gate(g.psr, band_db, AUDIO_PSR_GATE, AUDIO_SNR_GATE_DB) ? 1.0f : 0.0f;
}

/* CBF safety filter. out: v, w, limited */
void safety_filter(float v, float w, float d_front, int valid, float d_safe, float alpha, float v_rev, float *out)
{
    cbf_t c = {d_safe, alpha, v_rev};
    cbf_out_t o = cbf_filter(&c, v, w, d_front, valid != 0);
    out[0] = o.v;
    out[1] = o.w;
    out[2] = o.limited ? 1.0f : 0.0f;
}

typedef struct {
    pi_ctrl_t pi[4];
    stall_det_t stall;
    kinematics_t kin;
} drive_t;

void *drive_new(void)
{
    drive_t *d = calloc(1, sizeof(drive_t));
    for (int i = 0; i < 4; i++) pi_init(&d->pi[i], PI_KP, PI_KI, 1.0f / WHEEL_W_NOLOAD, -1.0f, 1.0f);
    d->stall.hold_s = STALL_HOLD_S;
    d->kin = (kinematics_t){WHEEL_RADIUS_M, TRACK_M, TRACK_SKID_GAIN};
    return d;
}

void drive_free(void *p) { free(p); }

/* Body command -> 4 wheel duties via the firmware PI loops. Returns stalled flag.
 * w_meas: measured wheel speeds (FL, RL, FR, RR); duty_out: 4 duties. */
int drive_step(void *p, float v, float w, const float *w_meas, float vbat, float dt, float *duty_out)
{
    drive_t *d = p;
    float wl, wr;
    kin_inverse(&d->kin, v, w, &wl, &wr);
    const float ref[4] = {wl, wl, wr, wr};
    float kff = 12.0f / (vbat * WHEEL_W_NOLOAD);
    for (int i = 0; i < 4; i++) {
        if (v == 0.0f && w == 0.0f) {
            pi_reset(&d->pi[i]);
            duty_out[i] = 0.0f;
        } else {
            d->pi[i].kff = kff;
            duty_out[i] = pi_update(&d->pi[i], ref[i], w_meas[i], dt);
        }
    }
    return stall_update(&d->stall, duty_out, w_meas, ref, dt) ? 1 : 0;
}

void kin_fwd(float wl, float wr, float *out)
{
    kinematics_t k = {WHEEL_RADIUS_M, TRACK_M, TRACK_SKID_GAIN};
    kin_forward(&k, wl, wr, &out[0], &out[1]);
}
