/*
 * Host unit tests for components/aegis_core.  Run:  make -C test/host
 */
#include "aegis_control.h"
#include "aegis_dsp.h"
#include "aegis_policy.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                                              \
    do {                                                              \
        if (cond) {                                                   \
            g_pass++;                                                 \
        } else {                                                      \
            g_fail++;                                                 \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);             \
            printf(__VA_ARGS__);                                      \
            printf("\n");                                             \
        }                                                             \
    } while (0)

/* deterministic noise */
static uint32_t rng = 12345u;
static float frand(void)
{
    rng = rng * 1664525u + 1013904223u;
    return ((rng >> 8) / 16777216.0f) * 2.0f - 1.0f;
}

#define FS 48000.0f
#define N 1024
#define MIC_D 0.100f

static float rms(const float *x, int n)
{
    double s = 0;
    for (int i = 0; i < n; i++) s += (double)x[i] * x[i];
    return (float)sqrt(s / n);
}

static void test_bandpass(void)
{
    printf("bandpass 300-3400 Hz\n");
    const float freqs[] = {60.0f, 1000.0f, 12000.0f};
    float gain[3];
    for (int f = 0; f < 3; f++) {
        bandpass_t bp;
        bandpass_init(&bp, FS, 300.0f, 3400.0f);
        static float x[8192];
        for (int i = 0; i < 8192; i++) x[i] = sinf(2.0f * (float)M_PI * freqs[f] * i / FS);
        bandpass_process(&bp, x, 8192);
        gain[f] = rms(x + 4096, 4096) / (float)(1.0 / sqrt(2.0));
    }
    CHECK(gain[1] > 0.9f && gain[1] < 1.05f, "1 kHz gain %.3f", gain[1]);
    CHECK(gain[0] < 0.02f, "60 Hz gain %.4f (expect < 0.02, 4th order ~ -48 dB)", gain[0]);
    CHECK(gain[2] < 0.02f, "12 kHz gain %.4f", gain[2]);
    printf("  gains: 60 Hz %.4f, 1 kHz %.3f, 12 kHz %.4f\n", gain[0], gain[1], gain[2]);
}

static void test_fft(void)
{
    printf("fft round trip\n");
    static fft_plan_t p;
    CHECK(fft_plan_init(&p, 256), "plan");
    static float d[512], o[512];
    for (int i = 0; i < 512; i++) o[i] = d[i] = frand();
    fft_run(&p, d, 0);
    fft_run(&p, d, 1);
    float err = 0;
    for (int i = 0; i < 512; i++) err = fmaxf(err, fabsf(d[i] / 256.0f - o[i]));
    CHECK(err < 1e-5f, "round trip max err %g", err);
    /* single tone lands in its bin */
    for (int i = 0; i < 256; i++) {
        d[2 * i] = cosf(2.0f * (float)M_PI * 10 * i / 256.0f);
        d[2 * i + 1] = 0;
    }
    fft_run(&p, d, 0);
    CHECK(fabsf(d[20] - 128.0f) < 1e-3f, "bin 10 re %.4f", d[20]);
    CHECK(!fft_plan_init(&p, 1000), "non power of two rejected");
}

/* Band-limited source sampled at 8x, so fractional delays are exact */
#define OS 8
static void make_pair(float delay_samples, float snr_db, float *l, float *r)
{
    static float hi[(N + 64) * OS];
    static bandpass_t bp;
    bandpass_init(&bp, FS * OS, 400.0f, 3000.0f);
    int nh = (N + 64) * OS;
    for (int i = 0; i < nh; i++) hi[i] = frand();
    bandpass_process(&bp, hi, nh);
    int shift = (int)lroundf(delay_samples * OS);
    float noise = powf(10.0f, -snr_db / 20.0f) * rms(hi + nh / 2, nh / 2);
    for (int i = 0; i < N; i++) {
        int base = (i + 32) * OS;
        l[i] = hi[base] + noise * frand() * 1.732f;
        r[i] = hi[base - shift] + noise * frand() * 1.732f;   /* r lags l by delay */
    }
}

static void test_gcc_phat(void)
{
    printf("gcc-phat delay recovery (fs 48 kHz, d 100 mm, N 1024)\n");
    static gcc_phat_t g;
    float c = speed_of_sound(25.0f);
    CHECK(gcc_phat_init(&g, FS, N, MIC_D / c, 300.0f, 3400.0f), "init");
    printf("  search window +-%d samples\n", g.max_lag);
    static float l[N], r[N];
    float worst = 0.0f;
    for (float dly = -13.0f; dly <= 13.0f; dly += 1.0f) {
        make_pair(dly, 40.0f, l, r);
        gcc_result_t res = gcc_phat_run(&g, l, r);
        worst = fmaxf(worst, fabsf(res.lag_samples - dly));
    }
    CHECK(worst < 0.25f, "integer delays: worst error %.3f samples", worst);
    printf("  integer delays -13..13: worst error %.3f samples\n", worst);

    float worst_f = 0.0f;
    for (float dly = -6.375f; dly <= 6.5f; dly += 1.375f) {
        make_pair(dly, 40.0f, l, r);
        gcc_result_t res = gcc_phat_run(&g, l, r);
        worst_f = fmaxf(worst_f, fabsf(res.lag_samples - dly));
    }
    CHECK(worst_f < 0.35f, "fractional delays: worst error %.3f samples", worst_f);
    printf("  fractional delays: worst error %.3f samples\n", worst_f);

    make_pair(5.0f, 6.0f, l, r);
    gcc_result_t noisy = gcc_phat_run(&g, l, r);
    CHECK(fabsf(noisy.lag_samples - 5.0f) < 0.6f, "6 dB SNR lag %.2f", noisy.lag_samples);
    printf("  6 dB SNR: lag %.2f (true 5.00), psr %.1f\n", noisy.lag_samples, noisy.psr);

    for (int i = 0; i < N; i++) {
        l[i] = frand();
        r[i] = frand();
    }
    gcc_result_t unc = gcc_phat_run(&g, l, r);
    printf("  uncorrelated noise: psr %.1f (firmware gate: psr >= 2.0)\n", unc.psr);
    CHECK(unc.psr < noisy.psr, "uncorrelated psr %.1f should be below correlated %.1f", unc.psr, noisy.psr);

    /* bearing: a source 30 deg to the left */
    float lag30 = sinf(30.0f * (float)M_PI / 180.0f) * MIC_D / c * FS;
    float th = doa_bearing_from_lag(lag30, FS, MIC_D, c);
    CHECK(fabsf(th - 30.0f * (float)M_PI / 180.0f) < 1e-3f, "bearing %.4f rad", th);
    float res_deg = asinf(c / FS / MIC_D) * 180.0f / (float)M_PI;
    printf("  bearing resolution at broadside, 1 sample: %.1f deg\n", res_deg);
}

static void test_pi(void)
{
    printf("wheel PI (first-order motor model)\n");
    pi_ctrl_t c;
    const float w_nl = 20.9f;   /* 200 rpm in rad/s */
    pi_init(&c, 0.02f, 0.4f, 1.0f / w_nl, -1.0f, 1.0f);
    float w = 0, dt = 0.01f, tau = 0.05f, load = 3.0f;   /* rad/s lost to load */
    float ref = 10.0f;
    for (int i = 0; i < 200; i++) {
        float u = pi_update(&c, ref, w, dt);
        float w_ss = u * w_nl - load;
        w += (w_ss - w) * dt / tau;
    }
    CHECK(fabsf(w - ref) < 0.1f, "tracks 10 rad/s under load: %.3f", w);

    /* saturation: ask for more than possible, then step down; no windup overshoot */
    pi_reset(&c);
    w = 0;
    for (int i = 0; i < 300; i++) {
        float u = pi_update(&c, 30.0f, w, dt);
        w += (u * w_nl - load - w) * dt / tau;
    }
    float peak = 0;
    for (int i = 0; i < 300; i++) {
        float u = pi_update(&c, 10.0f, w, dt);
        w += (u * w_nl - load - w) * dt / tau;
        if (i > 50) peak = fmaxf(peak, w);
    }
    CHECK(peak < 10.5f, "after saturation, settles without windup overshoot (peak %.2f)", peak);
}

static void test_kinematics_odom(void)
{
    printf("kinematics and odometry\n");
    kinematics_t k = {0.022f, 0.132f, 1.0f};
    float wl, wr, v, w;
    kin_inverse(&k, 0.3f, 1.2f, &wl, &wr);
    kin_forward(&k, wl, wr, &v, &w);
    CHECK(fabsf(v - 0.3f) < 1e-5f && fabsf(w - 1.2f) < 1e-5f, "round trip v %.4f w %.4f", v, w);

    odom_t o;
    odom_reset(&o, 1.0f);
    /* 1 m square: 4 x (straight 1 m, turn +90 deg) */
    for (int side = 0; side < 4; side++) {
        for (int i = 0; i < 100; i++) odom_update(&o, 0.5f, 0, 0, 0.02f);
        for (int i = 0; i < 100; i++) odom_update(&o, 0, 0, (float)M_PI / 2 / 1.0f, 0.01f);
    }
    CHECK(hypotf(o.x, o.y) < 1e-3f && fabsf(o.theta) < 1e-3f, "square closes: %.4f %.4f %.4f", o.x, o.y, o.theta);
}

static void test_cbf(void)
{
    printf("CBF safety filter\n");
    cbf_t c = {0.10f, 2.0f, 0.10f};
    cbf_out_t o = cbf_filter(&c, 0.30f, 0.0f, 0.20f, true);
    CHECK(fabsf(o.v - 0.20f) < 1e-6f && o.limited, "v at 0.2 m: %.3f", o.v);
    o = cbf_filter(&c, 0.30f, 0.5f, 0.05f, true);
    CHECK(o.v == 0.0f && o.w == 0.5f, "inside d_safe: forward blocked, turn allowed");
    o = cbf_filter(&c, 0.30f, 0.0f, 2.0f, false);
    CHECK(o.v == 0.0f, "invalid range fails safe");
    o = cbf_filter(&c, 0.10f, 0.0f, 1.0f, true);
    CHECK(o.v == 0.10f && !o.limited, "clear path unchanged");
    o = cbf_filter(&c, -0.3f, 0.0f, 1.0f, true);
    CHECK(fabsf(o.v + 0.10f) < 1e-6f, "blind reverse capped");

    /* closed loop: approach a wall at full speed, never cross d_safe */
    float d = 1.0f, dmin = 1.0f;
    for (int i = 0; i < 1000; i++) {
        cbf_out_t s = cbf_filter(&c, 0.37f, 0, d, true);
        d -= s.v * 0.01f;
        dmin = fminf(dmin, d);
    }
    CHECK(dmin >= c.d_safe - 1e-4f, "closed loop min distance %.4f >= d_safe", dmin);
    printf("  closed-loop approach: min distance %.4f m (d_safe %.2f)\n", dmin, c.d_safe);
}

static void test_policy(void)
{
    printf("policy\n");
    policy_cfg_t cfg = {0.30f, 0.20f, 0.30f, true};
    policy_state_t s = policy_encode(&cfg, 0.25f, true, 1.0f, true, 1.0f, false, 0.5f, true, 1, false);
    CHECK(s.front_blocked == 1 && s.left_blocked == 0 && s.right_blocked == 1 && s.audio_bin == 0,
          "encode: f%d l%d r%d a%d", s.front_blocked, s.left_blocked, s.right_blocked, s.audio_bin);
    s = policy_encode(&cfg, 1, true, 1, true, 1, true, -0.5f, true, 1, false);
    CHECK(s.audio_bin == 2, "negative bearing = right");
    s = policy_encode(&cfg, 1, true, 1, true, 1, true, 0, false, 2, false);
    CHECK(s.audio_bin == 2, "no bearing keeps last bin");

    int counts[5] = {0}, forward_blocked = 0, reverse = 0;
    for (int f = 0; f < 2; f++)
        for (int l = 0; l < 2; l++)
            for (int r = 0; r < 2; r++)
                for (int a = 0; a < 3; a++)
                    for (int t = 0; t < 2; t++) {
                        policy_state_t st = {(uint8_t)f, (uint8_t)l, (uint8_t)r, (uint8_t)a, (uint8_t)t};
                        policy_src_t src;
                        action_t act = policy_select(&cfg, &st, &src);
                        counts[src]++;
                        if (act == ACT_FORWARD && f) forward_blocked++;
                        if (act == ACT_REVERSE) reverse++;
                    }
    CHECK(forward_blocked == 0, "never drives forward into a blocked cell (%d)", forward_blocked);
    CHECK(reverse == 0, "never reverses blind (%d)", reverse);
    printf("  48 states: table %d, unvisited->rule %d, unsafe->rule %d, clamp->rule %d\n",
           counts[0], counts[1], counts[2], counts[3]);
}

static void test_path(void)
{
    printf("return path\n");
    static path_t p;
    path_init(&p, 0.15f, 0.20f);   /* loop radius > spacing so a crossing is always caught */
    /* go out 1 m, do a 0.5 m square loop, come back to the loop start, go on 0.5 m */
    float x = 0, y = 0;
    for (int i = 0; i <= 100; i++) path_record(&p, x = i * 0.01f, y);
    int before_loop = p.count;
    for (int i = 1; i <= 50; i++) path_record(&p, x, y = i * 0.01f);
    for (int i = 1; i <= 50; i++) path_record(&p, x = 1.0f + i * 0.01f, y);
    for (int i = 1; i <= 50; i++) path_record(&p, x, y = 0.5f - i * 0.01f);
    for (int i = 1; i <= 50; i++) path_record(&p, x = 1.5f - i * 0.01f, y);
    CHECK(p.count <= before_loop, "loop pruned (%d crumbs, %d before loop)", p.count, before_loop);
    for (int i = 1; i <= 50; i++) path_record(&p, x = 1.0f + i * 0.01f, y = -i * 0.01f);
    float px, py, last_x = 99, last_y = 99;
    int n = 0;
    while (path_pop(&p, &px, &py)) {
        last_x = px;
        last_y = py;
        n++;
    }
    CHECK(fabsf(last_x) < 1e-6f && fabsf(last_y) < 1e-6f, "last waypoint is home (%.2f, %.2f)", last_x, last_y);
    printf("  return route: %d waypoints\n", n);

    float v, w;
    goto_point(0, 0, 0, 1, 1, 0.3f, 2.0f, 2.0f, 1.0f, &v, &w);
    CHECK(v > 0 && w > 0, "target ahead-left: forward + left turn");
    goto_point(0, 0, 0, -1, 0, 0.3f, 2.0f, 2.0f, 1.0f, &v, &w);
    CHECK(v == 0 && fabsf(w) == 2.0f, "target behind: turn in place");
}

int main(void)
{
    test_bandpass();
    test_fft();
    test_gcc_phat();
    test_pi();
    test_kinematics_odom();
    test_cbf();
    test_policy();
    test_path();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
