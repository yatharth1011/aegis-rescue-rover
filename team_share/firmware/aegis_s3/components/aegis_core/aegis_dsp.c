#include "aegis_dsp.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Biquad: RBJ audio-EQ cookbook --------------------------------------- */
static void biquad_set(biquad_t *bq, double b0, double b1, double b2, double a0, double a1, double a2)
{
    bq->b0 = (float)(b0 / a0);
    bq->b1 = (float)(b1 / a0);
    bq->b2 = (float)(b2 / a0);
    bq->a1 = (float)(a1 / a0);
    bq->a2 = (float)(a2 / a0);
    bq->z1 = bq->z2 = 0.0f;
}

void biquad_design_lowpass(biquad_t *bq, float fs, float fc, float q)
{
    double w0 = 2.0 * M_PI * fc / fs, c = cos(w0), alpha = sin(w0) / (2.0 * q);
    biquad_set(bq, (1 - c) / 2, 1 - c, (1 - c) / 2, 1 + alpha, -2 * c, 1 - alpha);
}

void biquad_design_highpass(biquad_t *bq, float fs, float fc, float q)
{
    double w0 = 2.0 * M_PI * fc / fs, c = cos(w0), alpha = sin(w0) / (2.0 * q);
    biquad_set(bq, (1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + alpha, -2 * c, 1 - alpha);
}

void biquad_process(biquad_t *bq, float *x, int n)
{
    float z1 = bq->z1, z2 = bq->z2;
    for (int i = 0; i < n; i++) {
        float in = x[i];
        float out = bq->b0 * in + z1;
        z1 = bq->b1 * in - bq->a1 * out + z2;
        z2 = bq->b2 * in - bq->a2 * out;
        x[i] = out;
    }
    bq->z1 = z1;
    bq->z2 = z2;
}

/* Pole-pair Q values of a 4th-order Butterworth: 1 / (2 cos(pi/8)), 1 / (2 cos(3pi/8)) */
static const float BUTTER4_Q[2] = {0.54119610f, 1.30656296f};

void bandpass_init(bandpass_t *bp, float fs, float f_lo, float f_hi)
{
    biquad_design_highpass(&bp->s[0], fs, f_lo, BUTTER4_Q[0]);
    biquad_design_highpass(&bp->s[1], fs, f_lo, BUTTER4_Q[1]);
    biquad_design_lowpass(&bp->s[2], fs, f_hi, BUTTER4_Q[0]);
    biquad_design_lowpass(&bp->s[3], fs, f_hi, BUTTER4_Q[1]);
}

void bandpass_process(bandpass_t *bp, float *x, int n)
{
    for (int i = 0; i < BANDPASS_SECTIONS; i++) {
        biquad_process(&bp->s[i], x, n);
    }
}

/* ---- FFT --------------------------------------------------------------- */
bool fft_plan_init(fft_plan_t *p, int n)
{
    if (n < 8 || n > FFT_MAX_N || (n & (n - 1)) != 0) {
        return false;
    }
    p->n = n;
    p->log2n = 0;
    while ((1 << p->log2n) < n) {
        p->log2n++;
    }
    for (int k = 0; k < n / 2; k++) {
        double a = -2.0 * M_PI * k / n;
        p->cos_tab[k] = (float)cos(a);
        p->sin_tab[k] = (float)sin(a);
    }
    for (int i = 0; i < n; i++) {
        unsigned r = 0;
        for (int b = 0; b < p->log2n; b++) {
            r |= ((i >> b) & 1u) << (p->log2n - 1 - b);
        }
        p->bitrev[i] = (uint16_t)r;
    }
    return true;
}

void fft_run(const fft_plan_t *p, float *d, int inverse)
{
    const int n = p->n;
    for (int i = 0; i < n; i++) {
        int j = p->bitrev[i];
        if (j > i) {
            float tr = d[2 * i], ti = d[2 * i + 1];
            d[2 * i] = d[2 * j];
            d[2 * i + 1] = d[2 * j + 1];
            d[2 * j] = tr;
            d[2 * j + 1] = ti;
        }
    }
    const float sgn = inverse ? -1.0f : 1.0f;
    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1;
        int step = n / len;
        for (int start = 0; start < n; start += len) {
            for (int k = 0; k < half; k++) {
                float wr = p->cos_tab[k * step];
                float wi = sgn * p->sin_tab[k * step];
                int a = 2 * (start + k), b = 2 * (start + k + half);
                float xr = d[b] * wr - d[b + 1] * wi;
                float xi = d[b] * wi + d[b + 1] * wr;
                d[b] = d[a] - xr;
                d[b + 1] = d[a + 1] - xi;
                d[a] += xr;
                d[a + 1] += xi;
            }
        }
    }
}

/* ---- GCC-PHAT ------------------------------------------------------------ */
bool gcc_phat_init(gcc_phat_t *g, float fs, int n, float max_delay_s, float f_lo, float f_hi)
{
    if (!fft_plan_init(&g->plan, n)) {
        return false;
    }
    g->fs = fs;
    g->n = n;
    g->max_lag = (int)ceilf(max_delay_s * fs) + 1;
    if (g->max_lag >= n / 2) {
        return false;
    }
    g->bin_lo = (int)floorf(f_lo * n / fs);
    g->bin_hi = (int)ceilf(f_hi * n / fs);
    if (g->bin_lo < 1) g->bin_lo = 1;
    if (g->bin_hi > n / 2 - 1) g->bin_hi = n / 2 - 1;
    for (int i = 0; i < n; i++) {
        g->window[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / (n - 1));   /* Hann */
    }
    return true;
}

gcc_result_t gcc_phat_run(gcc_phat_t *g, const float *left, const float *right)
{
    const int n = g->n;
    float *z = g->work;
    float *s = g->spec;
    gcc_result_t res = {0.0f, 0.0f, 0.0f};

    /* Two real FFTs for the price of one: z = l + j r */
    for (int i = 0; i < n; i++) {
        z[2 * i] = left[i] * g->window[i];
        z[2 * i + 1] = right[i] * g->window[i];
    }
    fft_run(&g->plan, z, 0);

    memset(s, 0, sizeof(float) * 2 * n);
    for (int k = g->bin_lo; k <= g->bin_hi; k++) {
        int m = n - k;
        float zr = z[2 * k], zi = z[2 * k + 1];
        float mr = z[2 * m], mi = -z[2 * m + 1];          /* conj(Z[n-k]) */
        float lr = 0.5f * (zr + mr), li = 0.5f * (zi + mi);   /* L[k] */
        float rr = 0.5f * (zi - mi), ri = -0.5f * (zr - mr);  /* R[k] = (Z - conj)/2j */
        /* cross spectrum conj(L) * R */
        float cr = lr * rr + li * ri;
        float ci = lr * ri - li * rr;
        float mag = sqrtf(cr * cr + ci * ci) + 1e-12f;
        cr /= mag;
        ci /= mag;
        s[2 * k] = cr;
        s[2 * k + 1] = ci;
        s[2 * m] = cr;                                     /* Hermitian -> real IFFT */
        s[2 * m + 1] = -ci;
    }
    fft_run(&g->plan, s, 1);

    /* Correlation at lag L is s[L] (L >= 0) or s[n + L] (L < 0). */
    const float norm = 1.0f / (float)(2 * (g->bin_hi - g->bin_lo + 1));
    int best = 0;
    float best_v = -1e30f;
    for (int lag = -g->max_lag; lag <= g->max_lag; lag++) {
        int idx = lag >= 0 ? lag : n + lag;
        float v = s[2 * idx] * norm;
        if (v > best_v) {
            best_v = v;
            best = lag;
        }
    }

    /* Parabolic interpolation around the peak */
    float frac = 0.0f;
    if (best > -g->max_lag && best < g->max_lag) {
        int im = best - 1 >= 0 ? best - 1 : n + best - 1;
        int ip = best + 1 >= 0 ? best + 1 : n + best + 1;
        float ym = s[2 * im] * norm, yp = s[2 * ip] * norm;
        float den = ym - 2.0f * best_v + yp;
        if (fabsf(den) > 1e-12f) {
            frac = 0.5f * (ym - yp) / den;
            if (frac > 0.5f) frac = 0.5f;
            if (frac < -0.5f) frac = -0.5f;
        }
    }

    /* Peak-to-sidelobe: peak over mean |r| outside +-2 samples of the peak */
    float side = 0.0f;
    int cnt = 0;
    for (int lag = -g->max_lag; lag <= g->max_lag; lag++) {
        if (lag >= best - 2 && lag <= best + 2) continue;
        int idx = lag >= 0 ? lag : n + lag;
        side += fabsf(s[2 * idx] * norm);
        cnt++;
    }
    side = cnt ? side / cnt : 1e-6f;

    res.lag_samples = (float)best + frac;
    res.peak = best_v;
    res.psr = best_v / (side + 1e-6f);
    return res;
}

float doa_bearing_from_lag(float lag_samples, float fs, float mic_spacing_m, float c)
{
    float x = c * (lag_samples / fs) / mic_spacing_m;
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;
    return asinf(x);
}

float noise_floor_update(noise_floor_t *nf, float e)
{
    e += 1e-12f;
    if (nf->floor < 0 || e < nf->floor) {
        nf->floor = e;
    } else {
        nf->floor *= 1.011f;
    }
    return 10.0f * log10f(e / nf->floor);
}
