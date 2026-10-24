/*
 * Portable DSP for the two-microphone direction finder.
 * No ESP-IDF dependencies: unit-tested on the host (test/host).
 *
 *   biquad cascade  : 4th-order Butterworth band-pass, 300 Hz - 3.4 kHz
 *   fft             : in-place iterative radix-2 complex FFT
 *   gcc_phat        : band-limited GCC-PHAT, sub-sample lag by parabolic fit
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Biquad (direct form II transposed) -------------------------------- */
typedef struct {
    float b0, b1, b2, a1, a2;   /* a0 normalised to 1 */
    float z1, z2;
} biquad_t;

void biquad_design_lowpass(biquad_t *bq, float fs, float fc, float q);
void biquad_design_highpass(biquad_t *bq, float fs, float fc, float q);
void biquad_process(biquad_t *bq, float *x, int n);

#define BANDPASS_SECTIONS 4
typedef struct {
    biquad_t s[BANDPASS_SECTIONS];
} bandpass_t;

/* 4th-order Butterworth high-pass at f_lo followed by 4th-order low-pass at f_hi */
void bandpass_init(bandpass_t *bp, float fs, float f_lo, float f_hi);
void bandpass_process(bandpass_t *bp, float *x, int n);

/* ---- FFT ----------------------------------------------------------------- */
#define FFT_MAX_N 2048

typedef struct {
    int n;
    int log2n;
    float cos_tab[FFT_MAX_N / 2];
    float sin_tab[FFT_MAX_N / 2];
    uint16_t bitrev[FFT_MAX_N];
} fft_plan_t;

/* n must be a power of two, 8 <= n <= FFT_MAX_N. Returns false otherwise. */
bool fft_plan_init(fft_plan_t *p, int n);
/* data: interleaved re, im (2n floats). inverse != 0 computes the unscaled IFFT. */
void fft_run(const fft_plan_t *p, float *data, int inverse);

/* ---- GCC-PHAT ------------------------------------------------------------ */
typedef struct {
    float fs;
    int n;                 /* frame length (power of two) */
    int max_lag;           /* search window in samples */
    int bin_lo, bin_hi;    /* PHAT band, inclusive */
    fft_plan_t plan;
    float window[FFT_MAX_N];
    float work[2 * FFT_MAX_N];
    float spec[2 * FFT_MAX_N];
} gcc_phat_t;

typedef struct {
    float lag_samples;     /* + : right channel lags left (source on the left) */
    float peak;            /* normalised correlation peak (0..1) */
    float psr;             /* peak-to-sidelobe ratio inside the search window */
} gcc_result_t;

/*
 * max_delay_s = mic_spacing / speed_of_sound. The search window is
 * ceil(max_delay_s * fs) + 1 samples either side of zero.
 */
bool gcc_phat_init(gcc_phat_t *g, float fs, int n, float max_delay_s, float f_lo, float f_hi);
/* left/right: n samples each (already band-passed). Inputs are not modified. */
gcc_result_t gcc_phat_run(gcc_phat_t *g, const float *left, const float *right);

/* Bearing in radians from a lag: + = source on the left (+Y), range [-pi/2, pi/2] */
float doa_bearing_from_lag(float lag_samples, float fs, float mic_spacing_m, float speed_of_sound);

/* Band-energy noise floor: follows quiet frames down immediately and rises
 * slowly (x1.011 per frame, about 2 dB/s at 47 frames/s). Returns the frame
 * energy over the floor in dB. Start with floor = -1. */
typedef struct {
    float floor;
} noise_floor_t;
float noise_floor_update(noise_floor_t *nf, float energy);

/* DOA gate used by the firmware and the simulator */
static inline bool doa_gate(float psr, float band_db, float psr_min, float snr_min_db)
{
    return psr >= psr_min && band_db >= snr_min_db;
}

/* Speed of sound in air (m/s) at temperature t_c (deg C) */
static inline float speed_of_sound(float t_c) { return 331.3f + 0.606f * t_c; }

#ifdef __cplusplus
}
#endif
