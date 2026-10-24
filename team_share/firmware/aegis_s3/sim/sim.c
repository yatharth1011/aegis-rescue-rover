/*
 * A.E.G.I.S. software-in-the-loop simulator.
 *
 * The rover's decision code is the real firmware code, linked unchanged from
 * components/aegis_core:
 *   band-pass + GCC-PHAT (aegis_dsp), noise-floor DOA gate, Q-table policy,
 *   CBF safety filter, wheel PI, skid-steer kinematics, odometry, return path,
 *   and the mission state machine (aegis_mission).
 *
 * Simulated around it (everything the firmware would get from hardware):
 *   - room with random box obstacles, random target + sound beacon
 *   - 4 wheel motors (first-order, battery-scaled), true skid factor that
 *     differs from the firmware's calibrated value, lateral slip noise
 *   - quantised quadrature encoders, gyro with residual bias and noise
 *   - 3 VL53L0X rays (noise, 33 ms update, 2 m max, low objects invisible)
 *   - camera: 60 deg FOV, 1.0 m detection range, line of sight, dropouts
 *   - 48 kHz stereo microphone signals: beacon (pulsed band-limited noise),
 *     first-order wall reflections, obstacle shadowing, motor noise at the
 *     rover centre while driving, diffuse room noise, mic self-noise
 *   - the target object is pushed by the gripper and grasped only if it sits
 *     between the jaws when they close
 *
 * Usage: sim <scenario> <runs> <seed> <summary.csv> [trace_run trace_prefix]
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aegis_control.h"
#include "aegis_dsp.h"
#include "aegis_mission.h"
#include "aegis_params.h"
#include "aegis_policy.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------------------------------------------------------- rng */
static uint64_t g_rng;
static double urand(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 7;
    g_rng ^= g_rng << 17;
    return (g_rng >> 11) * (1.0 / 9007199254740992.0);
}
static double urange(double a, double b) { return a + (b - a) * urand(); }
static double gauss(void)
{
    double u1 = urand() + 1e-300, u2 = urand();
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* ---------------------------------------------------------------- scenario */
typedef struct {
    const char *name;
    float room_w, room_h;
    int obst_min, obst_max;
    float wall_refl;         /* first-order reflection amplitude */
    float motor_noise;       /* amplitude at the mics while driving, rel. to beacon at 1 m */
    float room_noise;        /* diffuse uncorrelated noise, rel. to beacon at 1 m */
    float skid_spread;       /* true skid factor = calibrated * (1 +- spread) */
    bool audio;              /* beacon on */
    float time_limit_s;
} scenario_t;

static const scenario_t SCENARIOS[] = {
    {"nominal", 4.0f, 3.0f, 3, 6, 0.5f, 0.30f, 0.010f, 0.10f, true, 240.0f},
    {"no_audio", 4.0f, 3.0f, 3, 6, 0.5f, 0.30f, 0.010f, 0.10f, false, 240.0f},
    {"harsh", 4.0f, 3.0f, 6, 9, 0.8f, 0.60f, 0.030f, 0.20f, true, 240.0f},
};

/* ---------------------------------------------------------------- world */
typedef struct {
    float x0, y0, x1, y1;
} box_t;

#define MAX_OBST 12
typedef struct {
    float w, h;
    box_t ob[MAX_OBST];
    int n;
    float tx, ty;            /* target object (and beacon) */
    float sx, sy, sth;       /* start pose */
} world_t;

static bool in_box(const box_t *b, float x, float y, float pad)
{
    return x > b->x0 - pad && x < b->x1 + pad && y > b->y0 - pad && y < b->y1 + pad;
}

static bool blocked_pt(const world_t *w, float x, float y, float pad)
{
    if (x < pad || y < pad || x > w->w - pad || y > w->h - pad) return true;
    for (int i = 0; i < w->n; i++)
        if (in_box(&w->ob[i], x, y, pad)) return true;
    return false;
}

/* Ray vs axis-aligned box (slab test). Returns hit distance or INFINITY. */
static float ray_box(float ox, float oy, float dx, float dy, const box_t *b)
{
    float tmin = 0.0f, tmax = INFINITY;
    float o[2] = {ox, oy}, d[2] = {dx, dy}, lo[2] = {b->x0, b->y0}, hi[2] = {b->x1, b->y1};
    for (int k = 0; k < 2; k++) {
        if (fabsf(d[k]) < 1e-9f) {
            if (o[k] < lo[k] || o[k] > hi[k]) return INFINITY;
        } else {
            float t1 = (lo[k] - o[k]) / d[k], t2 = (hi[k] - o[k]) / d[k];
            if (t1 > t2) { float t = t1; t1 = t2; t2 = t; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) return INFINITY;
        }
    }
    return tmin;
}

static float raycast(const world_t *w, float ox, float oy, float th)
{
    float dx = cosf(th), dy = sinf(th), best = INFINITY;
    /* room walls */
    if (dx > 1e-9f) best = fminf(best, (w->w - ox) / dx);
    if (dx < -1e-9f) best = fminf(best, -ox / dx);
    if (dy > 1e-9f) best = fminf(best, (w->h - oy) / dy);
    if (dy < -1e-9f) best = fminf(best, -oy / dy);
    for (int i = 0; i < w->n; i++) best = fminf(best, ray_box(ox, oy, dx, dy, &w->ob[i]));
    return best;
}

static bool segment_clear(const world_t *w, float ax, float ay, float bx, float by)
{
    float dx = bx - ax, dy = by - ay, len = hypotf(dx, dy);
    if (len < 1e-6f) return true;
    for (int i = 0; i < w->n; i++) {
        float t = ray_box(ax, ay, dx / len, dy / len, &w->ob[i]);
        if (t < len) return false;
    }
    return true;
}

/* Grid flood fill: is the target reachable for a rover of radius r? */
static bool reachable(const world_t *w, float r)
{
    const float cell = 0.05f;
    int gw = (int)(w->w / cell), gh = (int)(w->h / cell);
    static unsigned char seen[200 * 200];
    static int queue[200 * 200];
    if (gw > 200 || gh > 200) return false;
    memset(seen, 0, sizeof(seen));
    int sx = (int)(w->sx / cell), sy = (int)(w->sy / cell), head = 0, tail = 0;
    int tx = (int)(w->tx / cell), ty = (int)(w->ty / cell);
    seen[sy * gw + sx] = 1;
    queue[tail++] = sy * gw + sx;
    while (head < tail) {
        int c = queue[head++], cx = c % gw, cy = c / gw;
        if (abs(cx - tx) <= 3 && abs(cy - ty) <= 3) return true;   /* within reach of the jaws */
        static const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (int k = 0; k < 4; k++) {
            int nx = cx + nb[k][0], ny = cy + nb[k][1];
            if (nx < 0 || ny < 0 || nx >= gw || ny >= gh || seen[ny * gw + nx]) continue;
            if (blocked_pt(w, (nx + 0.5f) * cell, (ny + 0.5f) * cell, r)) continue;
            seen[ny * gw + nx] = 1;
            queue[tail++] = ny * gw + nx;
        }
    }
    return false;
}

static void make_world(world_t *w, const scenario_t *sc)
{
    for (;;) {
        w->w = sc->room_w;
        w->h = sc->room_h;
        w->sx = 0.35f;
        w->sy = 0.35f;
        w->sth = (float)urange(-M_PI, M_PI);
        w->n = sc->obst_min + (int)(urand() * (sc->obst_max - sc->obst_min + 1));
        for (int i = 0; i < w->n; i++) {
            for (int tries = 0; tries < 100; tries++) {
                float bw = (float)urange(0.15, 0.6), bh = (float)urange(0.15, 0.6);
                float x = (float)urange(0.0, w->w - bw), y = (float)urange(0.0, w->h - bh);
                box_t b = {x, y, x + bw, y + bh};
                if (in_box(&b, w->sx, w->sy, 0.45f)) continue;   /* keep the start clear */
                w->ob[i] = b;
                break;
            }
        }
        /* target: at least 2 m away, clear of obstacles */
        int tries = 0;
        do {
            w->tx = (float)urange(0.3, w->w - 0.3);
            w->ty = (float)urange(0.3, w->h - 0.3);
        } while ((hypotf(w->tx - w->sx, w->ty - w->sy) < 2.0f || blocked_pt(w, w->tx, w->ty, 0.25f)) &&
                 ++tries < 500);
        if (tries < 500 && reachable(w, 0.16f)) return;
    }
}

/* ---------------------------------------------------------------- rover body */
/* Footprint sample points in the body frame (m): chassis with wheels, and the
 * gripper base and jaws. */
#define FP_N 34
static float FP[FP_N][2];
static void init_footprint(void)
{
    int k = 0;
    for (int i = 0; i <= 6; i++) {
        float x = -0.085f + i * (0.17f / 6);
        FP[k][0] = x; FP[k++][1] = 0.075f;
        FP[k][0] = x; FP[k++][1] = -0.075f;
    }
    for (int i = 1; i < 6; i++) {
        float y = -0.075f + i * (0.15f / 6);
        FP[k][0] = -0.085f; FP[k++][1] = y;
    }
    for (int i = 0; i <= 4; i++) {
        float x = 0.085f + i * (0.09f / 4);
        FP[k][0] = x; FP[k++][1] = 0.035f;
        FP[k][0] = x; FP[k++][1] = -0.035f;
    }
    FP[k][0] = 0.175f; FP[k++][1] = 0.0f;
    FP[k][0] = 0.085f; FP[k++][1] = 0.0f;
    FP[k][0] = 0.13f; FP[k++][1] = 0.0f;
    while (k < FP_N) { FP[k][0] = 0; FP[k++][1] = 0; }
}

static bool pose_collides(const world_t *w, float x, float y, float th)
{
    float c = cosf(th), s = sinf(th);
    for (int i = 0; i < FP_N; i++) {
        float px = x + c * FP[i][0] - s * FP[i][1], py = y + s * FP[i][0] + c * FP[i][1];
        if (blocked_pt(w, px, py, 0.0f)) return true;
    }
    return false;
}

static float clearance(const world_t *w, float x, float y, float th)
{
    /* distance from the footprint to the nearest obstacle or wall (sampled) */
    float best = 1e9f, c = cosf(th), s = sinf(th);
    for (int i = 0; i < FP_N; i++) {
        float px = x + c * FP[i][0] - s * FP[i][1], py = y + s * FP[i][0] + c * FP[i][1];
        best = fminf(best, fminf(fminf(px, w->w - px), fminf(py, w->h - py)));
        for (int j = 0; j < w->n; j++) {
            const box_t *b = &w->ob[j];
            float dx = fmaxf(fmaxf(b->x0 - px, 0), px - b->x1), dy = fmaxf(fmaxf(b->y0 - py, 0), py - b->y1);
            best = fminf(best, hypotf(dx, dy));
        }
    }
    return best;
}

/* ---------------------------------------------------------------- audio */
#define SRC_RING 8192
typedef struct {
    float ring[SRC_RING];   /* beacon waveform history */
    float motor[SRC_RING];  /* motor noise history */
    long n;                 /* samples generated */
    bandpass_t shape;       /* beacon spectrum */
    bandpass_t mshape;      /* motor noise spectrum */
} audio_src_t;

static void audio_src_init(audio_src_t *a)
{
    memset(a, 0, sizeof(*a));
    bandpass_init(&a->shape, AUDIO_FS, 500.0f, 3000.0f);
    bandpass_init(&a->mshape, AUDIO_FS, 150.0f, 2500.0f);
}

/* beacon: 0.5 s bursts of band-limited noise, 0.5 s silence */
static void audio_src_advance(audio_src_t *a, int count, bool beacon_on)
{
    float tmp[1], mt[1];
    for (int i = 0; i < count; i++, a->n++) {
        tmp[0] = (float)gauss();
        bandpass_process(&a->shape, tmp, 1);
        bool on = beacon_on && ((a->n / (AUDIO_FS / 2)) % 2 == 0);
        a->ring[a->n % SRC_RING] = on ? 3.0f * tmp[0] : 0.0f;
        mt[0] = (float)gauss();
        bandpass_process(&a->mshape, mt, 1);
        a->motor[a->n % SRC_RING] = 3.0f * mt[0];
    }
}

/* Catmull-Rom fractional read, delay in samples back from sample index n */
static float ring_read(const float *r, long n, float delay)
{
    float pos = (float)n - delay;
    long i = (long)floorf(pos);
    float f = pos - i;
    float p0 = r[(i - 1 + SRC_RING) % SRC_RING], p1 = r[i % SRC_RING];
    float p2 = r[(i + 1) % SRC_RING], p3 = r[(i + 2) % SRC_RING];
    return p1 + 0.5f * f * (p2 - p0 + f * (2 * p0 - 5 * p1 + 4 * p2 - p3 + f * (3 * (p1 - p3) + p3 - p0)));
}

typedef struct {
    float x, y, gain;
} image_t;

/* Render one frame of stereo audio for mics at (lx,ly), (rx,ry). */
static void render_frame(const world_t *w, const scenario_t *sc, audio_src_t *a, float *L, float *R,
                         float lx, float ly, float rx, float ry, float rov_x, float rov_y, bool driving)
{
    const float c = speed_of_sound(AUDIO_TEMP_C);
    image_t im[5];
    int nim = 0;
    float shadow = segment_clear(w, w->tx, w->ty, rov_x, rov_y) ? 1.0f : 0.35f;   /* diffraction */
    im[nim++] = (image_t){w->tx, w->ty, shadow};
    if (sc->wall_refl > 0) {
        im[nim++] = (image_t){-w->tx, w->ty, sc->wall_refl};
        im[nim++] = (image_t){2 * w->w - w->tx, w->ty, sc->wall_refl};
        im[nim++] = (image_t){w->tx, -w->ty, sc->wall_refl};
        im[nim++] = (image_t){w->tx, 2 * w->h - w->ty, sc->wall_refl};
    }
    long base = a->n - AUDIO_FRAME;   /* frame covers [base, a->n) */
    for (int i = 0; i < AUDIO_FRAME; i++) {
        long n = base + i;
        float l = 0, r = 0;
        for (int k = 0; k < nim; k++) {
            float dl = hypotf(im[k].x - lx, im[k].y - ly), dr = hypotf(im[k].x - rx, im[k].y - ry);
            l += im[k].gain / fmaxf(dl, 0.1f) * ring_read(a->ring, n, dl / c * AUDIO_FS);
            r += im[k].gain / fmaxf(dr, 0.1f) * ring_read(a->ring, n, dr / c * AUDIO_FS);
        }
        if (driving) {   /* motor noise at the rover centre: equal path to both mics */
            float m = sc->motor_noise * a->motor[n % SRC_RING];
            l += m;
            r += m;
        }
        L[i] = l + sc->room_noise * (float)gauss() + 0.002f * (float)gauss();
        R[i] = r + sc->room_noise * (float)gauss() + 0.002f * (float)gauss();
    }
}

/* ---------------------------------------------------------------- run */
typedef enum { OUT_SUCCESS, OUT_RETURN_FAIL, OUT_GRASP_MISS, OUT_TIMEOUT, OUT_FAULT } outcome_t;
static const char *OUTCOME_NAME[] = {"success", "return_fail", "grasp_miss", "timeout", "fault"};

typedef struct {
    outcome_t outcome;
    float t_end, t_first_seen, t_grasp;
    int collisions;
    float min_clear;
    float path_len;
    float home_err_true;     /* true distance to start at the end */
    float odom_err;          /* |estimate - truth| at the end */
    int cbf_ticks;
    int doa_count;
    float doa_err_sum, doa_err_fold_sum;   /* deg, raw and front/back folded */
    int decisions, table_used, escapes;
} result_t;

static mission_cfg_t MCFG = {
    .v_max = V_MAX_MPS, .w_max = W_MAX_RPS,
    .cell_m = CELL_M, .turn_rad = TURN_RAD, .turn_tol_rad = TURN_TOL_RAD,
    .listen_us = 300000,
    .doa_stale_us = DOA_STALE_MS * 1000LL, .vision_stale_us = VISION_STALE_MS * 1000LL,
    .dock_area_frac = DOCK_AREA_FRAC, .dock_ex_tol = DOCK_EX_TOL,
    .path_spacing_m = PATH_SPACING_M, .path_loop_radius_m = PATH_LOOP_RADIUS_M, .home_tol_m = HOME_TOL_M,
    .vbat_low_v = VBAT_LOW_V,
    .servo_open_us = SERVO_US_OPEN, .servo_grip_us = SERVO_US_GRIP, .grip_dwell_us = 800000,
    .cbf = {CBF_D_SAFE_M, CBF_ALPHA, V_REVERSE_MAX_MPS},
    .policy = {POLICY_BLOCK_FRONT_M, POLICY_BLOCK_SIDE_M, POLICY_AUDIO_AHEAD, POLICY_USE_TABLE},
};

#define OBJ_W 0.018f            /* target dowel diameter (fits the 22 mm jaw gap) */
#define CAM_X 0.077f            /* camera position on the body x axis */
#define CAM_RANGE 1.0f
#define CAM_HFOV (60.0f * (float)M_PI / 180.0f)
#define JAW_FRONT_OF_HUBS 0.149f   /* object centre stops here when pushed (hub front + radius) */

static void write_world(const world_t *w, const char *prefix)
{
    char fn[512];
    snprintf(fn, sizeof(fn), "%s_world.csv", prefix);
    FILE *f = fopen(fn, "w");
    if (!f) return;
    fprintf(f, "kind,a,b,c,d\nroom,%g,%g,0,0\nstart,%g,%g,%g,0\ntarget,%g,%g,0,0\n", w->w, w->h, w->sx, w->sy,
            w->sth, w->tx, w->ty);
    for (int i = 0; i < w->n; i++)
        fprintf(f, "box,%g,%g,%g,%g\n", w->ob[i].x0, w->ob[i].y0, w->ob[i].x1, w->ob[i].y1);
    fclose(f);
}

static result_t run_once(const scenario_t *sc, const world_t *w, FILE *trace)
{
    result_t res;
    memset(&res, 0, sizeof(res));
    res.min_clear = 1e9f;
    res.t_first_seen = res.t_grasp = -1;

    /* truth */
    float x = w->sx, y = w->sy, th = w->sth;
    float ox = w->tx, oy = w->ty;       /* object position (may be pushed) */
    bool grasped = false, colliding = false;
    float skid_true = TRACK_SKID_GAIN * (1.0f + (float)urange(-sc->skid_spread, sc->skid_spread));
    float gyro_bias = 0.002f * (float)gauss();
    float wheel_w[4] = {0}, wheel_ang[4] = {0};
    const float vbat = 11.4f;

    /* firmware state */
    mission_ctx_t m;
    mission_init(&m, &MCFG);
    odom_t odom;
    odom_reset(&odom, ODOM_GYRO_WEIGHT);
    pi_ctrl_t pi[4];
    for (int i = 0; i < 4; i++) pi_init(&pi[i], PI_KP, PI_KI, 1.0f / WHEEL_W_NOLOAD, -1.0f, 1.0f);
    const kinematics_t kin = {WHEEL_RADIUS_M, TRACK_M, TRACK_SKID_GAIN};
    int32_t cnt_prev[4] = {0};
    stall_det_t stall = {.hold_s = STALL_HOLD_S};
    float last_ref[4] = {0};
    float ww_est[4] = {0}, duty[4] = {0};
    float tof_m[3] = {2, 2, 2};
    int64_t tof_t[3] = {0};
    bool vis_new = false, vis_visible = false;
    float vis_ex = 0, vis_area = 0;

    /* audio */
    static audio_src_t asrc;
    static gcc_phat_t gcc;
    static bandpass_t bpl, bpr;
    static float L[AUDIO_FRAME], R[AUDIO_FRAME];
    audio_src_init(&asrc);
    gcc_phat_init(&gcc, AUDIO_FS, AUDIO_FRAME, MIC_SPACING_M / speed_of_sound(AUDIO_TEMP_C), AUDIO_F_LO, AUDIO_F_HI);
    bandpass_init(&bpl, AUDIO_FS, AUDIO_F_LO, AUDIO_F_HI);
    bandpass_init(&bpr, AUDIO_FS, AUDIO_F_LO, AUDIO_F_HI);
    noise_floor_t nf = {.floor = -1.0f};
    long audio_next = 0;   /* sample index of the next frame end */
    bool doa_pending = false;
    float doa_pending_b = 0;

    const float dt_phys = 0.001f;
    const int steps_per_tick = NAV_PERIOD_MS;
    const float tof_x[3] = {0.0835f, 0.06f, 0.06f}, tof_y[3] = {0.03f, 0.0535f, -0.0535f};
    const float tof_dir[3] = {0.0f, (float)M_PI / 2, -(float)M_PI / 2};
    float v_body = 0, w_body = 0;
    bool driving = false;
    int64_t t_us = 0;
    bool started = false;
    float last_contact_t = -1.0f;

    for (long tick = 0;; tick++) {
        t_us = tick * NAV_PERIOD_MS * 1000LL;
        float t = t_us * 1e-6f;
        if (t > sc->time_limit_s) {
            res.outcome = OUT_TIMEOUT;
            break;
        }

        /* ---- physics at 1 kHz */
        for (int s = 0; s < steps_per_tick; s++) {
            for (int i = 0; i < 4; i++) {
                float w_ss = duty[i] * WHEEL_W_NOLOAD * vbat / 12.0f;
                w_ss -= copysignf(fminf(fabsf(w_ss), 1.0f), w_ss);   /* friction load, rad/s */
                wheel_w[i] += (w_ss - wheel_w[i]) * dt_phys / 0.05f;
                wheel_ang[i] += wheel_w[i] * dt_phys;
            }
            float wl = 0.5f * (wheel_w[0] + wheel_w[1]), wr = 0.5f * (wheel_w[2] + wheel_w[3]);
            v_body = WHEEL_RADIUS_M * 0.5f * (wl + wr) * 0.97f;                    /* 3 % slip */
            w_body = WHEEL_RADIUS_M * (wr - wl) / (TRACK_M * skid_true) + 0.02f * (float)gauss();
            float nth = wrap_pi(th + w_body * dt_phys);
            float nx = x + v_body * cosf(th) * dt_phys, ny = y + v_body * sinf(th) * dt_phys;
            if (pose_collides(w, nx, ny, nth)) {
                /* contact: the rover stalls against the obstacle (wheels stop) */
                if (!colliding && t - last_contact_t > 0.3f) res.collisions++;
                colliding = true;
                last_contact_t = t;
                for (int i = 0; i < 4; i++) wheel_w[i] *= 0.5f;
            } else {
                colliding = false;
                res.path_len += hypotf(nx - x, ny - y);
                x = nx;
                y = ny;
                th = nth;
            }
            /* object interaction */
            if (!grasped) {
                float c = cosf(th), s2 = sinf(th);
                float rx = c * (ox - x) + s2 * (oy - y), ry = -s2 * (ox - x) + c * (oy - y);
                if (rx > 0.0f && rx < JAW_FRONT_OF_HUBS && fabsf(ry) < 0.03f) {
                    rx = JAW_FRONT_OF_HUBS;                       /* pushed ahead of the gear hubs */
                    ox = x + c * rx - s2 * ry;
                    oy = y + s2 * rx + c * ry;
                }
            } else {
                ox = x + cosf(th) * 0.155f;
                oy = y + sinf(th) * 0.155f;
            }
        }
        res.min_clear = fminf(res.min_clear, clearance(w, x, y, th));
        driving = fabsf(v_body) > 0.01f || fabsf(w_body) > 0.05f;

        /* ---- audio: render every frame that completed during this tick */
        long now_samples = (long)((double)t * AUDIO_FS);
        while (audio_next + AUDIO_FRAME <= now_samples) {
            audio_src_advance(&asrc, AUDIO_FRAME, sc->audio);
            audio_next += AUDIO_FRAME;
            float c = cosf(th), s2 = sinf(th);
            float lx = x + c * 0.03f - s2 * 0.05f, ly = y + s2 * 0.03f + c * 0.05f;
            float rx = x + c * 0.03f + s2 * 0.05f, ry = y + s2 * 0.03f - c * 0.05f;
            render_frame(w, sc, &asrc, L, R, lx, ly, rx, ry, x, y, driving);
            bandpass_process(&bpl, L, AUDIO_FRAME);
            bandpass_process(&bpr, R, AUDIO_FRAME);
            float e = 0;
            for (int i = 0; i < AUDIO_FRAME; i++) e += 0.5f * (L[i] * L[i] + R[i] * R[i]);
            float band_db = noise_floor_update(&nf, e / AUDIO_FRAME);
            gcc_result_t g = gcc_phat_run(&gcc, L, R);
            if (doa_gate(g.psr, band_db, AUDIO_PSR_GATE, AUDIO_SNR_GATE_DB)) {
                doa_pending = true;
                doa_pending_b = doa_bearing_from_lag(g.lag_samples, AUDIO_FS, MIC_SPACING_M,
                                                     speed_of_sound(AUDIO_TEMP_C));
                float truth = wrap_pi(atan2f(w->ty - y, w->tx - x) - th);
                float err = fabsf(wrap_pi(doa_pending_b - truth));
                float folded = fabsf(truth) > (float)M_PI / 2 ? (truth > 0 ? (float)M_PI - truth : -(float)M_PI - truth) : truth;
                if (!driving) {
                    res.doa_count++;
                    res.doa_err_sum += err * 57.2958f;
                    res.doa_err_fold_sum += fabsf(doa_pending_b - folded) * 57.2958f;
                }
            }
        }

        /* ---- sensors at the firmware rate */
        for (int k = 0; k < 3; k++) {
            if (t_us - tof_t[k] >= 33000) {
                float c = cosf(th), s2 = sinf(th);
                float px = x + c * tof_x[k] - s2 * tof_y[k], py = y + s2 * tof_x[k] + c * tof_y[k];
                /* VL53L0X sees a ~25 deg cone: nearest return over 5 rays */
                float d = INFINITY;
                for (int ray = -2; ray <= 2; ray++)
                    d = fminf(d, raycast(w, px, py, th + tof_dir[k] + ray * 0.109f));
                d = d > 2.0f ? 2.0f : d * (1.0f + 0.02f * (float)gauss()) + 0.003f * (float)gauss();
                tof_m[k] = fmaxf(0.0f, d);
                tof_t[k] = t_us;
            }
        }
        if (tick % 7 == 0) {   /* camera ~14 Hz */
            float c = cosf(th), s2 = sinf(th);
            float cx = x + c * CAM_X, cy = y + s2 * CAM_X;
            float d = hypotf(ox - cx, oy - cy);
            float b = wrap_pi(atan2f(oy - cy, ox - cx) - th);
            bool seen = !grasped && d < CAM_RANGE && fabsf(b) < CAM_HFOV / 2 && segment_clear(w, cx, cy, ox, oy) &&
                        urand() < 0.95;
            vis_new = true;
            vis_visible = seen;
            vis_ex = seen ? -b / (CAM_HFOV / 2) + 0.02f * (float)gauss() : 0;
            float wf = OBJ_W / (2.0f * fmaxf(d, 0.02f) * tanf(CAM_HFOV / 2));
            vis_area = seen ? wf * wf : 0;
            if (seen && res.t_first_seen < 0 && started) res.t_first_seen = t;
        }

        /* ---- firmware nav tick (mirrors task_nav.c) */
        int32_t cnt[4];
        for (int i = 0; i < 4; i++) {
            cnt[i] = (int32_t)lroundf(wheel_ang[i] / (2.0f * (float)M_PI) * ENC_COUNTS_PER_WHEEL_REV);
            float w_raw = (cnt[i] - cnt_prev[i]) * (2.0f * (float)M_PI / ENC_COUNTS_PER_WHEEL_REV) / (NAV_PERIOD_MS / 1000.0f);
            ww_est[i] += 0.3f * (w_raw - ww_est[i]);
            cnt_prev[i] = cnt[i];
        }
        float vm, we;
        kin_forward(&kin, 0.5f * (ww_est[0] + ww_est[1]), 0.5f * (ww_est[2] + ww_est[3]), &vm, &we);
        float gz = w_body + gyro_bias + 0.003f * (float)gauss();
        odom_update(&odom, vm, we, gz, NAV_PERIOD_MS / 1000.0f);

        mission_in_t in = {.now_us = t_us, .x = odom.x, .y = odom.y, .theta = odom.theta, .vbat = vbat};
        for (int k = 0; k < 3; k++) {
            in.tof_m[k] = tof_m[k];
            in.tof_valid[k] = t_us - tof_t[k] < 150000;
        }
        in.doa_new = doa_pending;
        in.doa_bearing = doa_pending_b;
        doa_pending = false;
        in.vis_new = vis_new;
        in.vis_visible = vis_visible;
        in.vis_ex = vis_ex;
        in.vis_area = vis_area;
        vis_new = false;
        in.stalled = stall_update(&stall, duty, ww_est, last_ref, NAV_PERIOD_MS / 1000.0f);
        in.start_pressed = !started && t >= 0.5f;
        if (in.start_pressed) started = true;

        mission_t before = m.state;
        mission_out_t out;
        mission_step(&m, &in, &out);
        if (out.reset_odom) odom_reset(&odom, ODOM_GYRO_WEIGHT);
        if (out.cbf_limited && out.drive) res.cbf_ticks++;
        if (m.state == MISSION_SEARCH && m.decisions > (uint32_t)res.decisions) {
            res.decisions = (int)m.decisions;
            if (m.last_src == POLICY_SRC_TABLE) res.table_used++;
        }

        if (before != MISSION_RETURN && m.state == MISSION_RETURN && before == MISSION_GRASP) {
            /* jaws closed during GRASP: is the object between them? */
            float c = cosf(th), s2 = sinf(th);
            float rx = c * (ox - x) + s2 * (oy - y), ry = -s2 * (ox - x) + c * (oy - y);
            grasped = rx > 0.135f && rx < 0.18f && fabsf(ry) < 0.03f;
            res.t_grasp = t;
            if (!grasped) {
                res.outcome = OUT_GRASP_MISS;
                res.t_end = t;
                break;
            }
        }
        if (m.state == MISSION_DONE) {
            res.home_err_true = hypotf(x - w->sx, y - w->sy);
            res.outcome = res.home_err_true < 0.30f ? OUT_SUCCESS : OUT_RETURN_FAIL;
            break;
        }
        if (m.state == MISSION_FAULT) {
            res.outcome = OUT_FAULT;
            break;
        }

        /* wheel PI (mirrors task_nav.c) */
        float wl_ref, wr_ref;
        kin_inverse(&kin, out.v, out.w, &wl_ref, &wr_ref);
        const float ref[4] = {wl_ref, wl_ref, wr_ref, wr_ref};
        for (int i = 0; i < 4; i++) last_ref[i] = ref[i];
        float kff = 12.0f / (vbat * WHEEL_W_NOLOAD);
        for (int i = 0; i < 4; i++) {
            if (!out.drive) {
                pi_reset(&pi[i]);
                duty[i] = 0;
                continue;
            }
            pi[i].kff = kff;
            duty[i] = pi_update(&pi[i], ref[i], ww_est[i], NAV_PERIOD_MS / 1000.0f);
        }

        if (trace && tick % 10 == 0) {
            /* odometry estimate expressed in the world frame for plotting */
            float c0 = cosf(w->sth), s0 = sinf(w->sth);
            fprintf(trace, "%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.4f,%.4f,%.4f,%d\n", t, x, y,
                    th, w->sx + c0 * odom.x - s0 * odom.y, w->sy + s0 * odom.x + c0 * odom.y, m.state, out.v, out.w,
                    tof_m[0], tof_m[1], tof_m[2], (t_us - m.doa_t) < 3000000, wrap_pi(m.doa_world + w->sth), ox, oy,
                    (int)grasped);
        }
    }
    res.t_end = t_us * 1e-6f;
    res.escapes = (int)m.escapes;
    if (res.outcome != OUT_SUCCESS && res.outcome != OUT_RETURN_FAIL)
        res.home_err_true = hypotf(x - w->sx, y - w->sy);
    {
        float c0 = cosf(w->sth), s0 = sinf(w->sth);
        float ex = w->sx + c0 * odom.x - s0 * odom.y, ey = w->sy + s0 * odom.x + c0 * odom.y;
        res.odom_err = hypotf(ex - x, ey - y);
    }
    return res;
}

int main(int argc, char **argv)
{
    if (argc < 5) {
        fprintf(stderr, "usage: %s <nominal|no_audio|harsh> <runs> <seed> <summary.csv> [trace_run trace_prefix]\n", argv[0]);
        return 2;
    }
    const scenario_t *sc = NULL;
    for (size_t i = 0; i < sizeof(SCENARIOS) / sizeof(SCENARIOS[0]); i++)
        if (!strcmp(argv[1], SCENARIOS[i].name)) sc = &SCENARIOS[i];
    if (!sc) {
        fprintf(stderr, "unknown scenario %s\n", argv[1]);
        return 2;
    }
    int runs = atoi(argv[2]);
    unsigned long seed = strtoul(argv[3], NULL, 10);
    int trace_run = argc >= 7 ? atoi(argv[5]) : -1;
    const char *trace_prefix = argc >= 7 ? argv[6] : NULL;
    init_footprint();
    const char *pol = getenv("AEGIS_POLICY");
    if (pol && !strcmp(pol, "rule")) MCFG.policy.use_table = false;
    if (pol && !strcmp(pol, "table")) MCFG.policy.use_table = true;

    FILE *sum = fopen(argv[4], "w");
    if (!sum) return 1;
    fprintf(sum, "scenario,run,outcome,t_end,t_first_seen,t_grasp,collisions,min_clear,path_len,home_err,odom_err,"
                 "cbf_ticks,doa_n,doa_err_deg,doa_err_folded_deg,decisions,table_decisions,obstacles,escapes\n");
    int counts[5] = {0};
    for (int r = 0; r < runs; r++) {
        g_rng = 0x9E3779B97F4A7C15ull ^ (seed * 1000003ull + (uint64_t)r * 7919ull);
        if (!g_rng) g_rng = 1;
        world_t w;
        make_world(&w, sc);
        FILE *tr = NULL;
        if (r == trace_run) {
            char fn[512];
            snprintf(fn, sizeof(fn), "%s_trace.csv", trace_prefix);
            tr = fopen(fn, "w");
            if (tr) fprintf(tr, "t,x,y,th,ex,ey,state,v,w,tof_f,tof_l,tof_r,doa_ok,doa_world,obj_x,obj_y,grasped\n");
            write_world(&w, trace_prefix);
        }
        result_t res = run_once(sc, &w, tr);
        if (tr) fclose(tr);
        counts[res.outcome]++;
        fprintf(sum, "%s,%d,%s,%.2f,%.2f,%.2f,%d,%.4f,%.3f,%.3f,%.3f,%d,%d,%.2f,%.2f,%d,%d,%d,%d\n", sc->name, r,
                OUTCOME_NAME[res.outcome], res.t_end, res.t_first_seen, res.t_grasp, res.collisions, res.min_clear,
                res.path_len, res.home_err_true, res.odom_err, res.cbf_ticks, res.doa_count,
                res.doa_count ? res.doa_err_sum / res.doa_count : -1,
                res.doa_count ? res.doa_err_fold_sum / res.doa_count : -1, res.decisions, res.table_used, w.n,
                res.escapes);
        fflush(sum);
    }
    fclose(sum);
    printf("%-9s runs %d:", sc->name, runs);
    for (int i = 0; i < 5; i++) printf("  %s %d", OUTCOME_NAME[i], counts[i]);
    printf("\n");
    return 0;
}
