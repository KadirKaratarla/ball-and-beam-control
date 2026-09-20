#ifdef BB_TESTS // Faz 3.1 unit test: built only in the `tests` environment
#include "test_pid_sim.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "config.h"
#include "pid.h"
#include "linkage.h"

static const char *TAG = "pid_sim";

#define G_CM 981.0f
#define BALL_ACC_GAIN (5.0f / 7.0f) // solid sphere rolling without slip
#define BEAM_LEN_CM 45.0f
#define PLANT_DT 0.001f
#define CAM_DT 0.010f
#define CAM_DELAY_S 0.020f
#define CAM_NOISE_CM 0.03f
#define TRACE_DT 0.05f

// Crank simulated the way stepper_tick + the sqrt profile behave, in counts.
typedef struct {
    float vmax, amax; // counts/s, counts/s^2
    float pos, vel;
} crank_t;

static void crank_step(crank_t *c, float target, float dt)
{
    float e = target - c->pos;
    float mag = fabsf(e);
    float v_req = sqrtf(2.0f * c->amax * mag);
    if (v_req > c->vmax) v_req = c->vmax;
    if (e < 0) v_req = -v_req;
    float dv = v_req - c->vel;
    float dv_max = c->amax * dt;
    if (dv > dv_max) dv = dv_max;
    if (dv < -dv_max) dv = -dv_max;
    c->vel += dv;
    c->pos += c->vel * dt;
}

typedef struct {
    float rise_s, overshoot_pct, settle_s, final_err, peak_crank_dps, max_i;
    bool settled;
} metrics_t;

static float frand(void)
{
    return (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;
}

// Runs one scenario for `duration` seconds. Ball starts at x0 at rest,
// setpoint xs. If trace, prints a CSV row every TRACE_DT.
static metrics_t run(float x0, float xs, float ki, float vmax_counts, float duration, bool trace)
{
    pid_cfg_t cfg = {
        .kp = PID_KP, .ki = ki, .kd = PID_KD,
        .out_min = -BEAM_THETA_MAX_DEG, .out_max = BEAM_THETA_MAX_DEG,
        .i_max = PID_I_MAX_DEG, .d_tau = PID_D_TAU_S,
    };
    pid_t pid;
    pid_init(&pid, &cfg);

    crank_t crank = { .vmax = vmax_counts, .amax = vmax_counts * 10.0f, .pos = ENC_LEVEL_COUNTS };

    // camera delay line: samples every CAM_DT, delivered CAM_DELAY_S later
    enum { DLY = (int)(CAM_DELAY_S / CAM_DT + 0.5f) };
    float delay[DLY + 1];
    for (int i = 0; i <= DLY; i++) delay[i] = x0;
    int dhead = 0;

    float x = x0, v = 0.0f;
    float theta_cmd = 0.0f, theta_act = 0.0f, phi_cmd = 0.0f;
    float t = 0.0f, next_cam = 0.0f, next_trace = 0.0f;
    float step = xs - x0;
    metrics_t m = { .rise_s = -1, .settle_s = -1, .settled = false };
    float t10 = -1, t90 = -1, peak = x0, last_out_of_band = 0.0f;
    float peak_vel = 0.0f;

    if (trace) printf("T,t,x,x_meas,x_set,theta_cmd,theta_act,phi,P,I,D\n");

    while (t < duration) {
        // camera + controller at 100 Hz
        if (t >= next_cam) {
            next_cam += CAM_DT;
            float meas_now = x + CAM_NOISE_CM * frand();
            delay[dhead] = meas_now;
            dhead = (dhead + 1) % (DLY + 1);
            float meas = delay[dhead]; // oldest = delayed
            theta_cmd = pid_update(&pid, xs, meas, CAM_DT);
            if (fabsf(pid.i) > m.max_i) m.max_i = fabsf(pid.i);
            if (trace && t >= next_trace) {
                next_trace += TRACE_DT;
                float phi = linkage_phi_from_counts(crank.pos);
                printf("T,%.2f,%.3f,%.3f,%.1f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f\n",
                       t, x, meas, xs, theta_cmd, theta_act, phi, pid.p, pid.i, pid.d);
            }
        }

        // crank follows the commanded beam angle through the linkage
        // An unreachable theta (cannot happen inside BEAM_THETA_MAX_DEG,
        // but the fallback must be sane) keeps the last crank target.
        float phi_new;
        if (linkage_phi_from_theta(theta_cmd, &phi_new)) phi_cmd = phi_new;
        crank_step(&crank, linkage_counts_from_phi(phi_cmd), PLANT_DT);
        if (fabsf(crank.vel) > peak_vel) peak_vel = fabsf(crank.vel);
        theta_act = linkage_theta_from_phi(linkage_phi_from_counts(crank.pos));

        // ball
        float a = BALL_ACC_GAIN * G_CM * sinf(theta_act * 3.14159265f / 180.0f) - 0.2f * v;
        v += a * PLANT_DT;
        x += v * PLANT_DT;
        if (x < 0.0f) { x = 0.0f; v = 0.0f; }
        if (x > BEAM_LEN_CM) { x = BEAM_LEN_CM; v = 0.0f; }
        t += PLANT_DT;

        // metrics
        float frac = step != 0.0f ? (x - x0) / step : 1.0f;
        if (t10 < 0 && frac >= 0.1f) t10 = t;
        if (t90 < 0 && frac >= 0.9f) t90 = t;
        if ((step > 0 && x > peak) || (step < 0 && x < peak)) peak = x;
        if (fabsf(x - xs) > 0.02f * fabsf(step) + 0.05f) last_out_of_band = t;
    }

    m.rise_s = (t10 >= 0 && t90 >= 0) ? t90 - t10 : -1.0f;
    m.overshoot_pct = step != 0.0f ? (peak - xs) / step * 100.0f : 0.0f;
    m.settle_s = last_out_of_band;
    m.settled = duration - last_out_of_band > 1.0f;
    m.final_err = xs - x;
    m.peak_crank_dps = peak_vel * 360.0f / ENC_COUNTS_PER_REV;
    return m;
}

static void report(const char *name, metrics_t m)
{
    printf("  %-34s rise %.2fs  overshoot %.1f%%  settle(2%%) %.2fs%s  final err %+.2f cm  peak crank %.0f deg/s  max|I| %.2f deg\n",
           name, m.rise_s, m.overshoot_pct, m.settle_s, m.settled ? "" : " (NOT settled)",
           m.final_err, m.peak_crank_dps, m.max_i);
}

void test_pid_sim(void)
{
    ESP_LOGI(TAG, "geometry r=%.0f l=%.0f d=%.0f mm; Kp=%.2f Ki=%.2f Kd=%.2f; theta max %.1f deg",
             LINK_CRANK_R_MM, LINK_ROD_L_MM, LINK_BEAM_D_MM, PID_KP, PID_KI, PID_KD, BEAM_THETA_MAX_DEG);

    printf("--- linkage: phi -> theta -> phi round trip ---\n");
    printf("  phi_deg  theta_deg  theta_lin  phi_back  err_deg\n");
    for (int p = -90; p <= 90; p += 15) {
        float th = linkage_theta_from_phi((float)p);
        float lin = asinf(LINK_CRANK_R_MM / LINK_BEAM_D_MM * sinf(p * 3.14159265f / 180.0f)) * 180.0f / 3.14159265f;
        float back = 0.0f;
        bool ok = linkage_phi_from_theta(th, &back);
        printf("  %7d  %9.3f  %9.3f  %8.2f  %+.4f%s\n", p, th, lin, back, back - p, ok ? "" : "  UNREACHABLE");
    }
    float phi_max = 0.0f;
    linkage_phi_from_theta(BEAM_THETA_MAX_DEG, &phi_max);
    printf("  theta max %.1f deg <-> crank %.1f deg (%.0f counts from level)\n",
           BEAM_THETA_MAX_DEG, phi_max, phi_max * ENC_COUNTS_PER_REV / 360.0f);

    const float slow = STEP_VMAX / STEP_USTEPS_PER_COUNT;       // Faz 2 speed, counts/s
    const float fast = 43000.0f / STEP_USTEPS_PER_COUNT;        // proposed closed-loop speed

    printf("--- step 22.5 -> 30 cm (PD, Ki=0) ---\n");
    report("crank VMAX 3200 usteps/s (Faz 2)", run(22.5f, 30.0f, 0.0f, slow, 10.0f, false));
    report("crank VMAX 43000 usteps/s", run(22.5f, 30.0f, 0.0f, fast, 10.0f, false));

    printf("--- step 22.5 -> 30 cm with Ki=0.3 ---\n");
    report("Ki=0.3, fast crank", run(22.5f, 30.0f, 0.3f, fast, 10.0f, false));

    printf("--- anti-windup: ball starts at 3 cm, setpoint 22.5 (saturated for a while) ---\n");
    report("Ki=0.3, from 3 cm", run(3.0f, 22.5f, 0.3f, fast, 12.0f, false));
    report("Ki=0,   from 3 cm", run(3.0f, 22.5f, 0.0f, fast, 12.0f, false));

    printf("--- trace: step 22.5 -> 30 cm, Ki=0, fast crank ---\n");
    run(22.5f, 30.0f, 0.0f, fast, 6.0f, true);

    ESP_LOGI(TAG, "done");
}
#endif // BB_TESTS
