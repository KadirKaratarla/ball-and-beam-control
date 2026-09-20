#ifdef BB_TESTS // Faz 3.3 jog test: built only in the `tests` environment
#include "test_jog.h"

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_log.h"

#include "config.h"
#include "encoder.h"
#include "stepper.h"
#include "linkage.h"

static const char *TAG = "jog";

#define TICK_MS (CONTROL_LOOP_PERIOD_US / 1000)
#define JOG_VMAX 3200.0f   // jog speed: the Faz 2 proven-smooth value
#define JOG_AMAX 8000.0f
#define SWEEP_AMPL_COUNTS 600 // +-53 deg around level during the speed sweep
#define SWEEP_CYCLES 2
#define SWEEP_ABORT_FOLLOW 150.0f

static int32_t s_target;        // usteps, relative to engage
static float s_enc_at_engage;   // counts from 6 at engage
static float s_level_counts = ENC_LEVEL_COUNTS;
static encoder_sample_t s_enc;
static float s_follow_max;      // since last print

static void console_input_init(void)
{
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    uart_vfs_dev_use_driver(UART_NUM_0);
}

static int getkey(void)
{
    uint8_t c;
    return uart_read_bytes(UART_NUM_0, &c, 1, 0) == 1 ? c : -1;
}

// One 4 ms tick of the mini control loop: encoder, tracking, follow error.
static float tick(void)
{
    encoder_update(&s_enc);
    float vel = stepper_track(s_target);
    float cmd = stepper_get_step_count() / STEP_USTEPS_PER_COUNT;
    float follow = cmd - (s_enc.pos_counts - s_enc_at_engage);
    if (s_enc.ok && fabsf(follow) > fabsf(s_follow_max)) s_follow_max = follow;
    return vel;
}

static float shaft_counts(void)
{
    return s_enc.pos_counts; // measured, absolute from 6
}

static float cmd_counts(void)
{
    return stepper_get_step_count() / STEP_USTEPS_PER_COUNT + s_enc_at_engage; // commanded, absolute from 6
}

static void wait_arrival(void)
{
    TickType_t last = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TICK_MS));
        float vel = tick();
        int32_t e = s_target - stepper_get_step_count();
        if (vel == 0.0f && (e < 0 ? -e : e) <= 8) return;
    }
}

static void print_status(void)
{
    float counts = shaft_counts();
    float phi = (counts - s_level_counts) * (360.0f / ENC_COUNTS_PER_REV);
    float theta = linkage_theta_from_phi(phi);
    float cmd = stepper_get_step_count() / STEP_USTEPS_PER_COUNT + s_enc_at_engage;
    printf("  enc raw=%4u  shaft=%7.1f counts from 6 (%6.2f deg)  cmd=%7.1f  "
           "phi=%+7.2f deg  theta=%+6.3f deg  follow(max since last)=%+.1f%s\n",
           s_enc.raw, counts, counts * 360.0f / ENC_COUNTS_PER_REV, cmd, phi, theta, s_follow_max,
           s_enc.ok ? "" : "  ENC-BAD");
    s_follow_max = 0.0f;
}

static void goto_counts(float target_counts_from_6)
{
    s_target = (int32_t)((target_counts_from_6 - s_enc_at_engage) * STEP_USTEPS_PER_COUNT);
    wait_arrival();
}

// Speed sweep: +-SWEEP_AMPL_COUNTS around level at increasing VMAX.
static void speed_sweep(void)
{
    static const float vmax_list[] = { 3200, 10000, 20000, 30000, STEP_VMAX_CLOSED_LOOP };
    float start = cmd_counts();
    printf("--- speed sweep around %.0f counts, +-%d counts, %d cycles each ---\n",
           start, SWEEP_AMPL_COUNTS, SWEEP_CYCLES);
    printf("  vmax_usteps_s,rpm,cycle,leg,follow_max_counts,follow_end_counts\n");

    for (size_t i = 0; i < sizeof(vmax_list) / sizeof(vmax_list[0]); i++) {
        float vmax = vmax_list[i];
        stepper_set_limits(vmax, vmax * 10.0f);
        for (int cyc = 0; cyc < SWEEP_CYCLES; cyc++) {
            const float legs[4] = { start + SWEEP_AMPL_COUNTS, start, start - SWEEP_AMPL_COUNTS, start };
            for (int leg = 0; leg < 4; leg++) {
                s_follow_max = 0.0f;
                goto_counts(legs[leg]);
                vTaskDelay(pdMS_TO_TICKS(150));
                encoder_update(&s_enc);
                float cmd = stepper_get_step_count() / STEP_USTEPS_PER_COUNT;
                float follow_end = cmd - (s_enc.pos_counts - s_enc_at_engage);
                printf("  %.0f,%.1f,%d,%d,%+.1f,%+.1f\n", vmax, vmax * 60.0f / STEP_USTEPS_PER_REV,
                       cyc, leg, s_follow_max, follow_end);
                if (fabsf(follow_end) > SWEEP_ABORT_FOLLOW) {
                    printf("  ** following error %.0f counts: steps lost at %.0f usteps/s -- aborting sweep\n",
                           follow_end, vmax);
                    stepper_set_limits(JOG_VMAX, JOG_AMAX);
                    goto_counts(start);
                    return;
                }
            }
        }
    }
    stepper_set_limits(JOG_VMAX, JOG_AMAX);
    printf("--- sweep done, no step loss up to %.0f usteps/s ---\n", STEP_VMAX_CLOSED_LOOP);
}

void test_jog(void)
{
    console_input_init();
    ESP_ERROR_CHECK(encoder_init());
    encoder_update(&s_enc);
    if (!s_enc.ok || fabsf(s_enc.pos_counts) > ENC_START_WINDOW_COUNTS) {
        ESP_LOGE(TAG, "shaft at %.1f counts from 6 -- beam must rest at 6. Reset and retry.", s_enc.pos_counts);
        while (true) vTaskDelay(pdMS_TO_TICKS(10000));
    }
    ESP_ERROR_CHECK(stepper_init());
    stepper_set_limits(JOG_VMAX, JOG_AMAX);

    printf("\n=== jog: find beam level, verify linkage, sweep speed ===\n");
    printf("  shaft at %.1f counts from 6 (raw %u). Nominal level = %d counts.\n",
           s_enc.pos_counts, s_enc.raw, ENC_LEVEL_COUNTS);
    printf("  keys:  + / -  10 counts     [ / ]  100 counts     p  print\n");
    printf("         l  mark current position as LEVEL      s  speed sweep (ball OFF the beam!)\n");
    printf("         q  back to 6, driver off\n");
    printf("  press ENTER to engage the driver and move to nominal level (keep clear)\n");
    fflush(stdout);
    uart_flush_input(UART_NUM_0);
    while (true) {
        int c = getkey();
        if (c == '\r' || c == '\n') break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    stepper_enable(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    encoder_update(&s_enc);
    s_enc_at_engage = s_enc.pos_counts;
    s_target = 0;
    printf("  driver ON at %.1f counts. Moving to nominal level...\n", s_enc_at_engage);
    goto_counts(ENC_LEVEL_COUNTS);
    print_status();
    printf("  ready. Level the beam with +/-/[/], then press l.\n");
    fflush(stdout);
    uart_flush_input(UART_NUM_0);

    TickType_t last = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TICK_MS));
        tick();

        int c = getkey();
        if (c < 0) continue;
        int32_t step_counts = 0;
        switch (c) {
        case '+': case '=': step_counts = 10; break;
        case '-': case '_': step_counts = -10; break;
        case ']': step_counts = 100; break;
        case '[': step_counts = -100; break;
        case 'p': print_status(); break;
        case 'l':
            s_level_counts = shaft_counts();
            printf("  LEVEL marked: ENC_LEVEL_COUNTS = %.0f  (raw %u, %.2f deg from 6)\n",
                   s_level_counts, s_enc.raw, s_level_counts * 360.0f / ENC_COUNTS_PER_REV);
            print_status();
            break;
        case 's':
            speed_sweep();
            print_status();
            break;
        case 'q':
            printf("  returning to 6 and disabling...\n");
            goto_counts(s_enc_at_engage);
            stepper_halt();
            stepper_enable(false);
            printf("  driver OFF. Done -- reset to run again.\n");
            while (true) vTaskDelay(pdMS_TO_TICKS(10000));
        default: break;
        }
        if (step_counts) {
            float target = cmd_counts() + step_counts;
            // stay inside the working arc
            if (target < 50) target = 50;
            if (target > 2000) target = 2000;
            goto_counts(target);
            print_status();
        }
        fflush(stdout);
    }
}
#endif // BB_TESTS
