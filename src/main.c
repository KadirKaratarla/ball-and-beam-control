#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#ifdef BB_TESTS
// Faz 1 -- isolated module bring-up tests (build with `pio run -e tests`).
// Exactly one test runs per build; pick it with ACTIVE_TEST below.

#include "test_tmc_bringup.h"
#include "test_as5600.h"
#include "test_hcsr04.h"
#include "test_pc_link.h"
#include "test_motor_encoder.h"
#include "test_motor_free.h"
#include "test_timer_jitter.h"

#define TEST_TMC_BRINGUP   1
#define TEST_TIMER_JITTER  2 // Faz 1.1
#define TEST_AS5600        3
#define TEST_HCSR04        4
#define TEST_PC_LINK       5 // Faz D
#define TEST_MOTOR_ENCODER 6 // Faz 3.3 / 3.4 first data
#define TEST_MOTOR_FREE    7 // shaft free: 1 rev = 4096 counts?

#define ACTIVE_TEST TEST_TIMER_JITTER

void app_main(void)
{
#if ACTIVE_TEST == TEST_TMC_BRINGUP
    test_tmc_bringup();
#elif ACTIVE_TEST == TEST_TIMER_JITTER
    test_timer_jitter();
#elif ACTIVE_TEST == TEST_AS5600
    test_as5600();
#elif ACTIVE_TEST == TEST_HCSR04
    test_hcsr04();
#elif ACTIVE_TEST == TEST_PC_LINK
    test_pc_link();
#elif ACTIVE_TEST == TEST_MOTOR_ENCODER
    test_motor_encoder();
#elif ACTIVE_TEST == TEST_MOTOR_FREE
    test_motor_free();
#else
#error "ACTIVE_TEST selects a test that has not been implemented yet"
#endif
}

#else
// Faz 2 -- the real application: timing skeleton with open-loop motion.

#include "config.h"
#include "encoder.h"
#include "stepper.h"
#include "pc_link.h"
#include "diag_task.h"
#include "telemetry.h"
#include "control_task.h"

static const char *TAG = "main";

static void halt(const char *why)
{
    ESP_LOGE(TAG, "%s -- not starting. Fix and reset.", why);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ball_beam Faz 2: %d us loop, open-loop triangle 6 -> +%d counts -> 6",
             CONTROL_LOOP_PERIOD_US, MOTION_TRAVEL_COUNTS);

    telemetry_init();

    if (encoder_init() != ESP_OK) {
        halt("encoder not answering");
    }
    // K-019: no homing. The beam must be resting at 6 with the driver off.
    encoder_sample_t enc;
    encoder_update(&enc);
    if (!enc.ok || enc.pos_counts < -ENC_START_WINDOW_COUNTS || enc.pos_counts > ENC_START_WINDOW_COUNTS) {
        ESP_LOGE(TAG, "shaft at %.1f counts from 6 (raw %u); expected within +-%d",
                 enc.pos_counts, enc.raw, ENC_START_WINDOW_COUNTS);
        halt("beam is not resting at 6");
    }
    ESP_LOGI(TAG, "shaft at 6 (%.1f counts, raw %u)", enc.pos_counts, enc.raw);

    if (stepper_init() != ESP_OK) {
        halt("stepper/TMC2208 init failed");
    }

    pc_link_init();
    diag_start();
    control_start();
}
#endif
