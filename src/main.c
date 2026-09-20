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
#include "test_pid_sim.h"
#include "test_jog.h"

#define TEST_TMC_BRINGUP   1
#define TEST_TIMER_JITTER  2 // Faz 1.1
#define TEST_AS5600        3
#define TEST_HCSR04        4
#define TEST_PC_LINK       5 // Faz D
#define TEST_MOTOR_ENCODER 6 // Faz 3.3 / 3.4 first data
#define TEST_MOTOR_FREE    7 // shaft free: 1 rev = 4096 counts?
#define TEST_PID_SIM       8 // Faz 3.1, nothing connected
#define TEST_JOG           9 // Faz 3.3, interactive: level offset + speed sweep

#define ACTIVE_TEST TEST_JOG

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
#elif ACTIVE_TEST == TEST_PID_SIM
    test_pid_sim();
#elif ACTIVE_TEST == TEST_JOG
    test_jog();
#else
#error "ACTIVE_TEST selects a test that has not been implemented yet"
#endif
}

#else
// The application: 4 ms loop, PID on the camera ball position (Faz 3).

#include "config.h"
#include "encoder.h"
#include "stepper.h"
#include "pc_link.h"
#include "diag_task.h"
#include "telemetry.h"
#include "control_task.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "ball_beam Faz 4: %d us loop, PID on the camera position, setpoint %.1f cm, "
             "Kp %.2f Ki %.2f Kd %.2f, theta max %.1f deg, level %d counts",
             CONTROL_LOOP_PERIOD_US, BALL_SETPOINT_CM, PID_KP, PID_KI, PID_KD, BEAM_THETA_MAX_DEG, ENC_LEVEL_COUNTS);

    // The PC link comes up first, whatever else fails: the operator must
    // always be able to see the board's state and the reason it refuses to
    // run. Hardware faults start the control task in FAULT.
    telemetry_init();
    pc_link_init();

    fault_t initial = FAULT_NONE;
    if (encoder_init() != ESP_OK) {
        ESP_LOGE(TAG, "encoder not answering -- starting in FAULT");
        initial = FAULT_ENC_DEAD;
    } else if (stepper_init() != ESP_OK) {
        ESP_LOGE(TAG, "stepper/TMC2208 init failed -- starting in FAULT");
        initial = FAULT_TMC_UART;
    }
    // The rest-at-6 check (K-019) happens in the control task before it
    // engages, so a beam left mid-air is a recoverable FAULT_NOT_AT_REST.

    diag_start();
    control_start(initial);
}
#endif
