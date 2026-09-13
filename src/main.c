// Faz 1 -- isolated module bring-up tests.
// Exactly one test runs per build; pick it with ACTIVE_TEST below.

#include "esp_log.h"

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
