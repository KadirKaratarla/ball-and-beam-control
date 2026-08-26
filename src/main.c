// Faz 1 -- isolated module bring-up tests.
// Exactly one test runs per build; pick it with ACTIVE_TEST below.

#include "esp_log.h"

#include "test_tmc_bringup.h"

#define TEST_TMC_BRINGUP  1
#define TEST_TIMER_JITTER 2 // Faz 1.1, not written yet
#define TEST_AS5600       3 // Faz 1.2, not written yet
#define TEST_HCSR04       4 // Faz 1.4, not written yet

#define ACTIVE_TEST TEST_TMC_BRINGUP

void app_main(void)
{
#if ACTIVE_TEST == TEST_TMC_BRINGUP
    test_tmc_bringup();
#else
#error "ACTIVE_TEST selects a test that has not been implemented yet"
#endif
}
