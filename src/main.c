// Faz 1 -- isolated module bring-up tests.
// Exactly one test runs per build; pick it with ACTIVE_TEST below.

#include "esp_log.h"

#include "test_tmc_bringup.h"
#include "test_as5600.h"
#include "test_hcsr04.h"
#include "test_pc_link.h"

#define TEST_TMC_BRINGUP  1
#define TEST_TIMER_JITTER 2 // Faz 1.1, waiting on a logic analyser
#define TEST_AS5600       3
#define TEST_HCSR04       4
#define TEST_PC_LINK      5 // Faz D

#define ACTIVE_TEST TEST_PC_LINK

void app_main(void)
{
#if ACTIVE_TEST == TEST_TMC_BRINGUP
    test_tmc_bringup();
#elif ACTIVE_TEST == TEST_AS5600
    test_as5600();
#elif ACTIVE_TEST == TEST_HCSR04
    test_hcsr04();
#elif ACTIVE_TEST == TEST_PC_LINK
    test_pc_link();
#else
#error "ACTIVE_TEST selects a test that has not been implemented yet"
#endif
}
