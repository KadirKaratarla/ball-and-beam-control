#pragma once

// Core 1 housekeeping: the PC link poller and the TMC2208 watchdog.
//
// The TMC2208's registers are volatile and default to 1/8 stepping on a
// reset (K-007), so GSTAT.reset is polled and any reset is latched as a
// fault for the control task to act on. DRV_STATUS is captured for
// telemetry. None of this touches STEP/DIR/EN.

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    volatile bool driver_ok;      // false once a reset or UART failure is seen
    volatile uint32_t resets;
    volatile uint32_t uart_errors;
    volatile uint32_t drv_status; // last DRV_STATUS read
} diag_state_t;

void diag_start(void);
const diag_state_t *diag_get(void);

// Re-arm after the operator has cleared a fault: the TMC gets another
// chance to answer (it stays silent while the 12 V supply is off).
void diag_clear_driver(void);
