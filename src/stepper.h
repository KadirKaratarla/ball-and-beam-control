#pragma once

// Step generation for the control loop.
//
// STEP is a hardware square wave from LEDC whose frequency is the speed, so
// the control task never bit-bangs; it sets a new frequency once per tick.
// The same STEP pin is looped back into PCNT with DIR as the count
// direction, so the commanded shaft position is the true number of pulses
// the driver saw, not an integral of requested frequencies. Velocity is
// ramped here (STEP_AMAX) so the controller can ask for anything and the
// rotor still sees a continuous profile (K-009, K-020).

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// TMC UART bring-up with read-back verification, LEDC and PCNT setup.
// Leaves the driver DISABLED and pulses off.
esp_err_t stepper_init(void);

// Writes GCONF / CHOPCONF / IHOLD_IRUN and reads them back. Used at init
// and by the diag task after a driver reset. Only call from one task at a
// time (the UART is not locked).
bool stepper_configure_tmc(void);

void stepper_enable(bool on);
bool stepper_is_enabled(void);

// One tick: move the actual velocity toward `target_usteps_per_s` under
// the acceleration limit and program the pulse generator. Positive is
// toward 12. Returns the velocity now being generated.
float stepper_tick(float target_usteps_per_s);

// Immediately stops pulses and zeroes the velocity (no ramp). For faults.
void stepper_halt(void);

float stepper_get_velocity(void);

// Pulses emitted so far, signed, + toward 12 (PCNT).
int32_t stepper_get_step_count(void);
