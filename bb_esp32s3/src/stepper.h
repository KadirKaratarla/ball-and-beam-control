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
// Leaves the driver DISABLED and pulses off. Safe to call again after a
// failure (no 12 V at boot, say): it retries from scratch and returns
// ESP_OK immediately once it has succeeded.
esp_err_t stepper_init(void);

// False until init has succeeded; every other entry point is a no-op
// while it is false.
bool stepper_is_ready(void);

// Writes GCONF / CHOPCONF / IHOLD_IRUN and reads them back. Used at init
// and by the diag task after a driver reset. Only call from one task at a
// time (the UART is not locked).
bool stepper_configure_tmc(void);

void stepper_enable(bool on);
bool stepper_is_enabled(void);

// Velocity/acceleration limits used by stepper_tick / stepper_track.
// Defaults to STEP_VMAX / STEP_AMAX; the closed loop needs much more
// (see STEP_VMAX_CLOSED_LOOP), validated by test_jog before use.
void stepper_set_limits(float vmax_usteps_per_s, float amax_usteps_per_s2);

// One tick: move the actual velocity toward `target_usteps_per_s` under
// the acceleration limit and program the pulse generator. Positive is
// toward 12. Returns the velocity now being generated.
float stepper_tick(float target_usteps_per_s);

// One tick of position tracking: full speed toward `target_usteps`, then
// the sqrt deceleration that lands there under the acceleration limit.
// Returns the velocity now being generated. Arrived when it returns 0.
float stepper_track(int32_t target_usteps);

// Immediately stops pulses and zeroes the velocity (no ramp). For faults.
void stepper_halt(void);

// Zeroes the pulse counter (new position reference, e.g. at re-engage).
void stepper_reset_position(void);

float stepper_get_velocity(void);

// Pulses emitted so far, signed, + toward 12 (PCNT).
int32_t stepper_get_step_count(void);
