#pragma once

// The 4 ms control loop (Faz 2: timing skeleton, open-loop motion).
//
// GPTimer ISR on core 0 timestamps, toggles PIN_LOOP_PROBE and notifies the
// control task (highest priority, core 0). Each tick the task reads the
// encoder, snapshots the PC link, runs the guards, advances the open-loop
// profile, programs the step generator and pushes telemetry. It never
// logs (K-022). Every phase is timed so the Faz 2.2 budget comes from
// measurement.

#include "telemetry.h"

// Creates the task and starts the timer. With initial_fault == FAULT_NONE
// the driver engages after MOTION_START_DELAY_MS (if the beam rests near
// 6); otherwise the task starts latched in FAULT and waits for a
// RESET_FAULT command. Call after encoder/stepper/link init.
void control_start(fault_t initial_fault);
