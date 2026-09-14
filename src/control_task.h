#pragma once

// The 4 ms control loop (Faz 2: timing skeleton, open-loop motion).
//
// GPTimer ISR on core 0 timestamps, toggles PIN_LOOP_PROBE and notifies the
// control task (highest priority, core 0). Each tick the task reads the
// encoder, snapshots the PC link, runs the guards, advances the open-loop
// profile, programs the step generator and pushes telemetry. It never
// logs (K-022). Every phase is timed so the Faz 2.2 budget comes from
// measurement.

// Creates the task and starts the timer. The driver stays off until
// MOTION_START_DELAY_MS have passed. Call after encoder/stepper/link init.
void control_start(void);
