#pragma once

// Plain PID, no hardware, no globals: the same code runs in the control
// task and in the on-target simulation test.
//
// Derivative acts on the measurement (not the error) so a setpoint step
// does not kick the output, and is low-pass filtered because it comes
// from a 100 Hz camera position with sub-millimetre noise. Anti-windup is
// conditional integration: the integral stops moving in the direction
// that keeps the output saturated, and is clamped on its own.

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float out_min, out_max; // output saturation
    float i_max;            // |ki * integral| clamp
    float d_tau;            // derivative filter time constant, s (0 = none)
} pid_cfg_t;

typedef struct {
    pid_cfg_t cfg;
    float integral;  // ki-scaled contribution
    float d_filt;    // filtered derivative of the measurement
    float prev_meas;
    bool primed;
    // last update, for telemetry
    float p, i, d, out;
} pid_t;

void pid_init(pid_t *pid, const pid_cfg_t *cfg);
void pid_reset(pid_t *pid);

// One update: setpoint and measurement in the same unit, dt in seconds.
// Returns the saturated output (also kept in pid->out).
float pid_update(pid_t *pid, float setpoint, float meas, float dt);
