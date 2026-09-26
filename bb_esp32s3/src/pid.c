#include "pid.h"

void pid_init(pid_t *pid, const pid_cfg_t *cfg)
{
    pid->cfg = *cfg;
    pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->d_filt = 0.0f;
    pid->prev_meas = 0.0f;
    pid->primed = false;
    pid->p = pid->i = pid->d = pid->out = 0.0f;
}

float pid_update(pid_t *pid, float setpoint, float meas, float dt)
{
    const pid_cfg_t *c = &pid->cfg;
    float e = setpoint - meas;

    // Derivative of the measurement, first-order filtered. The first call
    // has no previous sample, so it contributes nothing rather than a spike.
    float d_raw = 0.0f;
    if (pid->primed && dt > 0.0f) {
        d_raw = (meas - pid->prev_meas) / dt;
    }
    pid->prev_meas = meas;
    pid->primed = true;
    if (c->d_tau > 0.0f && dt > 0.0f) {
        float a = dt / (c->d_tau + dt);
        pid->d_filt += a * (d_raw - pid->d_filt);
    } else {
        pid->d_filt = d_raw;
    }

    pid->p = c->kp * e;
    pid->d = -c->kd * pid->d_filt; // measurement rising == error falling

    // Unsaturated output with the integral as it stands decides whether
    // integrating further would only deepen saturation.
    float u = pid->p + pid->integral + pid->d;
    bool sat_hi = u > c->out_max;
    bool sat_lo = u < c->out_min;
    if (c->ki != 0.0f && dt > 0.0f && !((sat_hi && e > 0.0f) || (sat_lo && e < 0.0f))) {
        pid->integral += c->ki * e * dt;
        if (pid->integral > c->i_max) pid->integral = c->i_max;
        if (pid->integral < -c->i_max) pid->integral = -c->i_max;
    }
    pid->i = pid->integral;

    u = pid->p + pid->i + pid->d;
    if (u > c->out_max) u = c->out_max;
    if (u < c->out_min) u = c->out_min;
    pid->out = u;
    return u;
}
