#include "control_task.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "soc/gpio_reg.h"

#include "config.h"
#include "encoder.h"
#include "stepper.h"
#include "pc_link.h"
#include "diag_task.h"
#include "telemetry.h"
#include "pid.h"
#include "linkage.h"

static const char *TAG = "control";

// Live configuration, mirrored for CONFIG replies (written by the control
// task, read by the link task; a torn read of a float is harmless here).
static volatile float s_setpoint_cm = BALL_SETPOINT_CM;
static volatile float s_kp = PID_KP, s_ki = PID_KI, s_kd = PID_KD, s_d_tau = PID_D_TAU_S;
static volatile bool s_gains_default = true;

void control_get_config(proto_config_t *out)
{
    out->version = PROTO_VERSION;
    out->kp = s_kp;
    out->ki = s_ki;
    out->kd = s_kd;
    out->d_tau_s = s_d_tau;
    out->theta_max_deg = BEAM_THETA_MAX_DEG;
    out->x_set_0p1mm = (int16_t)(s_setpoint_cm * 100.0f);
    out->vmax = STEP_VMAX_CLOSED_LOOP;
    out->amax = STEP_AMAX_CLOSED_LOOP;
    out->level_counts = ENC_LEVEL_COUNTS;
    out->irun = STEP_TMC_IRUN;
    out->ihold = STEP_TMC_IHOLD;
    out->defaults = s_gains_default ? 1 : 0;
}

static gptimer_handle_t s_timer;
static TaskHandle_t s_task;
static volatile int64_t s_isr_us;
static volatile uint32_t s_isr_count;

static bool IRAM_ATTR on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *ev, void *arg)
{
    s_isr_us = esp_timer_get_time();
    if (s_isr_count & 1) {
        REG_WRITE(GPIO_OUT_W1TC_REG, 1u << PIN_LOOP_PROBE);
    } else {
        REG_WRITE(GPIO_OUT_W1TS_REG, 1u << PIN_LOOP_PROBE);
    }
    s_isr_count++;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woken);
    return woken == pdTRUE;
}

// --- helpers -----------------------------------------------------------------

static void trip_fault(ctrl_state_t *state, fault_t *fault, fault_t why)
{
    stepper_halt();
    stepper_enable(false);
    *state = CTRL_STATE_FAULT;
    *fault = why;
}

static inline void acc(uint32_t *sum, uint32_t *max, uint32_t v)
{
    *sum += v;
    if (v > *max) *max = v;
}

// Beam angle command -> crank target in microsteps relative to engage.
// The crank is clamped to +-CRANK_CMD_MAX_COUNTS around level regardless
// of what the linkage says, so a bad theta can never drive it toward 12.
static int32_t crank_target_usteps(float theta_deg, float enc_at_engage, float *phi_out)
{
    float phi = 0.0f;
    if (!linkage_phi_from_theta(theta_deg, &phi)) {
        phi = *phi_out; // unreachable: keep the previous command
    }
    float counts = linkage_counts_from_phi(phi);
    if (counts > ENC_LEVEL_COUNTS + CRANK_CMD_MAX_COUNTS) counts = ENC_LEVEL_COUNTS + CRANK_CMD_MAX_COUNTS;
    if (counts < ENC_LEVEL_COUNTS - CRANK_CMD_MAX_COUNTS) counts = ENC_LEVEL_COUNTS - CRANK_CMD_MAX_COUNTS;
    *phi_out = phi;
    return (int32_t)((counts - enc_at_engage) * STEP_USTEPS_PER_COUNT);
}

// --- the loop ----------------------------------------------------------------

static void control_task(void *arg)
{
    fault_t fault = (fault_t)(uintptr_t)arg;
    ctrl_state_t state = fault == FAULT_NONE ? CTRL_STATE_WAIT : CTRL_STATE_FAULT;
    encoder_sample_t enc;
    pc_link_state_t link;
    telem_stats_t st;
    memset(&st, 0, sizeof(st));

    pid_cfg_t pid_cfg = {
        .kp = PID_KP, .ki = PID_KI, .kd = PID_KD,
        .out_min = -BEAM_THETA_MAX_DEG, .out_max = BEAM_THETA_MAX_DEG,
        .i_max = PID_I_MAX_DEG, .d_tau = PID_D_TAU_S,
    };
    pid_t pid;
    pid_init(&pid, &pid_cfg);

    uint32_t tick = 0;
    uint32_t wakes = 0;
    uint32_t isr_at_stats = 0;
    uint32_t enc_bad_ticks = 0;
    float enc_at_engage = 0.0f; // the pulse counter starts at 0 wherever the crank hangs
    int64_t start_at = esp_timer_get_time() + (int64_t)MOTION_START_DELAY_MS * 1000;
    int64_t t_origin = esp_timer_get_time();

    float x_cm = 0.0f, theta_cmd = 0.0f, phi_cmd = 0.0f;
    int32_t target_usteps = 0;
    // Slow encoder correction of the crank command (counts). The rotor sits
    // behind the field by a load-dependent angle (up to ~40 counts seen at
    // engage); this integrates (target - measured) while the crank is at
    // rest so the measured shaft, not the pulse count, lands on target.
    float lag_comp = 0.0f;
    bool level_only = false; // MODE LEVEL: hold level even with the ball in view
    // Crank target actually handed to the tracker. In RUN it only follows
    // target_usteps when the change exceeds CRANK_CMD_DEADBAND_COUNTS, so
    // camera noise (0.06 deg of theta = 1 deg of crank) does not keep the
    // motor hunting at rest.
    int32_t held_usteps = 0;
    int64_t engaged_at_us = 0;
    uint8_t last_seq = 0;
    int64_t last_frame_us = 0;      // arrival time of the last frame fed to the PID
    int64_t ball_seen_us = 0;       // last time a valid ball position arrived

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t t0 = esp_timer_get_time();
        // Copy now: if this tick runs late the ISR overwrites s_isr_us
        // mid-tick and the latency would come out negative.
        int64_t isr_us = s_isr_us;
        wakes++;
        tick++;

        // 1. sensors
        encoder_update(&enc);
        int64_t t1 = esp_timer_get_time();

        pc_link_get(&link);
        bool stale = pc_link_is_stale();
        bool ball_valid = (link.flags & PC_LINK_FLAG_VALID) != 0;
        bool new_frame = link.packets > 0 && link.seq != last_seq;
        if (new_frame) {
            last_seq = link.seq;
            if (ball_valid) ball_seen_us = link.received_at_us;
        }
        bool ball_lost = stale || (t0 - ball_seen_us) > (int64_t)GUARD_BALL_LOST_MS * 1000;

        int32_t step_count = stepper_get_step_count();
        float cmd_pos = (float)step_count / STEP_USTEPS_PER_COUNT;
        // Following error is relative to where the driver engaged: the crank
        // rests anywhere within +-40 counts of the stored 6, and that rest
        // offset is not a tracking error.
        float follow = cmd_pos - (enc.pos_counts - enc_at_engage);
        const diag_state_t *diag = diag_get();

        // 2a. commands from the PC (already validated by pc_link)
        pc_cmd_t cmd;
        while (pc_link_pop_cmd(&cmd)) {
            switch (cmd.type) {
            case PC_CMD_SETPOINT:
                s_setpoint_cm = cmd.u.setpoint_cm;
                break;
            case PC_CMD_GAINS:
                pid.cfg.kp = s_kp = cmd.u.gains.kp;
                pid.cfg.ki = s_ki = cmd.u.gains.ki;
                pid.cfg.kd = s_kd = cmd.u.gains.kd;
                pid.cfg.d_tau = s_d_tau = cmd.u.gains.d_tau_s;
                pid.integral = 0.0f; // new gains start from a clean integral
                s_gains_default = false;
                break;
            case PC_CMD_MODE:
                switch (cmd.u.mode) {
                case PROTO_MODE_STOP:
                    if (state != CTRL_STATE_FAULT) {
                        stepper_halt();
                        stepper_enable(false);
                        state = CTRL_STATE_STOP;
                    }
                    break;
                case PROTO_MODE_LEVEL:
                    if (state == CTRL_STATE_RUN) {
                        state = CTRL_STATE_LEVEL_HOLD;
                    }
                    level_only = true;
                    break;
                case PROTO_MODE_RUN:
                    level_only = false;
                    break;
                case PROTO_MODE_RESET_FAULT:
                    if (state == CTRL_STATE_FAULT || state == CTRL_STATE_STOP) {
                        // Re-engage from wherever the crank came to rest: the
                        // beam dropped toward 6 when the driver went off.
                        fault = FAULT_NONE;
                        lag_comp = 0.0f;
                        enc_bad_ticks = 0;
                        start_at = t0 + (int64_t)MOTION_START_DELAY_MS * 1000;
                        state = CTRL_STATE_WAIT;
                    }
                    break;
                }
                break;
            }
        }

        // 2b. guards
        if (state != CTRL_STATE_FAULT && state != CTRL_STATE_STOP) {
            enc_bad_ticks = enc.ok ? 0 : enc_bad_ticks + 1;
            if (enc_bad_ticks >= GUARD_ENC_BAD_TICKS) {
                trip_fault(&state, &fault, FAULT_ENC_DEAD);
            } else if (enc.ok && (enc.pos_counts < GUARD_ENC_MIN_COUNTS || enc.pos_counts > GUARD_ENC_MAX_COUNTS)) {
                trip_fault(&state, &fault, FAULT_ENC_RANGE);
            } else if (state != CTRL_STATE_WAIT && enc.ok && fabsf(follow) > GUARD_FOLLOW_ERR_COUNTS) {
                trip_fault(&state, &fault, FAULT_FOLLOW_ERR);
            } else if (!diag->driver_ok) {
                trip_fault(&state, &fault, diag->resets ? FAULT_TMC_RESET : FAULT_TMC_UART);
            }
        }

        // 3. state machine -> crank target
        switch (state) {
        case CTRL_STATE_WAIT:
            if (t0 >= start_at) {
                // K-019: no homing, but the beam must be resting near 6 so
                // the driver engages under low load with a known geometry.
                if (!enc.ok || enc.pos_counts < -ENC_START_WINDOW_COUNTS || enc.pos_counts > ENC_START_WINDOW_COUNTS) {
                    state = CTRL_STATE_FAULT;
                    fault = FAULT_NOT_AT_REST;
                    break;
                }
                stepper_enable(true); // near 6: low load
                engaged_at_us = t0;
                enc_at_engage = enc.pos_counts;
                target_usteps = 0; // hold still while the rotor snaps to its pole
                stepper_set_limits(STEP_VMAX_ENGAGE, STEP_AMAX_ENGAGE);
                theta_cmd = 0.0f;
                state = CTRL_STATE_ENGAGE;
            }
            break;

        case CTRL_STATE_ENGAGE: {
            // The rotor snaps to the nearest pole when the driver energises:
            // take the reference after that has settled, then move to level.
            if (t0 - engaged_at_us < ENGAGE_SETTLE_MS * 1000) break;
            if (t0 - engaged_at_us < (ENGAGE_SETTLE_MS + 4) * 1000) {
                enc_at_engage = enc.pos_counts;
                target_usteps = crank_target_usteps(0.0f, enc_at_engage, &phi_cmd);
                break;
            }
            // Arrived at level once the tracker reports zero velocity there.
            int32_t e = target_usteps - step_count;
            if (stepper_get_velocity() == 0.0f && (e < 0 ? -e : e) <= 8) {
                stepper_set_limits(STEP_VMAX_CLOSED_LOOP, STEP_AMAX_CLOSED_LOOP);
                pid_reset(&pid);
                last_frame_us = 0;
                state = CTRL_STATE_LEVEL_HOLD;
            }
            break;
        }

        case CTRL_STATE_LEVEL_HOLD:
            theta_cmd = 0.0f;
            target_usteps = crank_target_usteps(0.0f, enc_at_engage, &phi_cmd);
            if (!ball_lost && !level_only) {
                pid_reset(&pid);
                last_frame_us = 0;
                state = CTRL_STATE_RUN;
            }
            break;

        case CTRL_STATE_RUN:
            if (ball_lost) {
                state = CTRL_STATE_LEVEL_HOLD;
                theta_cmd = 0.0f;
                target_usteps = crank_target_usteps(0.0f, enc_at_engage, &phi_cmd);
                break;
            }
            if (new_frame && ball_valid) {
                x_cm = link.pos_0p1mm * 0.01f;
                float dt = last_frame_us ? (link.received_at_us - last_frame_us) * 1e-6f : PID_DT_NOMINAL_S;
                if (dt < 0.005f) dt = 0.005f;
                if (dt > 0.050f) dt = 0.050f;
                last_frame_us = link.received_at_us;
                theta_cmd = pid_update(&pid, s_setpoint_cm, x_cm, dt);
                target_usteps = crank_target_usteps(theta_cmd, enc_at_engage, &phi_cmd);
            }
            break;

        case CTRL_STATE_FAULT:
        case CTRL_STATE_STOP:
            break;
        }
        int64_t t2 = esp_timer_get_time();

        // 4. actuate (deadband in RUN, encoder lag correction in LEVEL/RUN)
        {
            int32_t delta = target_usteps - held_usteps;
            if (state != CTRL_STATE_RUN || delta >= CRANK_CMD_DEADBAND_USTEPS || delta <= -CRANK_CMD_DEADBAND_USTEPS) {
                held_usteps = target_usteps;
            }
        }
        if ((state == CTRL_STATE_LEVEL_HOLD || state == CTRL_STATE_RUN) && enc.ok &&
            stepper_get_velocity() == 0.0f) {
            float target_counts = enc_at_engage + (float)held_usteps / STEP_USTEPS_PER_COUNT;
            lag_comp += (target_counts - enc.pos_counts) * (CONTROL_LOOP_PERIOD_US / 1e6f / LAG_COMP_TAU_S);
            if (lag_comp > LAG_COMP_MAX_COUNTS) lag_comp = LAG_COMP_MAX_COUNTS;
            if (lag_comp < -LAG_COMP_MAX_COUNTS) lag_comp = -LAG_COMP_MAX_COUNTS;
        }
        int32_t applied = held_usteps + (int32_t)(lag_comp * STEP_USTEPS_PER_COUNT);
        bool driving = state == CTRL_STATE_ENGAGE || state == CTRL_STATE_LEVEL_HOLD || state == CTRL_STATE_RUN;
        float vel = driving ? stepper_track(applied) : 0.0f;
        int64_t t3 = esp_timer_get_time();

        // 5. bookkeeping (cheap; the queue copy is the only real cost)
        acc(&st.enc_us_sum, &st.enc_us_max, (uint32_t)(t1 - t0));
        acc(&st.calc_us_sum, &st.calc_us_max, (uint32_t)(t2 - t1));
        acc(&st.step_us_sum, &st.step_us_max, (uint32_t)(t3 - t2));
        acc(&st.total_us_sum, &st.total_us_max, (uint32_t)(t3 - t0));
        acc(&st.lat_us_sum, &st.lat_us_max, t0 > isr_us ? (uint32_t)(t0 - isr_us) : 0);
        if (t3 - t0 > CONTROL_LOOP_PERIOD_US) st.overruns++;
        st.ticks++;

        if (tick % TELEM_SAMPLE_DECIMATION == 0) {
            telem_msg_t m = { .kind = TELEM_SAMPLE };
            m.u.sample = (telem_sample_t){
                .t_ms = (uint32_t)((t0 - t_origin) / 1000),
                .x_cm = x_cm,
                .x_set_cm = s_setpoint_cm,
                .ball_valid = ball_valid && !stale,
                .p = pid.p, .i = pid.i, .d = pid.d,
                .theta_cmd = theta_cmd,
                .phi_cmd = phi_cmd,
                .enc_pos = enc.pos_counts,
                .follow_err = follow,
                .lag_comp = lag_comp,
                .cmd_vel = vel,
                .link_age_ms = (uint16_t)((t0 - link.received_at_us) / 1000 > 65535 ? 65535 : (t0 - link.received_at_us) / 1000),
                .loop_us = (uint16_t)(t3 - t0),
                .state = state,
                .fault = fault,
                .link_stale = stale,
                .driver_on = stepper_is_enabled(),
                .last_seq = last_seq,
            };
            telemetry_push(&m);
        }

        if (tick % TELEM_STATS_TICKS == 0) {
            uint32_t isrs = s_isr_count - isr_at_stats;
            st.missed_ticks = isrs > wakes ? isrs - wakes : 0;
            st.enc_i2c_errors = enc.i2c_errors;
            st.enc_rejects = enc.rejects;
            st.enc_dir_faults = enc.dir_faults;
            st.link_packets = link.packets;
            st.link_crc_errors = link.crc_errors;
            st.link_seq_gaps = link.seq_gaps;
            st.cmd_rx = link.cmd_rx;
            st.cmd_bad = link.cmd_bad;
            st.tx_dropped = link.tx_dropped;
            st.tmc_resets = diag->resets;
            st.tmc_uart_errors = diag->uart_errors;
            st.drv_status = diag->drv_status;
            st.state = state;
            st.fault = fault;
            telem_msg_t m = { .kind = TELEM_STATS };
            m.u.stats = st;
            telemetry_push(&m);
            memset(&st, 0, sizeof(st));
            wakes = 0;
            isr_at_stats = s_isr_count;
        }
    }
}

void control_start(fault_t initial_fault)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_LOOP_PROBE,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_LOOP_PROBE, 0);

    xTaskCreatePinnedToCore(control_task, "control", 6144, (void *)(uintptr_t)initial_fault,
                            TASK_CONTROL_PRIO, &s_task, TASK_CONTROL_CORE);

    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &s_timer));
    gptimer_event_callbacks_t cbs = { .on_alarm = on_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, NULL));
    gptimer_alarm_config_t alarm = {
        .alarm_count = CONTROL_LOOP_PERIOD_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_timer, &alarm));
    ESP_ERROR_CHECK(gptimer_enable(s_timer));
    ESP_ERROR_CHECK(gptimer_start(s_timer));

    if (initial_fault == FAULT_NONE) {
        ESP_LOGI(TAG, "loop %d us on core %d prio %d; driver engages in %d ms and moves to level -- keep clear",
                 CONTROL_LOOP_PERIOD_US, TASK_CONTROL_CORE, TASK_CONTROL_PRIO, MOTION_START_DELAY_MS);
    } else {
        ESP_LOGW(TAG, "starting in FAULT %d; waiting for RESET_FAULT from the PC", initial_fault);
    }
}
