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

static const char *TAG = "control";

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

// --- open-loop profile: dwell at 6, move to +TRAVEL, dwell, move back ---

typedef enum { SEG_DWELL, SEG_MOVE } seg_t;

typedef struct {
    seg_t seg;
    int64_t dwell_until_us;
    int32_t target_usteps; // 0 or TRAVEL
} profile_t;

// Returns the velocity the profile wants this tick. Trapezoid: full speed
// until the stopping distance for STEP_AMAX is reached, then sqrt decel.
static float profile_step(profile_t *p, int32_t step_count, int64_t now)
{
    if (p->seg == SEG_DWELL) {
        if (now >= p->dwell_until_us) {
            p->seg = SEG_MOVE;
            p->target_usteps = p->target_usteps ? 0 : (int32_t)(MOTION_TRAVEL_COUNTS * STEP_USTEPS_PER_COUNT);
        }
        return 0.0f;
    }

    int32_t e = p->target_usteps - step_count;
    int32_t mag = e < 0 ? -e : e;
    if (mag <= 8) { // 8 usteps = 0.06 deg
        p->seg = SEG_DWELL;
        p->dwell_until_us = now + (int64_t)MOTION_DWELL_MS * 1000;
        return 0.0f;
    }
    float v = sqrtf(2.0f * STEP_AMAX * (float)mag);
    if (v > STEP_VMAX) v = STEP_VMAX;
    return e < 0 ? -v : v;
}

// --- the loop ---

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

static void control_task(void *arg)
{
    ctrl_state_t state = CTRL_STATE_WAIT;
    fault_t fault = FAULT_NONE;
    profile_t prof = { .seg = SEG_DWELL, .dwell_until_us = 0, .target_usteps = 0 };
    encoder_sample_t enc;
    pc_link_state_t link;
    telem_stats_t st;
    memset(&st, 0, sizeof(st));

    uint32_t tick = 0;
    uint32_t wakes = 0;
    uint32_t isr_at_stats = 0;
    uint32_t enc_bad_ticks = 0;
    float enc_at_engage = 0.0f; // the pulse counter starts at 0 wherever the crank hangs
    int64_t start_at = esp_timer_get_time() + (int64_t)MOTION_START_DELAY_MS * 1000;
    int64_t t_origin = esp_timer_get_time();

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
        int32_t step_count = stepper_get_step_count();
        float cmd_pos = (float)step_count / STEP_USTEPS_PER_COUNT;
        // Following error is relative to where the driver engaged: the crank
        // rests anywhere within +-40 counts of the stored 6, and that rest
        // offset is not a tracking error.
        float follow = cmd_pos - (enc.pos_counts - enc_at_engage);
        const diag_state_t *diag = diag_get();

        // 2. guards
        if (state != CTRL_STATE_FAULT) {
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

        // 3. state machine -> velocity request
        float v_req = 0.0f;
        switch (state) {
        case CTRL_STATE_WAIT:
            if (t0 >= start_at) {
                stepper_enable(true); // at 6: zero load, nothing moves
                enc_at_engage = enc.pos_counts;
                prof.seg = SEG_DWELL;
                prof.dwell_until_us = t0 + (int64_t)MOTION_DWELL_MS * 1000;
                prof.target_usteps = 0;
                state = CTRL_STATE_RUN;
            }
            break;
        case CTRL_STATE_RUN:
            if (GUARD_LINK_STALE_STOPS && stale) {
                state = CTRL_STATE_HOLD;
            } else {
                v_req = profile_step(&prof, step_count, t0);
            }
            break;
        case CTRL_STATE_HOLD:
            if (!stale) state = CTRL_STATE_RUN;
            break;
        case CTRL_STATE_FAULT:
            break;
        }
        int64_t t2 = esp_timer_get_time();

        // 4. actuate
        float vel = (state == CTRL_STATE_FAULT) ? 0.0f : stepper_tick(v_req);
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
                .enc_raw = enc.raw,
                .enc_pos = enc.pos_counts,
                .enc_ok = enc.ok,
                .pos_0p1mm = link.pos_0p1mm,
                .pos_valid = (link.flags & PC_LINK_FLAG_VALID) != 0,
                .link_stale = stale,
                .cmd_vel = vel,
                .cmd_pos = cmd_pos,
                .follow_err = follow,
                .loop_us = (uint16_t)(t3 - t0),
                .state = state,
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

void control_start(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_LOOP_PROBE,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_LOOP_PROBE, 0);

    xTaskCreatePinnedToCore(control_task, "control", 6144, NULL, TASK_CONTROL_PRIO, &s_task, TASK_CONTROL_CORE);

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

    ESP_LOGI(TAG, "loop %d us on core %d prio %d; driver engages in %d ms -- keep clear",
             CONTROL_LOOP_PERIOD_US, TASK_CONTROL_CORE, TASK_CONTROL_PRIO, MOTION_START_DELAY_MS);
}
