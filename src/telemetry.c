#include "telemetry.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "config.h"
#include "pc_link.h"
#include "protocol.h"

static const char *TAG = "telem";

static QueueHandle_t s_queue;
static uint32_t s_dropped;

static const char *state_name(uint8_t s)
{
    switch (s) {
    case CTRL_STATE_WAIT: return "WAIT";
    case CTRL_STATE_ENGAGE: return "ENGAGE";
    case CTRL_STATE_LEVEL_HOLD: return "LEVEL";
    case CTRL_STATE_RUN: return "RUN";
    case CTRL_STATE_FAULT: return "FAULT";
    case CTRL_STATE_STOP: return "STOP";
    case CTRL_STATE_OPENLOOP: return "OPEN";
    default: return "?";
    }
}

static const char *fault_name(uint8_t f)
{
    switch (f) {
    case FAULT_NONE: return "none";
    case FAULT_ENC_RANGE: return "ENC_RANGE";
    case FAULT_FOLLOW_ERR: return "FOLLOW_ERR";
    case FAULT_ENC_DEAD: return "ENC_DEAD";
    case FAULT_TMC_RESET: return "TMC_RESET";
    case FAULT_TMC_UART: return "TMC_UART";
    case FAULT_NOT_AT_REST: return "NOT_AT_REST";
    default: return "?";
    }
}

static void print_sample(const telem_sample_t *s)
{
    printf("S,%lu,%.2f,%.1f,%d,%+.2f,%+.2f,%+.2f,%+.2f,%+.1f,%.1f,%+.1f,%+.1f,%.0f,%u,%u,%s\n",
           (unsigned long)s->t_ms, s->x_cm, s->x_set_cm, s->ball_valid,
           s->p, s->i, s->d, s->theta_cmd, s->phi_cmd, s->enc_pos, s->follow_err, s->lag_comp,
           s->cmd_vel, s->link_age_ms, s->loop_us, state_name(s->state));
}

static void print_stats(const telem_stats_t *st)
{
    uint32_t n = st->ticks ? st->ticks : 1;
    printf("B,ticks=%lu enc=%lu/%lu calc=%lu/%lu step=%lu/%lu total=%lu/%lu lat=%lu/%lu us (mean/max) "
           "budget=%.1f%% overrun=%lu missed=%lu | enc i2c=%lu rej=%lu dir=%lu | link pkt=%lu crc=%lu gap=%lu "
           "| tmc reset=%lu uart=%lu drv=0x%08lX | %s%s%s | dropped=%lu\n",
           (unsigned long)st->ticks,
           (unsigned long)(st->enc_us_sum / n), (unsigned long)st->enc_us_max,
           (unsigned long)(st->calc_us_sum / n), (unsigned long)st->calc_us_max,
           (unsigned long)(st->step_us_sum / n), (unsigned long)st->step_us_max,
           (unsigned long)(st->total_us_sum / n), (unsigned long)st->total_us_max,
           (unsigned long)(st->lat_us_sum / n), (unsigned long)st->lat_us_max,
           100.0 * st->total_us_max / CONTROL_LOOP_PERIOD_US,
           (unsigned long)st->overruns, (unsigned long)st->missed_ticks,
           (unsigned long)st->enc_i2c_errors, (unsigned long)st->enc_rejects, (unsigned long)st->enc_dir_faults,
           (unsigned long)st->link_packets, (unsigned long)st->link_crc_errors, (unsigned long)st->link_seq_gaps,
           (unsigned long)st->tmc_resets, (unsigned long)st->tmc_uart_errors, (unsigned long)st->drv_status,
           state_name(st->state), st->fault ? " fault=" : "", st->fault ? fault_name(st->fault) : "",
           (unsigned long)s_dropped);
}

static inline int16_t clamp16(float v)
{
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

static void send_sample(const telem_sample_t *s)
{
    proto_telem_t t = {
        .t_ms = s->t_ms,
        .x_0p1mm = clamp16(s->x_cm * 100.0f),
        .x_set_0p1mm = clamp16(s->x_set_cm * 100.0f),
        .p_0p01deg = clamp16(s->p * 100.0f),
        .i_0p01deg = clamp16(s->i * 100.0f),
        .d_0p01deg = clamp16(s->d * 100.0f),
        .theta_0p01deg = clamp16(s->theta_cmd * 100.0f),
        .phi_0p1deg = clamp16(s->phi_cmd * 10.0f),
        .enc_counts = clamp16(s->enc_pos),
        .follow_0p1 = clamp16(s->follow_err * 10.0f),
        .lag_counts = (int8_t)(s->lag_comp > 127 ? 127 : s->lag_comp < -128 ? -128 : s->lag_comp),
        .state = s->state,
        .fault = s->fault,
        .flags = (uint8_t)((s->ball_valid ? PROTO_TF_BALL_VALID : 0) |
                           (s->link_stale ? PROTO_TF_LINK_STALE : 0) |
                           (s->driver_on ? PROTO_TF_DRIVER_ON : 0)),
        .last_seq = s->last_seq,
        .loop_us = s->loop_us,
        .cmd_vel_10 = clamp16(s->cmd_vel / 10.0f),
    };
    pc_link_send(PROTO_T_TELEM, &t, sizeof(t));
}

static inline uint16_t clampu16(uint32_t v)
{
    return v > 65535 ? 65535 : (uint16_t)v;
}

static void send_stats(const telem_stats_t *st)
{
    uint32_t n = st->ticks ? st->ticks : 1;
    proto_health_t h = {
        .ticks = clampu16(st->ticks),
        .loop_us_mean = clampu16(st->total_us_sum / n),
        .loop_us_max = clampu16(st->total_us_max),
        .overruns = clampu16(st->overruns),
        .missed = clampu16(st->missed_ticks),
        .enc_i2c_errors = st->enc_i2c_errors,
        .enc_rejects = st->enc_rejects,
        .enc_dir_faults = st->enc_dir_faults,
        .link_packets = st->link_packets,
        .link_crc_errors = st->link_crc_errors,
        .link_seq_gaps = st->link_seq_gaps,
        .cmd_rx = st->cmd_rx,
        .cmd_bad = st->cmd_bad,
        .tx_dropped = st->tx_dropped,
        .tmc_resets = clampu16(st->tmc_resets),
        .tmc_uart_errors = clampu16(st->tmc_uart_errors),
        .drv_status = st->drv_status,
        .state = st->state,
        .fault = st->fault,
    };
    pc_link_send(PROTO_T_HEALTH, &h, sizeof(h));
}

static void telemetry_task(void *arg)
{
    telem_msg_t msg;
    uint32_t n_samples = 0;
    while (true) {
        if (xQueueReceive(s_queue, &msg, portMAX_DELAY) != pdTRUE) continue;
        switch (msg.kind) {
        case TELEM_SAMPLE:
            send_sample(&msg.u.sample);
            if (TELEM_CONSOLE && (n_samples++ % TELEM_CONSOLE_DECIMATION) == 0) print_sample(&msg.u.sample);
            break;
        case TELEM_STATS:
            send_stats(&msg.u.stats);
            if (TELEM_CONSOLE) print_stats(&msg.u.stats);
            break;
        }
    }
}

void telemetry_init(void)
{
    s_queue = xQueueCreate(TELEM_QUEUE_LEN, sizeof(telem_msg_t));
    xTaskCreatePinnedToCore(telemetry_task, "telem", 4096, NULL, TASK_TELEM_PRIO, NULL, TASK_AUX_CORE);
    ESP_LOGI(TAG, "binary TELEM at %d Hz, HEALTH at 1 Hz on the USB link; console rows %s",
             1000000 / CONTROL_LOOP_PERIOD_US / TELEM_SAMPLE_DECIMATION, TELEM_CONSOLE ? "on" : "off");
    if (TELEM_CONSOLE) {
        ESP_LOGI(TAG, "S rows: t_ms,x_cm,x_set,ball,P,I,D,theta_cmd,phi_cmd,enc_pos,follow_err,lag_comp,cmd_vel,link_age_ms,loop_us,state");
    }
}

bool telemetry_push(const telem_msg_t *msg)
{
    if (xQueueSend(s_queue, msg, 0) != pdTRUE) {
        s_dropped++;
        return false;
    }
    return true;
}
