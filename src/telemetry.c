#include "telemetry.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "config.h"

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

static void telemetry_task(void *arg)
{
    telem_msg_t msg;
    while (true) {
        if (xQueueReceive(s_queue, &msg, portMAX_DELAY) != pdTRUE) continue;
        switch (msg.kind) {
        case TELEM_SAMPLE: print_sample(&msg.u.sample); break;
        case TELEM_STATS: print_stats(&msg.u.stats); break;
        }
    }
}

void telemetry_init(void)
{
    s_queue = xQueueCreate(TELEM_QUEUE_LEN, sizeof(telem_msg_t));
    xTaskCreatePinnedToCore(telemetry_task, "telem", 4096, NULL, TASK_TELEM_PRIO, NULL, TASK_AUX_CORE);
    ESP_LOGI(TAG, "S rows: t_ms,x_cm,x_set,ball,P,I,D,theta_cmd,phi_cmd,enc_pos,follow_err,lag_comp,cmd_vel,link_age_ms,loop_us,state");
}

bool telemetry_push(const telem_msg_t *msg)
{
    if (xQueueSend(s_queue, msg, 0) != pdTRUE) {
        s_dropped++;
        return false;
    }
    return true;
}
