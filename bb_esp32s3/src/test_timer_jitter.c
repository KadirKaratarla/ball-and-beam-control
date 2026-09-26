#ifdef BB_TESTS // Faz 1 bring-up test: built only in the `tests` environment
#include "test_timer_jitter.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "soc/gpio_reg.h"

#include "board_pins.h"

// Everything the control loop's timing will rest on, measured in isolation.
//
// The ISR does the absolute minimum: read the microsecond clock, flip the
// probe pin, hand off. The interval between consecutive ISR timestamps is
// the period as the loop will experience it; its spread is the jitter. The
// gap from the ISR timestamp to the woken task is the extra latency a
// task-based loop pays on top of that -- the number that decides whether
// the loop body can live in a task or has to be in the ISR itself.

#define PERIOD_US CONTROL_LOOP_PERIOD_US
#define REPORT_US 1000000
#define RING 4096 // power of two; ISR writes, task drains

#define JITTER_LIMIT_US 50 // acceptance from the plan

static const char *TAG = "jitter";

static gptimer_handle_t s_timer;
static TaskHandle_t s_task;

static volatile int64_t s_prev_isr_us;
static volatile int64_t s_last_isr_us;
static volatile uint32_t s_isr_count;
static volatile int32_t s_ring[RING];
static volatile uint32_t s_head;

static bool IRAM_ATTR on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *ev, void *arg)
{
    int64_t now = esp_timer_get_time();

    // Direct register write: cheapest possible toggle, no driver call in
    // the ISR. PIN_LOOP_PROBE is below 32 so the low OUT registers apply.
    if (s_isr_count & 1) {
        REG_WRITE(GPIO_OUT_W1TC_REG, 1u << PIN_LOOP_PROBE);
    } else {
        REG_WRITE(GPIO_OUT_W1TS_REG, 1u << PIN_LOOP_PROBE);
    }

    if (s_prev_isr_us != 0) {
        s_ring[s_head & (RING - 1)] = (int32_t)(now - s_prev_isr_us);
        s_head++;
    }
    s_prev_isr_us = now;
    s_last_isr_us = now;
    s_isr_count++;

    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woken);
    return woken == pdTRUE;
}

typedef struct {
    uint32_t n;
    int32_t min, max;
    int64_t sum, sumsq;
    uint32_t h_10, h_20, h_50, h_100, h_over; // |deviation| buckets, us
} stats_t;

static void stats_reset(stats_t *s)
{
    memset(s, 0, sizeof(*s));
    s->min = INT32_MAX;
    s->max = INT32_MIN;
}

static void stats_add(stats_t *s, int32_t v)
{
    s->n++;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    s->sum += v;
    s->sumsq += (int64_t)v * v;
}

static void stats_add_dev(stats_t *s, int32_t dev)
{
    uint32_t a = dev < 0 ? -dev : dev;
    if (a < 10) s->h_10++;
    else if (a < 20) s->h_20++;
    else if (a < 50) s->h_50++;
    else if (a < 100) s->h_100++;
    else s->h_over++;
}

static double stats_std(const stats_t *s)
{
    if (s->n < 2) return 0.0;
    double mean = (double)s->sum / s->n;
    double var = (double)s->sumsq / s->n - mean * mean;
    return var > 0 ? __builtin_sqrt(var) : 0.0;
}

static void loop_task(void *arg)
{
    stats_t period, latency;
    stats_reset(&period);
    stats_reset(&latency);
    uint32_t tail = 0;
    uint32_t wakes = 0;
    uint32_t isr_at_start = s_isr_count;
    int32_t worst_dev_ever = 0;
    int64_t next_report = esp_timer_get_time() + REPORT_US;
    int report_no = 0;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t woke = esp_timer_get_time();
        wakes++;

        stats_add(&latency, (int32_t)(woke - s_last_isr_us));

        while (tail != s_head) {
            int32_t iv = s_ring[tail & (RING - 1)];
            tail++;
            stats_add(&period, iv);
            int32_t dev = iv - PERIOD_US;
            stats_add_dev(&period, dev);
            if ((dev < 0 ? -dev : dev) > (worst_dev_ever < 0 ? -worst_dev_ever : worst_dev_ever)) {
                worst_dev_ever = dev;
            }
        }

        if (woke >= next_report) {
            report_no++;
            uint32_t isrs = s_isr_count - isr_at_start;
            // Notifications coalesce if the task is still busy when the next
            // ISR fires, so fewer wakes than ISRs means the task fell behind.
            uint32_t missed = isrs > wakes ? isrs - wakes : 0;

            printf("[%3d] period: n=%lu mean=%.2f min=%ld max=%ld std=%.2f us  "
                   "|dev| <10:%lu 10-20:%lu 20-50:%lu 50-100:%lu >100:%lu  "
                   "| isr->task: mean=%.1f max=%ld us  missed=%lu%s\n",
                   report_no, (unsigned long)period.n,
                   period.n ? (double)period.sum / period.n : 0.0,
                   (long)period.min, (long)period.max, stats_std(&period),
                   (unsigned long)period.h_10, (unsigned long)period.h_20, (unsigned long)period.h_50,
                   (unsigned long)period.h_100, (unsigned long)period.h_over,
                   latency.n ? (double)latency.sum / latency.n : 0.0, (long)latency.max,
                   (unsigned long)missed,
                   (period.max - PERIOD_US > JITTER_LIMIT_US || PERIOD_US - period.min > JITTER_LIMIT_US)
                       ? "  ** OVER LIMIT **" : "");

            if (report_no % 10 == 0) {
                printf("      worst deviation so far: %+ld us (limit +-%d)\n",
                       (long)worst_dev_ever, JITTER_LIMIT_US);
            }

            stats_reset(&period);
            stats_reset(&latency);
            wakes = 0;
            isr_at_start = s_isr_count;
            next_report += REPORT_US;
        }
    }
}

void test_timer_jitter(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_LOOP_PROBE,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_LOOP_PROBE, 0);

    // Highest sensible priority, pinned to core 0 alongside the timer
    // interrupt, so the ISR->task latency measured here is the best case
    // the real control task would see.
    xTaskCreatePinnedToCore(loop_task, "loop", 4096, NULL, configMAX_PRIORITIES - 2, &s_task, 0);

    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 us ticks
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &s_timer));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, NULL));

    gptimer_alarm_config_t alarm = {
        .alarm_count = PERIOD_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_timer, &alarm));
    ESP_ERROR_CHECK(gptimer_enable(s_timer));

    ESP_LOGI(TAG, "GPTimer %d us period, probe on GPIO%d (square wave, %.1f Hz), task on core 0 prio %d",
             PERIOD_US, PIN_LOOP_PROBE, 1e6 / (2.0 * PERIOD_US), configMAX_PRIORITIES - 2);
    ESP_LOGI(TAG, "acceptance: period deviation within +-%d us. Reporting every second.", JITTER_LIMIT_US);

    ESP_ERROR_CHECK(gptimer_start(s_timer));

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
#endif // BB_TESTS
