#include "diag_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "config.h"
#include "pc_link.h"
#include "tmc2208.h"

static const char *TAG = "diag";

static diag_state_t s_state = { .driver_ok = true };

static void link_task(void *arg)
{
    while (true) {
        pc_link_poll();
        vTaskDelay(pdMS_TO_TICKS(LINK_POLL_PERIOD_MS));
    }
}

static void diag_task(void *arg)
{
    // Three consecutive silent polls before declaring the UART dead: one
    // missed reply can be a collision with our own echo drain.
    int silent = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(DIAG_PERIOD_MS));

        uint32_t gstat = 0;
        if (!tmc2208_read_register(TMC_REG_GSTAT, &gstat)) {
            s_state.uart_errors++;
            if (++silent >= 3 && s_state.driver_ok) {
                s_state.driver_ok = false;
                ESP_LOGE(TAG, "TMC2208 not answering on UART");
            }
            continue;
        }
        silent = 0;

        if (gstat & 0x01) {
            // Registers are gone: the driver is at 1/8 stepping with default
            // current. Do not reconfigure on the fly under a running profile;
            // latch a fault, the control task stops, the operator restarts.
            s_state.resets++;
            tmc2208_write_register(TMC_REG_GSTAT, 0x01);
            if (s_state.driver_ok) {
                s_state.driver_ok = false;
                ESP_LOGE(TAG, "TMC2208 reset detected (GSTAT=0x%02lX) -- registers lost", (unsigned long)gstat);
            }
        }
        if (gstat & 0x06) {
            ESP_LOGW(TAG, "GSTAT: %s%s", (gstat & 0x02) ? "drv_err " : "", (gstat & 0x04) ? "uv_cp" : "");
            tmc2208_write_register(TMC_REG_GSTAT, gstat & 0x06);
        }

        uint32_t drv = 0;
        if (tmc2208_read_register(TMC_REG_DRV_STATUS, &drv)) {
            s_state.drv_status = drv;
        }
    }
}

void diag_start(void)
{
    xTaskCreatePinnedToCore(link_task, "link", 3072, NULL, TASK_LINK_PRIO, NULL, TASK_AUX_CORE);
    xTaskCreatePinnedToCore(diag_task, "diag", 4096, NULL, TASK_DIAG_PRIO, NULL, TASK_AUX_CORE);
    ESP_LOGI(TAG, "link poll every %d ms, TMC watchdog every %d ms, both on core %d",
             LINK_POLL_PERIOD_MS, DIAG_PERIOD_MS, TASK_AUX_CORE);
}

void diag_clear_driver(void)
{
    s_state.driver_ok = true;
}

const diag_state_t *diag_get(void)
{
    return &s_state;
}
