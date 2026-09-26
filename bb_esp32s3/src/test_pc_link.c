#ifdef BB_TESTS // Faz 1 bring-up test: built only in the `tests` environment
#include "test_pc_link.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "pc_link.h"

#define POLL_PERIOD_MS   2    // same cadence the control loop will use
#define REPORT_PERIOD_MS 1000

static const char *TAG = "pc_link";

void test_pc_link(void)
{
    pc_link_init();
    ESP_LOGI(TAG, "listening on USB-Serial-JTAG, polling every %d ms, stale after %d ms",
             POLL_PERIOD_MS, PC_LINK_STALE_US / 1000);

    int64_t next_report = esp_timer_get_time() + REPORT_PERIOD_MS * 1000;
    uint32_t last_packets = 0;
    bool was_stale = true;

    while (true) {
        pc_link_poll();

        bool stale = pc_link_is_stale();
        if (stale != was_stale) {
            // Edge-logged so a stall is visible the moment it happens, not
            // only at the next summary line.
            if (stale) {
                ESP_LOGW(TAG, "STALE -- no packet for %d ms", PC_LINK_STALE_US / 1000);
            } else {
                ESP_LOGI(TAG, "link live");
            }
            was_stale = stale;
        }

        int64_t now = esp_timer_get_time();
        if (now >= next_report) {
            pc_link_state_t s;
            pc_link_get(&s);
            uint32_t rate = s.packets - last_packets;
            last_packets = s.packets;

            if (s.packets == 0) {
                ESP_LOGI(TAG, "no packets yet");
            } else {
                ESP_LOGI(TAG, "pos %7.2f cm  valid=%d warn=%d  | %3lu pkt/s  total %lu  "
                              "crc_err %lu  seq_gaps %lu  resync %lu  age %lld ms%s",
                         s.pos_0p1mm / 100.0,
                         (s.flags & PC_LINK_FLAG_VALID) ? 1 : 0,
                         (s.flags & PC_LINK_FLAG_WARNING) ? 1 : 0,
                         (unsigned long)rate, (unsigned long)s.packets,
                         (unsigned long)s.crc_errors, (unsigned long)s.seq_gaps,
                         (unsigned long)s.resyncs,
                         (long long)((now - s.received_at_us) / 1000),
                         stale ? "  STALE" : "");
            }
            next_report += REPORT_PERIOD_MS * 1000;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}
#endif // BB_TESTS
