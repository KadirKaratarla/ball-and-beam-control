#ifdef BB_TESTS // Faz 1 bring-up test: built only in the `tests` environment
#include "test_tmc_bringup.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#include "board_pins.h"
#include "tmc2208.h"

// Bring-up values only: 16/31 of full scale. TUNE to the actual NEMA17's
// rated per-phase current before running for extended periods -- depends on
// this board's sense resistor value, see I_RMS formula in the datasheet ch.9.
#define TMC_IRUN        16
#define TMC_IHOLD       8
#define TMC_IHOLDDELAY  4

#define STEPS_PER_TEST_MOVE 40000
#define STEP_PULSE_HALF_PERIOD_US 12

static const char *TAG = "tmc_bringup";

static void configure_gpio(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_TMC_EN) | (1ULL << PIN_TMC_STEP) | (1ULL << PIN_TMC_DIR),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_TMC_EN, 1); // disabled while we configure over UART
    gpio_set_level(PIN_TMC_STEP, 0);
    gpio_set_level(PIN_TMC_DIR, 0);
}

static void configure_driver(void)
{
    // pdn_disable=1 (bit6, required for UART mode), mstep_reg_select=1
    // (bit7, resolution comes from CHOPCONF.MRES instead of MS1/MS2 pins),
    // i_scale_analog=0 (bit0, use internal 5VOUT-derived reference since
    // VREF isn't wired on this board -- leaving the reset default of 1 here
    // would scale current off a floating pin).
    tmc2208_write_register(TMC_REG_GCONF, 0xC0);

    // CHOPCONF: MRES=0000 (native 256 microstep), intpol=1, TBL=2 (32 clk),
    // HSTRT=4, HEND=0, TOFF=5 -- datasheet's own "quick configuration" basic
    // values (ch.14), just with MRES forced to 256 for our 1/256 test.
    tmc2208_write_register(TMC_REG_CHOPCONF, 0x100101B5);

    uint32_t ihold_irun = ((uint32_t)TMC_IHOLDDELAY << 16) |
                          ((uint32_t)TMC_IRUN << 8) |
                          (uint32_t)TMC_IHOLD;
    tmc2208_write_register(TMC_REG_IHOLD_IRUN, ihold_irun);
}

static void step_pulses(int count)
{
    for (int i = 0; i < count; i++) {
        gpio_set_level(PIN_TMC_STEP, 1);
        esp_rom_delay_us(STEP_PULSE_HALF_PERIOD_US);
        gpio_set_level(PIN_TMC_STEP, 0);
        esp_rom_delay_us(STEP_PULSE_HALF_PERIOD_US);
    }
}

void test_tmc_bringup(void)
{
    configure_gpio();

    tmc2208_uart_init(PIN_TMC_UART_TX, PIN_TMC_UART_RX, TMC_UART_BAUD);
    vTaskDelay(pdMS_TO_TICKS(50));

    configure_driver();

    uint32_t ifcnt = 0;
    if (tmc2208_read_register(TMC_REG_IFCNT, &ifcnt)) {
        ESP_LOGI(TAG, "IFCNT = %lu (+3 per run; the driver keeps counting across ESP resets)",
                 (unsigned long)ifcnt);
    } else {
        ESP_LOGE(TAG, "UART read failed -- check wiring (TX/1k/PDN_UART, RX direct), CRC, GND common");
    }

    uint32_t gconf_rb = 0;
    if (tmc2208_read_register(TMC_REG_GCONF, &gconf_rb)) {
        ESP_LOGI(TAG, "GCONF readback = 0x%08lX (expect 0xC0)", (unsigned long)gconf_rb);
    }

    uint32_t chopconf_rb = 0;
    if (tmc2208_read_register(TMC_REG_CHOPCONF, &chopconf_rb)) {
        ESP_LOGI(TAG, "CHOPCONF readback = 0x%08lX (TOFF=%lu, MRES=%lu, expect TOFF=5 MRES=0)",
                 (unsigned long)chopconf_rb,
                 (unsigned long)(chopconf_rb & 0xF),
                 (unsigned long)((chopconf_rb >> 24) & 0xF));
    }

    uint32_t gstat = 0;
    if (tmc2208_read_register(TMC_REG_GSTAT, &gstat)) {
        ESP_LOGI(TAG, "GSTAT = 0x%08lX -> reset=%lu drv_err=%lu uv_cp=%lu (uv_cp=1 means no/low VM)",
                 (unsigned long)gstat,
                 (unsigned long)(gstat & 1),
                 (unsigned long)((gstat >> 1) & 1),
                 (unsigned long)((gstat >> 2) & 1));
    }

    uint32_t drv_status = 0;
    if (tmc2208_read_register(TMC_REG_DRV_STATUS, &drv_status)) {
        ESP_LOGI(TAG, "DRV_STATUS = 0x%08lX", (unsigned long)drv_status);
    }

    gpio_set_level(PIN_TMC_EN, 0); // enable the driver
    vTaskDelay(pdMS_TO_TICKS(200));

    // IOIN reports the logic levels the chip actually sees on its own pins.
    // ENN must read 0 here -- if it reads 1 the EN wire isn't on PIN_TMC_EN.
    uint32_t ioin = 0;
    if (tmc2208_read_register(TMC_REG_IOIN, &ioin)) {
        ESP_LOGI(TAG, "IOIN = 0x%08lX -> ENN=%lu MS1=%lu MS2=%lu STEP=%lu DIR=%lu SEL_A=%lu VERSION=0x%02lX",
                 (unsigned long)ioin,
                 (unsigned long)(ioin & 1),
                 (unsigned long)((ioin >> 2) & 1),
                 (unsigned long)((ioin >> 3) & 1),
                 (unsigned long)((ioin >> 7) & 1),
                 (unsigned long)((ioin >> 9) & 1),
                 (unsigned long)((ioin >> 8) & 1),
                 (unsigned long)((ioin >> 24) & 0xFF));
    }

    ESP_LOGI(TAG, "Spinning forward...");
    gpio_set_level(PIN_TMC_DIR, 0);
    step_pulses(STEPS_PER_TEST_MOVE / 2);

    // Read mid-motion: stst must be 0 here if the chip is really seeing STEP
    // edges. If it stays 1, the STEP pulses never reach the driver.
    uint32_t drv_moving = 0;
    if (tmc2208_read_register(TMC_REG_DRV_STATUS, &drv_moving)) {
        ESP_LOGI(TAG, "DRV_STATUS mid-motion = 0x%08lX -> stst=%lu CS_ACTUAL=%lu (stst=0 means STEP is arriving)",
                 (unsigned long)drv_moving,
                 (unsigned long)((drv_moving >> 31) & 1),
                 (unsigned long)((drv_moving >> 16) & 0x1F));
    }

    step_pulses(STEPS_PER_TEST_MOVE / 2);

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Spinning backward...");
    gpio_set_level(PIN_TMC_DIR, 1);
    step_pulses(STEPS_PER_TEST_MOVE);

    gpio_set_level(PIN_TMC_EN, 1); // disable when done
    ESP_LOGI(TAG, "Test complete.");
}
#endif // BB_TESTS
