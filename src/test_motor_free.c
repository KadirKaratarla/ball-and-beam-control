#include "test_motor_free.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#include "board_pins.h"
#include "tmc2208.h"

// Under the beam the motor turned 2-4x what it was told and stalled on
// reversal. That is a stepper losing sync, not a load effect -- a stepper
// can't be pushed ahead of its own field -- and the likeliest causes are on
// the coil side: crossed coil pairs or an open coil. Both leave fingerprints
// the driver can report, and both show up cleanly with the shaft free.

#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_COUNTS 4096
#define USTEPS_PER_REV 51200 // 200 x 256

#define TMC_IRUN       20
#define TMC_IHOLD      12
#define TMC_IHOLDDELAY 4

#define CHUNKS_PER_REV 10
#define USTEPS_PER_CHUNK (USTEPS_PER_REV / CHUNKS_PER_REV) // 36 deg
#define STEP_HALF_PERIOD_US 40

static const char *TAG = "motor_free";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

// -- encoder ------------------------------------------------------------------

static void encoder_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_AS5600_SDA,
        .scl_io_num = PIN_AS5600_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));
}

static bool encoder_read(uint16_t *raw)
{
    uint8_t reg = AS5600_REG_RAW_ANGLE;
    uint8_t buf[2];
    if (i2c_master_transmit_receive(s_dev, &reg, 1, buf, 2, 50) != ESP_OK) {
        return false;
    }
    *raw = (uint16_t)(((buf[0] << 8) | buf[1]) & 0x0FFF);
    return true;
}

static int32_t enc_delta(uint16_t from, uint16_t to)
{
    int32_t d = (int32_t)to - (int32_t)from;
    if (d > AS5600_COUNTS / 2) d -= AS5600_COUNTS;
    else if (d < -AS5600_COUNTS / 2) d += AS5600_COUNTS;
    return d;
}

static double deg(int32_t counts)
{
    return counts * 360.0 / AS5600_COUNTS;
}

// -- driver -------------------------------------------------------------------

static void report_drv_status(const char *when)
{
    uint32_t s = 0;
    if (!tmc2208_read_register(TMC_REG_DRV_STATUS, &s)) {
        ESP_LOGE(TAG, "DRV_STATUS read failed");
        return;
    }
    printf("  DRV_STATUS %-12s = 0x%08lX  ola=%lu olb=%lu  s2ga=%lu s2gb=%lu s2vsa=%lu s2vsb=%lu  "
           "ot=%lu otpw=%lu  stst=%lu stealth=%lu  CS=%lu\n",
           when, (unsigned long)s,
           (unsigned long)((s >> 6) & 1), (unsigned long)((s >> 7) & 1),
           (unsigned long)((s >> 2) & 1), (unsigned long)((s >> 3) & 1),
           (unsigned long)((s >> 4) & 1), (unsigned long)((s >> 5) & 1),
           (unsigned long)((s >> 1) & 1), (unsigned long)(s & 1),
           (unsigned long)((s >> 31) & 1), (unsigned long)((s >> 30) & 1),
           (unsigned long)((s >> 16) & 0x1F));
    if ((s >> 6) & 1 || (s >> 7) & 1) {
        ESP_LOGW(TAG, "  OPEN LOAD flagged on phase %s%s -- a coil is not conducting",
                 ((s >> 6) & 1) ? "A " : "", ((s >> 7) & 1) ? "B" : "");
    }
    if ((s >> 2) & 0xF) {
        ESP_LOGE(TAG, "  SHORT flagged -- driver has shut the bridge down");
    }
}

static void stepper_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_TMC_EN) | (1ULL << PIN_TMC_STEP) | (1ULL << PIN_TMC_DIR),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_TMC_EN, 1);
    gpio_set_level(PIN_TMC_STEP, 0);
    gpio_set_level(PIN_TMC_DIR, 0);

    tmc2208_uart_init(PIN_TMC_UART_TX, PIN_TMC_UART_RX, TMC_UART_BAUD);
    vTaskDelay(pdMS_TO_TICKS(50));

    tmc2208_write_register(TMC_REG_GCONF, 0xC0);
    tmc2208_write_register(TMC_REG_CHOPCONF, 0x100101B5);
    tmc2208_write_register(TMC_REG_IHOLD_IRUN,
                           ((uint32_t)TMC_IHOLDDELAY << 16) |
                           ((uint32_t)TMC_IRUN << 8) |
                           (uint32_t)TMC_IHOLD);

    uint32_t gconf = 0, chop = 0, ifcnt = 0;
    tmc2208_read_register(TMC_REG_IFCNT, &ifcnt);
    tmc2208_read_register(TMC_REG_GCONF, &gconf);
    tmc2208_read_register(TMC_REG_CHOPCONF, &chop);
    unsigned mres = (chop >> 24) & 0xF;
    unsigned dedge = (chop >> 29) & 1;
    printf("\n  IFCNT=%lu  GCONF=0x%02lX  CHOPCONF=0x%08lX\n",
           (unsigned long)ifcnt, (unsigned long)gconf, (unsigned long)chop);
    printf("  MRES=%u -> 1/%u microsteps   dedge=%u   mstep_reg_select=%lu   TOFF=%lu\n",
           mres, 256u >> mres, dedge, (unsigned long)((gconf >> 7) & 1), (unsigned long)(chop & 0xF));
    if (chop != 0x100101B5 || gconf != 0xC0) {
        ESP_LOGE(TAG, "  config readback does NOT match what was written -- UART write lost?");
    }
    if (mres != 0) {
        ESP_LOGE(TAG, "  NOT at 1/256: every STEP pulse moves %u microsteps' worth", 1u << mres);
    }
    if (dedge) {
        ESP_LOGE(TAG, "  DEDGE set: both STEP edges count, every pulse moves twice");
    }
}

static void step_pulses(int count)
{
    for (int i = 0; i < count; i++) {
        gpio_set_level(PIN_TMC_STEP, 1);
        esp_rom_delay_us(STEP_HALF_PERIOD_US);
        gpio_set_level(PIN_TMC_STEP, 0);
        esp_rom_delay_us(STEP_HALF_PERIOD_US);
    }
}

// One full revolution in chunks, encoder after each, continuous (unwrapped)
// total returned in counts. Reads DRV_STATUS halfway so open-load is checked
// while the coils are actually being driven, which is when it is reliable.
static int32_t one_rev(const char *name, int dir_level, uint16_t *pos)
{
    gpio_set_level(PIN_TMC_DIR, dir_level);
    esp_rom_delay_us(20);

    int32_t total = 0;
    int32_t cmd_per_chunk = (int32_t)(USTEPS_PER_CHUNK * (double)AS5600_COUNTS / USTEPS_PER_REV + 0.5);

    printf("\n--- %s: DIR=%d, %d chunks x %d usteps (each should be %ld counts = 36 deg) ---\n",
           name, dir_level, CHUNKS_PER_REV, USTEPS_PER_CHUNK, (long)cmd_per_chunk);
    printf("  chunk   enc    d_counts   d_deg    ratio\n");

    for (int c = 1; c <= CHUNKS_PER_REV; c++) {
        uint16_t before = *pos;
        step_pulses(USTEPS_PER_CHUNK);
        vTaskDelay(pdMS_TO_TICKS(50));
        uint16_t after = 0;
        if (!encoder_read(&after)) {
            ESP_LOGE(TAG, "encoder read failed");
            continue;
        }
        int32_t d = enc_delta(before, after);
        total += d;
        printf("  %3d   %5u   %+8ld   %+7.2f   %5.2fx\n",
               c, after, (long)d, deg(d), (double)d / cmd_per_chunk);
        *pos = after;
        if (c == CHUNKS_PER_REV / 2) {
            report_drv_status("mid-motion");
        }
    }
    return total;
}

void test_motor_free(void)
{
    encoder_init();
    stepper_init();

    ESP_LOGW(TAG, "shaft must be FREE (rod detached from the crank) -- starting in 3 s");
    vTaskDelay(pdMS_TO_TICKS(3000));

    uint16_t pos = 0;
    encoder_read(&pos);
    uint16_t start = pos;
    printf("\n  start enc = %u\n", start);

    gpio_set_level(PIN_TMC_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    report_drv_status("enabled,idle");

    int32_t fwd = one_rev("forward", 0, &pos);
    report_drv_status("after fwd");
    vTaskDelay(pdMS_TO_TICKS(500));
    int32_t back = one_rev("reverse", 1, &pos);
    report_drv_status("after rev");

    int32_t drift = enc_delta(start, pos);

    printf("\n===== results =====\n");
    printf("  forward : %+6ld counts = %+8.2f deg   (expect +/-4096 = 360)\n", (long)fwd, deg(fwd));
    printf("  reverse : %+6ld counts = %+8.2f deg\n", (long)back, deg(back));
    printf("  usteps per encoder count : %.3f measured (fwd), 12.500 nominal\n",
           fwd ? (double)USTEPS_PER_REV / (fwd > 0 ? fwd : -fwd) : 0.0);
    printf("  drift after round trip   : %+ld counts = %+.2f deg\n", (long)drift, deg(drift));

    double ratio = fwd ? (double)(fwd > 0 ? fwd : -fwd) / AS5600_COUNTS : 0.0;
    if (ratio > 0.95 && ratio < 1.05) {
        printf("  => one commanded revolution = one measured revolution. Stepping is correct.\n");
    } else if (ratio > 1.9 && ratio < 2.1) {
        printf("  => moved 2x: every pulse is worth two microsteps (DEDGE, or 1/128 in effect).\n");
    } else if (ratio > 1.05) {
        printf("  => moved %.2fx the command: losing sync / lurching. Check coil pairing first.\n", ratio);
    } else {
        printf("  => moved only %.2fx the command: stalling / missing steps.\n", ratio);
    }
    printf("====================\n");

    gpio_set_level(PIN_TMC_EN, 1);
    ESP_LOGI(TAG, "driver OFF. Done -- reset to run again.");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
