#include "test_motor_encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#include "board_pins.h"
#include "tmc2208.h"

// Fine sweep of the beam's working range under load, non-interactive.
//
// The landmarks were mapped by hand earlier: 6 -> 12 o'clock is ~1980
// encoder counts and the encoder DEcreases going that way. The shaft-free
// test proved the motor and driver are exact (one commanded rev = one
// measured rev) and showed a periodic +-4 degree encoder error from the
// magnet sitting off-centre. This run does two things with one pass:
//
//   1. under the beam's load, checks that every 1.8 degree command produces
//      1.8 degrees of shaft -- the earlier 2-4x lurching turned out to be
//      mis-wired coils, since corrected, and this is the confirmation
//   2. records commanded-vs-measured at ~93 points across the range so the
//      encoder's eccentricity error can be fitted and corrected in software
//
// The driver engages at 6 (zero load) and never parks near 12 (unstable);
// it stops SWEEP_MARGIN_COUNTS short of it and comes back the same way.

#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_COUNTS 4096
#define USTEPS_PER_REV 51200
#define COUNTS_PER_USTEP ((double)AS5600_COUNTS / USTEPS_PER_REV)

#define TMC_IRUN       20
#define TMC_IHOLD      12
#define TMC_IHOLDDELAY 4

#define SPAN_6_TO_12_COUNTS 1980 // from the manual landmark mapping
#define SWEEP_MARGIN_COUNTS   80 // stop ~7 deg short of 12
#define USTEPS_PER_INCREMENT 256 // 1.8 deg, 20.48 counts
#define STEP_HALF_PERIOD_US   40
#define SETTLE_MS             25

#define STALL_LIMIT_COUNTS     6 // a 20-count increment that moved less than this
#define OVERSHOOT_LIMIT_COUNTS 57
#define MAX_INCREMENTS       130

static const char *TAG = "motor_enc";

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

static uint16_t encoder_read_avg(void)
{
    uint16_t first = 0;
    encoder_read(&first);
    int32_t acc = 0;
    for (int i = 0; i < 8; i++) {
        uint16_t r = 0;
        encoder_read(&r);
        int32_t d = (int32_t)r - (int32_t)first;
        if (d > AS5600_COUNTS / 2) d -= AS5600_COUNTS;
        if (d < -AS5600_COUNTS / 2) d += AS5600_COUNTS;
        acc += d;
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    return (uint16_t)(((int32_t)first + acc / 8 + AS5600_COUNTS) % AS5600_COUNTS);
}

static int32_t enc_delta(uint16_t from, uint16_t to)
{
    int32_t d = (int32_t)to - (int32_t)from;
    if (d > AS5600_COUNTS / 2) d -= AS5600_COUNTS;
    else if (d < -AS5600_COUNTS / 2) d += AS5600_COUNTS;
    return d;
}

static double deg(double counts)
{
    return counts * 360.0 / AS5600_COUNTS;
}

// -- driver -------------------------------------------------------------------

static void report_drv_status(const char *when)
{
    uint32_t s = 0;
    if (!tmc2208_read_register(TMC_REG_DRV_STATUS, &s)) {
        return;
    }
    printf("  DRV_STATUS %-9s ola=%lu olb=%lu short=%lu ot=%lu otpw=%lu stst=%lu CS=%lu\n",
           when, (unsigned long)((s >> 6) & 1), (unsigned long)((s >> 7) & 1),
           (unsigned long)((s >> 2) & 0xF), (unsigned long)((s >> 1) & 1), (unsigned long)(s & 1),
           (unsigned long)((s >> 31) & 1), (unsigned long)((s >> 16) & 0x1F));
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
                           ((uint32_t)TMC_IHOLDDELAY << 16) | ((uint32_t)TMC_IRUN << 8) | (uint32_t)TMC_IHOLD);

    uint32_t chop = 0;
    tmc2208_read_register(TMC_REG_CHOPCONF, &chop);
    printf("  CHOPCONF=0x%08lX  MRES=%lu (1/%lu)  IRUN=%d  chopper=%s\n",
           (unsigned long)chop, (unsigned long)((chop >> 24) & 0xF),
           256ul >> ((chop >> 24) & 0xF), TMC_IRUN, "StealthChop");
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

static void set_dir(int level)
{
    gpio_set_level(PIN_TMC_DIR, level);
    esp_rom_delay_us(20);
}

// One direction of the sweep. `sign` is the encoder direction of travel.
// Prints one CSV-ish row per increment: cumulative commanded counts,
// cumulative measured counts, the running error, and the raw encoder value
// -- everything the fit on the PC side needs. Returns increments taken or
// -1 on a guard.
static int sweep(const char *name, int dir_level, int sign, int32_t span_counts, uint16_t *pos)
{
    set_dir(dir_level);
    printf("\n--- %s: DIR=%d, sign=%+d, %d usteps/inc, target %ld counts ---\n",
           name, dir_level, sign, USTEPS_PER_INCREMENT, (long)span_counts);
    printf("  n,cmd_cum,meas_cum,err,raw\n");

    double cmd_per_inc = USTEPS_PER_INCREMENT * COUNTS_PER_USTEP;
    double cmd_cum = 0.0;
    int32_t meas_cum = 0;
    uint16_t start = *pos;

    for (int n = 1; n <= MAX_INCREMENTS; n++) {
        uint16_t before = *pos;
        step_pulses(USTEPS_PER_INCREMENT);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
        uint16_t after = 0;
        if (!encoder_read(&after)) {
            ESP_LOGE(TAG, "encoder read failed");
            return -1;
        }
        int32_t d = enc_delta(before, after) * sign; // positive = along travel
        cmd_cum += cmd_per_inc;
        meas_cum += d;
        *pos = after;

        printf("  %d,%.2f,%ld,%+.2f,%u\n", n, cmd_cum, (long)meas_cum, meas_cum - cmd_cum, after);

        if (d < STALL_LIMIT_COUNTS) {
            ESP_LOGE(TAG, "STALL at inc %d: moved %ld counts for %.1f commanded", n, (long)d, cmd_per_inc);
            return -1;
        }
        if (meas_cum > span_counts + OVERSHOOT_LIMIT_COUNTS) {
            ESP_LOGE(TAG, "OVERSHOOT at inc %d: %ld counts past target", n, (long)(meas_cum - span_counts));
            return -1;
        }
        if (meas_cum >= span_counts) {
            printf("  reached: %ld measured for %.1f commanded over %d increments (net error %+.1f counts = %+.2f deg)\n",
                   (long)meas_cum, cmd_cum, n, meas_cum - cmd_cum, deg(meas_cum - cmd_cum));
            return n;
        }
        if (n % 30 == 0) {
            report_drv_status("mid");
        }
    }
    (void)start;
    ESP_LOGE(TAG, "MAX_INCREMENTS without reaching target");
    return -1;
}

void test_motor_encoder(void)
{
    encoder_init();
    stepper_init();

    ESP_LOGW(TAG, "beam attached, resting at 6. Driver engages in 3 s -- keep clear.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    uint16_t enc6 = encoder_read_avg();
    printf("\n  enc6 (rest) = %u\n", enc6);

    gpio_set_level(PIN_TMC_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    uint16_t pos = encoder_read_avg();
    printf("  driver ON at %u (snap %+ld counts)\n", pos, (long)enc_delta(enc6, pos));
    report_drv_status("idle");

    // Probe one full step with DIR=0 to learn its encoder sign, then pick
    // the DIR level that heads toward 12 (encoder decreasing, from the map).
    set_dir(0);
    uint16_t before = pos;
    step_pulses(USTEPS_PER_INCREMENT);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
    encoder_read(&pos);
    int32_t probe = enc_delta(before, pos);
    int dir_to_12 = (probe < 0) ? 0 : 1;
    printf("  probe DIR=0: %+ld counts  =>  DIR=%d toward 12 (encoder decreasing)\n", (long)probe, dir_to_12);
    set_dir(1 - 0);
    step_pulses(USTEPS_PER_INCREMENT); // undo
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
    encoder_read(&pos);

    int32_t span = SPAN_6_TO_12_COUNTS - SWEEP_MARGIN_COUNTS;
    int n_up = sweep("6 -> ~12", dir_to_12, -1, span, &pos);
    if (n_up < 0) {
        ESP_LOGE(TAG, "sweep aborted; driver stays ON. Support the beam, then reset.");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    report_drv_status("at top");
    vTaskDelay(pdMS_TO_TICKS(500));

    int n_down = sweep("~12 -> 6", 1 - dir_to_12, +1, span, &pos);
    if (n_down < 0) {
        ESP_LOGE(TAG, "sweep aborted; driver stays ON. Support the beam, then reset.");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    report_drv_status("at 6");

    int32_t drift = enc_delta(enc6, pos);
    printf("\n===== results =====\n");
    printf("  up   : %d increments = %d usteps = %.1f deg commanded\n",
           n_up, n_up * USTEPS_PER_INCREMENT, deg(n_up * USTEPS_PER_INCREMENT * COUNTS_PER_USTEP));
    printf("  down : %d increments\n", n_down);
    printf("  drift after round trip : %+ld counts = %+.2f deg\n", (long)drift, deg(drift));
    printf("  DIR=%d -> toward 12 -> ball toward p2;  DIR=%d -> toward 6 -> ball toward p1\n",
           dir_to_12, 1 - dir_to_12);
    printf("====================\n");

    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(PIN_TMC_EN, 1);
    ESP_LOGI(TAG, "driver OFF at 6. Done.");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
