#include "test_motor_encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#include "board_pins.h"
#include "tmc2208.h"

// The rig: encoder on the motor shaft, short crank on the shaft, long rod
// from the crank tip to the beam. Beam level <=> crank horizontal ("9
// o'clock"). The beam's weight pulls the crank to 6 o'clock whenever the
// driver is off, and loads the motor hardest at 9 where the crank's lever
// arm is longest. The working range is 6 -> 9 -> 12, 180 degrees.

#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_COUNTS 4096
#define COUNTS_PER_MICROSTEP_NOMINAL (4096.0 / 51200.0) // 12.5 usteps per count

// Higher than the bring-up's 16: at 9 o'clock the motor is holding the
// beam against gravity at maximum lever arm, and slipping there is what
// this test is meant to detect, not cause.
#define TMC_IRUN       20
#define TMC_IHOLD      12
#define TMC_IHOLDDELAY 4

#define USTEPS_PER_MOVE     256   // one full step, 1.8 deg of shaft
#define MOVES_PER_LEG       10    // 18 deg each way -- well inside 6..12
#define STEP_HALF_PERIOD_US 40    // slow: 20 ms per full step

#define REST_CAPTURE_S      3
#define POSITION_WINDOW_S   12
#define RELEASE_COUNTDOWN_S 5

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

// Signed shortest difference on the 4096 circle.
static int32_t enc_delta(uint16_t from, uint16_t to)
{
    int32_t d = (int32_t)to - (int32_t)from;
    if (d > AS5600_COUNTS / 2) {
        d -= AS5600_COUNTS;
    } else if (d < -AS5600_COUNTS / 2) {
        d += AS5600_COUNTS;
    }
    return d;
}

static double counts_to_deg(int32_t counts)
{
    return counts * 360.0 / AS5600_COUNTS;
}

// -- stepper ------------------------------------------------------------------

static void stepper_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_TMC_EN) | (1ULL << PIN_TMC_STEP) | (1ULL << PIN_TMC_DIR),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_TMC_EN, 1); // off until the beam is positioned
    gpio_set_level(PIN_TMC_STEP, 0);
    gpio_set_level(PIN_TMC_DIR, 0);

    tmc2208_uart_init(PIN_TMC_UART_TX, PIN_TMC_UART_RX, TMC_UART_BAUD);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Same register set as the bring-up: pdn_disable, mstep_reg_select,
    // internal current reference; native 256 microsteps; run current raised.
    tmc2208_write_register(TMC_REG_GCONF, 0xC0);
    tmc2208_write_register(TMC_REG_CHOPCONF, 0x100101B5);
    tmc2208_write_register(TMC_REG_IHOLD_IRUN,
                           ((uint32_t)TMC_IHOLDDELAY << 16) |
                           ((uint32_t)TMC_IRUN << 8) |
                           (uint32_t)TMC_IHOLD);

    uint32_t gconf = 0;
    if (!tmc2208_read_register(TMC_REG_GCONF, &gconf) || gconf != 0xC0) {
        ESP_LOGE(TAG, "TMC2208 config readback failed (GCONF=0x%08lX)", (unsigned long)gconf);
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

// Move one leg of MOVES_PER_LEG full steps, logging the encoder after each,
// and return the total encoder delta over the leg.
static int32_t run_leg(const char *name, int dir_level, uint16_t *pos)
{
    gpio_set_level(PIN_TMC_DIR, dir_level);
    esp_rom_delay_us(20); // tDSU: DIR must settle before the first STEP edge

    uint16_t start = *pos;
    printf("\n--- %s: DIR=%d, %d x %d usteps ---\n", name, dir_level, MOVES_PER_LEG, USTEPS_PER_MOVE);
    printf("  move   enc_raw   d_counts   d_deg    counts/ustep\n");

    for (int m = 1; m <= MOVES_PER_LEG; m++) {
        uint16_t before = *pos;
        step_pulses(USTEPS_PER_MOVE);
        vTaskDelay(pdMS_TO_TICKS(30)); // let the rotor settle on the new detent
        uint16_t after = 0;
        if (!encoder_read(&after)) {
            ESP_LOGE(TAG, "encoder read failed");
            continue;
        }
        int32_t d = enc_delta(before, after);
        printf("  %4d   %7u   %8ld   %6.2f    %8.4f\n",
               m, after, (long)d, counts_to_deg(d), (double)d / USTEPS_PER_MOVE);
        *pos = after;
    }
    return enc_delta(start, *pos);
}

// -- test ---------------------------------------------------------------------

void test_motor_encoder(void)
{
    encoder_init();
    stepper_init();

    // Phase 0: driver off, the beam is resting where gravity leaves it.
    ESP_LOGI(TAG, "phase 0: driver OFF, %d s -- leave the beam resting (crank at 6 o'clock)",
             REST_CAPTURE_S);
    vTaskDelay(pdMS_TO_TICKS(REST_CAPTURE_S * 1000));
    uint16_t rest = 0;
    if (!encoder_read(&rest)) {
        ESP_LOGE(TAG, "encoder not responding, aborting");
        return;
    }
    ESP_LOGI(TAG, "rest (6 o'clock) = %u", rest);

    // Phase 1: the user lifts the beam to level and HOLDS it; the driver is
    // then enabled so the motor takes over before they let go. Enabling
    // snaps the rotor to the nearest detent of the energised phase pattern,
    // so the value that counts is the one read after the snap.
    ESP_LOGI(TAG, "phase 1: lift the beam to LEVEL and HOLD it -- %d s, encoder streaming",
             POSITION_WINDOW_S);
    for (int s = POSITION_WINDOW_S; s > 0; s--) {
        uint16_t now = 0;
        encoder_read(&now);
        printf("  %2d s   enc %4u   (%+7.2f deg from rest)\n",
               s, now, counts_to_deg(enc_delta(rest, now)));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    gpio_set_level(PIN_TMC_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    uint16_t level = 0;
    encoder_read(&level);
    int32_t rest_to_level = enc_delta(rest, level);
    ESP_LOGI(TAG, "driver ON -- holding. You can let go.");
    ESP_LOGI(TAG, "level = %u, rest->level = %ld counts = %.2f deg (crank 6->9 should be ~90 deg / 1024 counts)",
             level, (long)rest_to_level, counts_to_deg(rest_to_level));

    // Phase 2: +18, -36, +18 degrees around level.
    uint16_t pos = level;
    int32_t leg_fwd  = run_leg("leg A forward",  0, &pos);
    int32_t leg_back = run_leg("leg B back",     1, &pos);
    leg_back        += run_leg("leg B back (2)", 1, &pos);
    int32_t leg_ret  = run_leg("leg C return",   0, &pos);

    // Phase 3: what it all says.
    int32_t drift = enc_delta(level, pos);
    double counts_per_ustep = (double)leg_fwd / (MOVES_PER_LEG * USTEPS_PER_MOVE);

    printf("\n===== results =====\n");
    printf("  leg A (DIR=0, +%d steps) : %+6ld counts  %+7.2f deg\n",
           MOVES_PER_LEG, (long)leg_fwd, counts_to_deg(leg_fwd));
    printf("  leg B (DIR=1, -%d steps) : %+6ld counts  %+7.2f deg\n",
           2 * MOVES_PER_LEG, (long)leg_back, counts_to_deg(leg_back));
    printf("  leg C (DIR=0, +%d steps) : %+6ld counts  %+7.2f deg\n",
           MOVES_PER_LEG, (long)leg_ret, counts_to_deg(leg_ret));
    printf("  counts per microstep     : %.5f measured, %.5f nominal (12.5 usteps/count)\n",
           counts_per_ustep, COUNTS_PER_MICROSTEP_NOMINAL);
    printf("  drift after round trip   : %+ld counts = %+.2f deg  (step loss if far from 0)\n",
           (long)drift, counts_to_deg(drift));

    // Which way is 12 o'clock? Rest is 6, level is 9; the sign of rest->level
    // says whether the encoder counts up or down going 6->9->12, and leg A's
    // sign says which way DIR=0 pushes. Together: does DIR=0 lift the beam
    // toward 12 (ball toward p2) or lower it toward 6 (ball toward p1)?
    bool enc_up_toward_12 = rest_to_level > 0;
    bool dir0_enc_up = leg_fwd > 0;
    bool dir0_toward_12 = (enc_up_toward_12 == dir0_enc_up);
    printf("  encoder counts %s going 6 -> 9 -> 12\n", enc_up_toward_12 ? "UP" : "DOWN");
    printf("  DIR=0 moves the crank toward %s  =>  ball toward %s\n",
           dir0_toward_12 ? "12 o'clock" : "6 o'clock",
           dir0_toward_12 ? "p2 (45 cm)" : "p1 (0 cm)");
    printf("===================\n\n");

    // Phase 4: hand the beam back to the user before the driver lets go.
    ESP_LOGW(TAG, "driver will release in %d s -- HOLD THE BEAM, it will fall to 6 o'clock",
             RELEASE_COUNTDOWN_S);
    for (int s = RELEASE_COUNTDOWN_S; s > 0; s--) {
        printf("  release in %d\n", s);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    gpio_set_level(PIN_TMC_EN, 1);
    ESP_LOGI(TAG, "driver OFF. Done -- reset to run again.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
