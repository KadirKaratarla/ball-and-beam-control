#include "test_motor_encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#include "board_pins.h"
#include "tmc2208.h"

// The rig: encoder on the motor shaft, short crank on the shaft, long rod
// from the crank tip to the beam. Beam level <=> crank horizontal ("9
// o'clock"). The beam's weight drops the crank to 6 o'clock whenever the
// driver is off, loads the motor hardest at 9 (longest lever arm), and
// makes 12 an unstable equilibrium. The working range is 6 -> 9 -> 12.
//
// Two halves. First, with the driver off, the user moves the beam to 6, 9
// and 12 by hand and presses Enter at each so the encoder value of every
// landmark is known before anything moves under power. Then the driver
// takes over at 6 -- where the load is zero -- and walks the crank to the
// recorded 12 and back, comparing what was commanded against what the
// encoder saw at every increment.

#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_COUNTS 4096
#define USTEPS_PER_REV 51200 // 200 full steps x 256 microsteps (CHOPCONF.MRES=0)
#define COUNTS_PER_USTEP ((double)AS5600_COUNTS / USTEPS_PER_REV)

// Higher than the bring-up's 16: crossing 9 o'clock the motor works
// against the beam's full lever arm, and slipping there is what this
// test measures, not what it should cause.
#define TMC_IRUN       20
#define TMC_IHOLD      12
#define TMC_IHOLDDELAY 4

#define USTEPS_PER_INCREMENT 512  // 3.6 deg of shaft per increment
#define STEP_HALF_PERIOD_US  40   // 12.5k usteps/s ~ 15 rpm, gentle under load
#define PROBE_USTEPS         256  // one full step to learn which way DIR=0 goes

// Guards for the powered sweep.
#define OVERSHOOT_LIMIT_COUNTS 57 // ~5 deg past the recorded landmark
#define STALL_LIMIT_COUNTS     10 // an increment that moved < this is a stall
#define MAX_INCREMENTS         60 // 216 deg -- past 180 something is wrong

static const char *TAG = "motor_enc";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

// -- console input ------------------------------------------------------------

static void console_input_init(void)
{
    // The console UART has no RX driver by default; install one so Enter
    // from the monitor can be read. printf output is routed through the same
    // driver so the two paths don't interleave on the FIFO.
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    uart_vfs_dev_use_driver(UART_NUM_0);
}

static void wait_enter(const char *prompt)
{
    printf("\n>>> %s -- then press ENTER\n", prompt);
    fflush(stdout);
    uart_flush_input(UART_NUM_0);
    while (true) {
        uint8_t c;
        if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(100)) == 1 && (c == '\r' || c == '\n')) {
            return;
        }
    }
}

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

// Average a few samples so a landmark isn't one noisy reading.
static uint16_t encoder_read_avg(void)
{
    int32_t acc = 0;
    uint16_t first = 0;
    encoder_read(&first);
    for (int i = 0; i < 8; i++) {
        uint16_t r = 0;
        encoder_read(&r);
        // accumulate relative to the first sample so a seam crossing
        // inside the window doesn't wreck the mean
        int32_t d = (int32_t)r - (int32_t)first;
        if (d > AS5600_COUNTS / 2) d -= AS5600_COUNTS;
        if (d < -AS5600_COUNTS / 2) d += AS5600_COUNTS;
        acc += d;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    int32_t v = (int32_t)first + acc / 8;
    return (uint16_t)((v + AS5600_COUNTS) % AS5600_COUNTS);
}

// Signed shortest difference on the 4096 circle.
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

// -- stepper ------------------------------------------------------------------

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
    tmc2208_write_register(TMC_REG_CHOPCONF, 0x100101B5); // MRES=0 -> 1/256
    tmc2208_write_register(TMC_REG_IHOLD_IRUN,
                           ((uint32_t)TMC_IHOLDDELAY << 16) |
                           ((uint32_t)TMC_IRUN << 8) |
                           (uint32_t)TMC_IHOLD);

    uint32_t gconf = 0, chop = 0;
    tmc2208_read_register(TMC_REG_GCONF, &gconf);
    tmc2208_read_register(TMC_REG_CHOPCONF, &chop);
    ESP_LOGI(TAG, "TMC2208 GCONF=0x%02lX CHOPCONF=0x%08lX MRES=%lu -> 1/%d microsteps, IRUN=%d",
             (unsigned long)gconf, (unsigned long)chop, (unsigned long)((chop >> 24) & 0xF),
             256 >> ((chop >> 24) & 0xF), TMC_IRUN);
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
    esp_rom_delay_us(20); // tDSU before the first STEP edge
}

// Walk toward `target` in increments, logging commanded vs measured, until
// the encoder reaches it. `sign` is the encoder direction of travel (+1/-1).
// Returns the number of increments taken, or -1 on a guard trip.
static int sweep_to(const char *name, int dir_level, int sign, uint16_t target, uint16_t *pos)
{
    set_dir(dir_level);
    printf("\n--- %s: DIR=%d, %d usteps per increment ---\n", name, dir_level, USTEPS_PER_INCREMENT);
    printf("  inc   enc     d_meas   d_cmd   err    to_target\n");

    int32_t cmd_counts = (int32_t)(USTEPS_PER_INCREMENT * COUNTS_PER_USTEP + 0.5);
    int32_t err_acc = 0;

    for (int n = 1; n <= MAX_INCREMENTS; n++) {
        uint16_t before = *pos;
        step_pulses(USTEPS_PER_INCREMENT);
        vTaskDelay(pdMS_TO_TICKS(30));
        uint16_t after = 0;
        if (!encoder_read(&after)) {
            ESP_LOGE(TAG, "encoder read failed");
            return -1;
        }
        int32_t d = enc_delta(before, after);
        int32_t err = d * sign - cmd_counts; // measured minus commanded, in the travel direction
        err_acc += err;
        int32_t remaining = enc_delta(after, target) * sign;
        printf("  %3d  %5u   %+6ld   %+6ld  %+4ld   %+6ld\n",
               n, after, (long)d, (long)(cmd_counts * sign), (long)err, (long)remaining);
        *pos = after;

        if (d * sign < STALL_LIMIT_COUNTS) {
            ESP_LOGE(TAG, "STALL: commanded %ld counts, encoder moved %ld -- stopping", (long)cmd_counts, (long)d);
            return -1;
        }
        if (remaining < -OVERSHOOT_LIMIT_COUNTS) {
            ESP_LOGE(TAG, "OVERSHOOT: %ld counts past target -- stopping", (long)-remaining);
            return -1;
        }
        if (remaining <= cmd_counts / 2) {
            printf("  reached target (accumulated error %+ld counts = %+.2f deg over %d increments)\n",
                   (long)err_acc, deg(err_acc), n);
            return n;
        }
    }
    ESP_LOGE(TAG, "MAX_INCREMENTS hit without reaching target -- stopping");
    return -1;
}

// -- test ---------------------------------------------------------------------

void test_motor_encoder(void)
{
    console_input_init();
    encoder_init();
    stepper_init();

    printf("\n================ manual landmark mapping (driver OFF) ================\n");

    wait_enter("Let the beam rest at 6 o'clock (crank straight down)");
    uint16_t enc6 = encoder_read_avg();
    printf("  enc6  = %4u\n", enc6);

    wait_enter("Lift the beam to LEVEL (crank at 9) and HOLD it");
    uint16_t enc9 = encoder_read_avg();
    int32_t d69 = enc_delta(enc6, enc9);
    printf("  enc9  = %4u    6->9  = %+5ld counts = %+7.2f deg\n", enc9, (long)d69, deg(d69));

    wait_enter("Move the beam on to 12 o'clock (crank straight up) and HOLD it");
    uint16_t enc12 = encoder_read_avg();
    int32_t d912 = enc_delta(enc9, enc12);
    int32_t d612 = d69 + d912;
    printf("  enc12 = %4u    9->12 = %+5ld counts = %+7.2f deg\n", enc12, (long)d912, deg(d912));

    int sign = d612 > 0 ? 1 : -1;
    printf("\n  trajectory 6 -> 12: %+ld counts = %+.2f deg  (expect ~180)\n", (long)d612, deg(d612));
    printf("  encoder counts %s from 6 toward 12\n", sign > 0 ? "UP" : "DOWN");
    if (d69 * d912 <= 0) {
        ESP_LOGW(TAG, "6->9 and 9->12 have different signs -- landmarks look wrong, check and reset");
    }

    wait_enter("Let the beam back down to 6 and let go");
    uint16_t pos = encoder_read_avg();
    int32_t back = enc_delta(enc6, pos);
    printf("  now at %4u  (%+.2f deg from enc6)\n", pos, deg(back));
    if (back < -OVERSHOOT_LIMIT_COUNTS || back > OVERSHOOT_LIMIT_COUNTS) {
        ESP_LOGE(TAG, "not back at 6 -- refusing to power the driver here. Reset and retry.");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("\n================ powered sweep (driver ON at 6, zero load) ================\n");
    gpio_set_level(PIN_TMC_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    pos = encoder_read_avg();
    printf("  driver ON, holding at %u\n", pos);

    // Probe: one full step with DIR=0, see which way the encoder goes.
    set_dir(0);
    uint16_t before = pos;
    step_pulses(PROBE_USTEPS);
    vTaskDelay(pdMS_TO_TICKS(50));
    encoder_read(&pos);
    int32_t probe = enc_delta(before, pos);
    int dir_toward_12 = (probe * sign > 0) ? 0 : 1;
    printf("  probe: DIR=0 x %d usteps -> %+ld counts  =>  DIR=%d goes toward 12, DIR=%d toward 6\n",
           PROBE_USTEPS, (long)probe, dir_toward_12, 1 - dir_toward_12);
    // undo the probe so the sweep starts from the recorded 6
    set_dir(1 - 0);
    step_pulses(PROBE_USTEPS);
    vTaskDelay(pdMS_TO_TICKS(50));
    encoder_read(&pos);

    int n_up = sweep_to("6 -> 12", dir_toward_12, sign, enc12, &pos);
    if (n_up > 0) {
        vTaskDelay(pdMS_TO_TICKS(500));
        int n_down = sweep_to("12 -> 6", 1 - dir_toward_12, -sign, enc6, &pos);
        int32_t drift = enc_delta(enc6, pos);
        printf("\n===== results =====\n");
        printf("  up   : %d increments = %d usteps for %+.2f deg measured\n",
               n_up, n_up * USTEPS_PER_INCREMENT, deg(d612));
        if (n_down > 0) {
            printf("  down : %d increments\n", n_down);
        }
        printf("  usteps per encoder count : %.3f measured, %.3f nominal\n",
               n_up > 0 && d612 ? (double)(n_up * USTEPS_PER_INCREMENT) / (d612 * sign) : 0.0,
               1.0 / COUNTS_PER_USTEP);
        printf("  drift after round trip   : %+ld counts = %+.2f deg  (step loss if far from 0)\n",
               (long)drift, deg(drift));
        printf("  DIR=%d -> toward 12 -> ball toward p2 (45 cm)\n", dir_toward_12);
        printf("  DIR=%d -> toward 6  -> ball toward p1 (0 cm)\n", 1 - dir_toward_12);
        printf("====================\n");
    } else {
        ESP_LOGE(TAG, "sweep aborted -- see above. Leaving the driver ON so the beam doesn't drop.");
        ESP_LOGE(TAG, "Support the beam, then reset.");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Back at 6 the crank carries no load, so releasing here is safe.
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(PIN_TMC_EN, 1);
    ESP_LOGI(TAG, "driver OFF at 6. Done -- reset to run again.");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
