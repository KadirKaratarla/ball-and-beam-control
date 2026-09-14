#include "test_motor_free.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#include "board_pins.h"
#include "tmc2208.h"

// Fine, continuous, full-revolution encoder map with the shaft free.
//
// The coarse 10-chunk version of this test proved the motor and driver are
// exact; this one exists for the encoder. The magnet sits off-centre, so
// RAW_ANGLE carries a periodic error of a few degrees, and a correction
// table needs the error sampled densely around the whole circle. Motion is
// slow and never pauses -- the earlier tests' start/stop pattern rang the
// rotor at every stop, and a smooth constant-rate turn both avoids that and
// lets the user confirm by eye that the motor runs clean.

#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_COUNTS 4096
#define USTEPS_PER_REV 51200

#define TMC_IRUN       20
#define TMC_IHOLD      12
#define TMC_IHOLDDELAY 4

#define STEP_HALF_PERIOD_US 160  // 3.2k usteps/s = 3.75 rpm, ~16 s per rev
#define USTEPS_PER_SAMPLE    32  // 0.225 deg = 2.56 counts between reads
#define SAMPLES_PER_REV (USTEPS_PER_REV / USTEPS_PER_SAMPLE) // 1600

static const char *TAG = "motor_free";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

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
    printf("  CHOPCONF=0x%08lX  MRES=%lu (1/%lu)  IRUN=%d\n",
           (unsigned long)chop, (unsigned long)((chop >> 24) & 0xF), 256ul >> ((chop >> 24) & 0xF), TMC_IRUN);
}

static inline void step_pulses(int count)
{
    for (int i = 0; i < count; i++) {
        gpio_set_level(PIN_TMC_STEP, 1);
        esp_rom_delay_us(STEP_HALF_PERIOD_US);
        gpio_set_level(PIN_TMC_STEP, 0);
        esp_rom_delay_us(STEP_HALF_PERIOD_US);
    }
}

// One continuous revolution, encoder sampled every USTEPS_PER_SAMPLE.
// Rows are "sample,raw": the commanded position is sample*USTEPS_PER_SAMPLE
// microsteps, exact, so nothing else needs transmitting.
static void one_rev(const char *name, int dir_level)
{
    gpio_set_level(PIN_TMC_DIR, dir_level);
    esp_rom_delay_us(20);

    uint16_t start = 0;
    encoder_read(&start);
    printf("\n--- %s: DIR=%d, %d samples, %d usteps each, start raw %u ---\n",
           name, dir_level, SAMPLES_PER_REV, USTEPS_PER_SAMPLE, start);
    printf("n,raw\n");

    for (int s = 1; s <= SAMPLES_PER_REV; s++) {
        step_pulses(USTEPS_PER_SAMPLE);
        uint16_t raw = 0;
        if (!encoder_read(&raw)) {
            printf("%d,ERR\n", s);
            continue;
        }
        printf("%d,%u\n", s, raw);
    }

    uint16_t end = 0;
    encoder_read(&end);
    int32_t d = (int32_t)end - (int32_t)start;
    printf("--- %s end raw %u, net %+ld counts (expect ~0 mod 4096) ---\n", name, end, (long)d);
}

void test_motor_free(void)
{
    encoder_init();
    stepper_init();

    ESP_LOGW(TAG, "shaft must be FREE (rod detached). Starting in 3 s -- watch the motor for smoothness.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    gpio_set_level(PIN_TMC_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    one_rev("forward", 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    one_rev("reverse", 1);

    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(PIN_TMC_EN, 1);
    ESP_LOGI(TAG, "driver OFF. Done.");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
