#include "test_as5600.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "board_pins.h"

#define AS5600_REG_STATUS    0x0B
#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_REG_ANGLE     0x0E
#define AS5600_REG_AGC       0x1A
#define AS5600_REG_MAGNITUDE 0x1B

// STATUS bits (datasheet: MD/ML/MH report what the AGC loop had to do).
#define AS5600_STATUS_MH 0x08 // gain floor hit -> magnet too strong / too close
#define AS5600_STATUS_ML 0x10 // gain ceiling hit -> magnet too weak / too far
#define AS5600_STATUS_MD 0x20 // magnet detected

#define AS5600_COUNTS 4096 // 12-bit, one full turn

#define CONTROL_LOOP_BUDGET_US 2000
#define READOUT_PERIOD_MS 50 // 20 Hz live readout

#define AS5600_POWER_UP_MS 10 // datasheet T_PU
#define I2C_TIMEOUT_MS 100
#define TIMING_ITERATIONS 200

// Start slow and only then push the clock up: a link that fails at 100 kHz
// has a wiring fault, not a timing problem.
#define I2C_SPEED_INITIAL 100000
#define I2C_SPEED_OPERATING 400000

static const char *TAG = "as5600";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static esp_err_t as5600_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

// RAW_ANGLE / ANGLE / MAGNITUDE are 12-bit, high byte first.
static esp_err_t as5600_read_u12(uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];
    esp_err_t err = as5600_read(reg, buf, sizeof(buf));
    if (err == ESP_OK) {
        *out = (uint16_t)(((buf[0] << 8) | buf[1]) & 0x0FFF);
    }
    return err;
}

// Reads the idle level of both lines as plain inputs with every internal pull
// disabled, so the only thing that can hold them high is the breakout's own
// 10k pull-ups. A line reading 0 here is not connected to a powered module.
static void report_line_levels(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_AS5600_SDA) | (1ULL << PIN_AS5600_SCL),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io);
    vTaskDelay(pdMS_TO_TICKS(AS5600_POWER_UP_MS));

    int sda = gpio_get_level(PIN_AS5600_SDA);
    int scl = gpio_get_level(PIN_AS5600_SCL);
    ESP_LOGI(TAG, "idle levels, internal pulls off: SDA(GPIO%d)=%d SCL(GPIO%d)=%d (both must be 1)",
             PIN_AS5600_SDA, sda, PIN_AS5600_SCL, scl);
    if (!sda || !scl) {
        ESP_LOGE(TAG, "  a line stuck at 0 means: broken/loose wire, wrong GPIO, or module not powered");
    } else {
        ESP_LOGI(TAG, "  both lines pulled up -> the board's 10k pull-ups are reaching these pins");
    }

    gpio_reset_pin(PIN_AS5600_SDA);
    gpio_reset_pin(PIN_AS5600_SCL);
}

static void bus_init(gpio_num_t sda, gpio_num_t scl)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .glitch_ignore_cnt = 7,
        // Off on purpose: the breakout carries 10k pull-ups, and the ESP32's
        // ~45k internal ones would only sit in parallel with them.
        .flags.enable_internal_pullup = false,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));
}

static void bus_deinit(void)
{
    ESP_ERROR_CHECK(i2c_del_master_bus(s_bus));
    s_bus = NULL;
}

static void device_attach(uint32_t scl_hz)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = scl_hz,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));
}

static void device_detach(void)
{
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(s_dev));
    s_dev = NULL;
}

static bool scan_bus(void)
{
    bool found_encoder = false;
    int found_any = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus, addr, I2C_TIMEOUT_MS) == ESP_OK) {
            found_any++;
            ESP_LOGI(TAG, "  found device at 0x%02X%s", addr,
                     addr == AS5600_I2C_ADDR ? "  <-- AS5600" : "");
            if (addr == AS5600_I2C_ADDR) {
                found_encoder = true;
            }
        }
    }
    if (!found_any) {
        ESP_LOGW(TAG, "  no devices responded");
    }
    return found_encoder;
}

static void report_magnet(void)
{
    uint8_t status = 0;
    uint8_t agc = 0;
    uint16_t magnitude = 0;

    if (as5600_read(AS5600_REG_STATUS, &status, 1) != ESP_OK ||
        as5600_read(AS5600_REG_AGC, &agc, 1) != ESP_OK ||
        as5600_read_u12(AS5600_REG_MAGNITUDE, &magnitude) != ESP_OK) {
        ESP_LOGE(TAG, "magnet status read failed");
        return;
    }

    ESP_LOGI(TAG, "STATUS = 0x%02X -> MD=%d ML=%d MH=%d", status,
             (status & AS5600_STATUS_MD) ? 1 : 0,
             (status & AS5600_STATUS_ML) ? 1 : 0,
             (status & AS5600_STATUS_MH) ? 1 : 0);

    // At 3.3V the AGC range is 0..128 (0..255 at 5V); mid-scale means the
    // magnet sits at a comfortable distance.
    ESP_LOGI(TAG, "AGC = %u (3V3 range 0..128, aim for mid-scale), MAGNITUDE = %u",
             agc, magnitude);

    if (!(status & AS5600_STATUS_MD)) {
        ESP_LOGE(TAG, "  no magnet detected -- is it diametrically magnetised and centred?");
    }
    if (status & AS5600_STATUS_ML) {
        ESP_LOGW(TAG, "  magnet too weak / too far -- move it closer");
    }
    if (status & AS5600_STATUS_MH) {
        ESP_LOGW(TAG, "  magnet too strong / too close -- move it away");
    }
    if ((status & AS5600_STATUS_MD) && !(status & (AS5600_STATUS_ML | AS5600_STATUS_MH))) {
        ESP_LOGI(TAG, "  magnet mounting OK");
    }
}

// Times the exact transaction the 2 ms control loop will run: address the
// RAW_ANGLE register, then read its two bytes.
static void report_read_time(uint32_t scl_hz)
{
    uint8_t reg = AS5600_REG_RAW_ANGLE;
    uint8_t buf[2];

    // Warm up so the first (slower) transaction doesn't skew the average.
    i2c_master_transmit_receive(s_dev, &reg, 1, buf, sizeof(buf), I2C_TIMEOUT_MS);

    int64_t start = esp_timer_get_time();
    for (int i = 0; i < TIMING_ITERATIONS; i++) {
        if (i2c_master_transmit_receive(s_dev, &reg, 1, buf, sizeof(buf), I2C_TIMEOUT_MS) != ESP_OK) {
            ESP_LOGE(TAG, "timing run failed at %u kHz", (unsigned)(scl_hz / 1000));
            return;
        }
    }
    int64_t elapsed = esp_timer_get_time() - start;

    double per_read_us = (double)elapsed / TIMING_ITERATIONS;
    ESP_LOGI(TAG, "RAW_ANGLE read @ %4u kHz: %6.1f us  (%.1f%% of the %d us loop budget)",
             (unsigned)(scl_hz / 1000), per_read_us,
             100.0 * per_read_us / CONTROL_LOOP_BUDGET_US, CONTROL_LOOP_BUDGET_US);
}

void test_as5600(void)
{
    report_line_levels();

    ESP_LOGI(TAG, "Scanning I2C bus at %u kHz (SDA=GPIO%d SCL=GPIO%d)...",
             I2C_SPEED_INITIAL / 1000, PIN_AS5600_SDA, PIN_AS5600_SCL);
    bus_init(PIN_AS5600_SDA, PIN_AS5600_SCL);

    if (!scan_bus()) {
        // Swapping the two wires is the classic mistake; rule it out here
        // instead of asking for a rewire.
        ESP_LOGW(TAG, "retrying with SDA/SCL swapped (SDA=GPIO%d SCL=GPIO%d)...",
                 PIN_AS5600_SCL, PIN_AS5600_SDA);
        bus_deinit();
        bus_init(PIN_AS5600_SCL, PIN_AS5600_SDA);

        if (!scan_bus()) {
            ESP_LOGE(TAG, "AS5600 (0x%02X) not found either way -- check VCC at the module and the wires",
                     AS5600_I2C_ADDR);
            return;
        }
        ESP_LOGE(TAG, "*** SDA and SCL are swapped: SDA is on GPIO%d, SCL on GPIO%d ***",
                 PIN_AS5600_SCL, PIN_AS5600_SDA);
        ESP_LOGE(TAG, "*** swap the two wires, or update board_pins.h to match ***");
    }

    device_attach(I2C_SPEED_INITIAL);
    report_magnet();

    ESP_LOGI(TAG, "--- read time budget (Faz 2.2 input) ---");
    report_read_time(I2C_SPEED_INITIAL);
    device_detach();

    device_attach(I2C_SPEED_OPERATING);
    report_read_time(I2C_SPEED_OPERATING);
    device_detach();

    device_attach(1000000);
    report_read_time(1000000);
    device_detach();

    // 400 kHz is the settled operating speed: it costs 10.8% of the 2 ms loop
    // against 7.9% at 1 MHz, and that 60 us is not worth pushing this board's
    // 10k pull-ups past what Fast-mode Plus asks for.
    device_attach(I2C_SPEED_OPERATING);

    ESP_LOGI(TAG, "--- turn the shaft by hand ---");

    // Printed with printf rather than ESP_LOG so the columns stay readable:
    // no level/timestamp/tag prefix in front of every sample.
    printf("\n   raw    angle     step     turns   continuous\n");
    printf(  "-----------------------------------------------\n");

    uint16_t prev_raw = 0;
    bool have_prev = false;
    int32_t turns = 0;

    while (true) {
        uint16_t raw = 0;
        if (as5600_read_u12(AS5600_REG_RAW_ANGLE, &raw) != ESP_OK) {
            ESP_LOGE(TAG, "read failed");
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // Unwrap the 0<->4095 seam so a full turn keeps counting instead of
        // jumping. Anything past half a turn between two samples is read as a
        // wrap, which holds as long as we sample far faster than the shaft
        // turns -- at 20 Hz that means below 10 rev/s.
        int32_t step = 0;
        if (have_prev) {
            step = (int32_t)raw - (int32_t)prev_raw;
            if (step > AS5600_COUNTS / 2) {
                step -= AS5600_COUNTS;
                turns--;
            } else if (step < -AS5600_COUNTS / 2) {
                step += AS5600_COUNTS;
                turns++;
            }
        }
        have_prev = true;
        prev_raw = raw;

        double deg = raw * 360.0 / AS5600_COUNTS;
        double step_deg = step * 360.0 / AS5600_COUNTS;
        double continuous_deg = turns * 360.0 + deg;

        printf("%6u  %7.2f  %+7.2f  %8ld  %10.2f\n",
               raw, deg, step_deg, (long)turns, continuous_deg);

        vTaskDelay(pdMS_TO_TICKS(READOUT_PERIOD_MS));
    }
}
