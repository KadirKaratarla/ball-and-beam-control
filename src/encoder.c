#include "encoder.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_check.h"

#include "config.h"
#include "encoder_lut.h"

#define AS5600_REG_RAW_ANGLE 0x0C

static const char *TAG = "encoder";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static encoder_sample_t s_last;   // last accepted state, counters included
static uint32_t s_consec_rejects;
static float s_pos_at_6;          // corrected angle of the 6 o'clock rest

static esp_err_t read_raw(uint16_t *out)
{
    uint8_t reg = AS5600_REG_RAW_ANGLE;
    uint8_t buf[2];
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, buf, sizeof(buf), ENC_I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        *out = (uint16_t)(((buf[0] << 8) | buf[1]) & 0x0FFF);
    }
    return err;
}

static float wrap_counts(float d)
{
    while (d > 2048.0f) d -= 4096.0f;
    while (d <= -2048.0f) d += 4096.0f;
    return d;
}

float encoder_pos_from_corrected(float corrected)
{
    return wrap_counts((corrected - s_pos_at_6) * (float)ENC_SIGN_TOWARD_12);
}

// A chip reset that lands in the middle of a read leaves the AS5600 driving
// SDA low; the IDF driver then spins forever on a busy bus (seen 2026-09-20,
// task watchdog on the control task). Standard recovery before the driver
// touches the pins: clock SCL until the slave releases SDA, then a STOP.
static void bus_recover(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_AS5600_SDA) | (1ULL << PIN_AS5600_SCL),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE, // breakout has 10k pull-ups
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(PIN_AS5600_SDA, 1);
    gpio_set_level(PIN_AS5600_SCL, 1);
    esp_rom_delay_us(10);
    bool sda_low = gpio_get_level(PIN_AS5600_SDA) == 0;
    // Always clock out a full byte plus ACK slot: a slave interrupted mid
    // transfer may be waiting for clocks with SDA released, which a level
    // check cannot see.
    for (int i = 0; i < 9; i++) {
        gpio_set_level(PIN_AS5600_SCL, 0);
        esp_rom_delay_us(5);
        gpio_set_level(PIN_AS5600_SCL, 1);
        esp_rom_delay_us(5);
    }
    // STOP: SDA low -> high while SCL is high
    gpio_set_level(PIN_AS5600_SDA, 0);
    esp_rom_delay_us(5);
    gpio_set_level(PIN_AS5600_SDA, 1);
    esp_rom_delay_us(10);
    if (sda_low) {
        ESP_LOGW(TAG, "SDA was held low at boot; after recovery SDA=%d", gpio_get_level(PIN_AS5600_SDA));
    }
    gpio_reset_pin(PIN_AS5600_SDA);
    gpio_reset_pin(PIN_AS5600_SCL);
}

esp_err_t encoder_init(void)
{
    bus_recover();

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_AS5600_SDA,
        .scl_io_num = PIN_AS5600_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, // breakout has 10k pull-ups
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "bus");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = ENC_I2C_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev), TAG, "device");

    // The driver logs two ESP_LOGE lines from the calling task on every
    // failed transaction -- from the control task that is ~14 ms per tick
    // (K-022). Failures are counted in i2c_errors instead.
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    s_pos_at_6 = encoder_correct(ENC_RAW_AT_6);

    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_raw(&raw), TAG, "first read");
    s_last.raw = raw;
    s_last.corrected = encoder_correct(raw);
    s_last.pos_counts = encoder_pos_from_corrected(s_last.corrected);
    s_last.ok = true;

    ESP_LOGI(TAG, "first read raw=%u corrected=%.1f -> %.1f counts from 6 (%.2f deg)",
             raw, s_last.corrected, s_last.pos_counts, s_last.pos_counts * 360.0f / 4096.0f);
    return ESP_OK;
}

void encoder_update(encoder_sample_t *out)
{
    uint16_t raw;
    if (read_raw(&raw) != ESP_OK) {
        s_last.i2c_errors++;
        s_last.ok = false;
        *out = s_last;
        return;
    }

    float corrected = encoder_correct(raw);
    float delta = wrap_counts(corrected - s_last.corrected);
    float mag = delta < 0 ? -delta : delta;

    if (mag > ENC_MAX_DELTA_PER_TICK && s_consec_rejects < ENC_REJECT_RESYNC) {
        s_consec_rejects++;
        s_last.rejects++;
        int32_t sum = (int32_t)raw + (int32_t)s_last.raw;
        if (sum > 4096 - ENC_DIR_FAULT_TOL && sum < 4096 + ENC_DIR_FAULT_TOL) {
            s_last.dir_faults++;
        }
        s_last.ok = false;
        *out = s_last;
        return;
    }

    s_consec_rejects = 0;
    s_last.raw = raw;
    s_last.corrected = corrected;
    s_last.pos_counts = encoder_pos_from_corrected(corrected);
    s_last.ok = true;
    *out = s_last;
}
