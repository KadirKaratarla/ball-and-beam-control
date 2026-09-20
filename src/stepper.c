#include "stepper.h"

#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_check.h"

#include "config.h"
#include "tmc2208.h"

static const char *TAG = "stepper";

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CH LEDC_CHANNEL_0
#define LEDC_RES LEDC_TIMER_10_BIT
#define LEDC_DUTY_50 (1u << 9)

// PCNT counters are 16 bit; accum_count folds the overflows into the
// value returned by pcnt_unit_get_count as long as both limits are watched.
#define PCNT_HIGH_LIMIT 32000
#define PCNT_LOW_LIMIT (-32000)

static pcnt_unit_handle_t s_pcnt;
static float s_vel;          // usteps/s, signed, currently generated
static uint32_t s_freq_hz;   // 0 = pulses off
static int s_dir_level = -1; // last level written to DIR
static bool s_enabled;
static float s_vmax = STEP_VMAX;
static float s_amax = STEP_AMAX;
static float s_dv_max = STEP_AMAX * (CONTROL_LOOP_PERIOD_US / 1e6f);

bool stepper_configure_tmc(void)
{
    uint32_t ihold_irun = ((uint32_t)STEP_TMC_IHOLDDELAY << 16) |
                          ((uint32_t)STEP_TMC_IRUN << 8) | (uint32_t)STEP_TMC_IHOLD;

    bool ok = tmc2208_write_register(TMC_REG_GCONF, STEP_TMC_GCONF) &&
              tmc2208_write_register(TMC_REG_CHOPCONF, STEP_TMC_CHOPCONF) &&
              tmc2208_write_register(TMC_REG_IHOLD_IRUN, ihold_irun);
    if (!ok) {
        ESP_LOGE(TAG, "TMC register write failed (no UART echo)");
        return false;
    }

    // IHOLD_IRUN is write-only; GCONF and CHOPCONF read back.
    uint32_t gconf = 0, chop = 0;
    if (!tmc2208_read_register(TMC_REG_GCONF, &gconf) ||
        !tmc2208_read_register(TMC_REG_CHOPCONF, &chop)) {
        ESP_LOGE(TAG, "TMC read-back failed");
        return false;
    }
    if (gconf != STEP_TMC_GCONF || chop != STEP_TMC_CHOPCONF) {
        ESP_LOGE(TAG, "TMC read-back mismatch: GCONF=0x%08lX (want 0x%08lX) CHOPCONF=0x%08lX (want 0x%08lX)",
                 (unsigned long)gconf, (unsigned long)STEP_TMC_GCONF,
                 (unsigned long)chop, (unsigned long)STEP_TMC_CHOPCONF);
        return false;
    }
    return true;
}

static void pulses_off(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CH, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CH);
    s_freq_hz = 0;
}

static void pulses_set(uint32_t hz, int dir_level)
{
    if (dir_level != s_dir_level) {
        // Only reached with pulses off: the ramp always passes through the
        // stopped band, so DIR never changes under a running pulse train.
        gpio_set_level(PIN_TMC_DIR, dir_level);
        s_dir_level = dir_level;
    }
    if (hz != s_freq_hz) {
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, hz);
        if (s_freq_hz == 0) {
            ledc_set_duty(LEDC_MODE, LEDC_CH, LEDC_DUTY_50);
            ledc_update_duty(LEDC_MODE, LEDC_CH);
        }
        s_freq_hz = hz;
    }
}

esp_err_t stepper_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_TMC_STEP) | (1ULL << PIN_TMC_DIR) | (1ULL << PIN_TMC_EN),
        .mode = GPIO_MODE_INPUT_OUTPUT, // STEP/DIR also feed PCNT
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio");
    gpio_set_level(PIN_TMC_EN, 1); // disabled
    gpio_set_level(PIN_TMC_STEP, 0);
    gpio_set_level(PIN_TMC_DIR, 0);
    s_dir_level = 0;

    tmc2208_uart_init(PIN_TMC_UART_TX, PIN_TMC_UART_RX, TMC_UART_BAUD);
    if (!stepper_configure_tmc()) {
        return ESP_FAIL;
    }
    uint32_t gstat = 0;
    if (tmc2208_read_register(TMC_REG_GSTAT, &gstat)) {
        // Clear the power-on reset flag so the diag watchdog only sees
        // resets that happen from here on.
        tmc2208_write_register(TMC_REG_GSTAT, 0x07);
        ESP_LOGI(TAG, "TMC configured and verified: 1/256, IRUN=%d (GSTAT was 0x%02lX)",
                 STEP_TMC_IRUN, (unsigned long)gstat);
    }

    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_RES,
        .freq_hz = 1000,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&tcfg), TAG, "ledc timer");
    ledc_channel_config_t ccfg = {
        .gpio_num = PIN_TMC_STEP,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CH,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ccfg), TAG, "ledc channel");

    pcnt_unit_config_t ucfg = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &s_pcnt), TAG, "pcnt unit");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_pcnt, PCNT_HIGH_LIMIT), TAG, "pcnt wp");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_pcnt, PCNT_LOW_LIMIT), TAG, "pcnt wp");

    pcnt_chan_config_t chcfg = {
        .edge_gpio_num = PIN_TMC_STEP,
        .level_gpio_num = PIN_TMC_DIR,
    };
    pcnt_channel_handle_t ch;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_pcnt, &chcfg, &ch), TAG, "pcnt channel");
    // Count rising edges; DIR high (toward 12) counts up, DIR low counts down.
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_HOLD), TAG, "pcnt edge");
#if STEP_DIR_LEVEL_TOWARD_12
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(ch, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "pcnt level");
#else
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(ch, PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP), TAG, "pcnt level");
#endif
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_pcnt), TAG, "pcnt enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_pcnt), TAG, "pcnt clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_pcnt), TAG, "pcnt start");

    ESP_LOGI(TAG, "LEDC step on GPIO%d (10 bit, %d..%d Hz), PCNT loop-back with DIR on GPIO%d",
             PIN_TMC_STEP, STEP_VMIN_HZ, (int)STEP_VMAX, PIN_TMC_DIR);
    return ESP_OK;
}

void stepper_enable(bool on)
{
    gpio_set_level(PIN_TMC_EN, on ? 0 : 1);
    s_enabled = on;
}

bool stepper_is_enabled(void)
{
    return s_enabled;
}

void stepper_set_limits(float vmax, float amax)
{
    s_vmax = vmax;
    s_amax = amax;
    s_dv_max = amax * (CONTROL_LOOP_PERIOD_US / 1e6f);
}

float stepper_tick(float target)
{
    if (target > s_vmax) target = s_vmax;
    if (target < -s_vmax) target = -s_vmax;

    float dv = target - s_vel;
    if (dv > s_dv_max) dv = s_dv_max;
    if (dv < -s_dv_max) dv = -s_dv_max;
    s_vel += dv;

    float mag = s_vel < 0 ? -s_vel : s_vel;
    if (mag < STEP_VMIN_HZ) {
        if (s_freq_hz) pulses_off();
    } else {
        int dir = s_vel > 0 ? STEP_DIR_LEVEL_TOWARD_12 : !STEP_DIR_LEVEL_TOWARD_12;
        pulses_set((uint32_t)(mag + 0.5f), dir);
    }
    return s_vel;
}

float stepper_track(int32_t target_usteps)
{
    int32_t e = target_usteps - stepper_get_step_count();
    int32_t mag = e < 0 ? -e : e;
    if (mag <= 8) { // 8 usteps = 0.06 deg: close enough, stop
        return stepper_tick(0.0f);
    }
    float v = sqrtf(2.0f * s_amax * (float)mag);
    if (v > s_vmax) v = s_vmax;
    return stepper_tick(e < 0 ? -v : v);
}

void stepper_halt(void)
{
    pulses_off();
    s_vel = 0;
}

float stepper_get_velocity(void)
{
    return s_vel;
}

int32_t stepper_get_step_count(void)
{
    int count = 0;
    pcnt_unit_get_count(s_pcnt, &count);
    return count;
}
