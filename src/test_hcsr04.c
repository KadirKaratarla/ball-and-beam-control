#include "test_hcsr04.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "board_pins.h"

// Least-squares fit over measured (caliper distance, echo width) pairs at
// 5.1 / 10 / 15 / 15.6 cm; residuals stay within +-0.15 cm there. Note the
// slope works out to 54.8 us/cm rather than the 58.3 us/cm that 343 m/s
// predicts, so it is absorbing some bias in the reference measurements as
// well as the sensor's own -- it is only trusted across the calibrated span.
// Re-fit with points further out before relying on longer distances.
#define CAL_CM_PER_US 0.0182489
#define CAL_OFFSET_CM (-0.39153)

// Kept for showing what the uncalibrated conversion would have said.
#define US_PER_CM_NOMINAL (2.0 / 0.0343)

#define TRIG_PULSE_US 10 // datasheet: at least 10 us high

// This also caps how far we can measure: the period minus the module's
// ~460 us ECHO start delay is the flight budget, and an echo that outlasts it
// finishes in the following cycle and lands on the wrong measurement. At
// 10 ms that ceiling is ~163 cm, which was being exceeded -- a ping-pong ball
// at the far end of the beam reflects so little that the burst carried on to
// the room behind it and came back as an aliased ~180 cm. 30 ms lifts the
// ceiling to ~5 m so those far returns at least resolve honestly, which is
// what tells us whether the ball is being seen at all.
#define TRIGGER_PERIOD_MS 30

// Live samples go out at every 5th measurement: 20 Hz is fast enough to
// follow a ball being moved by hand and slow enough to read, and it keeps the
// console well inside what 115200 baud carries.
#define LIVE_PRINT_EVERY 5

// One statistics line per ~1 s window on top of the live stream.
#define SUMMARY_EVERY 100

// A ping-pong ball is a poor ultrasonic target -- small, round, and it
// scatters most of the burst away from the sensor -- so the reading can jump
// to a completely different reflector (the beam surface, a wall) between one
// sample and the next. 3 cm in 10 ms would be 3 m/s, far quicker than the
// ball moves on this beam, so anything past that is counted as a spurious
// reading rather than motion.
#define OUTLIER_JUMP_CM 3.0

// 4 m of range is ~23.3 ms of round trip; anything longer is the module
// reporting "nothing found" rather than a real distance.
#define ECHO_MAX_US 25000

// Below ~5 cm the returning echo overlaps the transmitter's own ringing: the
// module latches onto it late and over-reports (measured 3.50 cm as 4.06 cm),
// and under ~2 cm it reports nothing at all. Readings here are not a
// calibration problem, they are physically unusable.
#define MIN_VALID_CM 5.0

static const char *TAG = "hcsr04";

static volatile int64_t s_rise_us;      // set on the rising edge
static volatile uint32_t s_width_us;    // last completed pulse width
static volatile int64_t s_result_at_us; // when that pulse finished
static volatile uint32_t s_count;       // completed measurements

static void IRAM_ATTR echo_isr(void *arg)
{
    int64_t now = esp_timer_get_time();

    if (gpio_get_level(PIN_HCSR04_ECHO)) {
        s_rise_us = now;
    } else if (s_rise_us > 0) {
        s_width_us = (uint32_t)(now - s_rise_us);
        s_result_at_us = now;
        s_count++;
        s_rise_us = 0;
    }
}

static void configure_gpio(void)
{
    gpio_config_t trig = {
        .pin_bit_mask = 1ULL << PIN_HCSR04_TRIG,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&trig));
    gpio_set_level(PIN_HCSR04_TRIG, 0);

    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << PIN_HCSR04_ECHO,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_HCSR04_ECHO, echo_isr, NULL));
}

// The only part the control loop would pay for: three GPIO writes and a 10 us
// wait. Everything after this happens in the interrupt.
static void fire_trigger(void)
{
    gpio_set_level(PIN_HCSR04_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level(PIN_HCSR04_TRIG, 1);
    esp_rom_delay_us(TRIG_PULSE_US);
    gpio_set_level(PIN_HCSR04_TRIG, 0);
}

void test_hcsr04(void)
{
    configure_gpio();

    ESP_LOGI(TAG, "TRIG=GPIO%d ECHO=GPIO%d, triggering every %d ms",
             PIN_HCSR04_TRIG, PIN_HCSR04_ECHO, TRIGGER_PERIOD_MS);

    int64_t t0 = esp_timer_get_time();
    fire_trigger();
    ESP_LOGI(TAG, "trigger call blocks for %lld us -- the echo arrives later, in the ISR",
             esp_timer_get_time() - t0);

    ESP_LOGI(TAG, "--- live samples at 20 Hz, statistics once per second ---");
    printf("\n echo_us   cal_cm  status\n");
    printf(  "--------------------------\n");

    uint32_t prev_count = s_count;
    uint32_t cycles = 0, ok = 0, missed = 0, outliers = 0;
    double sum_cm = 0.0, min_cm = 1e9, max_cm = -1e9;
    double prev_cm = 0.0;
    bool have_prev = false;
    uint32_t live_tick = 0;

    while (true) {
        fire_trigger();
        vTaskDelay(pdMS_TO_TICKS(TRIGGER_PERIOD_MS));
        cycles++;
        live_tick++;

        uint32_t count = s_count;
        uint32_t width = s_width_us;

        if (count == prev_count) {
            // No falling edge completed within this cycle: either nothing is in
            // range, or the previous measurement is still running because we
            // triggered again too soon.
            missed++;
            // A gap breaks the continuity the jump test relies on, so don't
            // let the sample after it count as an outlier.
            have_prev = false;
            if (live_tick >= LIVE_PRINT_EVERY) {
                live_tick = 0;
                printf("      --       --  no echo\n");
            }
        } else {
            double cal_cm = width * CAL_CM_PER_US + CAL_OFFSET_CM;

            const char *status = "ok";
            if (width > ECHO_MAX_US) {
                status = "out of range";
            } else if (cal_cm < MIN_VALID_CM) {
                status = "TOO CLOSE"; // inside the ringing zone, do not trust
            }

            ok++;
            sum_cm += cal_cm;
            if (cal_cm < min_cm) {
                min_cm = cal_cm;
            }
            if (cal_cm > max_cm) {
                max_cm = cal_cm;
            }

            if (have_prev && fabs(cal_cm - prev_cm) > OUTLIER_JUMP_CM) {
                outliers++;
                status = "JUMP";
            }
            prev_cm = cal_cm;
            have_prev = true;

            if (live_tick >= LIVE_PRINT_EVERY) {
                live_tick = 0;
                printf("%8lu %8.2f  %s\n", (unsigned long)width, cal_cm, status);
            }
        }
        prev_count = count;

        if (cycles >= SUMMARY_EVERY) {
            if (ok > 0) {
                printf("=== n=%lu ok=%lu missed=%lu (%.1f%%) outliers=%lu | "
                       "mean=%.2f min=%.2f max=%.2f spread=%.2f\n",
                       (unsigned long)cycles, (unsigned long)ok, (unsigned long)missed,
                       100.0 * missed / cycles, (unsigned long)outliers,
                       sum_cm / ok, min_cm, max_cm, max_cm - min_cm);
            } else {
                printf("=== n=%lu ok=0 missed=%lu (100%%) -- nothing detected at all\n",
                       (unsigned long)cycles, (unsigned long)missed);
            }
            cycles = ok = missed = outliers = 0;
            sum_cm = 0.0;
            min_cm = 1e9;
            max_cm = -1e9;
        }
    }
}
