#pragma once

// AS5600 shaft encoder for the control loop: one blocking I2C read per
// tick, LUT correction (encoder_lut.h), and a plausibility filter that
// keeps a bad read from reaching the controller.

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    uint16_t raw;        // RAW_ANGLE as read (last accepted read if this tick failed)
    float corrected;     // LUT-corrected, 0..4096
    float pos_counts;    // shaft angle from the 6 o'clock rest, + toward 12
    bool ok;             // this tick's read was accepted
    uint32_t i2c_errors; // cumulative
    uint32_t rejects;    // cumulative, implausible delta
    uint32_t dir_faults; // cumulative, x / 4096-x signature (K-018)
} encoder_sample_t;

// Brings up the bus at ENC_I2C_HZ and takes the first read to seed the
// filter. Fails if the encoder does not answer.
esp_err_t encoder_init(void);

// One tick: read, correct, filter. Never blocks longer than the I2C
// timeout. Safe to call only from one task.
void encoder_update(encoder_sample_t *out);

// Counts from 6 (+ toward 12) for a corrected angle, wrapped to
// (-2048, 2048].
float encoder_pos_from_corrected(float corrected);
