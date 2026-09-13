#pragma once

// AS5600 angle correction table for the ball_beam rig's encoder.
//
// Generated 2026-09-14 from a free-shaft sweep: two continuous
// revolutions (forward and reverse) at 3.75 rpm, RAW_ANGLE sampled every
// 32 microsteps = 1600 points per revolution, commanded position exact from
// the 1/256 microstep count. The motor was separately verified to turn
// exactly one revolution per 51200 microsteps.
//
// The magnet sits slightly off-centre and tilted, so RAW_ANGLE carries a
// smooth periodic error: -2.8 .. +6.0 degrees over one turn. Harmonic
// content (degrees): 1st 3.05, 2nd 1.83, 3rd 0.31, 4th 0.05; a 4-harmonic
// model fits to 0.043 deg rms. Forward/reverse hysteresis is 0.17 deg, the
// rotor's lag in continuous motion, and is not part of this table.
//
// Built from the forward sweep, validated against the reverse sweep:
// uncorrected max 6.04 deg, corrected max 0.50 deg / 0.17 deg rms (what
// remains is the rotor lag above).
//
// THIS TABLE BELONGS TO THIS MAGNET IN THIS POSITION. Re-seating or
// re-centring the magnet invalidates it -- rerun test_motor_free and
// regenerate.
//
// Model used to build it, error in counts as a function of absolute raw
// angle theta = 2*pi*raw/4096 (kept here so the table can be regenerated
// or replaced by direct evaluation if ever preferred):
//   err = +19.649
//       + -29.813*sin(theta)  + -17.735*cos(theta)
//       + +13.179*sin(2theta) + +16.188*cos(2theta)
//       + +2.156*sin(3theta) + -2.725*cos(3theta)
//       + -0.435*sin(4theta) + +0.428*cos(4theta)
//   corrected = raw - err(raw)

#include <stdint.h>

#define ENCODER_LUT_SIZE  256
#define ENCODER_LUT_SHIFT 4     // raw (0..4095) >> 4 -> index

// Correction to ADD to the raw reading, in counts, tabulated at raw = i*16.
static const float encoder_lut_correction[ENCODER_LUT_SIZE] = {
     -15.77f,  -15.81f,  -15.83f,  -15.83f,  -15.81f,  -15.77f,  -15.71f,  -15.62f,
     -15.50f,  -15.36f,  -15.18f,  -14.98f,  -14.74f,  -14.47f,  -14.17f,  -13.83f,
     -13.46f,  -13.05f,  -12.60f,  -12.12f,  -11.59f,  -11.03f,  -10.43f,   -9.79f,
      -9.12f,   -8.40f,   -7.65f,   -6.85f,   -6.03f,   -5.16f,   -4.26f,   -3.33f,
      -2.36f,   -1.36f,   -0.34f,    0.72f,    1.80f,    2.90f,    4.02f,    5.16f,
       6.31f,    7.48f,    8.65f,    9.83f,   11.01f,   12.19f,   13.36f,   14.52f,
      15.67f,   16.79f,   17.90f,   18.98f,   20.03f,   21.04f,   22.01f,   22.94f,
      23.83f,   24.66f,   25.44f,   26.16f,   26.82f,   27.41f,   27.94f,   28.39f,
      28.78f,   29.09f,   29.32f,   29.47f,   29.55f,   29.55f,   29.46f,   29.30f,
      29.05f,   28.72f,   28.31f,   27.83f,   27.26f,   26.62f,   25.90f,   25.10f,
      24.24f,   23.30f,   22.30f,   21.23f,   20.09f,   18.90f,   17.65f,   16.34f,
      14.98f,   13.58f,   12.12f,   10.62f,    9.08f,    7.50f,    5.88f,    4.23f,
       2.55f,    0.85f,   -0.89f,   -2.64f,   -4.42f,   -6.21f,   -8.01f,   -9.83f,
     -11.66f,  -13.50f,  -15.34f,  -17.18f,  -19.02f,  -20.86f,  -22.70f,  -24.53f,
     -26.34f,  -28.15f,  -29.94f,  -31.71f,  -33.47f,  -35.20f,  -36.91f,  -38.59f,
     -40.24f,  -41.86f,  -43.45f,  -45.00f,  -46.51f,  -47.98f,  -49.41f,  -50.79f,
     -52.13f,  -53.41f,  -54.65f,  -55.83f,  -56.95f,  -58.02f,  -59.03f,  -59.99f,
     -60.88f,  -61.72f,  -62.49f,  -63.20f,  -63.85f,  -64.43f,  -64.96f,  -65.42f,
     -65.81f,  -66.15f,  -66.43f,  -66.64f,  -66.80f,  -66.90f,  -66.94f,  -66.93f,
     -66.86f,  -66.74f,  -66.57f,  -66.35f,  -66.08f,  -65.77f,  -65.41f,  -65.01f,
     -64.57f,  -64.09f,  -63.57f,  -63.02f,  -62.43f,  -61.81f,  -61.16f,  -60.48f,
     -59.77f,  -59.04f,  -58.29f,  -57.51f,  -56.70f,  -55.88f,  -55.04f,  -54.18f,
     -53.31f,  -52.42f,  -51.51f,  -50.59f,  -49.66f,  -48.72f,  -47.76f,  -46.80f,
     -45.83f,  -44.86f,  -43.88f,  -42.89f,  -41.90f,  -40.91f,  -39.92f,  -38.93f,
     -37.94f,  -36.96f,  -35.98f,  -35.00f,  -34.03f,  -33.08f,  -32.13f,  -31.19f,
     -30.27f,  -29.36f,  -28.47f,  -27.60f,  -26.74f,  -25.91f,  -25.10f,  -24.31f,
     -23.54f,  -22.80f,  -22.09f,  -21.40f,  -20.74f,  -20.11f,  -19.51f,  -18.94f,
     -18.41f,  -17.90f,  -17.42f,  -16.98f,  -16.56f,  -16.18f,  -15.83f,  -15.51f,
     -15.22f,  -14.97f,  -14.74f,  -14.54f,  -14.37f,  -14.22f,  -14.11f,  -14.01f,
     -13.95f,  -13.90f,  -13.88f,  -13.88f,  -13.89f,  -13.93f,  -13.98f,  -14.04f,
     -14.12f,  -14.21f,  -14.31f,  -14.42f,  -14.53f,  -14.65f,  -14.77f,  -14.89f,
     -15.02f,  -15.14f,  -15.25f,  -15.36f,  -15.46f,  -15.56f,  -15.64f,  -15.71f,
};

// Corrected shaft angle in counts (0..4096), linear interpolation between
// table entries, wrapping at the seam.
static inline float encoder_correct(uint16_t raw)
{
    uint32_t i = raw >> ENCODER_LUT_SHIFT;
    float frac = (raw & ((1u << ENCODER_LUT_SHIFT) - 1)) / (float)(1u << ENCODER_LUT_SHIFT);
    float c0 = encoder_lut_correction[i];
    float c1 = encoder_lut_correction[(i + 1) & (ENCODER_LUT_SIZE - 1)];
    float corrected = (float)raw + c0 + (c1 - c0) * frac;
    if (corrected < 0.0f) corrected += 4096.0f;
    if (corrected >= 4096.0f) corrected -= 4096.0f;
    return corrected;
}
