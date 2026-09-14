#pragma once

// Faz 2 application constants. Pin assignments live in board_pins.h; this
// file holds everything that was *measured* during Faz 1 and the limits the
// control loop is built on. Every number here has a source noted next to it.

#include "board_pins.h"

// ---------------------------------------------------------------------------
// Encoder geometry (K-019, K-020, K-021)
// ---------------------------------------------------------------------------
#define ENC_COUNTS_PER_REV 4096

// RAW_ANGLE with the driver off and the crank hanging at 6 o'clock, read at
// the start of the loaded sweep on 2026-09-14. The magnet is fixed to the
// shaft, so this does not drift; it changes only if the magnet is reseated
// (then encoder_lut.h must be regenerated too).
#define ENC_RAW_AT_6 535

// Going 6 -> 12 the encoder DEcreases (K-020). Shaft position is reported
// as counts from 6, positive toward 12, so 12 o'clock is about +2048.
#define ENC_SIGN_TOWARD_12 (-1)

// Startup sanity: the beam must be resting at 6 (driver off) within this
// window, else the firmware refuses to power the driver (K-019).
#define ENC_START_WINDOW_COUNTS 150

// Plausibility filter: the largest shaft movement one 4 ms tick can
// legitimately show. 64 counts/4 ms = 5.6 deg/4 ms = 234 rpm, far above
// anything the crank will do; anything larger is a bad read.
#define ENC_MAX_DELTA_PER_TICK 64

// After this many consecutive rejects the filter accepts the new value
// anyway -- otherwise one real jump would lock it out forever.
#define ENC_REJECT_RESYNC 5

// raw + previous raw within this of 4096 while the delta is implausible is
// the x / 4096-x flip signature of a floating DIR pin (K-018).
#define ENC_DIR_FAULT_TOL 24

#define ENC_I2C_HZ 400000     // 222 us per read (K-011)
// A failed read costs the full timeout inside the control task; 1 ms is
// 4x the read time and keeps one bad tick under a quarter of the period.
#define ENC_I2C_TIMEOUT_MS 1

// ---------------------------------------------------------------------------
// Stepper (K-007, K-009, K-020)
// ---------------------------------------------------------------------------
#define STEP_USTEPS_PER_REV 51200 // 200 full steps x 1/256
#define STEP_USTEPS_PER_COUNT ((float)STEP_USTEPS_PER_REV / ENC_COUNTS_PER_REV) // 12.5

#define STEP_DIR_LEVEL_TOWARD_12 1 // DIR=1 -> crank toward 12 -> ball toward p2

// Velocity is signed, positive toward 12, in microsteps per second.
// 3200 usteps/s = 3.75 rpm, the speed the free-shaft run proved smooth.
#define STEP_VMAX 3200.0f
// 8000 usteps/s^2 reaches VMAX in 0.4 s.
#define STEP_AMAX 8000.0f
// Below this LEDC's divider runs out (80 MHz / 1024 / 1024 = 76 Hz), and
// 80 usteps/s is 0.56 deg/s anyway: treated as stopped, pulses off.
#define STEP_VMIN_HZ 80

#define STEP_TMC_IRUN 20 // holds at level with margin (K-020)
#define STEP_TMC_IHOLD 12
#define STEP_TMC_IHOLDDELAY 4
#define STEP_TMC_GCONF 0x000000C0u    // pdn_disable, mstep_reg_select
#define STEP_TMC_CHOPCONF 0x100101B5u // MRES=0 (1/256), intpol, TBL=2, HSTRT=4, TOFF=5

// ---------------------------------------------------------------------------
// Faz 2 open-loop motion: slow triangle 6 -> +MOTION_TRAVEL_COUNTS -> 6
// ---------------------------------------------------------------------------
#define MOTION_TRAVEL_COUNTS 1000 // ~88 deg, just past level (level ~ 87 deg, K-021)
#define MOTION_DWELL_MS 1000
#define MOTION_START_DELAY_MS 3000 // "keep clear" warning before the driver engages

// ---------------------------------------------------------------------------
// Guards -- any of these trips a latched fault: pulses off, driver disabled
// ---------------------------------------------------------------------------
// Shaft outside [6 - 100, 12 + 100] counts.
#define GUARD_ENC_MIN_COUNTS (-100)
#define GUARD_ENC_MAX_COUNTS (2048 + 100)
// Commanded (pulse count) vs measured (encoder) disagreement, in counts.
// 150 counts = 13 deg; the LUT-corrected sweep closed within 3 counts.
#define GUARD_FOLLOW_ERR_COUNTS 150
// Consecutive ticks the encoder may fail (I2C error or reject) before fault.
#define GUARD_ENC_BAD_TICKS 25 // 100 ms
// PC link older than PC_LINK_STALE_US stops motion (ramps to zero, holds).
// The position is not used in Faz 2, but the fail-safe path is exercised
// from day one. Run gui/link.py (--synthetic is enough) to keep it live.
#define GUARD_LINK_STALE_STOPS 1

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
#define TELEM_SAMPLE_DECIMATION 25 // 250 Hz / 25 = 10 Hz rows
#define TELEM_STATS_TICKS 250      // 1 Hz budget/health line
#define TELEM_QUEUE_LEN 32

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------
#define TASK_CONTROL_PRIO (configMAX_PRIORITIES - 2)
#define TASK_CONTROL_CORE 0
#define TASK_LINK_PRIO 10
#define TASK_DIAG_PRIO 5
#define TASK_TELEM_PRIO 3
#define TASK_AUX_CORE 1
#define LINK_POLL_PERIOD_MS 2
#define DIAG_PERIOD_MS 100
