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

// Startup sanity: the beam must be resting near 6 (driver off) within this
// window, else the firmware refuses to power the driver (K-019). Friction
// leaves the crank anywhere up to ~20 deg off 6; 400 counts = 35 deg still
// means a third of the level-position torque at engage.
#define ENC_START_WINDOW_COUNTS 400

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
// What the closed loop needs (test_pid_sim: unstable at 3200, settles in
// 1 s at 43000 = 50 rpm). Validated by test_jog's speed sweep on
// 2026-09-20: +-53 deg swings, no step loss, 20 counts dynamic lag,
// +4..+9 counts static lag under gravity (rotor behind the field).
#define STEP_VMAX_CLOSED_LOOP 43000.0f
#define STEP_AMAX_CLOSED_LOOP 430000.0f
// Position tracker: v = min(vmax, sqrt(2 a e), KV * e). The linear zone
// (1/KV = 25 ms) makes sub-degree corrections glide at a few rpm instead
// of bursting at full acceleration; large errors still get the sqrt/vmax
// response (test_pid_sim / bench 2026-09-20).
#define STEP_TRACK_KV 40.0f
// Below this LEDC's divider runs out (80 MHz / 1024 / 1024 = 76 Hz), and
// 80 usteps/s is 0.56 deg/s anyway: treated as stopped, pulses off.
#define STEP_VMIN_HZ 80

#define STEP_TMC_IRUN 10 // 0.61 A rms (vsense=0: CS 31 = 1.77 A). 20 (1.16 A) and 14 (0.83 A) ran hot; load needs far less
#define STEP_TMC_IHOLD 4
#define STEP_TMC_IHOLDDELAY 4
#define STEP_TMC_GCONF 0x000000C0u    // pdn_disable, mstep_reg_select
#define STEP_TMC_CHOPCONF 0x100101B5u // MRES=0 (1/256), intpol, TBL=2, HSTRT=4, TOFF=5

// ---------------------------------------------------------------------------
// Linkage geometry (mm) -- from the reference STLs (IVProjects) and the
// user's measurement, 2026-09-20. Side view, origin at the motor shaft,
// x toward the hinge tower, y up. At beam level the crank points at 9
// o'clock (away from the hinge tower), the rod is vertical and the pivot
// sits exactly one rod length above the shaft (150 - 60 = 90 mm).
// ---------------------------------------------------------------------------
#define LINK_CRANK_R_MM 31.0f    // shaft bore to pin, Crank.STL
#define LINK_ROD_L_MM 90.0f      // bearing seat to bearing seat, Coupler.STL
#define LINK_BEAM_D_MM 477.0f    // hinge pin to beam-end pin, measured
#define LINK_PIVOT_DY_MM 90.0f   // pivot height above the shaft
// Pivot x: beam-end pin is directly above the 9 o'clock crank pin (x = -r).
#define LINK_PIVOT_DX_MM (LINK_BEAM_D_MM - LINK_CRANK_R_MM)

// Encoder counts (from 6) at which the beam is level: measured with
// test_jog and a spirit level on 2026-09-20 (raw 3545, 96.8 deg from 6).
// Depends only on the magnet seating; re-measure if the magnet moves.
#define ENC_LEVEL_COUNTS 1101

// Beam angle limit. The 90 mm rod tilts as the pin swings, so the exact
// curve is asymmetric: theta(-90) = -4.33, theta(+90) = +3.09 with a peak
// of +3.26 at phi = +75 (test_pid_sim table). Past the peak the gain
// reverses, so stay well inside: 3.0 deg <-> crank about +57 / -52 deg.
#define BEAM_THETA_MAX_DEG 3.0f

// ---------------------------------------------------------------------------
// Ball controller (Faz 3). Plant: x'' = (5/7) g sin(theta) ~ 700 theta cm/s^2
// (theta in rad). PD for wn = 3 rad/s, zeta = 0.7: Kp = wn^2/700,
// Kd = 2 zeta wn / 700, converted to deg/cm. Starting point for 3.5.
// ---------------------------------------------------------------------------
#define BALL_SETPOINT_CM 22.5f   // beam centre (camera axis 0..45 cm)
#define PID_KP 0.74f             // deg per cm
#define PID_KI 0.15f             // deg per cm*s; first run held 1 cm off the setpoint with Ki=0
#define PID_KD 0.34f             // deg per cm/s
#define PID_I_MAX_DEG 1.5f       // integral contribution clamp
#define PID_D_TAU_S 0.15f        // derivative low-pass; 0.06 let 0.3 mm camera noise dither the crank +-1.8 deg at rest
#define PID_DT_NOMINAL_S 0.01f   // camera frame period; the loop runs the PID per frame

// ---------------------------------------------------------------------------
// Start-up motion
// ---------------------------------------------------------------------------
#define MOTION_START_DELAY_MS 3000 // "keep clear" warning before the driver engages
// 6 -> level (1101 counts, 97 deg) at a moderate speed, not the closed-loop one.
#define STEP_VMAX_ENGAGE 10000.0f
#define STEP_AMAX_ENGAGE 100000.0f
// Hard clamp on the crank command around level, whatever the linkage says
// (theta max 3.0 deg is ~650 counts; the +theta peak is at ~850).
#define CRANK_CMD_MAX_COUNTS 650
// After the driver energises the rotor snaps to a pole; the shaft
// reference is taken after this settle time.
#define ENGAGE_SETTLE_MS 200
// Encoder correction of the crank command: integrates (target - measured)
// with this time constant, bounded, and only while the crank is not being
// driven (so it estimates the static rotor lag, not the dynamic one, and
// adds no phase lag inside the ball loop's band). Cancels the
// load-dependent lag (+4..+40 counts seen) so the beam angle is what the
// PID asked for.
#define LAG_COMP_TAU_S 1.5f
#define LAG_COMP_MAX_COUNTS 60.0f
// Crank command deadband (RUN only): changes smaller than this are not
// sent to the motor. 5 counts = 0.4 deg crank = 0.025 deg beam, enough to
// swallow pure camera noise; 14 produced a visible 1.2 deg step limit
// cycle (+-0.5 cm of ball) on the bench.
#define CRANK_CMD_DEADBAND_COUNTS 5
#define CRANK_CMD_DEADBAND_USTEPS ((int32_t)(CRANK_CMD_DEADBAND_COUNTS * STEP_USTEPS_PER_COUNT))

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
// No valid ball position for this long (link stale or ball not seen)
// -> LEVEL_HOLD: beam level, controller reset, resumes when it returns.
#define GUARD_BALL_LOST_MS 200

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
#define TELEM_SAMPLE_DECIMATION 5 // 250 Hz / 5 = 50 Hz rows (tuning); 25 for normal use
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
