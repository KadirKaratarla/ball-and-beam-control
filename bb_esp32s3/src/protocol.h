#pragma once

// PC <-> ESP32 binary protocol over the USB-Serial-JTAG port (Faz 4).
// gui/protocol.py mirrors this file field for field; change both.
//
// Frame:  [0xAA 0x55] [type u8] [len u8] [payload: len bytes] [crc8]
//   crc8 (TMC2208 polynomial 0x07, same as before) covers type, len and
//   payload. All multi-byte fields little-endian, structs packed.
//   len <= PROTO_MAX_PAYLOAD.

#include <stdint.h>

#define PROTO_SYNC0 0xAA
#define PROTO_SYNC1 0x55
#define PROTO_MAX_PAYLOAD 64
#define PROTO_HEADER_LEN 4 // sync0 sync1 type len
#define PROTO_MAX_FRAME (PROTO_HEADER_LEN + PROTO_MAX_PAYLOAD + 1)
#define PROTO_VERSION 1

// --- PC -> ESP ---------------------------------------------------------------
#define PROTO_T_POS 0x01        // proto_pos_t, one per camera frame
#define PROTO_T_SETPOINT 0x20   // proto_setpoint_t
#define PROTO_T_GAINS 0x21      // proto_gains_t
#define PROTO_T_MODE 0x22       // proto_mode_t
#define PROTO_T_PING 0x23       // proto_ping_t -> ACK with the same nonce
#define PROTO_T_GET_CONFIG 0x24 // no payload -> CONFIG
#define PROTO_T_THETA 0x25      // proto_theta_t, open-loop beam angle

// --- ESP -> PC ---------------------------------------------------------------
#define PROTO_T_TELEM 0x10  // proto_telem_t, every TELEM_BIN_DECIMATION ticks
#define PROTO_T_HEALTH 0x11 // proto_health_t, 1 Hz
#define PROTO_T_ACK 0x12    // proto_ack_t, one per received command
#define PROTO_T_CONFIG 0x13 // proto_config_t, at boot and on request

// ACK results
#define PROTO_ACK_OK 0
#define PROTO_ACK_BAD_LEN 1
#define PROTO_ACK_OUT_OF_RANGE 2
#define PROTO_ACK_UNKNOWN_TYPE 3
#define PROTO_ACK_REJECTED 4 // valid but not applicable in the current state

// MODE commands
#define PROTO_MODE_RUN 0         // normal: engage, level, follow the ball
#define PROTO_MODE_LEVEL 1       // hold the beam level, ignore the ball
#define PROTO_MODE_STOP 2        // pulses off, driver off (beam drops to 6)
#define PROTO_MODE_RESET_FAULT 3 // leave FAULT/STOP, re-engage
#define PROTO_MODE_OPENLOOP 4    // autotune: beam angle from THETA commands

// POS flags
#define PROTO_POS_VALID 0x01
#define PROTO_POS_WARNING 0x02

// TELEM flags
#define PROTO_TF_BALL_VALID 0x01
#define PROTO_TF_LINK_STALE 0x02
#define PROTO_TF_DRIVER_ON 0x04

typedef struct __attribute__((packed)) {
    uint8_t seq;
    int16_t pos_0p1mm;
    uint8_t flags;
} proto_pos_t;

typedef struct __attribute__((packed)) {
    int16_t x_0p1mm;
} proto_setpoint_t;

typedef struct __attribute__((packed)) {
    float kp, ki, kd; // deg/cm, deg/(cm*s), deg/(cm/s)
    float d_tau_s;
} proto_gains_t;

typedef struct __attribute__((packed)) {
    uint8_t mode;
} proto_mode_t;

typedef struct __attribute__((packed)) {
    uint32_t nonce;
} proto_ping_t;

// Open-loop beam angle (MODE_OPENLOOP only). The firmware levels the beam
// if no THETA arrives within OPENLOOP_TIMEOUT_MS, so a stalled PC or a
// dropped link cannot leave it tilted.
typedef struct __attribute__((packed)) {
    int16_t theta_0p01deg;
} proto_theta_t;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    int16_t x_0p1mm;      // last camera position
    int16_t x_set_0p1mm;
    int16_t p_0p01deg, i_0p01deg, d_0p01deg, theta_0p01deg;
    int16_t phi_0p1deg;   // crank command from level
    int16_t enc_counts;   // measured shaft, counts from 6
    int16_t follow_0p1;   // following error, 0.1 counts
    int8_t lag_counts;    // encoder lag correction
    uint8_t state;        // ctrl_state_t
    uint8_t fault;        // fault_t
    uint8_t flags;        // PROTO_TF_*
    uint8_t last_seq;     // seq of the last POS packet used
    uint16_t loop_us;
    int16_t cmd_vel_10;   // step velocity / 10 usteps/s
} proto_telem_t;

typedef struct __attribute__((packed)) {
    uint16_t ticks;
    uint16_t loop_us_mean, loop_us_max;
    uint16_t overruns, missed;
    uint32_t enc_i2c_errors, enc_rejects, enc_dir_faults;
    uint32_t link_packets, link_crc_errors, link_seq_gaps;
    uint32_t cmd_rx, cmd_bad;
    uint32_t tx_dropped;
    uint16_t tmc_resets, tmc_uart_errors;
    uint32_t drv_status;
    uint8_t state, fault;
} proto_health_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd_type;
    uint8_t result;
    uint32_t nonce; // PING nonce, else 0
} proto_ack_t;

typedef struct __attribute__((packed)) {
    uint16_t version;
    float kp, ki, kd, d_tau_s;
    float theta_max_deg;
    int16_t x_set_0p1mm;
    float vmax, amax;     // usteps/s, usteps/s^2
    int16_t level_counts;
    uint8_t irun, ihold;
    uint8_t defaults;     // 1 while the gains are the compiled-in ones
    float kp_def, ki_def, kd_def, d_tau_def; // the compiled-in gains
} proto_config_t;
