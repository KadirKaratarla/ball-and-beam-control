#pragma once

// Everything the control task wants to say goes through here as a struct
// in a queue; a low-priority task on the other core turns it into text.
// The control task itself never touches the console (K-022).

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CTRL_STATE_WAIT = 0,       // start delay, driver off
    CTRL_STATE_ENGAGE = 1,     // driver on, crank moving from 6 to level
    CTRL_STATE_LEVEL_HOLD = 2, // beam held level: no ball / link stale
    CTRL_STATE_RUN = 3,        // closed loop on the ball position
    CTRL_STATE_FAULT = 4,      // latched, driver off
    CTRL_STATE_STOP = 5,       // stopped by command, driver off
} ctrl_state_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_ENC_RANGE,   // shaft outside the guard window
    FAULT_FOLLOW_ERR,  // pulses vs encoder disagree
    FAULT_ENC_DEAD,    // too many consecutive bad reads
    FAULT_TMC_RESET,   // driver lost its registers
    FAULT_TMC_UART,    // driver stopped answering (or never did at boot)
    FAULT_NOT_AT_REST, // beam not resting near 6 when asked to engage (K-019)
} fault_t;

// 10 Hz row (Faz 3.6 format; becomes the Faz 4 telemetry packet)
typedef struct {
    uint32_t t_ms;
    float x_cm, x_set_cm; // ball position and setpoint
    uint8_t ball_valid;
    float p, i, d;        // controller terms, deg
    float theta_cmd;      // beam angle command, deg
    float phi_cmd;        // crank angle command, deg from level
    float enc_pos;        // measured shaft, counts from 6
    float follow_err;     // commanded - measured shaft, counts
    float lag_comp;       // encoder correction added to the command, counts
    float cmd_vel;        // usteps/s
    uint16_t link_age_ms; // age of the last valid packet
    uint16_t loop_us;
    uint8_t state;
    uint8_t fault;
    uint8_t link_stale;
    uint8_t driver_on;
    uint8_t last_seq;     // seq of the last POS packet used by the PID
} telem_sample_t;

// 1 Hz budget + health line
typedef struct {
    uint32_t ticks;
    uint32_t enc_us_sum, enc_us_max;
    uint32_t calc_us_sum, calc_us_max;
    uint32_t step_us_sum, step_us_max;
    uint32_t total_us_sum, total_us_max;
    uint32_t lat_us_sum, lat_us_max; // ISR -> task
    uint32_t overruns;               // total > period
    uint32_t missed_ticks;           // ISRs that found the task still busy
    uint32_t enc_i2c_errors, enc_rejects, enc_dir_faults; // cumulative
    uint32_t link_packets, link_crc_errors, link_seq_gaps;
    uint32_t cmd_rx, cmd_bad, tx_dropped;
    uint32_t tmc_resets, tmc_uart_errors;
    uint32_t drv_status;
    uint8_t state;
    uint8_t fault;
} telem_stats_t;

typedef struct {
    enum { TELEM_SAMPLE, TELEM_STATS } kind;
    union {
        telem_sample_t sample;
        telem_stats_t stats;
    } u;
} telem_msg_t;

void telemetry_init(void);

// Non-blocking; drops the message if the queue is full (counted).
bool telemetry_push(const telem_msg_t *msg);
