#pragma once

// Everything the control task wants to say goes through here as a struct
// in a queue; a low-priority task on the other core turns it into text.
// The control task itself never touches the console (K-022).

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CTRL_STATE_WAIT = 0,  // start delay, driver off
    CTRL_STATE_RUN = 1,   // open-loop profile running
    CTRL_STATE_HOLD = 2,  // stopped by a stale link, resumes when it returns
    CTRL_STATE_FAULT = 3, // latched, driver off
} ctrl_state_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_ENC_RANGE,   // shaft outside the guard window
    FAULT_FOLLOW_ERR,  // pulses vs encoder disagree
    FAULT_ENC_DEAD,    // too many consecutive bad reads
    FAULT_TMC_RESET,   // driver lost its registers
    FAULT_TMC_UART,    // driver stopped answering
} fault_t;

// 10 Hz row
typedef struct {
    uint32_t t_ms;
    uint16_t enc_raw;
    float enc_pos;    // counts from 6, + toward 12
    uint8_t enc_ok;
    int16_t pos_0p1mm;
    uint8_t pos_valid;
    uint8_t link_stale;
    float cmd_vel;    // usteps/s
    float cmd_pos;    // counts from 6, from the pulse counter
    float follow_err; // cmd_pos - enc_pos
    uint16_t loop_us;
    uint8_t state;
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
