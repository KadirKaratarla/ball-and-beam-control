#pragma once

// PC <-> ESP32 link over the built-in USB-Serial-JTAG port, framed as in
// protocol.h. The receiver keeps the latest ball position plus arrival
// time for the control loop (stale after PC_LINK_STALE_US), validates
// commands and queues them for the control task, and ACKs every command
// as soon as it is parsed. Sending is serialised by a mutex and never
// blocks: a full USB buffer drops the frame and counts it.

#include <stdint.h>
#include <stdbool.h>

#include "protocol.h"

// Five missed 10 ms frames. Long enough that one dropped packet doesn't
// trip it, short enough that a frozen PC is caught inside a fraction of a
// ball transit.
#define PC_LINK_STALE_US 50000

#define PC_LINK_FLAG_VALID PROTO_POS_VALID
#define PC_LINK_FLAG_WARNING PROTO_POS_WARNING

typedef struct {
    int16_t pos_0p1mm;      // last received position
    uint8_t flags;
    uint8_t seq;
    int64_t received_at_us; // esp_timer time of the last valid POS packet
    uint32_t packets;       // valid POS packets so far
    uint32_t crc_errors;
    uint32_t seq_gaps;      // number of times seq skipped (not the size)
    uint32_t resyncs;       // bytes discarded hunting for sync
    uint32_t cmd_rx;        // commands accepted (ACK ok)
    uint32_t cmd_bad;       // commands rejected (any other ACK)
    uint32_t tx_dropped;    // frames dropped because the USB buffer was full
} pc_link_state_t;

typedef enum {
    PC_CMD_SETPOINT,
    PC_CMD_GAINS,
    PC_CMD_MODE,
} pc_cmd_type_t;

typedef struct {
    pc_cmd_type_t type;
    union {
        float setpoint_cm;
        struct { float kp, ki, kd, d_tau_s; } gains;
        uint8_t mode;
    } u;
} pc_cmd_t;

void pc_link_init(void);

// Drain whatever has arrived and parse it. Cheap; call from any task.
void pc_link_poll(void);

// Snapshot of the receiver state (copied out, safe to hold).
void pc_link_get(pc_link_state_t *out);

// True if no valid POS packet has arrived within PC_LINK_STALE_US.
bool pc_link_is_stale(void);

// Next validated command for the control task, non-blocking.
bool pc_link_pop_cmd(pc_cmd_t *out);

// Frame and send; returns false (and counts) if it could not be queued.
bool pc_link_send(uint8_t type, const void *payload, uint8_t len);

// Supplied by the control task: current configuration for CONFIG replies.
void control_get_config(proto_config_t *out);
