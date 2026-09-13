#pragma once

// PC -> ESP32 ball position link over the built-in USB-Serial-JTAG port.
//
// The PC's camera tracker sends one 7-byte packet per frame at 100 Hz:
//
//   [0] 0xAA   sync
//   [1] 0x55   sync
//   [2] seq    u8, wraps -- lets the receiver count dropped packets
//   [3] pos lo i16 little-endian, position along the beam in 0.1 mm
//   [4] pos hi        (45 cm = 4500; +-3.2 m range is far more than needed)
//   [5] flags  bit0 valid (ball seen this frame), bit1 health warning
//   [6] crc8   CRC8-ATM (x^8+x^2+x+1) over bytes 0..5, same as the TMC2208
//
// Each packet is answered with a 4-byte echo [0xAA 0x55 seq crc8] the
// instant it is parsed, so the PC can time the round trip.
//
// The receiver keeps only the latest reading plus its arrival time. The
// control loop reads that and, if it is older than PC_LINK_STALE_US, treats
// the position as unknown -- that is the fail-safe against the PC stalling.

#include <stdint.h>
#include <stdbool.h>

#define PC_LINK_PACKET_LEN 7
#define PC_LINK_ECHO_LEN   4
#define PC_LINK_SYNC0      0xAA
#define PC_LINK_SYNC1      0x55

#define PC_LINK_FLAG_VALID   0x01
#define PC_LINK_FLAG_WARNING 0x02

// Five missed 10 ms frames. Long enough that one dropped packet doesn't
// trip it, short enough that a frozen PC is caught inside a fraction of a
// ball transit.
#define PC_LINK_STALE_US 50000

typedef struct {
    int16_t pos_0p1mm;      // last received position
    uint8_t flags;
    uint8_t seq;
    int64_t received_at_us; // esp_timer time of the last valid packet
    uint32_t packets;       // valid packets so far
    uint32_t crc_errors;
    uint32_t seq_gaps;      // number of times seq skipped (not the size)
    uint32_t resyncs;       // bytes discarded hunting for sync
} pc_link_state_t;

void pc_link_init(void);

// Drain whatever has arrived and parse it. Cheap; call from any task.
void pc_link_poll(void);

// Snapshot of the receiver state (copied out, safe to hold).
void pc_link_get(pc_link_state_t *out);

// True if no valid packet has arrived within PC_LINK_STALE_US.
bool pc_link_is_stale(void);
