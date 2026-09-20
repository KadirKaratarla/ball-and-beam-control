"""PC <-> ESP32 binary protocol (Faz 4). Mirrors bb_esp32s3/src/protocol.h.

Frame:  AA 55 | type u8 | len u8 | payload (len bytes) | crc8
crc8 = polynomial 0x07 (TMC2208 style) over type, len, payload.
All fields little-endian; struct formats below match the packed C structs.
"""

import struct
from collections import namedtuple

SYNC0, SYNC1 = 0xAA, 0x55
MAX_PAYLOAD = 64
VERSION = 1

# PC -> ESP
T_POS, T_SETPOINT, T_GAINS, T_MODE, T_PING, T_GET_CONFIG = 0x01, 0x20, 0x21, 0x22, 0x23, 0x24
# ESP -> PC
T_TELEM, T_HEALTH, T_ACK, T_CONFIG = 0x10, 0x11, 0x12, 0x13

ACK_OK, ACK_BAD_LEN, ACK_OUT_OF_RANGE, ACK_UNKNOWN_TYPE, ACK_REJECTED = range(5)
ACK_NAMES = {ACK_OK: "ok", ACK_BAD_LEN: "bad_len", ACK_OUT_OF_RANGE: "out_of_range",
             ACK_UNKNOWN_TYPE: "unknown_type", ACK_REJECTED: "rejected"}

MODE_RUN, MODE_LEVEL, MODE_STOP, MODE_RESET_FAULT = range(4)

POS_VALID, POS_WARNING = 0x01, 0x02
TF_BALL_VALID, TF_LINK_STALE, TF_DRIVER_ON = 0x01, 0x02, 0x04

STATE_NAMES = {0: "WAIT", 1: "ENGAGE", 2: "LEVEL", 3: "RUN", 4: "FAULT", 5: "STOP"}
FAULT_NAMES = {0: "none", 1: "ENC_RANGE", 2: "FOLLOW_ERR", 3: "ENC_DEAD", 4: "TMC_RESET", 5: "TMC_UART", 6: "NOT_AT_REST"}

# --- payload layouts (struct format, field names) ---------------------------
POS_FMT = "<BhB"
SETPOINT_FMT = "<h"
GAINS_FMT = "<ffff"
MODE_FMT = "<B"
PING_FMT = "<I"

TELEM_FMT = "<IhhhhhhhhhbBBBBHh"
Telem = namedtuple("Telem", "t_ms x_0p1mm x_set_0p1mm p i d theta phi_0p1deg enc_counts "
                            "follow_0p1 lag_counts state fault flags last_seq loop_us cmd_vel_10")

HEALTH_FMT = "<HHHHHIIIIIIIIIHHIBB"
Health = namedtuple("Health", "ticks loop_us_mean loop_us_max overruns missed enc_i2c_errors "
                              "enc_rejects enc_dir_faults link_packets link_crc_errors link_seq_gaps "
                              "cmd_rx cmd_bad tx_dropped tmc_resets tmc_uart_errors drv_status state fault")

ACK_FMT = "<BBI"
Ack = namedtuple("Ack", "cmd_type result nonce")

CONFIG_FMT = "<HfffffhffhBBB"
Config = namedtuple("Config", "version kp ki kd d_tau_s theta_max_deg x_set_0p1mm vmax amax "
                              "level_counts irun ihold defaults")

DECODERS = {
    T_TELEM: (TELEM_FMT, Telem),
    T_HEALTH: (HEALTH_FMT, Health),
    T_ACK: (ACK_FMT, Ack),
    T_CONFIG: (CONFIG_FMT, Config),
}


def crc8(data):
    crc = 0
    for b in data:
        for _ in range(8):
            if (crc >> 7) ^ (b & 1):
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            b >>= 1
    return crc


def frame(ptype, payload=b""):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([ptype, len(payload)]) + payload
    return bytes([SYNC0, SYNC1]) + body + bytes([crc8(body)])


def pack_pos(seq, pos_cm, valid, warning=False):
    v = int(round(pos_cm * 100))
    v = max(-32768, min(32767, v))
    return frame(T_POS, struct.pack(POS_FMT, seq & 0xFF, v, (POS_VALID if valid else 0) | (POS_WARNING if warning else 0)))


def pack_setpoint(x_cm):
    return frame(T_SETPOINT, struct.pack(SETPOINT_FMT, int(round(x_cm * 100))))


def pack_gains(kp, ki, kd, d_tau_s):
    return frame(T_GAINS, struct.pack(GAINS_FMT, kp, ki, kd, d_tau_s))


def pack_mode(mode):
    return frame(T_MODE, struct.pack(MODE_FMT, mode))


def pack_ping(nonce):
    return frame(T_PING, struct.pack(PING_FMT, nonce & 0xFFFFFFFF))


def pack_get_config():
    return frame(T_GET_CONFIG)


def decode(ptype, payload):
    """Returns a namedtuple for known ESP->PC types, else the raw payload."""
    dec = DECODERS.get(ptype)
    if dec is None:
        return payload
    fmt, cls = dec
    if len(payload) != struct.calcsize(fmt):
        raise ValueError(f"type 0x{ptype:02X}: {len(payload)} bytes, expected {struct.calcsize(fmt)}")
    return cls._make(struct.unpack(fmt, payload))


class Parser:
    """Byte-at-a-time frame parser. feed() returns a list of (type, payload)."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.resyncs = 0

    def feed(self, data):
        out = []
        self.buf += data
        while True:
            # hunt for sync
            i = self.buf.find(bytes([SYNC0, SYNC1]))
            if i < 0:
                if len(self.buf) > 1:
                    self.resyncs += len(self.buf) - 1
                    del self.buf[:-1]
                return out
            if i > 0:
                self.resyncs += i
                del self.buf[:i]
            if len(self.buf) < 4:
                return out
            ptype, plen = self.buf[2], self.buf[3]
            if plen > MAX_PAYLOAD:
                self.resyncs += 1
                del self.buf[:1]
                continue
            total = 4 + plen + 1
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2:4 + plen])
            if crc8(body) != self.buf[4 + plen]:
                self.crc_errors += 1
                del self.buf[:1]
                continue
            out.append((ptype, bytes(self.buf[4:4 + plen])))
            del self.buf[:total]
