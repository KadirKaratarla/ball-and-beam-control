#include "pc_link.h"

#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "config.h"

#define RX_BUF 1024
#define TX_BUF 2048
#define CMD_QUEUE_LEN 8

static pc_link_state_t s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_cmds;
static SemaphoreHandle_t s_tx_mutex;

// Receive state machine
static uint8_t s_frame[PROTO_MAX_FRAME];
static int s_fill;
static bool s_have_seq;

// Same polynomial and bit order as the TMC2208 datagram CRC, so the PC side
// can reuse one implementation for both links.
static uint8_t crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (b & 1)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            b >>= 1;
        }
    }
    return crc;
}

void pc_link_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = RX_BUF,
        .tx_buffer_size = TX_BUF,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    memset(&s_state, 0, sizeof(s_state));
    s_fill = 0;
    s_have_seq = false;
    s_cmds = xQueueCreate(CMD_QUEUE_LEN, sizeof(pc_cmd_t));
    s_tx_mutex = xSemaphoreCreateMutex();
}

bool pc_link_send(uint8_t type, const void *payload, uint8_t len)
{
    if (len > PROTO_MAX_PAYLOAD) return false;
    uint8_t buf[PROTO_MAX_FRAME];
    buf[0] = PROTO_SYNC0;
    buf[1] = PROTO_SYNC1;
    buf[2] = type;
    buf[3] = len;
    if (len) memcpy(&buf[4], payload, len);
    buf[4 + len] = crc8(&buf[2], 2 + len);
    int total = PROTO_HEADER_LEN + len + 1;

    // Non-blocking on purpose: if the host isn't draining, dropping a frame
    // costs one sample, whereas blocking here would stall the caller.
    bool ok = false;
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        ok = usb_serial_jtag_write_bytes(buf, total, 0) == total;
        xSemaphoreGive(s_tx_mutex);
    }
    if (!ok) {
        taskENTER_CRITICAL(&s_lock);
        s_state.tx_dropped++;
        taskEXIT_CRITICAL(&s_lock);
    }
    return ok;
}

static void ack(uint8_t cmd_type, uint8_t result, uint32_t nonce)
{
    proto_ack_t a = { .cmd_type = cmd_type, .result = result, .nonce = nonce };
    pc_link_send(PROTO_T_ACK, &a, sizeof(a));
    taskENTER_CRITICAL(&s_lock);
    if (result == PROTO_ACK_OK) s_state.cmd_rx++; else s_state.cmd_bad++;
    taskEXIT_CRITICAL(&s_lock);
}

static void handle_pos(const proto_pos_t *p)
{
    taskENTER_CRITICAL(&s_lock);
    if (s_have_seq && (uint8_t)(s_state.seq + 1) != p->seq) {
        s_state.seq_gaps++;
    }
    s_have_seq = true;
    s_state.seq = p->seq;
    s_state.pos_0p1mm = p->pos_0p1mm;
    s_state.flags = p->flags;
    s_state.received_at_us = esp_timer_get_time();
    s_state.packets++;
    taskEXIT_CRITICAL(&s_lock);
}

static void handle_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    pc_cmd_t cmd;
    switch (type) {
    case PROTO_T_POS:
        if (len == sizeof(proto_pos_t)) handle_pos((const proto_pos_t *)payload);
        // no ACK: the telemetry stream carries last_seq
        return;

    case PROTO_T_SETPOINT: {
        if (len != sizeof(proto_setpoint_t)) { ack(type, PROTO_ACK_BAD_LEN, 0); return; }
        float x = ((const proto_setpoint_t *)payload)->x_0p1mm * 0.01f;
        if (x < BALL_SETPOINT_MIN_CM || x > BALL_SETPOINT_MAX_CM) { ack(type, PROTO_ACK_OUT_OF_RANGE, 0); return; }
        cmd.type = PC_CMD_SETPOINT;
        cmd.u.setpoint_cm = x;
        break;
    }
    case PROTO_T_GAINS: {
        if (len != sizeof(proto_gains_t)) { ack(type, PROTO_ACK_BAD_LEN, 0); return; }
        proto_gains_t g;
        memcpy(&g, payload, sizeof(g));
        if (!(g.kp >= 0.0f && g.kp <= PID_KP_MAX) || !(g.ki >= 0.0f && g.ki <= PID_KI_MAX) ||
            !(g.kd >= 0.0f && g.kd <= PID_KD_MAX) || !(g.d_tau_s >= 0.0f && g.d_tau_s <= 1.0f)) {
            ack(type, PROTO_ACK_OUT_OF_RANGE, 0);
            return;
        }
        cmd.type = PC_CMD_GAINS;
        cmd.u.gains.kp = g.kp;
        cmd.u.gains.ki = g.ki;
        cmd.u.gains.kd = g.kd;
        cmd.u.gains.d_tau_s = g.d_tau_s;
        break;
    }
    case PROTO_T_MODE: {
        if (len != sizeof(proto_mode_t)) { ack(type, PROTO_ACK_BAD_LEN, 0); return; }
        uint8_t m = ((const proto_mode_t *)payload)->mode;
        if (m > PROTO_MODE_OPENLOOP) { ack(type, PROTO_ACK_OUT_OF_RANGE, 0); return; }
        cmd.type = PC_CMD_MODE;
        cmd.u.mode = m;
        break;
    }
    case PROTO_T_THETA: {
        if (len != sizeof(proto_theta_t)) { ack(type, PROTO_ACK_BAD_LEN, 0); return; }
        float th = ((const proto_theta_t *)payload)->theta_0p01deg * 0.01f;
        if (th < -OPENLOOP_THETA_MAX_DEG || th > OPENLOOP_THETA_MAX_DEG) { ack(type, PROTO_ACK_OUT_OF_RANGE, 0); return; }
        cmd.type = PC_CMD_THETA;
        cmd.u.theta_deg = th;
        break;
    }
    case PROTO_T_PING: {
        if (len != sizeof(proto_ping_t)) { ack(type, PROTO_ACK_BAD_LEN, 0); return; }
        uint32_t nonce;
        memcpy(&nonce, payload, sizeof(nonce));
        ack(type, PROTO_ACK_OK, nonce);
        return;
    }
    case PROTO_T_GET_CONFIG: {
        proto_config_t c;
        control_get_config(&c);
        pc_link_send(PROTO_T_CONFIG, &c, sizeof(c));
        ack(type, PROTO_ACK_OK, 0);
        return;
    }
    default:
        ack(type, PROTO_ACK_UNKNOWN_TYPE, 0);
        return;
    }

    if (xQueueSend(s_cmds, &cmd, 0) == pdTRUE) {
        ack(type, PROTO_ACK_OK, 0);
    } else {
        ack(type, PROTO_ACK_REJECTED, 0);
    }
}

// Byte-at-a-time state machine. Any byte that doesn't fit the expected
// position throws the partial frame away and hunts for sync again.
static void feed(uint8_t b)
{
    if (s_fill == 0) {
        if (b == PROTO_SYNC0) s_frame[s_fill++] = b; else s_state.resyncs++;
        return;
    }
    if (s_fill == 1) {
        if (b == PROTO_SYNC1) {
            s_frame[s_fill++] = b;
        } else {
            s_state.resyncs++;
            s_fill = (b == PROTO_SYNC0) ? 1 : 0;
        }
        return;
    }
    if (s_fill == 3 && b > PROTO_MAX_PAYLOAD) { // impossible length
        s_state.resyncs++;
        s_fill = 0;
        return;
    }
    s_frame[s_fill++] = b;
    if (s_fill >= PROTO_HEADER_LEN) {
        int len = s_frame[3];
        if (s_fill == PROTO_HEADER_LEN + len + 1) {
            if (crc8(&s_frame[2], 2 + len) == s_frame[4 + len]) {
                handle_frame(s_frame[2], &s_frame[4], (uint8_t)len);
            } else {
                taskENTER_CRITICAL(&s_lock);
                s_state.crc_errors++;
                taskEXIT_CRITICAL(&s_lock);
            }
            s_fill = 0;
        }
    }
}

void pc_link_poll(void)
{
    uint8_t buf[64];
    int n;
    while ((n = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0)) > 0) {
        for (int i = 0; i < n; i++) {
            feed(buf[i]);
        }
    }
}

void pc_link_get(pc_link_state_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_state;
    taskEXIT_CRITICAL(&s_lock);
}

bool pc_link_is_stale(void)
{
    taskENTER_CRITICAL(&s_lock);
    int64_t at = s_state.received_at_us;
    uint32_t packets = s_state.packets;
    taskEXIT_CRITICAL(&s_lock);
    if (packets == 0) {
        return true;
    }
    return (esp_timer_get_time() - at) > PC_LINK_STALE_US;
}

bool pc_link_pop_cmd(pc_cmd_t *out)
{
    return xQueueReceive(s_cmds, out, 0) == pdTRUE;
}
