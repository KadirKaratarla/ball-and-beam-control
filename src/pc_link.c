#include "pc_link.h"

#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define RX_BUF 512
#define TX_BUF 256

static pc_link_state_t s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Assembly buffer for the packet currently being received.
static uint8_t s_pkt[PC_LINK_PACKET_LEN];
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
}

static void send_echo(uint8_t seq)
{
    uint8_t echo[PC_LINK_ECHO_LEN] = { PC_LINK_SYNC0, PC_LINK_SYNC1, seq, 0 };
    echo[3] = crc8(echo, 3);
    // Non-blocking on purpose: if the host isn't draining, dropping an echo
    // costs one RTT sample, whereas blocking here would stall the caller.
    usb_serial_jtag_write_bytes(echo, sizeof(echo), 0);
}

static void accept_packet(void)
{
    if (crc8(s_pkt, PC_LINK_PACKET_LEN - 1) != s_pkt[PC_LINK_PACKET_LEN - 1]) {
        taskENTER_CRITICAL(&s_lock);
        s_state.crc_errors++;
        taskEXIT_CRITICAL(&s_lock);
        return;
    }

    uint8_t seq = s_pkt[2];
    int16_t pos = (int16_t)(s_pkt[3] | (s_pkt[4] << 8));
    uint8_t flags = s_pkt[5];

    taskENTER_CRITICAL(&s_lock);
    if (s_have_seq && (uint8_t)(s_state.seq + 1) != seq) {
        s_state.seq_gaps++;
    }
    s_have_seq = true;
    s_state.seq = seq;
    s_state.pos_0p1mm = pos;
    s_state.flags = flags;
    s_state.received_at_us = esp_timer_get_time();
    s_state.packets++;
    taskEXIT_CRITICAL(&s_lock);

    send_echo(seq);
}

// Byte-at-a-time state machine. Sync is two bytes so a stray 0xAA inside a
// payload can't restart a packet on its own; any byte that doesn't fit the
// expected position throws the partial packet away and hunts again.
static void feed(uint8_t b)
{
    if (s_fill == 0) {
        if (b == PC_LINK_SYNC0) {
            s_pkt[s_fill++] = b;
        } else {
            s_state.resyncs++;
        }
        return;
    }
    if (s_fill == 1) {
        if (b == PC_LINK_SYNC1) {
            s_pkt[s_fill++] = b;
        } else {
            s_state.resyncs++;
            s_fill = (b == PC_LINK_SYNC0) ? 1 : 0;
        }
        return;
    }
    s_pkt[s_fill++] = b;
    if (s_fill == PC_LINK_PACKET_LEN) {
        accept_packet();
        s_fill = 0;
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
