#include "tmc2208.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#define TMC_UART_PORT UART_NUM_1
#define TMC_SYNC_BYTE 0x05
#define TMC_SLAVE_ADDR 0x00
#define TMC_MASTER_REPLY_ADDR 0xFF

static uint8_t tmc_crc8(const uint8_t *datagram, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t current_byte = datagram[i];
        for (int j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (current_byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            current_byte >>= 1;
        }
    }
    return crc;
}

void tmc2208_uart_init(int tx_gpio, int rx_gpio, int baud_rate)
{
    uart_config_t cfg = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(TMC_UART_PORT, 256, 0, 0, NULL, 0);
    uart_param_config(TMC_UART_PORT, &cfg);
    // Both pins land on the same PDN_UART node, so our own transmitted bytes
    // still echo back on rx and have to be drained before the reply.
    uart_set_pin(TMC_UART_PORT, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

bool tmc2208_write_register(uint8_t reg, uint32_t value)
{
    uint8_t dg[8];
    dg[0] = TMC_SYNC_BYTE;
    dg[1] = TMC_SLAVE_ADDR;
    dg[2] = reg | 0x80; // write access
    dg[3] = (value >> 24) & 0xFF;
    dg[4] = (value >> 16) & 0xFF;
    dg[5] = (value >> 8) & 0xFF;
    dg[6] = value & 0xFF;
    dg[7] = tmc_crc8(dg, 7);

    uart_flush_input(TMC_UART_PORT);
    uart_write_bytes(TMC_UART_PORT, (const char *)dg, sizeof(dg));
    uart_wait_tx_done(TMC_UART_PORT, pdMS_TO_TICKS(20));

    // Drain our own echo (write access gets no reply from the driver).
    uint8_t echo[8];
    uart_read_bytes(TMC_UART_PORT, echo, sizeof(echo), pdMS_TO_TICKS(20));
    return true;
}

bool tmc2208_read_register(uint8_t reg, uint32_t *out_value)
{
    uint8_t dg[4];
    dg[0] = TMC_SYNC_BYTE;
    dg[1] = TMC_SLAVE_ADDR;
    dg[2] = reg & 0x7F; // read access
    dg[3] = tmc_crc8(dg, 3);

    uart_flush_input(TMC_UART_PORT);
    uart_write_bytes(TMC_UART_PORT, (const char *)dg, sizeof(dg));
    uart_wait_tx_done(TMC_UART_PORT, pdMS_TO_TICKS(20));

    // Discard our own echoed request bytes first.
    uint8_t echo[4];
    uart_read_bytes(TMC_UART_PORT, echo, sizeof(echo), pdMS_TO_TICKS(20));

    // Then the driver's reply, sent after its SENDDELAY.
    uint8_t reply[8];
    int got = uart_read_bytes(TMC_UART_PORT, reply, sizeof(reply), pdMS_TO_TICKS(20));
    if (got != sizeof(reply)) {
        return false;
    }
    if (reply[0] != TMC_SYNC_BYTE || reply[1] != TMC_MASTER_REPLY_ADDR) {
        return false;
    }
    if (tmc_crc8(reply, 7) != reply[7]) {
        return false;
    }

    *out_value = ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16) |
                 ((uint32_t)reply[5] << 8) | (uint32_t)reply[6];
    return true;
}
