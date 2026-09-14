#pragma once

#include <stdint.h>
#include <stdbool.h>

#define TMC_REG_GCONF       0x00
#define TMC_REG_GSTAT       0x01
#define TMC_REG_IFCNT       0x02
#define TMC_REG_IOIN        0x06
#define TMC_REG_IHOLD_IRUN  0x10
#define TMC_REG_TPOWERDOWN  0x11
#define TMC_REG_TSTEP       0x12
#define TMC_REG_CHOPCONF    0x6C
#define TMC_REG_DRV_STATUS  0x6F

// Half-duplex UART: tx_gpio drives PDN_UART through a 1k series resistor,
// rx_gpio taps the same node directly (no resistor) so the driver can pull
// the line low against our idling push-pull output.
void tmc2208_uart_init(int tx_gpio, int rx_gpio, int baud_rate);

bool tmc2208_write_register(uint8_t reg, uint32_t value);
bool tmc2208_read_register(uint8_t reg, uint32_t *out_value);
