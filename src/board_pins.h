#pragma once

// Verified pin map -- see docs/decision_log.md ("Doğrulanmış pin haritası").
// Keep this file as the single source of truth so the isolated Faz 1 tests
// can't drift apart from each other.

#include "driver/gpio.h"

// --- TMC2208 stepper driver ---
#define PIN_TMC_STEP    GPIO_NUM_4
#define PIN_TMC_DIR     GPIO_NUM_5
#define PIN_TMC_UART_TX GPIO_NUM_6  // through 1k series resistor to PDN_UART
#define PIN_TMC_UART_RX GPIO_NUM_7  // straight to the PDN_UART pin, no resistor
#define PIN_TMC_EN      GPIO_NUM_15 // LOW = driver enabled

#define TMC_UART_BAUD 115200
