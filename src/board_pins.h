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

// --- AS5600 magnetic encoder (I2C) ---
// DIR is strapped to GND on the board; OUT and GPO are left unconnected.
#define PIN_AS5600_SDA GPIO_NUM_8
#define PIN_AS5600_SCL GPIO_NUM_9

#define AS5600_I2C_ADDR 0x36

// --- HC-SR04 ultrasonic range finder ---
// Runs on 5V, so ECHO is a 5V output and reaches this pin through a
// 1k/2k divider (5V -> 3.33V). TRIG takes our 3.3V drive directly.
#define PIN_HCSR04_TRIG GPIO_NUM_10
#define PIN_HCSR04_ECHO GPIO_NUM_11
