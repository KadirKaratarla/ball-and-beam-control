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
// DIR is NOT strapped on this breakout despite an early reading suggesting
// it was: left floating it picks up I2C line activity and the chip flips
// polarity between reads, reporting the same angle alternately as x and
// 4096-x. It needs its own wire to GND. OUT and GPO stay unconnected.
#define PIN_AS5600_SDA GPIO_NUM_8
#define PIN_AS5600_SCL GPIO_NUM_9

#define AS5600_I2C_ADDR 0x36

// --- HC-SR04 ultrasonic range finder ---
// Runs on 5V, so ECHO is a 5V output and reaches this pin through a
// 1k/2k divider (5V -> 3.33V). TRIG takes our 3.3V drive directly.
#define PIN_HCSR04_TRIG GPIO_NUM_10
#define PIN_HCSR04_ECHO GPIO_NUM_11

// --- Control loop timing probe ---
// Toggled in the loop timer ISR so a logic analyser can read the period
// and jitter independently of the software measurement.
#define PIN_LOOP_PROBE GPIO_NUM_16

// --- Control loop period ---
// 4 ms (250 Hz). Started at 2 ms in the plan; relaxed to 4 ms so every
// per-cycle budget (encoder read, PID, step update) halves in relative cost.
// The camera delivers position at 100 Hz, so each update still gets 2.5
// loop cycles.
#define CONTROL_LOOP_PERIOD_US 4000
