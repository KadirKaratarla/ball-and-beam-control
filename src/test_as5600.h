#pragma once

// Faz 1.2 -- AS5600 magnetic encoder: bus scan, magnet mounting check,
// read-time budget measurement, then a continuous angle readout so the
// shaft can be turned by hand to check linearity. Does not return.
void test_as5600(void);
