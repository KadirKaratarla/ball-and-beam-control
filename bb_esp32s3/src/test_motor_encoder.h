#pragma once

// Faz 3.3 / 3.4 first data: drive the stepper in known increments while
// reading the AS5600 on the same shaft. Establishes the microstep-to-count
// ratio, which DIR level moves the crank toward 12 o'clock, and whether
// steps are lost holding the beam near horizontal. Does not return.
void test_motor_encoder(void);
