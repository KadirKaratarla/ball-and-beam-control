#pragma once

// Motor characterisation with the shaft free (rod detached from the crank):
// verifies the driver really is at 1/256, reads the open-load / short /
// overtemperature flags, then turns exactly one revolution each way while
// the AS5600 counts. One revolution must read 4096 counts. Does not return.
void test_motor_free(void);
