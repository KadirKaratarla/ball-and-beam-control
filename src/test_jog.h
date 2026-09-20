#pragma once

// Faz 3.3 -- interactive jog over the console (PlatformIO monitor, COM11).
// Finds the encoder count at which the beam is level (ENC_LEVEL_COUNTS),
// lets the user check the linkage table against a phone inclinometer,
// and sweeps the crank speed up to STEP_VMAX_CLOSED_LOOP while watching
// the following error. Keys are listed on start. Does not return.
void test_jog(void);
