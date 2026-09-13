#pragma once

// Faz 1.4 -- HC-SR04 distance measurement without blocking the caller.
// The trigger pulse costs ~12 us; the echo is timed by a GPIO interrupt and
// the result is picked up later, which is the pattern the 2 ms control loop
// needs (see K-010 in docs/decision_log.md). Does not return.
void test_hcsr04(void);
