#pragma once

// Faz 3.1 -- PID + linkage checked on the target against a simulated
// plant (no motor, no camera, nothing connected). Ball: x'' = (5/7) g
// sin(theta) with the real crank kinematics, a rate/acceleration-limited
// crank, a 100 Hz camera with 20 ms delay and 0.3 mm noise. Prints step
// response metrics, an anti-windup check, the linkage table and a CSV
// trace. Returns when done.
void test_pid_sim(void);
