#pragma once

// Faz 1.1 -- does a hardware timer give us a stable control-loop tick?
// GPTimer fires every CONTROL_LOOP_PERIOD_US; the ISR only timestamps,
// toggles PIN_LOOP_PROBE and wakes a task. Reports period jitter (software
// measured, histogrammed) and ISR-to-task latency once a second. Acceptance
// from the plan: jitter within +-50 us. Does not return.
void test_timer_jitter(void);
