#pragma once

// Faz D -- exercise the PC position link: receive packets over
// USB-Serial-JTAG, echo them for round-trip timing, and report packet /
// CRC / sequence-gap counters and the staleness flag once a second on the
// UART console. Does not return.
void test_pc_link(void);
