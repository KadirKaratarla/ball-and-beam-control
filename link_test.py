"""Faz 4.5 loopback test of the ESP32 protocol -- no camera needed.

Sends a synthetic position at 100 Hz, receives TELEM/HEALTH, exercises
every command with ACK checks, measures PING RTT, telemetry period jitter
and seq loss, then (optionally) pulls DTR to reset the ESP and times the
automatic reconnect.

    python link_test.py [--seconds 20] [--reset]
"""

import argparse
import statistics
import time

import serial

import protocol as P
from esp_link import EspLink, find_esp_port


def pct(v, q):
    if not v:
        return float("nan")
    s = sorted(v)
    return s[min(len(s) - 1, int(q * len(s)))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--reset", metavar="UART_PORT", default=None,
                    help="reset the ESP through its UART port's RTS->EN circuit (e.g. COM11) and time the reconnect")
    args = ap.parse_args()

    events = []
    link = EspLink(on_event=lambda kind, port: events.append((time.perf_counter(), kind, port)))
    link.start()

    t0 = time.perf_counter()
    while not link.connected and time.perf_counter() - t0 < 10:
        time.sleep(0.05)
    if not link.connected:
        print("no ESP32 found / no PONG")
        return
    print(f"connected on {link.port} in {(time.perf_counter() - t0) * 1000:.0f} ms, handshake RTT {link.stats['rtt_ms']:.2f} ms")

    # wait for CONFIG
    t1 = time.perf_counter()
    while link.config is None and time.perf_counter() - t1 < 2:
        link.drain()
        time.sleep(0.02)
    cfg = link.config
    if cfg:
        print(f"CONFIG v{cfg.version}: Kp {cfg.kp:.2f} Ki {cfg.ki:.2f} Kd {cfg.kd:.2f} d_tau {cfg.d_tau_s:.2f} "
              f"theta_max {cfg.theta_max_deg:.1f} setpoint {cfg.x_set_0p1mm / 100:.1f} cm vmax {cfg.vmax:.0f} "
              f"level {cfg.level_counts} IRUN {cfg.irun} IHOLD {cfg.ihold} defaults={cfg.defaults}")
    else:
        print("no CONFIG received")

    # --- command ACK checks ---
    def expect_ack(name, send, want=P.ACK_OK, timeout=0.5, cmd_type=None):
        link.drain()
        send()
        t = time.perf_counter()
        while time.perf_counter() - t < timeout:
            for ptype, msg in link.drain():
                if ptype == P.T_ACK and msg.cmd_type != P.T_PING and (cmd_type is None or msg.cmd_type == cmd_type):
                    ok = msg.result == want
                    print(f"  {name:32s} ACK {P.ACK_NAMES.get(msg.result, msg.result):13s} {'OK' if ok else 'UNEXPECTED'}")
                    return ok
            time.sleep(0.005)
        print(f"  {name:32s} NO ACK")
        return False

    # first TELEM tells the board's state
    t2 = time.perf_counter()
    first = None
    while first is None and time.perf_counter() - t2 < 1.0:
        for ptype, msg in link.drain():
            if ptype == P.T_TELEM:
                first = msg
                break
        time.sleep(0.01)
    if first:
        print(f"board state {P.STATE_NAMES.get(first.state, first.state)} fault {P.FAULT_NAMES.get(first.fault, first.fault)}")
        if first.state == 4 and first.fault == 6:
            print("  beam is not resting near 6: lower it by hand, then RESET_FAULT is sent below")

    print("commands:")
    expect_ack("SETPOINT 22.5", lambda: link.set_setpoint(22.5), cmd_type=P.T_SETPOINT)
    expect_ack("SETPOINT 99 (out of range)", lambda: link.set_setpoint(99), P.ACK_OUT_OF_RANGE, cmd_type=P.T_SETPOINT)
    expect_ack("GAINS same as default", lambda: link.set_gains(cfg.kp, cfg.ki, cfg.kd, cfg.d_tau_s) if cfg else link.set_gains(0.74, 0.15, 0.34, 0.15), cmd_type=P.T_GAINS)
    expect_ack("GAINS Kp=50 (out of range)", lambda: link.set_gains(50, 0, 0, 0.1), P.ACK_OUT_OF_RANGE, cmd_type=P.T_GAINS)
    expect_ack("MODE LEVEL", lambda: link.set_mode(P.MODE_LEVEL), cmd_type=P.T_MODE)
    expect_ack("MODE RUN", lambda: link.set_mode(P.MODE_RUN), cmd_type=P.T_MODE)
    expect_ack("MODE RESET_FAULT", lambda: link.set_mode(P.MODE_RESET_FAULT), cmd_type=P.T_MODE)
    expect_ack("GET_CONFIG", lambda: link.request_config(), cmd_type=P.T_GET_CONFIG)
    link.drain()

    # --- streaming ---
    print(f"streaming synthetic position for {args.seconds:.0f} s ...")
    seq = 0
    n_sent = 0
    telem_t = []
    telem_seq_seen = []
    health = []
    rtts = []
    last_ping = 0
    t_start = time.perf_counter()
    next_send = t_start
    last_state = None
    while time.perf_counter() - t_start < args.seconds:
        now = time.perf_counter()
        if now >= next_send:
            phase = ((now - t_start) % 8.0) / 8.0
            pos = 45.0 * (2 * phase if phase < 0.5 else 2 - 2 * phase)
            link.send_position(seq, pos, True)
            seq = (seq + 1) & 0xFF
            n_sent += 1
            next_send += 0.01
        if now - last_ping > 0.5:
            link.ping()
            last_ping = now
        for ptype, msg in link.drain():
            if ptype == P.T_TELEM:
                telem_t.append(now)
                telem_seq_seen.append(msg.last_seq)
                if msg.state != last_state:
                    print(f"  t={now - t_start:5.1f}s state {P.STATE_NAMES.get(msg.state, msg.state)}"
                          f"{' fault=' + P.FAULT_NAMES.get(msg.fault, str(msg.fault)) if msg.fault else ''}")
                    last_state = msg.state
            elif ptype == P.T_HEALTH:
                health.append(msg)
            elif ptype == P.T_ACK and msg.cmd_type == P.T_PING:
                if link.stats["rtt_ms"] is not None:
                    rtts.append(link.stats["rtt_ms"])
        time.sleep(0.001)

    dur = time.perf_counter() - t_start
    print(f"sent {n_sent} POS in {dur:.1f} s; received {len(telem_t)} TELEM ({len(telem_t) / dur:.1f} Hz), {len(health)} HEALTH")
    if len(telem_t) > 2:
        d = [(b - a) * 1000 for a, b in zip(telem_t, telem_t[1:])]
        print(f"  TELEM period ms: mean {statistics.mean(d):.2f} p50 {pct(d, .5):.2f} p99 {pct(d, .99):.2f} max {max(d):.2f}")
    if rtts:
        print(f"  PING RTT ms: p50 {pct(rtts, .5):.2f} p95 {pct(rtts, .95):.2f} max {max(rtts):.2f}  (n={len(rtts)})")
    if health:
        h = health[-1]
        print(f"  ESP HEALTH: loop {h.loop_us_mean}/{h.loop_us_max} us, overruns {h.overruns}, missed {h.missed}, enc i2c {h.enc_i2c_errors} rej {h.enc_rejects}, "
              f"link pkt {h.link_packets} crc {h.link_crc_errors} gaps {h.link_seq_gaps}, cmd ok {h.cmd_rx} bad {h.cmd_bad}, "
              f"tx dropped {h.tx_dropped}, drv 0x{h.drv_status:08X}, state {P.STATE_NAMES.get(h.state, h.state)}")
    print(f"  PC parser: crc errors {link.parser.crc_errors}, resync bytes {link.parser.resyncs}, "
          f"tx dropped {link.stats['tx_dropped']}")

    if args.reset:
        print(f"resetting the ESP through {args.reset} (RTS -> EN) ...")
        link.stop()
        t_reset = time.perf_counter()
        try:
            s = serial.Serial()
            s.port = args.reset
            s.dtr = False
            s.rts = False
            s.open()
            s.rts = True   # EN low
            time.sleep(0.1)
            s.rts = False  # EN released
            s.close()
        except Exception as e:
            print(f"  could not pulse EN: {e}")
        link2 = EspLink(on_event=lambda kind, port: events.append((time.perf_counter(), kind, port)))
        link2.start()
        while not link2.connected and time.perf_counter() - t_reset < 15:
            time.sleep(0.05)
        if link2.connected:
            print(f"  reconnected on {link2.port} {(time.perf_counter() - t_reset):.2f} s after reset")
        else:
            print("  reconnect FAILED within 15 s")
        link2.stop()
    else:
        link.stop()


if __name__ == "__main__":
    main()
