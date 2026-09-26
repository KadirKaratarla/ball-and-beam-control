"""PC -> ESP32 position link over the Faz 4 binary protocol.

Streams the ball position to the ESP32 at the camera frame rate (100 Hz)
through esp_link.EspLink (auto-connect, auto-reconnect, framed protocol in
protocol.py) and prints the board's own telemetry once a second.

Two sources for the position:

    --synthetic   a triangle wave 0..45 cm, no camera (link exercise only:
                  the crank will chase it with no ball on the beam)
    default       the calibration wizard, then the live tracker

Options:
    --stall N          after N seconds stop sending for 200 ms, then resume
                       (the ESP32 must drop to LEVEL and come back)
    --setpoint X       send a setpoint in cm once connected
    --log FILE         CSV of every packet sent (tuning)
    --skip-calibration reuse calibration.json (bench only)

Usage:
    python link.py --synthetic
    python link.py
"""

import argparse
import time

import protocol as P
from esp_link import EspLink

SEND_HZ = 100
REPORT_S = 1.0


def synthetic_positions(beam_cm, period_s=4.0):
    """Triangle wave end to end: easy to eyeball in the telemetry."""
    t0 = time.perf_counter()
    while True:
        phase = ((time.perf_counter() - t0) % period_s) / period_s
        yield beam_cm * (2 * phase if phase < 0.5 else 2 - 2 * phase), True, False


def tracker_positions(args):
    import calibrate
    import tracker as trk

    cam = calibrate.open_camera(args.id, args.fps)
    try:
        if args.skip_calibration:
            calib_dict = trk.load_calibration(args.calibration)
            trk.apply_calibration_to_camera(cam, calib_dict)
            print(f"WARNING: reusing calibration from {calib_dict['created']}")
        else:
            calib = calibrate.run(cam, args.beam_cm, args.scale, args.calibration)
            if calib is None or not calib.validation or not calib.validation.get("passed"):
                return
            calib_dict = calib.to_dict()
        tr = trk.BallTracker(calib_dict)
        while True:
            frame, _ = cam.read(timestamp=True)
            reading, _ = tr.process(frame, time.perf_counter())
            yield (reading.pos_cm if reading.valid else 0.0,
                   reading.valid,
                   tr.health.warning() is not None)
    finally:
        cam.end()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="override auto-detection of the ESP32 USB-Serial-JTAG port")
    parser.add_argument("--synthetic", action="store_true", help="triangle wave instead of the camera")
    parser.add_argument("--stall", type=float, default=0, help="seconds until a 200 ms send pause")
    parser.add_argument("--setpoint", type=float, default=None, help="setpoint in cm to send once connected")
    parser.add_argument("--beam-cm", type=float, default=45.0)
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--fps", type=int, default=100)
    parser.add_argument("--scale", type=int, default=2)
    parser.add_argument("--skip-calibration", action="store_true", help="DEV ONLY")
    parser.add_argument("--calibration", default=None)
    parser.add_argument("--log", default=None,
                        help="CSV of every packet sent: t_s, seq, pos_cm, valid, warning (tuning)")
    args = parser.parse_args()

    if args.calibration is None:
        import calibrate
        args.calibration = calibrate.CALIB_PATH

    link = EspLink(port=args.port,
                   on_event=lambda kind, port: print(f"\n--- ESP32 {kind} ({port}) ---"))
    link.start()

    source = synthetic_positions(args.beam_cm) if args.synthetic else tracker_positions(args)
    mode = "synthetic triangle wave" if args.synthetic else "camera tracker"
    print(f"sending {mode} at {SEND_HZ} Hz  (ctrl-c to stop)\n")

    log = open(args.log, "w") if args.log else None
    if log:
        log.write("t_s,seq,pos_cm,valid,warning\n")

    seq = 0
    sent = 0
    period = 1.0 / SEND_HZ
    next_send = time.perf_counter()
    next_report = next_send + REPORT_S
    start = next_send
    stalled = False
    setpoint_sent = False
    last_telem = None
    last_health = None
    telem_n = 0

    try:
        for pos_cm, valid, warning in source:
            now = time.perf_counter()

            if args.stall and not stalled and now - start >= args.stall:
                print("\n--- stalling for 200 ms (the ESP32 must drop to LEVEL) ---")
                time.sleep(0.2)
                stalled = True
                next_send = time.perf_counter()

            # In synthetic mode the loop paces itself; with the camera the
            # frame rate paces it and this just never sleeps.
            if args.synthetic:
                sleep = next_send - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                next_send += period

            t_send = time.perf_counter()
            link.send_position(seq, pos_cm, valid, warning)
            if log:
                log.write(f"{t_send:.4f},{seq},{pos_cm:.3f},{int(valid)},{int(warning)}\n")
            seq = (seq + 1) & 0xFF
            sent += 1

            if args.setpoint is not None and link.connected and not setpoint_sent:
                link.set_setpoint(args.setpoint)
                setpoint_sent = True

            for ptype, msg in link.drain():
                if ptype == P.T_TELEM:
                    last_telem = msg
                    telem_n += 1
                elif ptype == P.T_HEALTH:
                    last_health = msg
                elif ptype == P.T_ACK and msg.cmd_type != P.T_PING and msg.result != P.ACK_OK:
                    print(f"\nACK for 0x{msg.cmd_type:02X}: {P.ACK_NAMES.get(msg.result, msg.result)}")

            if t_send >= next_report:
                if link.connected and last_telem:
                    t = last_telem
                    rtt = link.stats["rtt_ms"]
                    print(f"pos {pos_cm:6.2f} cm  sent {sent:6d}  | ESP {P.STATE_NAMES.get(t.state, t.state):6s}"
                          f"{' ' + P.FAULT_NAMES.get(t.fault, str(t.fault)) if t.fault else ''}"
                          f"  x {t.x_0p1mm / 100:6.2f} set {t.x_set_0p1mm / 100:5.1f}  theta {t.theta / 100:+5.2f}"
                          f"  enc {t.enc_counts:5d}  loop {t.loop_us:3d} us  telem {telem_n:4d}/s"
                          f"  rtt {rtt:.1f} ms" if rtt is not None else "")
                    if last_health and (last_health.overruns or last_health.link_crc_errors or last_health.enc_i2c_errors):
                        h = last_health
                        print(f"   health: overruns {h.overruns} link crc {h.link_crc_errors} gaps {h.link_seq_gaps} "
                              f"enc i2c {h.enc_i2c_errors} rej {h.enc_rejects}")
                else:
                    print(f"pos {pos_cm:6.2f} cm  sent {sent:6d}  | ESP not connected (dropped {link.stats['tx_dropped']})")
                telem_n = 0
                next_report += REPORT_S
                link.ping()
    except KeyboardInterrupt:
        pass
    finally:
        if log:
            log.close()
        link.stop()
        print(f"\nsent {sent}, tx dropped {link.stats['tx_dropped']}, reconnects {link.stats['reconnects']}")


if __name__ == "__main__":
    main()
