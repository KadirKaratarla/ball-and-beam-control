"""PC -> ESP32 position link -- Faz D.

Sends the ball position to the ESP32 over its built-in USB-Serial-JTAG port
at 100 Hz and times the echo the firmware returns for every packet, which
is how the round-trip latency of the whole link is measured.

Packet (7 bytes, matches bb_esp32s3/src/pc_link.h):

    AA 55 | seq u8 | pos i16 LE in 0.1 mm | flags u8 | crc8

Echo from the ESP32 (4 bytes): AA 55 | seq | crc8

Two sources for the position:

    --synthetic   a triangle wave 0..45 cm, no camera. Exercises the link
                  on its own (D1 counters, D2 latency, D3 stall watchdog).
    default       the calibration wizard, then the live tracker (D4).

--stall N        after N seconds, stop sending for 200 ms then resume, so
                 the ESP32's staleness watchdog can be seen to fire and
                 clear on its console.

Usage:
    python link.py --synthetic
    python link.py --synthetic --stall 5
    python link.py
"""

import argparse
import struct
import threading
import time
from collections import deque

import numpy as np
import serial
from serial.tools import list_ports

SYNC = b"\xAA\x55"
PACKET_LEN = 7
ECHO_LEN = 4

FLAG_VALID = 0x01
FLAG_WARNING = 0x02

# Espressif's USB-Serial-JTAG controller enumerates with this pair on every
# ESP32-S3; the CH343 bridge that carries the console is a different VID.
ESP_USB_VID = 0x303A
ESP_USB_PID = 0x1001

SEND_HZ = 100
REPORT_S = 1.0


def crc8(data):
    """CRC8-ATM, bit order as in the TMC2208 datasheet -- shared with pc_link.c."""
    crc = 0
    for byte in data:
        for _ in range(8):
            if (crc >> 7) ^ (byte & 1):
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            byte >>= 1
    return crc


def build_packet(seq, pos_cm, valid, warning):
    pos_0p1mm = int(round(pos_cm * 100)) if valid else 0
    pos_0p1mm = max(-32768, min(32767, pos_0p1mm))
    flags = (FLAG_VALID if valid else 0) | (FLAG_WARNING if warning else 0)
    body = SYNC + struct.pack("<BhB", seq & 0xFF, pos_0p1mm, flags)
    return body + bytes([crc8(body)])


def find_esp_port():
    for p in list_ports.comports():
        if p.vid == ESP_USB_VID and p.pid == ESP_USB_PID:
            return p.device
    return None


class EchoReader(threading.Thread):
    """Pulls echoes off the port and turns them into round-trip times."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.pending = {}          # seq -> send time
        self.rtts = deque(maxlen=2000)
        self.echoes = 0
        self.crc_errors = 0
        self.lock = threading.Lock()
        self.running = True

    def sent(self, seq, t):
        with self.lock:
            self.pending[seq] = t

    def run(self):
        buf = bytearray()
        while self.running:
            # Block for the first byte only, then sweep up whatever else is
            # already waiting. read(n) with a timeout returns on the timeout
            # when fewer than n bytes arrive, which for 4-byte echoes every
            # 10 ms meant every call sat out the full timeout and the RTT
            # figure was mostly that wait.
            chunk = self.ser.read(1)
            if not chunk:
                continue
            waiting = self.ser.in_waiting
            if waiting:
                chunk += self.ser.read(waiting)
            buf += chunk
            while True:
                i = buf.find(SYNC)
                if i < 0:
                    buf.clear()
                    break
                if len(buf) - i < ECHO_LEN:
                    del buf[:i]
                    break
                frame = bytes(buf[i:i + ECHO_LEN])
                del buf[:i + ECHO_LEN]
                if crc8(frame[:3]) != frame[3]:
                    self.crc_errors += 1
                    continue
                seq = frame[2]
                now = time.perf_counter()
                with self.lock:
                    t_sent = self.pending.pop(seq, None)
                    self.echoes += 1
                    if t_sent is not None:
                        self.rtts.append((now - t_sent) * 1000.0)

    def report(self, sent):
        with self.lock:
            rtts = np.array(self.rtts) if self.rtts else None
            lost = len(self.pending)
        line = f"sent {sent:6d}  echoed {self.echoes:6d}  unanswered {lost:4d}  echo_crc_err {self.crc_errors}"
        if rtts is not None and rtts.size:
            line += (f"  | RTT ms  p50 {np.percentile(rtts, 50):5.2f}  p95 {np.percentile(rtts, 95):5.2f}"
                     f"  p99 {np.percentile(rtts, 99):5.2f}  max {rtts.max():5.2f}")
        return line


def synthetic_positions(beam_cm, period_s=4.0):
    """Triangle wave end to end: easy to eyeball on the ESP32 console."""
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
    parser.add_argument("--stall", type=float, default=0, help="seconds until a 200 ms send pause (D3)")
    parser.add_argument("--beam-cm", type=float, default=45.0)
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--fps", type=int, default=100)
    parser.add_argument("--scale", type=int, default=2)
    parser.add_argument("--skip-calibration", action="store_true", help="DEV ONLY")
    parser.add_argument("--calibration", default=None)
    args = parser.parse_args()

    if args.calibration is None:
        import calibrate
        args.calibration = calibrate.CALIB_PATH

    port = args.port or find_esp_port()
    if not port:
        print(f"no ESP32 USB-Serial-JTAG port found (VID {ESP_USB_VID:04X} PID {ESP_USB_PID:04X}) "
              "-- is the board's USB (not UART) connector plugged in?")
        return
    print(f"ESP32 link on {port}")

    # DTR/RTS left low so opening the port can't be mistaken by the ROM for
    # the reset-into-bootloader sequence esptool uses.
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200  # ignored by USB CDC, but pyserial wants one
    ser.timeout = 0.02
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.reset_input_buffer()

    reader = EchoReader(ser)
    reader.start()

    source = synthetic_positions(args.beam_cm) if args.synthetic else tracker_positions(args)
    mode = "synthetic triangle wave" if args.synthetic else "camera tracker"
    print(f"sending {mode} at {SEND_HZ} Hz  (ctrl-c to stop)\n")

    seq = 0
    sent = 0
    period = 1.0 / SEND_HZ
    next_send = time.perf_counter()
    next_report = next_send + REPORT_S
    start = next_send
    stalled = False

    try:
        for pos_cm, valid, warning in source:
            now = time.perf_counter()

            if args.stall and not stalled and now - start >= args.stall:
                print(f"\n--- stalling for 200 ms (watch the ESP32 console for STALE) ---")
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

            pkt = build_packet(seq, pos_cm, valid, warning)
            t_send = time.perf_counter()
            ser.write(pkt)
            reader.sent(seq, t_send)
            seq = (seq + 1) & 0xFF
            sent += 1

            if t_send >= next_report:
                print(f"pos {pos_cm:6.2f} cm  " + reader.report(sent))
                next_report += REPORT_S
    except KeyboardInterrupt:
        pass
    finally:
        reader.running = False
        ser.close()
        print("\n" + reader.report(sent))


if __name__ == "__main__":
    main()
