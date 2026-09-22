"""ESP32 link: auto-connect, reconnect, framed protocol (Faz 4.4).

    link = EspLink()             # finds the USB-Serial-JTAG port by VID/PID
    link.start()                 # background reader + reconnect loop
    link.send_position(seq, cm, valid)
    link.set_setpoint(30.0)      # -> ACK
    for ptype, msg in link.drain(): ...   # Telem / Health / Ack / Config

Reconnect: any I/O error closes the port; the thread then rescans every
RECONNECT_S until the ESP re-enumerates (it does on every reset), pings
it, and requests CONFIG. Callers keep calling send_*(); while
disconnected the frames are dropped and counted.
"""

import queue
import struct
import threading
import time

import serial
from serial.tools import list_ports

import protocol as P

# Windows timer resolution defaults to ~15.6 ms, which makes every sleep()
# and serial timeout coarse; 1 ms keeps RTT and send pacing honest.
try:
    import ctypes
    ctypes.windll.winmm.timeBeginPeriod(1)
except Exception:
    pass

ESP_USB_VID = 0x303A
ESP_USB_PID = 0x1001
RECONNECT_S = 0.5
PING_TIMEOUT_S = 1.0


def find_esp_port():
    for p in list_ports.comports():
        if p.vid == ESP_USB_VID and p.pid == ESP_USB_PID:
            return p.device
    return None


class EspLink:
    def __init__(self, port=None, on_event=None):
        self.port_override = port
        self.on_event = on_event or (lambda *a: None)  # ("connected"|"disconnected", detail)
        self.ser = None
        self.connected = False
        self.port = None
        self.rx = queue.Queue(maxsize=10000)
        self.parser = P.Parser()
        self.stats = dict(tx_frames=0, tx_dropped=0, rx_frames=0, rx_bytes=0, reconnects=0,
                          rtt_ms=None, connected_at=None)
        self.config = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._pending_pings = {}

    # --- lifecycle ---------------------------------------------------------
    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self._close()

    def _open(self):
        port = self.port_override or find_esp_port()
        if not port:
            return False
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = 115200  # ignored by USB CDC
            ser.timeout = 0.05
            ser.write_timeout = 0.2
            # DTR/RTS low so opening can't look like esptool's reset sequence
            ser.dtr = False
            ser.rts = False
            ser.open()
            ser.reset_input_buffer()
        except (serial.SerialException, OSError):
            return False
        with self._lock:
            self.ser = ser
            self.port = port
            self.parser = P.Parser()
        return True

    def _close(self):
        with self._lock:
            ser, self.ser = self.ser, None
            was = self.connected
            self.connected = False
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        if was:
            self.on_event("disconnected", self.port)

    def _run(self):
        while self._running:
            if self.ser is None:
                if not self._open():
                    time.sleep(RECONNECT_S)
                    continue
                # handshake: ping, then config
                if not self._handshake():
                    self._close()
                    time.sleep(RECONNECT_S)
                    continue
                with self._lock:
                    self.connected = True
                self.stats["connected_at"] = time.time()
                self.stats["reconnects"] += 1
                self.on_event("connected", self.port)
            # One byte blocks (up to the timeout), then whatever else has
            # arrived: frames are handed over the moment they land instead of
            # at the end of a 20 ms read window.
            try:
                data = self.ser.read(1)
                if data:
                    n = self.ser.in_waiting
                    if n:
                        data += self.ser.read(n)
            except (serial.SerialException, OSError, AttributeError):
                self._close()
                continue
            if data:
                self._on_bytes(data)

    def _handshake(self):
        nonce = int(time.time() * 1000) & 0xFFFFFFFF
        t0 = time.perf_counter()
        if not self._write(P.pack_ping(nonce)):
            return False
        deadline = t0 + PING_TIMEOUT_S
        got_pong = False
        while time.perf_counter() < deadline:
            try:
                data = self.ser.read(256)
            except (serial.SerialException, OSError):
                return False
            if not data:
                continue
            for ptype, payload in self.parser.feed(data):
                msg = self._decode(ptype, payload)
                if ptype == P.T_ACK and msg.cmd_type == P.T_PING and msg.nonce == nonce:
                    self.stats["rtt_ms"] = (time.perf_counter() - t0) * 1000
                    got_pong = True
                else:
                    self._enqueue(ptype, msg)
            if got_pong:
                break
        if not got_pong:
            return False
        self._write(P.pack_get_config())
        return True

    # --- rx ----------------------------------------------------------------
    def _decode(self, ptype, payload):
        try:
            return P.decode(ptype, payload)
        except ValueError:
            return payload

    def _enqueue(self, ptype, msg):
        self.stats["rx_frames"] += 1
        if ptype == P.T_CONFIG:
            self.config = msg
        if ptype == P.T_ACK and msg.cmd_type == P.T_PING:
            t0 = self._pending_pings.pop(msg.nonce, None)
            if t0 is not None:
                self.stats["rtt_ms"] = (time.perf_counter() - t0) * 1000
        try:
            self.rx.put_nowait((ptype, msg))
        except queue.Full:
            pass

    def _on_bytes(self, data):
        self.stats["rx_bytes"] += len(data)
        for ptype, payload in self.parser.feed(data):
            self._enqueue(ptype, self._decode(ptype, payload))

    def drain(self):
        """All frames received since the last call, as (type, message)."""
        out = []
        while True:
            try:
                out.append(self.rx.get_nowait())
            except queue.Empty:
                return out

    # --- tx ----------------------------------------------------------------
    def _write(self, frame):
        with self._lock:
            ser = self.ser
        if ser is None:
            self.stats["tx_dropped"] += 1
            return False
        try:
            ser.write(frame)
            self.stats["tx_frames"] += 1
            return True
        except (serial.SerialException, OSError):
            self.stats["tx_dropped"] += 1
            self._close()
            return False

    def send_position(self, seq, pos_cm, valid, warning=False):
        return self._write(P.pack_pos(seq, pos_cm, valid, warning))

    def set_setpoint(self, x_cm):
        return self._write(P.pack_setpoint(x_cm))

    def set_gains(self, kp, ki, kd, d_tau_s):
        return self._write(P.pack_gains(kp, ki, kd, d_tau_s))

    def set_mode(self, mode):
        return self._write(P.pack_mode(mode))

    def set_theta(self, theta_deg):
        """Open-loop beam angle (MODE_OPENLOOP); the firmware levels the
        beam if these stop arriving."""
        return self._write(P.pack_theta(theta_deg))

    def ping(self):
        nonce = int(time.perf_counter() * 1e6) & 0xFFFFFFFF
        self._pending_pings[nonce] = time.perf_counter()
        # keep the dict bounded if pongs never come
        if len(self._pending_pings) > 64:
            oldest = min(self._pending_pings, key=self._pending_pings.get)
            self._pending_pings.pop(oldest, None)
        return self._write(P.pack_ping(nonce))

    def request_config(self):
        return self._write(P.pack_get_config())
