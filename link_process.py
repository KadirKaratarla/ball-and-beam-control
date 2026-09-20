"""Camera + ESP link in their own process; the GUI talks to it via queues.

pseyepy's frame read blocks inside C++ while holding the GIL, so anything
sharing the interpreter with the camera loop starves (a Qt GUI freezes and
positions get delayed past the ESP's 50 ms staleness limit). A separate
process has its own GIL: the loop here runs exactly as link.py does, and
the GUI process only ever touches queues.

Child -> GUI (out_q):  ("status", text) | ("event", kind, port)
                       ("batch", pos_tuple_or_None, [(ptype, msg), ...], stats_dict, frames)
GUI -> child (cmd_q):  ("setpoint", cm) | ("gains", kp, ki, kd, tau) | ("mode", m)
                       ("ping",) | ("config",) | ("stop",)
"""

import multiprocessing as mp
import queue
import time

from PySide6.QtCore import QObject, Signal


def _run(cmd_q, out_q, opts):
    import protocol as P  # noqa: F401  (namedtuples must be importable in both processes)
    from esp_link import EspLink

    link = EspLink(port=opts.get("port"), on_event=lambda kind, port: out_q.put(("event", kind, port)))
    link.start()
    seq = 0
    frames = 0
    running = True

    def handle_cmds():
        nonlocal running
        while True:
            try:
                c = cmd_q.get_nowait()
            except queue.Empty:
                return
            if c[0] == "setpoint":
                link.set_setpoint(c[1])
            elif c[0] == "gains":
                link.set_gains(c[1], c[2], c[3], c[4])
            elif c[0] == "mode":
                link.set_mode(c[1])
            elif c[0] == "ping":
                link.ping()
            elif c[0] == "config":
                link.request_config()
            elif c[0] == "stop":
                running = False

    def emit(pos):
        nonlocal seq, frames
        if pos is not None:
            pos_cm, valid, warning = pos
            t = time.perf_counter()
            link.send_position(seq, pos_cm, valid, warning)
            seq = (seq + 1) & 0xFF
            frames += 1
            pos = (pos_cm, valid, warning, t)
        out_q.put(("batch", pos, link.drain(), dict(link.stats, connected=link.connected, port=link.port,
                                                   config=link.config), frames))
        handle_cmds()

    try:
        if opts.get("synthetic"):
            out_q.put(("status", "synthetic triangle wave (no camera)"))
            t0 = time.perf_counter()
            next_t = t0
            period = 1.0 / opts.get("fps", 100)
            beam = opts.get("beam_cm", 45.0)
            while running:
                now = time.perf_counter()
                phase = ((now - t0) % 4.0) / 4.0
                emit((beam * (2 * phase if phase < 0.5 else 2 - 2 * phase), True, False))
                next_t += period
                dt = next_t - time.perf_counter()
                if dt > 0:
                    time.sleep(dt)
        else:
            import calibrate
            import tracker as trk

            out_q.put(("status", "opening PS3 Eye ..."))
            cam = calibrate.open_camera(opts.get("id", 0), opts.get("fps", 100))
            try:
                if opts.get("skip_calibration"):
                    calib_dict = trk.load_calibration(opts["calibration_path"])
                    trk.apply_calibration_to_camera(cam, calib_dict)
                    out_q.put(("status", f"reusing calibration from {calib_dict['created']} (dev only)"))
                else:
                    out_q.put(("status", "calibration wizard running -- see the OpenCV window"))
                    calib = calibrate.run(cam, opts.get("beam_cm", 45.0), opts.get("scale", 2), opts["calibration_path"])
                    if calib is None or not calib.validation or not calib.validation.get("passed"):
                        out_q.put(("status", "calibration aborted or failed"))
                        return
                    calib_dict = calib.to_dict()
                out_q.put(("status", f"tracking (calibration {calib_dict.get('created', '?')})"))
                tr = trk.BallTracker(calib_dict)
                while running:
                    frame, _ = cam.read(timestamp=True)
                    reading, _ = tr.process(frame, time.perf_counter())
                    emit((reading.pos_cm if reading.valid else 0.0, reading.valid, tr.health.warning() is not None))
            finally:
                cam.end()
    except Exception as e:
        out_q.put(("status", f"camera process error: {e!r}"))
    finally:
        link.stop()
        out_q.put(("status", "camera process stopped"))


class LinkProxy(QObject):
    """GUI-side handle: same surface as EspLink + CameraThread, fed from the queues."""

    status = Signal(str)
    position = Signal(float, bool, bool, float)

    def __init__(self, opts):
        super().__init__()
        self.cmd_q = mp.Queue()
        self.out_q = mp.Queue()
        self.proc = mp.Process(target=_run, args=(self.cmd_q, self.out_q, opts), daemon=True)
        self.connected = False
        self.port = None
        self.config = None
        self.stats = dict(tx_frames=0, tx_dropped=0, rx_frames=0, rx_bytes=0, reconnects=0, rtt_ms=None)
        self.frames = 0
        self._frames = []
        self.events = []

    def start(self):
        self.proc.start()

    def stop(self):
        try:
            self.cmd_q.put(("stop",))
        except Exception:
            pass
        self.proc.join(timeout=3)
        if self.proc.is_alive():
            self.proc.terminate()

    # --- pull everything the child produced since the last call ---
    def pump(self):
        while True:
            try:
                m = self.out_q.get_nowait()
            except queue.Empty:
                return
            kind = m[0]
            if kind == "status":
                self.status.emit(m[1])
            elif kind == "event":
                self.events.append((m[1], m[2]))
                self.status.emit(f"ESP32 {m[1]} ({m[2]})")
            elif kind == "batch":
                pos, frames, stats, nframes = m[1], m[2], m[3], m[4]
                self.connected = stats.pop("connected")
                self.port = stats.pop("port")
                cfg = stats.pop("config")
                if cfg is not None:
                    self.config = cfg
                self.stats = stats
                self.frames = nframes
                if pos is not None:
                    self.position.emit(*pos)
                self._frames.extend(frames)

    def drain(self):
        self.pump()
        out, self._frames = self._frames, []
        return out

    # --- commands ---
    def set_setpoint(self, cm):
        self.cmd_q.put(("setpoint", cm))

    def set_gains(self, kp, ki, kd, tau):
        self.cmd_q.put(("gains", kp, ki, kd, tau))

    def set_mode(self, mode):
        self.cmd_q.put(("mode", mode))

    def ping(self):
        self.cmd_q.put(("ping",))

    def request_config(self):
        self.cmd_q.put(("config",))

    # CameraThread compatibility for MainWindow.closeEvent
    def wait(self, ms=0):
        self.proc.join(timeout=ms / 1000)
