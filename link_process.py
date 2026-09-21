"""Camera + ESP link in their own process; the GUI talks to it via queues.

pseyepy's frame read blocks inside C++ while holding the GIL, so anything
sharing the interpreter with the camera loop starves (a Qt GUI freezes and
positions get delayed past the ESP's 50 ms staleness limit). A separate
process has its own GIL: the loop here runs exactly as link.py does, and
the GUI process only ever touches queues.

Child -> GUI (out_q):  ("status", text) | ("event", kind, port)
                       ("camera", "missing" | "opening" | "calibrating" | "tracking" | "error", detail)
                       ("batch", pos_tuple_or_None, [(ptype, msg), ...], stats_dict, frames)
                       ("image", jpeg_bytes)   every 4th frame while the view is on
GUI -> child (cmd_q):  ("setpoint", cm) | ("gains", kp, ki, kd, tau) | ("mode", m)
                       ("ping",) | ("config",) | ("view", on) | ("rescan",) | ("stop",)

The ESP link lives for the whole process regardless of the camera: with no
camera the loop idles (telemetry still flows to the GUI), retries the
camera every CAMERA_RETRY_S and immediately on "rescan".
"""

import multiprocessing as mp
import queue
import time

from PySide6.QtCore import QObject, Signal

CAMERA_RETRY_S = 3.0


def _run(cmd_q, out_q, opts):
    import protocol as P  # noqa: F401  (namedtuples must be importable in both processes)
    from esp_link import EspLink

    link = EspLink(port=opts.get("port"), on_event=lambda kind, port: out_q.put(("event", kind, port)))
    link.start()
    seq = 0
    frames = 0
    running = True
    view_on = False
    rescan = False

    def handle_cmds():
        nonlocal running, view_on, rescan
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
            elif c[0] == "view":
                view_on = bool(c[1])
            elif c[0] == "rescan":
                rescan = True
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

    def idle(seconds):
        """Keep the link/GUI traffic alive while there is no camera."""
        nonlocal rescan
        end = time.perf_counter() + seconds
        while running and time.perf_counter() < end and not rescan:
            emit(None)
            time.sleep(0.05)
        rescan = False

    try:
        if opts.get("synthetic"):
            out_q.put(("camera", "tracking", "synthetic triangle wave (no camera)"))
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
            return

        import calibrate
        import tracker as trk
        import cv2
        from pseyepy import cam_count

        while running:
            # --- find the camera ---
            try:
                n = cam_count()
            except Exception as e:
                n = 0
                out_q.put(("camera", "error", f"cam_count: {e!r}"))
            if n < 1:
                out_q.put(("camera", "missing", "PS3 Eye bulunamadı (libusbK, Interface 0)"))
                idle(CAMERA_RETRY_S)
                continue

            out_q.put(("camera", "opening", f"{n} PS3 Eye"))
            try:
                cam = calibrate.open_camera(opts.get("id", 0), opts.get("fps", 100))
            except Exception as e:
                out_q.put(("camera", "missing", f"açılamadı: {e}"))
                idle(CAMERA_RETRY_S)
                continue

            try:
                if opts.get("skip_calibration"):
                    calib_dict = trk.load_calibration(opts["calibration_path"])
                    trk.apply_calibration_to_camera(cam, calib_dict)
                    out_q.put(("camera", "calibrating", f"kayıtlı kalibrasyon {calib_dict.get('created', '?')}"))
                else:
                    out_q.put(("camera", "calibrating", "sihirbaz açık -- OpenCV penceresi"))
                    calib = calibrate.run(cam, opts.get("beam_cm", 45.0), opts.get("scale", 2), opts["calibration_path"])
                    if calib is None or not calib.validation or not calib.validation.get("passed"):
                        out_q.put(("camera", "error", "kalibrasyon iptal edildi veya geçmedi -- Yeniden tara ile tekrar"))
                        cam.end()
                        idle(3600)  # wait for a rescan
                        continue
                    calib_dict = calib.to_dict()
                out_q.put(("camera", "tracking", f"kalibrasyon {calib_dict.get('created', '?')}"))
                tr = trk.BallTracker(calib_dict)
                n = 0
                while running:
                    frame, _ = cam.read(timestamp=True)
                    reading, centroid = tr.process(frame, time.perf_counter())
                    warning = tr.health.warning()
                    emit((reading.pos_cm if reading.valid else 0.0, reading.valid, warning is not None))
                    n += 1
                    if view_on and n % 4 == 0:
                        view = trk.render_view(frame, tr, centroid, reading, warning, 2)
                        ok, buf = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if ok:
                            out_q.put(("image", buf.tobytes()))
            except Exception as e:
                out_q.put(("camera", "error", f"{e!r} -- yeniden deneniyor"))
                try:
                    cam.end()
                except Exception:
                    pass
                idle(CAMERA_RETRY_S)
                continue
            cam.end()
    except Exception as e:
        out_q.put(("status", f"camera process error: {e!r}"))
    finally:
        link.stop()
        out_q.put(("status", "camera process stopped"))


class LinkProxy(QObject):
    """GUI-side handle: same surface as EspLink + the camera, fed from the queues."""

    status = Signal(str)
    camera_state = Signal(str, str)  # state, detail
    position = Signal(float, bool, bool, float)
    image = Signal(bytes)  # JPEG of the tracker view

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
        self.cam_state = "starting"
        self.cam_detail = ""

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
            elif kind == "camera":
                self.cam_state, self.cam_detail = m[1], m[2]
                self.camera_state.emit(m[1], m[2])
            elif kind == "image":
                self.image.emit(m[1])
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

    def set_view(self, on):
        self.cmd_q.put(("view", bool(on)))

    def rescan(self):
        self.cmd_q.put(("rescan",))
