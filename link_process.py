"""Camera and ESP link, each in its own process; the GUI talks by queues.

Two child processes, not one:

    camera process  PS3 Eye -> tracker -> position queue (+ JPEG view)
    link process    position queue -> ESP; telemetry/ACK/CONFIG -> GUI

pseyepy's frame read blocks inside C++ *holding the GIL*, so a camera that
is unplugged mid-run freezes every Python thread in its process for good:
no watchdog inside that process can ever run. Keeping the link in a
separate process means a hung camera can be killed and restarted (by the
GUI's watchdog below) while the ESP link, its telemetry and the operator's
commands carry on.

Child -> GUI (out_q):  ("status", text) | ("event", kind, port)
                       ("camera", state, detail) | ("calib", dict)
                       ("batch", pos_tuple_or_None, [(ptype, msg), ...], stats, frames)
                       ("image", jpeg_bytes)
GUI -> link  (cmd_q):  ("setpoint", cm) | ("gains", ...) | ("mode", m) | ("theta", deg)
                       ("ping",) | ("config",) | ("stop",)
GUI -> camera (cam_cmd_q): ("view", on) | ("rescan",) | ("stop",)
"""

import multiprocessing as mp
import queue
import time

from PySide6.QtCore import QObject, Signal

CAMERA_RETRY_S = 3.0
CAMERA_STALL_S = 2.5      # no frame for this long while tracking -> restart
CAMERA_RESTART_GRACE_S = 4.0


# --------------------------------------------------------------------------
# link process: owns the serial link, forwards positions, drains telemetry
# --------------------------------------------------------------------------
def _link_run(cmd_q, out_q, pos_q, opts):
    import protocol as P  # noqa: F401  (namedtuples must exist in both processes)
    from esp_link import EspLink

    link = EspLink(port=opts.get("port"), on_event=lambda kind, port: out_q.put(("event", kind, port)))
    link.start()
    seq = 0
    frames = 0
    running = True

    while running:
        # commands from the GUI
        while True:
            try:
                c = cmd_q.get_nowait()
            except queue.Empty:
                break
            if c[0] == "setpoint":
                link.set_setpoint(c[1])
            elif c[0] == "gains":
                link.set_gains(c[1], c[2], c[3], c[4])
            elif c[0] == "mode":
                link.set_mode(c[1])
            elif c[0] == "theta":
                link.set_theta(c[1])
            elif c[0] == "ping":
                link.ping()
            elif c[0] == "config":
                link.request_config()
            elif c[0] == "stop":
                running = False

        # positions from the camera process (or none: the ESP's own
        # staleness watchdog then levels the beam, which is what we want)
        pos = None
        while True:
            try:
                pos = pos_q.get_nowait()
            except queue.Empty:
                break
            pos_cm, valid, warning, t_capture = pos
            link.send_position(seq, pos_cm, valid, warning)
            seq = (seq + 1) & 0xFF
            frames += 1

        out_q.put(("batch", pos, link.drain(),
                   dict(link.stats, connected=link.connected, port=link.port, config=link.config), frames))
        time.sleep(0.004)

    link.stop()
    out_q.put(("status", "link process stopped"))


# --------------------------------------------------------------------------
# camera process: owns the PS3 Eye, publishes positions and the view
# --------------------------------------------------------------------------
def _camera_run(cam_cmd_q, out_q, pos_q, opts):
    view_on = False
    rescan = False

    def handle_cmds():
        nonlocal view_on, rescan
        stop = False
        while True:
            try:
                c = cam_cmd_q.get_nowait()
            except queue.Empty:
                return stop
            if c[0] == "view":
                view_on = bool(c[1])
            elif c[0] == "rescan":
                rescan = True
            elif c[0] == "stop":
                stop = True

    def idle(seconds):
        nonlocal rescan
        end = time.perf_counter() + seconds
        while time.perf_counter() < end and not rescan:
            if handle_cmds():
                return True
            time.sleep(0.05)
        rescan = False
        return False

    try:
        if opts.get("synthetic"):
            out_q.put(("camera", "tracking", "synthetic triangle wave (no camera)"))
            t0 = time.perf_counter()
            period = 1.0 / opts.get("fps", 100)
            beam = opts.get("beam_cm", 45.0)
            next_t = t0
            while True:
                if handle_cmds():
                    return
                now = time.perf_counter()
                phase = ((now - t0) % 4.0) / 4.0
                pos_q.put((beam * (2 * phase if phase < 0.5 else 2 - 2 * phase), True, False, now))
                next_t += period
                dt = next_t - time.perf_counter()
                if dt > 0:
                    time.sleep(dt)

        import calibrate
        import tracker as trk
        import cv2
        from pseyepy import cam_count

        while True:
            if handle_cmds():
                return
            try:
                n = cam_count()
            except Exception as e:
                n = 0
                out_q.put(("camera", "error", f"cam_count: {e!r}"))
            if n < 1:
                out_q.put(("camera", "missing", "PS3 Eye bulunamadı -- USB'yi takın"))
                if idle(CAMERA_RETRY_S):
                    return
                continue

            out_q.put(("camera", "opening", f"{n} PS3 Eye"))
            try:
                cam = calibrate.open_camera(opts.get("id", 0), opts.get("fps", 100))
            except Exception as e:
                out_q.put(("camera", "missing", f"açılamadı: {e}"))
                if idle(CAMERA_RETRY_S):
                    return
                continue

            try:
                calib_dict = opts.get("calibration")
                if calib_dict:
                    # A calibration handed over by the GUI (session restart
                    # after the camera was unplugged): no wizard, the light
                    # and the camera pose have not changed.
                    trk.apply_calibration_to_camera(cam, calib_dict)
                    out_q.put(("camera", "calibrating", f"kayıtlı kalibrasyon {calib_dict.get('created', '?')}"))
                elif opts.get("skip_calibration"):
                    calib_dict = trk.load_calibration(opts["calibration_path"])
                    trk.apply_calibration_to_camera(cam, calib_dict)
                    out_q.put(("camera", "calibrating", f"kayıtlı kalibrasyon {calib_dict.get('created', '?')}"))
                else:
                    out_q.put(("camera", "calibrating", "sihirbaz açık -- OpenCV penceresi"))
                    calib = calibrate.run(cam, opts.get("beam_cm", 45.0), opts.get("scale", 2),
                                          opts["calibration_path"])
                    if calib is None or not calib.validation or not calib.validation.get("passed"):
                        out_q.put(("camera", "error", "kalibrasyon iptal/başarısız -- 'Yeniden tara'"))
                        cam.end()
                        if idle(3600):
                            return
                        continue
                    calib_dict = calib.to_dict()
                opts["calibration"] = calib_dict
                out_q.put(("calib", calib_dict))
                out_q.put(("camera", "tracking", f"kalibrasyon {calib_dict.get('created', '?')}"))

                tr = trk.BallTracker(calib_dict)
                k = 0
                while True:
                    if handle_cmds():
                        cam.end()
                        return
                    # Blocks in C++ with the GIL held; if the camera is
                    # unplugged this never returns and the GUI restarts us.
                    frame, _ = cam.read(timestamp=True)
                    reading, centroid = tr.process(frame, time.perf_counter())
                    warning = tr.health.warning()
                    pos_q.put((reading.pos_cm if reading.valid else 0.0, reading.valid,
                               warning is not None, time.perf_counter()))
                    k += 1
                    if view_on and k % 4 == 0:
                        view = trk.render_view(frame, tr, centroid, reading, warning, 2)
                        ok, buf = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if ok:
                            out_q.put(("image", buf.tobytes()))
            except Exception as e:
                out_q.put(("camera", "error", f"{e!r}"))
                try:
                    cam.end()
                except Exception:
                    pass
                if idle(CAMERA_RETRY_S):
                    return
    except Exception as e:
        out_q.put(("camera", "error", f"kamera süreci hatası: {e!r}"))


# --------------------------------------------------------------------------
# GUI-side handle
# --------------------------------------------------------------------------
class LinkProxy(QObject):
    """One object for the GUI: ESP link + camera, both out of process."""

    status = Signal(str)
    camera_state = Signal(str, str)  # state, detail
    position = Signal(float, bool, bool, float)
    image = Signal(bytes)

    def __init__(self, opts):
        super().__init__()
        self.opts = dict(opts)
        self.cmd_q = mp.Queue()
        self.cam_cmd_q = mp.Queue()
        self.out_q = mp.Queue()
        self.pos_q = mp.Queue(maxsize=200)
        self.link_proc = None
        self.cam_proc = None
        self.connected = False
        self.port = None
        self.config = None
        self.stats = dict(tx_frames=0, tx_dropped=0, rx_frames=0, rx_bytes=0, reconnects=0, rtt_ms=None)
        self.frames = 0
        self._frames = []
        self.events = []
        self.cam_state = "starting"
        self.cam_detail = ""
        self.cam_restarts = 0
        self.view_on = False
        self._last_pos_t = time.perf_counter()
        self._cam_started_at = 0.0

    # --- lifecycle --------------------------------------------------------
    def start(self):
        self.link_proc = mp.Process(target=_link_run, args=(self.cmd_q, self.out_q, self.pos_q, self.opts),
                                    daemon=True)
        self.link_proc.start()
        self._start_camera()

    def _start_camera(self):
        self.cam_proc = mp.Process(target=_camera_run,
                                   args=(self.cam_cmd_q, self.out_q, self.pos_q, self.opts), daemon=True)
        self.cam_proc.start()
        self._cam_started_at = time.perf_counter()
        self._last_pos_t = time.perf_counter()
        if self.view_on:
            self.cam_cmd_q.put(("view", True))

    def restart_camera(self, reason=""):
        """Kill a wedged camera process and start a fresh one.

        The read blocks in C++ with the GIL held, so terminate() is the only
        way out. The ESP link is untouched -- it lives in the other process.
        """
        self.cam_restarts += 1
        self.cam_state, self.cam_detail = "missing", reason or "kamera yanıt vermiyor"
        self.camera_state.emit(self.cam_state, self.cam_detail)
        try:
            if self.cam_proc and self.cam_proc.is_alive():
                self.cam_proc.terminate()
                self.cam_proc.join(timeout=2)
        except Exception:
            pass
        # drain anything the dead process left behind
        for q in (self.pos_q,):
            while True:
                try:
                    q.get_nowait()
                except queue.Empty:
                    break
        self._start_camera()

    def stop(self):
        for q, proc in ((self.cmd_q, self.link_proc), (self.cam_cmd_q, self.cam_proc)):
            try:
                q.put(("stop",))
            except Exception:
                pass
        for proc in (self.cam_proc, self.link_proc):
            if not proc:
                continue
            proc.join(timeout=2)
            if proc.is_alive():
                proc.terminate()

    # --- pump -------------------------------------------------------------
    def pump(self):
        while True:
            try:
                m = self.out_q.get_nowait()
            except queue.Empty:
                break
            kind = m[0]
            if kind == "status":
                self.status.emit(m[1])
            elif kind == "event":
                self.events.append((m[1], m[2]))
                self.status.emit(f"ESP32 {m[1]} ({m[2]})")
            elif kind == "camera":
                self.cam_state, self.cam_detail = m[1], m[2]
                self.camera_state.emit(m[1], m[2])
            elif kind == "calib":
                # keep it so a restarted camera skips the wizard
                self.opts["calibration"] = m[1]
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
                    self._last_pos_t = time.perf_counter()
                    self.position.emit(*pos)
                self._frames.extend(frames)

        # camera watchdog: tracking but no frames -> the read is wedged
        if self.cam_state == "tracking" and \
                time.perf_counter() - self._last_pos_t > CAMERA_STALL_S and \
                time.perf_counter() - self._cam_started_at > CAMERA_RESTART_GRACE_S:
            self.restart_camera("kamera çekildi / yanıt vermiyor")
        elif self.cam_proc and not self.cam_proc.is_alive() and \
                time.perf_counter() - self._cam_started_at > CAMERA_RESTART_GRACE_S:
            self.restart_camera("kamera süreci kapandı")

    def drain(self):
        self.pump()
        out, self._frames = self._frames, []
        return out

    # --- commands ---------------------------------------------------------
    def set_setpoint(self, cm):
        self.cmd_q.put(("setpoint", cm))

    def set_gains(self, kp, ki, kd, tau):
        self.cmd_q.put(("gains", kp, ki, kd, tau))

    def set_mode(self, mode):
        self.cmd_q.put(("mode", mode))

    def set_theta(self, theta_deg):
        self.cmd_q.put(("theta", theta_deg))

    def ping(self):
        self.cmd_q.put(("ping",))

    def request_config(self):
        self.cmd_q.put(("config",))

    def set_view(self, on):
        self.view_on = bool(on)
        self.cam_cmd_q.put(("view", self.view_on))

    def rescan(self):
        """Operator asked for a camera rescan: restart it outright."""
        self.restart_camera("yeniden tarama istendi")
