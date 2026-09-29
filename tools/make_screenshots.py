"""Render the GUI's documentation screenshots without the rig attached.

The interface is the real one -- it is built, laid out and grabbed exactly
as it runs -- but the telemetry behind it is replayed rather than live: a
step sequence through the identified plant (K = 605 cm/s^2/rad, 45 ms
delay) driven by the firmware's own gains, plus the health counters the rig
actually reported. That way the screenshots can be regenerated whenever the
interface changes, instead of going stale the moment a label moves.

    python tools/make_screenshots.py

Writes the five PNGs the README links to, into assets/images/.
"""
import math
import os
import sys

import cv2
import numpy as np
from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "gui"))

import protocol as P                                    # noqa: E402
from ui_main import MainWindow                          # noqa: E402

IMG = os.path.join(ROOT, "assets", "images")
CAM_FRAME_SRC = os.path.join(ROOT, "assets", "video", "normal çalışma ekran görüntüsü.mp4")
CAM_FRAME_T = 30.0
CAM_FRAME_CROP = (60, 95, 330, 250)                     # the camera panel in that recording

WIN_W, WIN_H = 1900, 1000

# What the rig reported on the bench, straight out of a HEALTH frame.
HEALTH = P.Health(ticks=250, loop_us_mean=314, loop_us_max=467, overruns=0, missed=0,
                  enc_i2c_errors=0, enc_rejects=0, enc_dir_faults=0,
                  link_packets=12480, link_crc_errors=0, link_seq_gaps=0,
                  cmd_rx=37, cmd_bad=0, tx_dropped=0,
                  tmc_resets=0, tmc_uart_errors=0, drv_status=0x400A0000,
                  state=P.MODE_RUN if hasattr(P, "MODE_RUN") else 0, fault=0)

CONFIG = P.Config(version=1, kp=0.74, ki=0.06, kd=0.50, d_tau_s=0.15, theta_max_deg=3.13,
                  x_set_0p1mm=2250, vmax=43000.0, amax=430000.0, level_counts=1101,
                  irun=10, ihold=4, defaults=1,
                  kp_def=0.74, ki_def=0.06, kd_def=0.50, d_tau_def=0.15)


# --- replayed telemetry -----------------------------------------------------

def simulate(seconds=19.6, dt=0.01):
    """The identified plant under the firmware's own controller.

    Not a live capture, but the same model the tuning was done against, so
    the traces have the shape and the overshoot the rig actually shows.
    """
    K = 605.0                       # cm/s^2 per rad
    delay = int(round(0.045 / dt))  # 45 ms, camera + link + loop
    kp, ki, kd, tau = 0.74, 0.06, 0.50, 0.15
    x, v = 22.5, 0.0
    hist = [x] * (delay + 1)
    integ, d_state, x_prev = 0.0, 0.0, x
    out = []
    for k in range(int(seconds / dt)):
        t = k * dt
        target = (22.5 if t < 2 else 27.5 if t < 6.5 else 17.5 if t < 11 else
                  30.0 if t < 15.5 else 22.5)
        meas = hist[-(delay + 1)]
        err = target - meas
        integ = max(-1.5 / max(ki, 1e-6), min(1.5 / max(ki, 1e-6), integ + err * dt))
        d_raw = -(meas - x_prev) / dt
        d_state += (d_raw - d_state) * dt / max(tau, 1e-6)
        x_prev = meas
        theta = kp * err + ki * integ + kd * d_state
        theta = max(-3.13, min(3.13, theta))
        v += K * math.radians(theta) * dt
        v *= 0.995
        x = min(45.0, max(0.0, x + v * dt))
        if x in (0.0, 45.0):
            v = 0.0
        hist.append(x)
        phi = theta * 20.5                       # linkage, near enough for a picture
        out.append(P.Telem(
            t_ms=int(t * 1000), x_0p1mm=int(round(x * 100)), x_set_0p1mm=int(round(target * 100)),
            p=int(kp * err * 100), i=int(ki * integ * 100), d=int(kd * d_state * 100),
            theta=int(round(theta * 100)), phi_0p1deg=int(round(phi * 10)),
            enc_counts=int(1101 + phi * 4096 / 360), follow_0p1=int(np.random.randint(-30, 30)),
            lag_counts=0, state=3, fault=0, flags=0b101, last_seq=k & 0xFF,
            loop_us=314, cmd_vel_10=int(abs(v) * 300)))
    return out


def camera_jpeg():
    """A real frame of the tracking view, lifted from the recorded session."""
    cap = cv2.VideoCapture(CAM_FRAME_SRC)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(CAM_FRAME_T * (cap.get(cv2.CAP_PROP_FPS) or 30)))
    ok, fr = cap.read()
    cap.release()
    if not ok:
        return None
    x0, y0, x1, y1 = CAM_FRAME_CROP
    crop = cv2.resize(fr[y0:y1, x0:x1], (640, 360), interpolation=cv2.INTER_CUBIC)
    ok, buf = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 92])
    return buf.tobytes() if ok else None


# --- stubs the window talks to ---------------------------------------------

class FakeLink:
    def __init__(self):
        self.connected = True
        self.port = "COM12"
        self.stats = {"rtt_ms": 1.2, "reconnects": 0}
        self.queue = []

    def drain(self):
        q, self.queue = self.queue, []
        return q

    def ping(self):
        pass

    def set_setpoint(self, cm):
        pass

    def set_gains(self, *a):
        pass

    def set_mode(self, m):
        pass

    def request_config(self):
        pass

    def stop(self):
        pass


class FakeCamera(QObject):
    status = Signal(str)
    camera_state = Signal(str, str)
    position = Signal(float, bool, bool, float)
    image = Signal(bytes)

    def __init__(self):
        super().__init__()
        self.frames = 0
        self.cam_restarts = 0

    def rescan(self):
        pass

    def set_view(self, on):
        pass


def main():
    os.makedirs(IMG, exist_ok=True)
    app = QApplication(sys.argv)
    link, cam = FakeLink(), FakeCamera()
    win = MainWindow(link, cam)
    win.resize(WIN_W, WIN_H)
    win.show()

    # config first, so the gain boxes show the firmware defaults
    link.queue.append((P.T_CONFIG, CONFIG))
    win.poll()

    # replay the step sequence into the plots
    frames = simulate()
    for k, tm in enumerate(frames):
        link.queue.append((P.T_TELEM, tm))
        if k % 250 == 0:
            link.queue.append((P.T_HEALTH, HEALTH))
        if k % 4 == 0:
            win.poll()
            app.processEvents()
    link.queue.append((P.T_HEALTH, HEALTH))
    win.telem_rate, win.cam_rate = 125, 100
    win.on_camera_state("tracking", "kalibrasyon 2026-09-20T23:48:55")
    win.poll()
    app.processEvents()

    def shot(name):
        path = os.path.join(IMG, name + ".png")
        win.telem_rate, win.cam_rate = 125, 100     # the timer would recount these
        win.refresh()
        app.processEvents()
        win.grab().save(path)
        print("  %-24s %5.2f MB" % (name + ".png", os.path.getsize(path) / 1048576))

    shot("gui_overview")

    # camera view instead of the schematic
    jpg = camera_jpeg()
    win.btn_view.setChecked(True)
    app.processEvents()
    if jpg:
        win.on_image(jpg)
    app.processEvents()
    shot("gui_camera_view")
    win.btn_view.setChecked(False)
    app.processEvents()

    # the Motor health tab, cropped to the panel
    win.tabs.setCurrentIndex(2)
    app.processEvents()
    win.poll()
    app.processEvents()
    win.tabs.grab().save(os.path.join(IMG, "gui_health_motor.png"))
    print("  %-24s %5.2f MB" % ("gui_health_motor.png",
                                os.path.getsize(os.path.join(IMG, "gui_health_motor.png")) / 1048576))
    win.tabs.setCurrentIndex(0)

    # device-missing states
    link.connected = False
    win.poll()
    app.processEvents()
    shot("gui_esp_missing")

    link.connected = True
    win.on_camera_state("missing", "PS3 Eye not found -- plug in the USB")
    win.poll()
    app.processEvents()
    shot("gui_camera_missing")

    win.close()


if __name__ == "__main__":
    main()
