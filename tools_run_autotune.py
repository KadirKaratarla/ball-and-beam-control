"""Full autotune on the rig: camera + link + tuner in one process."""
import sys, time
sys.path.insert(0, r"E:\projeler\ball_beam\gui")

import protocol as P
from esp_link import EspLink
from autotune import Autotuner, Aborted
import calibrate
import tracker as trk

profile = sys.argv[1] if len(sys.argv) > 1 else "normal"

link = EspLink()
link.start()
t0 = time.perf_counter()
while not link.connected and time.perf_counter() - t0 < 10:
    time.sleep(0.05)
print("ESP32:", link.connected, link.port, flush=True)
if not link.connected:
    sys.exit(1)

cam = calibrate.open_camera(0, 100)
calib = trk.load_calibration(calibrate.CALIB_PATH)
trk.apply_calibration_to_camera(cam, calib)
tracker = trk.BallTracker(calib)
print("kamera açık, kalibrasyon", calib.get("created"), flush=True)

st = {"seq": 0, "frames": 0, "lost": 0}


def pump():
    frame, _ = cam.read(timestamp=True)
    reading, _ = tracker.process(frame, time.perf_counter())
    link.send_position(st["seq"], reading.pos_cm if reading.valid else 0.0, reading.valid,
                       tracker.health.warning() is not None)
    st["seq"] = (st["seq"] + 1) & 0xFF
    st["frames"] += 1
    if not reading.valid:
        st["lost"] += 1


tuner = Autotuner(link, on_status=lambda s: print(s, flush=True), pump_cb=pump)
rc = 0
try:
    link.request_config()
    for _ in range(150):
        pump()
        tuner._drain()
    cfg = link.config
    old = (cfg.kp, cfg.ki, cfg.kd, cfg.d_tau_s) if cfg else None
    result = tuner.run(profile=profile, old_gains=old)
    print("\n--- ölçüm ---", flush=True)
    print(tuner.report(), flush=True)
    print("\n--- ayar ---", flush=True)
    print(tuner.format_result(result), flush=True)
    print(f"\nkare {st['frames']}, top görünmeyen {st['lost']}", flush=True)
    for pl in tuner.m.pulses:
        print("  pulse", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in pl.items()}, flush=True)
except Aborted as e:
    print(f"\nDURDURULDU: {e}", flush=True)
    rc = 2
finally:
    try:
        link.set_theta(0.0)
        link.set_mode(P.MODE_RUN)
        link.set_setpoint(22.5)
        for _ in range(60):
            pump()
    except Exception:
        pass
    cam.end()
    link.stop()
sys.exit(rc)
