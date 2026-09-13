"""Ball position tracker -- the production side of Faz C.

Turns camera frames into "ball is at X cm along the beam" readings, using
the detection chain and the numbers the calibration wizard produced.

Every start runs the wizard first. That is deliberate and not optional by
default: the light, the camera and the rig all drift between sessions, and
a tracker fed yesterday's thresholds fails quietly rather than loudly. The
--skip-calibration flag exists for development only.

While running, a health monitor watches the detection rate and the contour
size against what the wizard measured. Drift past the limits raises a
warning -- the reading is still delivered, but the operator is told the
calibration no longer matches the room.

Faz D builds on this: the serial sender imports BallTracker and the frame
loop, and forwards each Reading to the ESP32.

Usage:
    python tracker.py                    # wizard, then track, console output
    python tracker.py --view             # also show a small live window
    python tracker.py --skip-calibration # dev only: reuse calibration.json
"""

import argparse
import json
import os
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

import calibrate

HERE = os.path.dirname(os.path.abspath(__file__))

# Health limits. The detection rate floor is a little under the wizard's
# pass threshold so ordinary noise doesn't trip it; the area band is wide
# because the contour legitimately changes size along the beam.
HEALTH_WINDOW = 100
HEALTH_MIN_RATE = 0.90
HEALTH_AREA_RATIO = (0.45, 2.0)

CONSOLE_EVERY = 10  # print every Nth reading -> 10 Hz at 100 fps


@dataclass
class Reading:
    t_capture: float   # perf_counter() when the frame was read
    valid: bool
    pos_cm: float      # along the axis, 0 at p1, beam_length at p2
    perp_px: float     # distance off the axis, for diagnostics
    area: float


class HealthMonitor:
    """Rolling check that the room still looks like it did at calibration."""

    def __init__(self, area_reference):
        self.area_reference = area_reference
        self.detected = deque(maxlen=HEALTH_WINDOW)
        self.areas = deque(maxlen=HEALTH_WINDOW)

    def update(self, reading):
        self.detected.append(reading.valid)
        if reading.valid:
            self.areas.append(reading.area)

    @property
    def rate(self):
        return sum(self.detected) / len(self.detected) if self.detected else 1.0

    @property
    def area_ratio(self):
        if not self.areas or not self.area_reference:
            return 1.0
        return float(np.median(self.areas) / self.area_reference)

    def warning(self):
        if len(self.detected) < HEALTH_WINDOW:
            return None
        if self.rate < HEALTH_MIN_RATE:
            return f"detection rate {self.rate:.2f} -- recalibrate"
        lo, hi = HEALTH_AREA_RATIO
        if not lo <= self.area_ratio <= hi:
            return f"ball area {self.area_ratio:.2f}x calibration -- lighting changed, recalibrate"
        return None


class BallTracker:
    """Frame in, Reading out. Holds no camera; pure processing."""

    def __init__(self, calib_dict):
        self.beam_length_cm = calib_dict["beam_length_cm"]
        self.hsv = calib_dict["hsv"]
        axis = calib_dict["axis"]
        self.calib = calibrate.Calibration(self.beam_length_cm)
        self.calib.p1 = tuple(axis["p1"])
        self.calib.p2 = tuple(axis["p2"])
        self.calib.band_halfwidth = axis["band_halfwidth"]
        self.band_mask = None
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.health = HealthMonitor(calib_dict.get("validation", {}).get("area_median"))

    def process(self, frame_rgb, t_capture):
        if self.band_mask is None:
            self.band_mask = self.calib.band_mask(frame_rgb.shape)

        centroid, area, _ = calibrate.detect(frame_rgb, self.hsv, self.band_mask, self.kernel)
        if centroid is None:
            reading = Reading(t_capture, False, float("nan"), float("nan"), area)
        else:
            t, perp = self.calib.project(centroid)
            reading = Reading(t_capture, True, t * self.beam_length_cm, perp, area)

        self.health.update(reading)
        return reading, centroid


def load_calibration(path):
    with open(path) as f:
        return json.load(f)


def apply_calibration_to_camera(cam, calib_dict):
    """Pin the sensor at the wizard's exposure/gain when the wizard didn't run."""
    calibrate.set_exposure(cam, calib_dict["exposure"])
    calibrate.set_gain(cam, calib_dict["gain"])


def draw_view(frame_rgb, tracker, centroid, reading, warning, scale):
    view = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    poly = (tracker.calib.band_polygon() * scale).astype(np.int32)
    cv2.polylines(view, [poly], True, (255, 200, 0), 1)
    if centroid is not None:
        c = tuple((np.array(centroid) * scale).astype(int))
        cv2.drawMarker(view, c, (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
        label = f"{reading.pos_cm:5.1f} cm   area {reading.area:.0f}"
        colour = (0, 255, 0)
    else:
        label = "no ball"
        colour = (0, 0, 255)
    cv2.putText(view, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 1, cv2.LINE_AA)
    if warning:
        cv2.putText(view, warning, (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)
    cv2.imshow("tracker", view)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--fps", type=int, default=100)
    parser.add_argument("--beam-cm", type=float, default=45.0)
    parser.add_argument("--scale", type=int, default=2)
    parser.add_argument("--view", action="store_true", help="show a live window (lowers the rate)")
    parser.add_argument("--skip-calibration", action="store_true",
                        help="DEV ONLY: reuse calibration.json instead of running the wizard")
    parser.add_argument("--calibration", default=calibrate.CALIB_PATH)
    args = parser.parse_args()

    cam = calibrate.open_camera(args.id, args.fps)
    try:
        if args.skip_calibration:
            calib_dict = load_calibration(args.calibration)
            apply_calibration_to_camera(cam, calib_dict)
            print(f"WARNING: using saved calibration from {calib_dict['created']} -- "
                  f"lighting and rig may have moved since")
        else:
            calib = calibrate.run(cam, args.beam_cm, args.scale, args.calibration)
            if calib is None:
                return
            if not calib.validation or not calib.validation.get("passed"):
                print("calibration did not pass validation, not tracking")
                return
            calib_dict = calib.to_dict()

        tracker = BallTracker(calib_dict)
        print(f"\ntracking: 0 cm at p1, {tracker.beam_length_cm:g} cm at p2. q to quit.\n")

        intervals = deque(maxlen=200)
        proc_times = deque(maxlen=200)
        prev = None
        n = 0
        last_warning = None

        while True:
            frame, _ = cam.read(timestamp=True)
            t_capture = time.perf_counter()
            if prev is not None:
                intervals.append((t_capture - prev) * 1000.0)
            prev = t_capture

            t0 = time.perf_counter()
            reading, centroid = tracker.process(frame, t_capture)
            proc_times.append((time.perf_counter() - t0) * 1000.0)

            warning = tracker.health.warning()
            if warning != last_warning:
                if warning:
                    print(f"!! {warning}")
                last_warning = warning

            n += 1
            if n % CONSOLE_EVERY == 0:
                fps = 1000.0 / np.mean(intervals) if intervals else 0.0
                pos = f"{reading.pos_cm:6.2f} cm" if reading.valid else "   --   "
                print(f"{pos}  area {reading.area:5.0f}  perp {reading.perp_px if reading.valid else 0:4.1f}  "
                      f"| {fps:5.1f} fps  proc {np.mean(proc_times):4.2f} ms  "
                      f"rate {tracker.health.rate:.2f}", end="\r")

            if args.view:
                draw_view(frame, tracker, centroid, reading, warning, args.scale)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        cam.end()
        cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
