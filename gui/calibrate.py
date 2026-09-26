"""Start-up calibration wizard for the ball tracker -- Faz C.

Runs before every session because two things move between sessions: the
light and the hardware. Fixed HSV thresholds tuned by day fail by night, and
the beam's place in the frame shifts whenever the camera or the rig is
nudged. So instead of stored magic numbers, each start re-derives them from
measurements the user drives with a few clicks:

  1. exposure   - pin the sensor so the frame lands at a target brightness,
                  whatever the room is doing; nothing else is tuned until
                  this baseline exists
  2. axis       - click both ends of the beam: gives the axis to project the
                  ball onto, the band of pixels worth processing, and the
                  pixel-to-cm scale
  3. colour     - click the ball a few times: the HSV range is fitted to the
                  sampled pixels, hue tight and saturation/value as loose
                  floors so the remaining light variation passes through
  4. validate   - roll the ball end to end while detection rate, covered
                  range and contour size are checked against pass criteria
  5. save       - write everything, timestamped, for the tracker to load

The camera looks at the beam from the side, so the ball's position is its
projection onto the axis measured at rest. Tilt shortens that projection by
cos(theta), about 1.5% at 10 degrees, which the controller absorbs; the
encoder angle is available later if it ever matters.

Keys (shown on screen for the active step):
    n      next step          b   back one step
    r      redo current step  q   quit without saving
    +/-    widen / narrow the beam band (axis step)
    space  finish validation early
"""

import argparse
import json
import os
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
from pseyepy import Camera

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB_PATH = os.path.join(HERE, "calibration.json")

WINDOW = "calibration"

STEP_EXPOSURE, STEP_AXIS, STEP_COLOUR, STEP_VALIDATE, STEP_DONE = range(5)
STEP_NAMES = ["1/4 exposure", "2/4 beam axis", "3/4 ball colour", "4/4 validate", "done"]

# Brightness the exposure loop aims the frame at. Mid-scale leaves room in
# both directions for the ball's highlights and shadows to stay inside the
# sampled HSV range.
TARGET_V = 110
TARGET_V_TOL = 8

# The band is a strip around the axis. It has to hold the ball at every
# tilt the beam will reach, so it is wider than the ball itself.
DEFAULT_BAND_HALFWIDTH = 45

# Colour sampling reads a small patch around each click rather than the one
# pixel under it: a single pixel is noise, a patch is a distribution.
SAMPLE_PATCH = 5
MIN_COLOUR_SAMPLES = 3

# Validation: what "the tracker works across the beam" means in numbers.
VALIDATE_SECONDS = 10
PASS_DETECTION_RATE = 0.95
PASS_RANGE_COVER = 0.80   # ball must have visited this much of the axis
PASS_MIN_AREA_RATIO = 0.4  # smallest contour vs median: far end shrinkage


class Calibration:
    """Everything the wizard determines, in frame (not display) coordinates."""

    def __init__(self, beam_length_cm):
        self.beam_length_cm = beam_length_cm
        self.exposure = None
        self.gain = None
        self.p1 = None          # left end of the beam, (x, y)
        self.p2 = None          # right end
        self.band_halfwidth = DEFAULT_BAND_HALFWIDTH
        self.hsv = None         # dict h_min..v_max, min_area
        self.colour_samples = []
        self.validation = None

    # -- axis geometry -------------------------------------------------------

    def axis_ready(self):
        return self.p1 is not None and self.p2 is not None

    def axis(self):
        p1 = np.array(self.p1, dtype=float)
        p2 = np.array(self.p2, dtype=float)
        d = p2 - p1
        length = float(np.hypot(*d))
        return p1, d / length if length else d, length

    def project(self, point):
        """Return (t along axis 0..1, perpendicular distance in px)."""
        p1, u, length = self.axis()
        v = np.array(point, dtype=float) - p1
        along = float(np.dot(v, u))
        perp = float(abs(u[0] * v[1] - u[1] * v[0]))
        return along / length if length else 0.0, perp

    def band_polygon(self):
        p1, u, length = self.axis()
        n = np.array([-u[1], u[0]]) * self.band_halfwidth
        margin = u * self.band_halfwidth
        corners = np.array([
            p1 - margin + n,
            p1 + u * length + margin + n,
            p1 + u * length + margin - n,
            p1 - margin - n,
        ])
        return corners.astype(np.int32)

    def band_mask(self, shape):
        mask = np.zeros(shape[:2], dtype=np.uint8)
        if self.axis_ready():
            cv2.fillPoly(mask, [self.band_polygon()], 255)
        return mask

    # -- persistence ---------------------------------------------------------

    def to_dict(self):
        return {
            "created": datetime.now().isoformat(timespec="seconds"),
            "beam_length_cm": self.beam_length_cm,
            "exposure": self.exposure,
            "gain": self.gain,
            "axis": {"p1": self.p1, "p2": self.p2, "band_halfwidth": self.band_halfwidth},
            "hsv": self.hsv,
            "validation": self.validation,
        }


# -- camera helpers ----------------------------------------------------------

def set_exposure(cam, value):
    value = int(max(0, min(255, value)))
    try:
        cam.exposure = value
    except Exception:
        cam.exposure = [value]
    return value


def set_gain(cam, value):
    value = int(max(0, min(63, value)))
    try:
        cam.gain = value
    except Exception:
        cam.gain = [value]
    return value


def mean_brightness(frame_rgb, mask=None):
    v = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)[:, :, 2]
    if mask is not None and mask.any():
        return float(v[mask > 0].mean())
    return float(v.mean())


def normalize_exposure(cam, calib, region_mask, show):
    """Walk exposure (then gain) until the region sits at TARGET_V, then hold.

    The camera's own auto-exposure is deliberately not used: it would keep
    adapting during the run and drag the frame time around with it. This
    converges once and leaves the sensor pinned.
    """
    exposure = calib.exposure if calib.exposure is not None else 120
    gain = calib.gain if calib.gain is not None else 20
    exposure = set_exposure(cam, exposure)
    gain = set_gain(cam, gain)

    for iteration in range(40):
        # Let the new setting propagate through the pipeline before judging it.
        for _ in range(3):
            frame, _ = cam.read(timestamp=True)
        mean_v = mean_brightness(frame, region_mask)
        error = TARGET_V - mean_v

        show(frame, f"normalising: exposure={exposure} gain={gain} mean V={mean_v:.0f} target={TARGET_V}")

        if abs(error) <= TARGET_V_TOL:
            break

        if error > 0 and exposure >= 255:
            if gain >= 63:
                break
            gain = set_gain(cam, gain + 6)
            continue
        if error < 0 and exposure <= 0:
            if gain <= 0:
                break
            gain = set_gain(cam, gain - 6)
            continue

        step = max(-30, min(30, error * 0.6))
        exposure = set_exposure(cam, exposure + step)

    calib.exposure = exposure
    calib.gain = gain
    return mean_v


# -- detection (same chain the tracker uses) ---------------------------------

def hsv_range_from_samples(samples):
    """Fit thresholds to clicked pixels: hue tight, saturation/value floors only.

    Hue barely moves with lighting, so it carries the discrimination and gets
    a narrow window. Saturation and value are exactly what lighting shifts,
    so they only get a floor low enough to keep the ball's shaded side, and
    no ceiling at all.
    """
    arr = np.concatenate(samples).astype(float)
    h, s, v = arr[:, 0], arr[:, 1], arr[:, 2]

    h_med = np.median(h)
    h_mad = np.median(np.abs(h - h_med))
    h_half = max(8.0, 3.0 * h_mad)

    return {
        "h_min": int(max(0, h_med - h_half)),
        "h_max": int(min(179, h_med + h_half)),
        "s_min": int(max(0, np.percentile(s, 10) - 25)),
        "s_max": 255,
        "v_min": int(max(0, np.percentile(v, 10) - 25)),
        "v_max": 255,
        "min_area": 30,
    }


def detect(frame_rgb, hsv_range, band_mask, kernel):
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    lower = np.array([hsv_range["h_min"], hsv_range["s_min"], hsv_range["v_min"]])
    upper = np.array([hsv_range["h_max"], hsv_range["s_max"], hsv_range["v_max"]])
    mask = cv2.inRange(hsv, lower, upper)
    if band_mask is not None:
        mask = cv2.bitwise_and(mask, band_mask)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0, mask
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < hsv_range["min_area"]:
        return None, area, mask
    m = cv2.moments(largest)
    if m["m00"] == 0:
        return None, area, mask
    return (m["m10"] / m["m00"], m["m01"] / m["m00"]), area, mask


# -- wizard ------------------------------------------------------------------

class Wizard:
    def __init__(self, cam, calib, scale):
        self.cam = cam
        self.calib = calib
        self.scale = scale
        self.step = STEP_EXPOSURE
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.message = ""
        self.last_frame = None
        self.validate_started = None
        self.validate_stats = None
        cv2.namedWindow(WINDOW)
        cv2.setMouseCallback(WINDOW, self.on_mouse)

    # display ---------------------------------------------------------------

    def show(self, frame_rgb, status, mask=None, centroid=None, area=0.0):
        view = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        s = self.scale
        view = cv2.resize(view, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)

        if self.calib.axis_ready():
            poly = (self.calib.band_polygon() * s).astype(np.int32)
            cv2.polylines(view, [poly], True, (255, 200, 0), 1)
            p1 = tuple((np.array(self.calib.p1) * s).astype(int))
            p2 = tuple((np.array(self.calib.p2) * s).astype(int))
            cv2.line(view, p1, p2, (0, 200, 255), 1)
            cv2.circle(view, p1, 5, (0, 200, 255), -1)
            cv2.circle(view, p2, 5, (0, 200, 255), -1)
        elif self.calib.p1 is not None:
            p1 = tuple((np.array(self.calib.p1) * s).astype(int))
            cv2.circle(view, p1, 5, (0, 200, 255), -1)

        for pt in getattr(self, "sample_points", []):
            cv2.circle(view, tuple((np.array(pt) * s).astype(int)), 4, (255, 0, 255), 1)

        if centroid is not None:
            c = tuple((np.array(centroid) * s).astype(int))
            cv2.drawMarker(view, c, (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
            t, perp = self.calib.project(centroid) if self.calib.axis_ready() else (0, 0)
            cv2.putText(view, f"pos {t * self.calib.beam_length_cm:5.1f} cm  perp {perp:4.1f}px  area {area:.0f}",
                        (8, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        header = f"[{STEP_NAMES[self.step]}]  {status}"
        cv2.rectangle(view, (0, 0), (view.shape[1], 46), (0, 0, 0), -1)
        cv2.putText(view, header, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(view, self.message, (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        if mask is not None:
            m = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            m = cv2.resize(m, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
            view = np.hstack([view, m])

        cv2.imshow(WINDOW, view)

    # input -----------------------------------------------------------------

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        fx, fy = x / self.scale, y / self.scale
        if self.step == STEP_AXIS:
            if self.calib.p1 is None:
                self.calib.p1 = (fx, fy)
                self.message = "now click the RIGHT end of the beam"
            elif self.calib.p2 is None:
                self.calib.p2 = (fx, fy)
                self.message = "axis set -- +/- adjusts band width, n to continue"
        elif self.step == STEP_COLOUR and self.last_frame is not None:
            self.sample_colour(int(fx), int(fy))

    def sample_colour(self, x, y):
        h = SAMPLE_PATCH // 2
        frame = self.last_frame
        patch = frame[max(0, y - h):y + h + 1, max(0, x - h):x + h + 1]
        if patch.size == 0:
            return
        hsv = cv2.cvtColor(patch, cv2.COLOR_RGB2HSV).reshape(-1, 3)
        self.calib.colour_samples.append(hsv)
        self.sample_points.append((x, y))
        self.calib.hsv = hsv_range_from_samples(self.calib.colour_samples)
        n = len(self.calib.colour_samples)
        r = self.calib.hsv
        self.message = (f"{n} samples  H {r['h_min']}-{r['h_max']}  S>={r['s_min']}  V>={r['v_min']}"
                        + ("   -- n to continue" if n >= MIN_COLOUR_SAMPLES else f"   -- need {MIN_COLOUR_SAMPLES}"))

    # steps -----------------------------------------------------------------

    def enter_step(self, step):
        self.step = step
        if step == STEP_EXPOSURE:
            self.message = "measuring..."
        elif step == STEP_AXIS:
            self.calib.p1 = self.calib.p2 = None
            self.message = "click the LEFT end of the beam"
        elif step == STEP_COLOUR:
            self.calib.colour_samples = []
            self.calib.hsv = None
            self.sample_points = []
            self.message = f"click the ball {MIN_COLOUR_SAMPLES}+ times (centre, edge, shaded side)"
        elif step == STEP_VALIDATE:
            self.validate_started = time.perf_counter()
            self.validate_stats = {"frames": 0, "detected": 0, "t": [], "area": [], "perp": []}
            self.message = f"roll the ball END TO END for {VALIDATE_SECONDS}s (space to finish early)"
        elif step == STEP_DONE:
            self.message = "s to save, q to quit without saving"

    def run_exposure_step(self, region_mask):
        def show(frame, status):
            self.show(frame, status)
            cv2.waitKey(1)
        mean_v = normalize_exposure(self.cam, self.calib, region_mask, show)
        self.message = (f"pinned: exposure={self.calib.exposure} gain={self.calib.gain} "
                        f"mean V={mean_v:.0f} -- n to continue, r to redo")

    def finish_validation(self):
        st = self.validate_stats
        frames = max(1, st["frames"])
        rate = st["detected"] / frames
        t = np.array(st["t"]) if st["t"] else np.array([0.0])
        area = np.array(st["area"]) if st["area"] else np.array([0.0])
        perp = np.array(st["perp"]) if st["perp"] else np.array([0.0])
        cover = float(t.max() - t.min()) if t.size else 0.0
        area_ratio = float(area.min() / np.median(area)) if area.size and np.median(area) > 0 else 0.0

        checks = {
            "detection_rate": (rate, rate >= PASS_DETECTION_RATE, f">= {PASS_DETECTION_RATE}"),
            "range_covered": (cover, cover >= PASS_RANGE_COVER, f">= {PASS_RANGE_COVER}"),
            "min_area_ratio": (area_ratio, area_ratio >= PASS_MIN_AREA_RATIO, f">= {PASS_MIN_AREA_RATIO}"),
            "max_perp_px": (float(perp.max()), perp.max() <= self.calib.band_halfwidth,
                            f"<= band {self.calib.band_halfwidth}"),
        }
        passed = all(ok for _, ok, _ in checks.values())

        print("\n--- validation ---")
        for name, (value, ok, rule) in checks.items():
            print(f"  {name:<16} {value:7.3f}   {rule:<14} {'OK' if ok else 'FAIL'}")
        print(f"  result: {'PASS' if passed else 'FAIL'}\n")

        self.calib.validation = {
            "passed": passed,
            "frames": frames,
            "detection_rate": round(rate, 4),
            "range_covered": round(cover, 4),
            "area_median": float(np.median(area)),
            "area_min": float(area.min()),
            "max_perp_px": float(perp.max()),
        }
        self.enter_step(STEP_DONE)
        self.message = (("PASS" if passed else "FAIL") +
                        f"  rate {rate:.2f}  cover {cover:.2f}  area {area_ratio:.2f} -- s save, r redo, q quit")

    # main loop -------------------------------------------------------------

    def run(self):
        self.enter_step(STEP_EXPOSURE)
        self.run_exposure_step(None)

        while True:
            frame, _ = self.cam.read(timestamp=True)
            self.last_frame = frame
            band = self.calib.band_mask(frame.shape) if self.calib.axis_ready() else None

            centroid, area, mask = None, 0.0, None
            if self.calib.hsv is not None:
                centroid, area, mask = detect(frame, self.calib.hsv, band, self.kernel)

            if self.step == STEP_VALIDATE:
                st = self.validate_stats
                st["frames"] += 1
                if centroid is not None:
                    t, perp = self.calib.project(centroid)
                    st["detected"] += 1
                    st["t"].append(t)
                    st["area"].append(area)
                    st["perp"].append(perp)
                elapsed = time.perf_counter() - self.validate_started
                rate = st["detected"] / st["frames"]
                self.message = (f"{elapsed:4.1f}s  detect {rate:.2f}  "
                                f"cover {(max(st['t']) - min(st['t'])) if st['t'] else 0:.2f}  (space to finish)")
                if elapsed >= VALIDATE_SECONDS:
                    self.finish_validation()

            status = {
                STEP_EXPOSURE: "n next  r redo",
                STEP_AXIS: "click ends  +/- band  n next  b back",
                STEP_COLOUR: "click ball  n next  b back  r redo",
                STEP_VALIDATE: "rolling...  space finish",
                STEP_DONE: "s save  r redo  q quit",
            }[self.step]
            self.show(frame, status, mask, centroid, area)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                return False
            if key == ord("n"):
                self.advance()
            elif key == ord("b") and self.step > STEP_EXPOSURE:
                self.enter_step(self.step - 1)
            elif key == ord("r"):
                if self.step == STEP_EXPOSURE:
                    self.run_exposure_step(band)
                elif self.step == STEP_DONE:
                    self.enter_step(STEP_VALIDATE)
                else:
                    self.enter_step(self.step)
            elif key in (ord("+"), ord("=")) and self.step == STEP_AXIS:
                self.calib.band_halfwidth += 5
            elif key == ord("-") and self.step == STEP_AXIS and self.calib.band_halfwidth > 10:
                self.calib.band_halfwidth -= 5
            elif key == ord(" ") and self.step == STEP_VALIDATE:
                self.finish_validation()
            elif key == ord("s") and self.step == STEP_DONE:
                return True

    def advance(self):
        if self.step == STEP_EXPOSURE:
            self.enter_step(STEP_AXIS)
        elif self.step == STEP_AXIS:
            if not self.calib.axis_ready():
                self.message = "both ends are needed first"
                return
            # Re-pin exposure on the band now that we know where the beam is:
            # the room average and the beam's own brightness can differ a lot.
            band = self.calib.band_mask(self.last_frame.shape)
            self.enter_step(STEP_EXPOSURE)
            self.run_exposure_step(band)
            self.enter_step(STEP_COLOUR)
        elif self.step == STEP_COLOUR:
            if len(self.calib.colour_samples) < MIN_COLOUR_SAMPLES:
                self.message = f"need at least {MIN_COLOUR_SAMPLES} samples"
                return
            self.enter_step(STEP_VALIDATE)
        elif self.step == STEP_VALIDATE:
            self.finish_validation()


def open_camera(cam_id=0, fps=100):
    """The one place the capture mode is chosen, so wizard and tracker agree."""
    return Camera(cam_id, resolution=Camera.RES_SMALL, fps=fps, colour=True,
                  auto_gain=False, auto_whitebalance=False, gain=20, exposure=120)


def run(cam, beam_cm=45.0, scale=2, out_path=CALIB_PATH):
    """Drive the wizard on an already-open camera.

    Returns the Calibration on success (also written to out_path) or None if
    the user quit. The camera is left pinned at the exposure/gain the wizard
    settled on, so a caller can carry straight on with it.
    """
    calib = Calibration(beam_cm)
    wizard = Wizard(cam, calib, scale)
    ok = wizard.run()
    cv2.destroyWindow(WINDOW)
    if not ok:
        print("calibration not saved")
        return None
    with open(out_path, "w") as f:
        json.dump(calib.to_dict(), f, indent=2)
    print(f"saved {out_path}")
    return calib


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--fps", type=int, default=100)
    parser.add_argument("--scale", type=int, default=2)
    parser.add_argument("--beam-cm", type=float, default=45.0, help="real length between the two clicked ends")
    parser.add_argument("--out", default=CALIB_PATH)
    args = parser.parse_args()

    cam = open_camera(args.id, args.fps)
    try:
        run(cam, args.beam_cm, args.scale, args.out)
    finally:
        cam.end()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
