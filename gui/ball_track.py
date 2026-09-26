"""Live view and colour tuning for the ball tracker -- Faz C of the vision route.

Two jobs in one window: aim the camera at the beam, and find the HSV range
that separates the ball from everything else. Both are done by eye, so the
mask and the detection are drawn next to the raw frame while the camera runs
at its real rate.

Detection is the usual chain: convert to HSV so the threshold survives
lighting changes, cut a binary mask with the tuned range, open it to drop
speckle, take the largest contour, and read its centroid as the ball
position. Per-frame cost is measured because the whole chain has to fit in
the 10 ms frame budget with room to spare.

Keys:
    q / esc   quit
    s         save the current HSV range to hsv_config.json
    r         reset the range to the orange default
    space     freeze / unfreeze (useful while positioning the camera)

Usage:
    python ball_track.py
    python ball_track.py --fps 100 --scale 3
"""

import argparse
import json
import os
import time
from collections import deque

import cv2
import numpy as np
from pseyepy import Camera

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hsv_config.json")

# A ping-pong ball's orange sits low in the hue circle. Saturation and value
# floors keep pale walls and dark corners out; both ceilings stay open.
DEFAULT_HSV = {
    "h_min": 5, "h_max": 25,
    "s_min": 120, "s_max": 255,
    "v_min": 80, "v_max": 255,
    "min_area": 30,
}

WINDOW = "ball tracker"
TRACKBARS = "controls"


def load_config():
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH) as f:
                saved = json.load(f)
            merged = dict(DEFAULT_HSV)
            merged.update({k: v for k, v in saved.items() if k in DEFAULT_HSV})
            print(f"loaded HSV range from {CONFIG_PATH}")
            return merged
        except (OSError, ValueError) as exc:
            print(f"could not read {CONFIG_PATH} ({exc}), using defaults")
    return dict(DEFAULT_HSV)


def save_config(values):
    with open(CONFIG_PATH, "w") as f:
        json.dump(values, f, indent=2)
    print(f"saved HSV range to {CONFIG_PATH}")


def build_trackbars(values):
    cv2.namedWindow(TRACKBARS, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(TRACKBARS, 420, 260)
    # Hue is 0..179 in OpenCV rather than 0..359, so the ceiling differs.
    cv2.createTrackbar("H min", TRACKBARS, values["h_min"], 179, lambda v: None)
    cv2.createTrackbar("H max", TRACKBARS, values["h_max"], 179, lambda v: None)
    cv2.createTrackbar("S min", TRACKBARS, values["s_min"], 255, lambda v: None)
    cv2.createTrackbar("S max", TRACKBARS, values["s_max"], 255, lambda v: None)
    cv2.createTrackbar("V min", TRACKBARS, values["v_min"], 255, lambda v: None)
    cv2.createTrackbar("V max", TRACKBARS, values["v_max"], 255, lambda v: None)
    cv2.createTrackbar("min area", TRACKBARS, values["min_area"], 2000, lambda v: None)


def read_trackbars():
    return {
        "h_min": cv2.getTrackbarPos("H min", TRACKBARS),
        "h_max": cv2.getTrackbarPos("H max", TRACKBARS),
        "s_min": cv2.getTrackbarPos("S min", TRACKBARS),
        "s_max": cv2.getTrackbarPos("S max", TRACKBARS),
        "v_min": cv2.getTrackbarPos("V min", TRACKBARS),
        "v_max": cv2.getTrackbarPos("V max", TRACKBARS),
        "min_area": cv2.getTrackbarPos("min area", TRACKBARS),
    }


def set_trackbars(values):
    cv2.setTrackbarPos("H min", TRACKBARS, values["h_min"])
    cv2.setTrackbarPos("H max", TRACKBARS, values["h_max"])
    cv2.setTrackbarPos("S min", TRACKBARS, values["s_min"])
    cv2.setTrackbarPos("S max", TRACKBARS, values["s_max"])
    cv2.setTrackbarPos("V min", TRACKBARS, values["v_min"])
    cv2.setTrackbarPos("V max", TRACKBARS, values["v_max"])
    cv2.setTrackbarPos("min area", TRACKBARS, values["min_area"])


def detect(frame_rgb, values, kernel):
    """Find the ball and return (centroid, radius, area, mask).

    Returns centroid None when nothing passes the area floor, which is what
    the control loop would see as "no reading this frame".
    """
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    lower = np.array([values["h_min"], values["s_min"], values["v_min"]])
    upper = np.array([values["h_max"], values["s_max"], values["v_max"]])
    mask = cv2.inRange(hsv, lower, upper)

    # Opening = erode then dilate: speckle smaller than the kernel disappears
    # while the ball keeps its size.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0, 0.0, mask

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < values["min_area"]:
        return None, 0.0, area, mask

    # Moments give a sub-pixel centre, which matters here: the ball is only a
    # few dozen pixels across, so rounding to whole pixels would quantise the
    # position far more coarsely than the sensor actually resolves.
    m = cv2.moments(largest)
    if m["m00"] == 0:
        return None, 0.0, area, mask
    cx = m["m10"] / m["m00"]
    cy = m["m01"] / m["m00"]
    _, radius = cv2.minEnclosingCircle(largest)
    return (cx, cy), radius, area, mask


def annotate(view, centroid, radius, area, scale, stats):
    if centroid is not None:
        cx, cy = centroid
        cv2.circle(view, (int(cx * scale), int(cy * scale)), int(radius * scale), (0, 255, 0), 2)
        cv2.drawMarker(view, (int(cx * scale), int(cy * scale)), (0, 0, 255),
                       cv2.MARKER_CROSS, 14, 2)
        label = f"x={cx:6.1f}  y={cy:6.1f}  area={area:.0f}"
        colour = (0, 255, 0)
    else:
        label = "no ball"
        colour = (0, 0, 255)

    cv2.putText(view, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
    cv2.putText(view, stats, (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--fps", type=int, default=100, help="100 measured clean; 125 drops frames")
    parser.add_argument("--exposure", type=int, default=40, help="0-255; timing is independent of this at 100 fps")
    parser.add_argument("--gain", type=int, default=20, help="0-63")
    parser.add_argument("--scale", type=int, default=3, help="display magnification for the 320x240 frame")
    args = parser.parse_args()

    values = load_config()

    cam = Camera(args.id,
                 resolution=Camera.RES_SMALL,
                 fps=args.fps,
                 colour=True,
                 auto_gain=False,
                 auto_whitebalance=False,
                 gain=args.gain,
                 exposure=args.exposure)

    build_trackbars(values)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    proc_times = deque(maxlen=200)
    frame_times = deque(maxlen=200)
    prev = None
    frozen = False
    last_frame = None

    print("\nq/esc quit   s save   r reset   space freeze\n")

    try:
        while True:
            if not frozen:
                frame, _ = cam.read(timestamp=True)
                last_frame = frame
            frame = last_frame
            if frame is None:
                continue

            now = time.perf_counter()
            if prev is not None and not frozen:
                frame_times.append((now - prev) * 1000.0)
            prev = now

            values = read_trackbars()

            t0 = time.perf_counter()
            centroid, radius, area, mask = detect(frame, values, kernel)
            proc_times.append((time.perf_counter() - t0) * 1000.0)

            view = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            view = cv2.resize(view, None, fx=args.scale, fy=args.scale,
                              interpolation=cv2.INTER_NEAREST)

            fps = 1000.0 / np.mean(frame_times) if frame_times else 0.0
            stats = f"{fps:5.1f} fps   proc {np.mean(proc_times):4.2f} ms" + ("   FROZEN" if frozen else "")
            annotate(view, centroid, radius, area, args.scale, stats)

            mask_view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_view = cv2.resize(mask_view, None, fx=args.scale, fy=args.scale,
                                   interpolation=cv2.INTER_NEAREST)
            cv2.putText(mask_view, "mask", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)

            cv2.imshow(WINDOW, np.hstack([view, mask_view]))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                save_config(values)
            if key == ord("r"):
                set_trackbars(DEFAULT_HSV)
            if key == ord(" "):
                frozen = not frozen
    finally:
        cam.end()
        cv2.destroyAllWindows()

        if proc_times:
            pt = np.array(proc_times)
            budget = 1000.0 / args.fps
            print(f"\nprocessing per frame: mean {pt.mean():.2f} ms, "
                  f"p95 {np.percentile(pt, 95):.2f} ms, max {pt.max():.2f} ms")
            print(f"frame budget at {args.fps} fps: {budget:.2f} ms "
                  f"-- detection uses {100.0 * pt.mean() / budget:.1f}% of it")


if __name__ == "__main__":
    main()
