"""Camera capture probe -- Faz A3 + B1..B4 of the vision route.

Answers the questions that decide whether a camera can carry the ball
position at a given rate:

  * which capture devices exist and what they open at
  * whether a requested mode (e.g. 320x240 @ 125 fps) is actually applied
  * what the frame interval really looks like -- not just the mean, but the
    p95/p99/max tail, which is what "runs at 125 Hz without problems"
    actually means
  * whether exposure and gain can be pinned manually, since auto exposure
    stretches the frame time in dim light and quietly destroys the rate

Usage:
    python camera_probe.py --list
    python camera_probe.py --index 0
    python camera_probe.py --index 1 --width 320 --height 240 --fps 125
"""

import argparse
import time

import cv2
import numpy as np

# DirectShow generally honours resolution/fps/exposure requests on Windows
# webcams where the Media Foundation backend silently ignores them.
BACKENDS = {
    "dshow": cv2.CAP_DSHOW,
    "msmf": cv2.CAP_MSMF,
    "any": cv2.CAP_ANY,
}

# DirectShow reports auto exposure as a two-state property with these values
# rather than a plain 0/1.
AUTO_EXPOSURE_MANUAL = 0.25
AUTO_EXPOSURE_AUTO = 0.75


def open_capture(index, backend):
    cap = cv2.VideoCapture(index, BACKENDS[backend])
    return cap if cap.isOpened() else None


def list_cameras(backend, max_index=6):
    print(f"Scanning indices 0..{max_index - 1} on the {backend} backend\n")
    found = 0
    for index in range(max_index):
        cap = open_capture(index, backend)
        if cap is None:
            continue
        ok, frame = cap.read()
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        shape = f"{frame.shape[1]}x{frame.shape[0]}" if ok and frame is not None else "no frame"
        print(f"  [{index}] opens at {width}x{height} @ {fps:g} fps, first read: {shape}")
        cap.release()
        found += 1
    if not found:
        print("  no capture devices responded")
    print()


def apply_mode(cap, width, height, fps, fourcc):
    """Request a mode and report what the driver actually accepted.

    Every property is set through the driver, which is free to ignore or
    substitute values, so the requested numbers are treated as a proposal and
    the readback as the truth.
    """
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)

    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join(chr((actual_fourcc >> (8 * i)) & 0xFF) for i in range(4))
    return {
        "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "fps": cap.get(cv2.CAP_PROP_FPS),
        "fourcc": fourcc_str.strip("\x00"),
    }


def probe_exposure(cap):
    """Check whether exposure and gain can be pinned, and report the result.

    A rate target only holds if the sensor is not free to lengthen its
    integration time, so this is a gating check rather than a nicety.
    """
    print("--- exposure / gain control (B4) ---")

    before = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, AUTO_EXPOSURE_MANUAL)
    after = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
    manual_ok = after != before or after == AUTO_EXPOSURE_MANUAL
    print(f"  auto exposure : {before} -> {after}  "
          f"{'manual mode accepted' if manual_ok else 'DID NOT TAKE'}")

    for name, prop in (("exposure", cv2.CAP_PROP_EXPOSURE), ("gain", cv2.CAP_PROP_GAIN)):
        current = cap.get(prop)
        probe = current - 1 if current else -6
        cap.set(prop, probe)
        readback = cap.get(prop)
        cap.set(prop, current)
        print(f"  {name:<13}: {current} -> set {probe} -> reads {readback}  "
              f"{'writable' if readback != current else 'ignored / read-only'}")
    print()


def measure(cap, frames, warmup):
    """Time the gaps between successive frames.

    The mean says almost nothing on its own: a stream that averages 8 ms but
    stalls for 60 ms every second is not running at 125 Hz in any way the
    control loop cares about. So the tail percentiles and the worst case are
    reported alongside, plus how long each read() call itself blocks.
    """
    print(f"--- capture timing (B1..B3): {warmup} warm-up + {frames} frames ---")

    for _ in range(warmup):
        cap.read()

    intervals = []
    read_times = []
    failures = 0
    prev = None
    start = time.perf_counter()

    for _ in range(frames):
        t0 = time.perf_counter()
        ok, frame = cap.read()
        t1 = time.perf_counter()
        if not ok or frame is None:
            failures += 1
            continue
        read_times.append((t1 - t0) * 1000.0)
        if prev is not None:
            intervals.append((t1 - prev) * 1000.0)
        prev = t1

    elapsed = time.perf_counter() - start
    if not intervals:
        print("  no frames captured\n")
        return

    iv = np.array(intervals)
    rt = np.array(read_times)
    effective = len(read_times) / elapsed

    print(f"  effective rate : {effective:.1f} fps over {elapsed:.2f} s")
    print(f"  failed reads   : {failures}")
    print()
    print(f"  {'':<16}{'mean':>8}{'p50':>8}{'p95':>8}{'p99':>8}{'max':>8}")
    print(f"  {'interval (ms)':<16}{iv.mean():>8.2f}{np.percentile(iv, 50):>8.2f}"
          f"{np.percentile(iv, 95):>8.2f}{np.percentile(iv, 99):>8.2f}{iv.max():>8.2f}")
    print(f"  {'read() (ms)':<16}{rt.mean():>8.2f}{np.percentile(rt, 50):>8.2f}"
          f"{np.percentile(rt, 95):>8.2f}{np.percentile(rt, 99):>8.2f}{rt.max():>8.2f}")

    target = cap.get(cv2.CAP_PROP_FPS)
    if target > 0:
        budget = 1000.0 / target
        # Anything past 1.5 frame times is a frame the sensor owed us and did
        # not deliver, which is what a stale position looks like downstream.
        late = int((iv > budget * 1.5).sum())
        print()
        print(f"  frame budget   : {budget:.2f} ms at {target:g} fps")
        print(f"  late intervals : {late} of {len(iv)} ({100.0 * late / len(iv):.1f}%) over 1.5x budget")
    print()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="enumerate capture devices and exit")
    parser.add_argument("--index", type=int, default=0, help="capture device index")
    parser.add_argument("--backend", choices=BACKENDS, default="dshow")
    parser.add_argument("--width", type=int, default=0)
    parser.add_argument("--height", type=int, default=0)
    parser.add_argument("--fps", type=float, default=0)
    parser.add_argument("--fourcc", default="", help="e.g. MJPG, YUY2 -- some high rate modes need a specific one")
    parser.add_argument("--frames", type=int, default=500)
    parser.add_argument("--warmup", type=int, default=30)
    args = parser.parse_args()

    if args.list:
        list_cameras(args.backend)
        return

    cap = open_capture(args.index, args.backend)
    if cap is None:
        print(f"could not open camera index {args.index} on the {args.backend} backend")
        return

    try:
        print(f"--- requested mode (A3) on index {args.index}, {args.backend} backend ---")
        requested = f"{args.width or '?'}x{args.height or '?'} @ {args.fps or '?'} fps"
        actual = apply_mode(cap, args.width, args.height, args.fps, args.fourcc)
        print(f"  requested : {requested} {args.fourcc}")
        print(f"  actual    : {actual['width']}x{actual['height']} @ {actual['fps']:g} fps {actual['fourcc']}")
        if args.fps and abs(actual["fps"] - args.fps) > 1:
            print("  NOTE: the driver did not accept the requested rate")
        print()

        probe_exposure(cap)
        measure(cap, args.frames, args.warmup)
    finally:
        cap.release()


if __name__ == "__main__":
    main()
