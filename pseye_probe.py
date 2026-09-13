"""PS3 Eye capture probe via pseyepy -- Faz A3 + B1..B4 of the vision route.

Talks to the camera through libusb rather than DirectShow, which is what
makes the native high-rate modes and the exposure/gain registers reachable
at all. The DirectShow route could not set resolution or exposure and stalled
for a full second at a time, so the numbers here are the ones that decide
whether 125 Hz is usable.

Reports the same things as camera_probe.py: whether the requested mode is
really applied, what the frame interval distribution looks like out in the
tail, and whether exposure and gain can be pinned.

Usage:
    python pseye_probe.py --list
    python pseye_probe.py --fps 125
    python pseye_probe.py --fps 125 --exposure 40 --gain 20 --frames 1000
"""

import argparse
import time

import numpy as np
from pseyepy import Camera, cam_count

RESOLUTIONS = {
    "small": (Camera.RES_SMALL, "320x240"),
    "large": (Camera.RES_LARGE, "640x480"),
}

# The sensor only runs at these rates; asking for anything else gets silently
# rounded, which is why the empirical measurement below is the real check.
NATIVE_RATES = {
    "small": (30, 40, 50, 60, 75, 100, 125),
    "large": (15, 30, 40, 50, 60),
}


def report_settings(cam, label):
    """Read back whatever the library exposes about the current settings."""
    print(f"  {label}:")
    for name in ("exposure", "gain", "auto_gain", "auto_whitebalance"):
        value = getattr(cam, name, None)
        print(f"    {name:<18} {value if value is not None else '(not exposed)'}")


def measure(cam, frames, warmup):
    """Time successive reads, from both the wall clock and the camera's own stamps.

    Two clocks on purpose: the wall clock covers everything the control loop
    would wait for, while the camera timestamps show whether a gap came from
    the sensor or from the host being busy. If the wall clock stutters and the
    stamps do not, the fault is on this side of the USB cable.
    """
    print(f"--- capture timing (B1..B3): {warmup} warm-up + {frames} frames ---")

    for _ in range(warmup):
        cam.read()

    host_intervals = []
    read_times = []
    stamps = []
    failures = 0
    prev = None
    start = time.perf_counter()

    for _ in range(frames):
        t0 = time.perf_counter()
        try:
            frame, stamp = cam.read(timestamp=True)
        except Exception as exc:  # a dropped frame surfaces as a read error
            failures += 1
            print(f"  read failed: {exc}")
            continue
        t1 = time.perf_counter()
        if frame is None:
            failures += 1
            continue
        read_times.append((t1 - t0) * 1000.0)
        stamps.append(stamp)
        if prev is not None:
            host_intervals.append((t1 - prev) * 1000.0)
        prev = t1

    elapsed = time.perf_counter() - start
    if not host_intervals:
        print("  no frames captured\n")
        return

    hi = np.array(host_intervals)
    rt = np.array(read_times)
    effective = len(read_times) / elapsed

    print(f"  effective rate : {effective:.1f} fps over {elapsed:.2f} s")
    print(f"  failed reads   : {failures}")
    print(f"  frame shape    : {frame.shape}  dtype {frame.dtype}")
    print()
    print(f"  {'':<18}{'mean':>8}{'p50':>8}{'p95':>8}{'p99':>8}{'max':>8}")
    print(f"  {'host interval (ms)':<18}{hi.mean():>8.2f}{np.percentile(hi, 50):>8.2f}"
          f"{np.percentile(hi, 95):>8.2f}{np.percentile(hi, 99):>8.2f}{hi.max():>8.2f}")
    print(f"  {'read() (ms)':<18}{rt.mean():>8.2f}{np.percentile(rt, 50):>8.2f}"
          f"{np.percentile(rt, 95):>8.2f}{np.percentile(rt, 99):>8.2f}{rt.max():>8.2f}")

    stamp_arr = np.array(stamps, dtype=float)
    if stamp_arr.size > 2 and np.all(np.diff(stamp_arr) >= 0):
        cam_iv = np.diff(stamp_arr) * 1000.0
        print(f"  {'cam interval (ms)':<18}{cam_iv.mean():>8.2f}{np.percentile(cam_iv, 50):>8.2f}"
              f"{np.percentile(cam_iv, 95):>8.2f}{np.percentile(cam_iv, 99):>8.2f}{cam_iv.max():>8.2f}")

    return hi


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="count connected cameras and exit")
    parser.add_argument("--id", type=int, default=0, help="camera id")
    parser.add_argument("--res", choices=RESOLUTIONS, default="small")
    parser.add_argument("--fps", type=int, default=125)
    parser.add_argument("--colour", action="store_true", help="RGB frames instead of greyscale")
    parser.add_argument("--exposure", type=int, default=40, help="0-255, fixed")
    parser.add_argument("--gain", type=int, default=20, help="0-63, fixed")
    parser.add_argument("--frames", type=int, default=1000)
    parser.add_argument("--warmup", type=int, default=50)
    args = parser.parse_args()

    count = cam_count()
    print(f"cameras detected: {count}")
    if args.list:
        return
    if count == 0:
        print("nothing to open -- is libusbK assigned to Interface 0 of the PS3 Eye?")
        return

    res_const, res_label = RESOLUTIONS[args.res]
    if args.fps not in NATIVE_RATES[args.res]:
        print(f"note: {args.fps} is not one of the native {args.res} rates "
              f"{NATIVE_RATES[args.res]}, the sensor will pick its nearest")

    print(f"\n--- requested mode (A3) ---")
    print(f"  {res_label} @ {args.fps} fps, {'colour' if args.colour else 'greyscale'}")

    # Auto gain is turned off and both exposure and gain pinned: a sensor left
    # to adapt stretches its integration time in dim light, which quietly caps
    # the frame rate and makes the latency depend on the room lighting.
    cam = Camera(args.id,
                 resolution=res_const,
                 fps=args.fps,
                 colour=args.colour,
                 auto_gain=False,
                 auto_whitebalance=False,
                 gain=args.gain,
                 exposure=args.exposure)

    try:
        print("\n--- exposure / gain control (B4) ---")
        report_settings(cam, "after pinning")

        hi = measure(cam, args.frames, args.warmup)

        if hi is not None:
            budget = 1000.0 / args.fps
            late = int((hi > budget * 1.5).sum())
            print()
            print(f"  frame budget   : {budget:.2f} ms at {args.fps} fps")
            print(f"  late intervals : {late} of {len(hi)} ({100.0 * late / len(hi):.1f}%) over 1.5x budget")
            print()
    finally:
        cam.end()


if __name__ == "__main__":
    main()
