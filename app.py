"""Ball & Beam GUI (Faz 5).

    python app.py                 camera: calibration wizard, then tracking
    python app.py --synthetic     no camera, triangle wave (link/GUI check)
    python app.py --skip-calibration   reuse calibration.json (bench only)

Processes: the camera + ESP link run in a child process (link_process.py;
pseyepy blocks with the GIL held, so they cannot share the GUI's
interpreter); the GUI polls the process queues at 30 Hz.
"""

import argparse
import multiprocessing as mp
import sys

from PySide6.QtWidgets import QApplication

from link_process import LinkProxy
from ui_main import MainWindow


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic", action="store_true")
    ap.add_argument("--skip-calibration", action="store_true", help="DEV ONLY")
    ap.add_argument("--port", default=None)
    ap.add_argument("--beam-cm", type=float, default=45.0)
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--fps", type=int, default=100)
    ap.add_argument("--scale", type=int, default=2)
    args = ap.parse_args()

    import calibrate
    app = QApplication(sys.argv)
    proxy = LinkProxy(dict(port=args.port, synthetic=args.synthetic, beam_cm=args.beam_cm, id=args.id,
                           fps=args.fps, scale=args.scale, skip_calibration=args.skip_calibration,
                           calibration_path=calibrate.CALIB_PATH))
    win = MainWindow(proxy, proxy, beam_cm=args.beam_cm)
    win.show()
    proxy.start()
    rc = app.exec()
    proxy.stop()
    sys.exit(rc)


if __name__ == "__main__":
    mp.freeze_support()
    main()
