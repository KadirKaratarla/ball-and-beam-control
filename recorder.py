"""Session recorder: every TELEM frame (and the sent positions) to CSV."""

import csv
import os
import time

import protocol as P


class Recorder:
    def __init__(self, directory="logs"):
        self.directory = directory
        self.file = None
        self.writer = None
        self.path = None
        self.rows = 0
        self.t0 = None

    @property
    def active(self):
        return self.file is not None

    def start(self):
        os.makedirs(self.directory, exist_ok=True)
        self.path = os.path.join(self.directory, time.strftime("%Y-%m-%d_%H%M%S") + ".csv")
        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["t_pc_s", "kind", "t_ms", "x_cm", "x_set_cm", "P", "I", "D", "theta_deg", "phi_deg",
                              "enc_counts", "follow_counts", "lag_counts", "state", "fault", "ball_valid",
                              "link_stale", "driver_on", "last_seq", "loop_us", "cmd_vel"])
        self.rows = 0
        self.t0 = time.perf_counter()
        return self.path

    def stop(self):
        if self.file:
            self.file.close()
        self.file = None
        self.writer = None

    def telem(self, t: P.Telem):
        if not self.writer:
            return
        self.writer.writerow([f"{time.perf_counter() - self.t0:.4f}", "T", t.t_ms, t.x_0p1mm / 100, t.x_set_0p1mm / 100,
                              t.p / 100, t.i / 100, t.d / 100, t.theta / 100, t.phi_0p1deg / 10,
                              t.enc_counts, t.follow_0p1 / 10, t.lag_counts,
                              P.STATE_NAMES.get(t.state, t.state), P.FAULT_NAMES.get(t.fault, t.fault),
                              int(bool(t.flags & P.TF_BALL_VALID)), int(bool(t.flags & P.TF_LINK_STALE)),
                              int(bool(t.flags & P.TF_DRIVER_ON)), t.last_seq, t.loop_us, t.cmd_vel_10 * 10])
        self.rows += 1

    def sent(self, pos_cm, valid, t_send):
        if not self.writer:
            return
        self.writer.writerow([f"{t_send - self.t0:.4f}", "P", "", f"{pos_cm:.3f}", "", "", "", "", "", "",
                              "", "", "", "", "", int(valid), "", "", "", "", ""])
