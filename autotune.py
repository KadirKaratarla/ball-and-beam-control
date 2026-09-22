"""Plant identification for the ball-and-beam controller (Faz 6).

NOT PART OF THE APPLICATION. The tuner measured the plant (K = 605
cm/s^2/rad, delay 45 ms, camera noise 0.28 mm, dead band 0.15 deg) and
its pole-placement proposal verified *worse* than the hand-tuned gains,
which the user then kept: Kp 0.74, Ki 0.15, Kd 0.34, tau_D 0.15. So
app.py has no autotune button -- this file stays as a bench tool for
re-measuring the plant after a mechanical change (new ball, new rail,
different linkage). Run it with the camera loop feeding positions, e.g.
the runner in the session scratchpad, or wire pump_cb to your own loop.

Instead of relay/Ziegler-Nichols (which is not defined for a double
integrator and would swing the ball across a 45 cm beam), this measures
the plant with short, bounded open-loop experiments and computes the
gains from what it finds:

    noise      2 s level    -> camera sigma, natural drift
    dead band  theta steps  -> the smallest tilt that starts the ball
    gain K     +-theta pulse pairs, both directions
                            -> K = dv / (theta_rad * dt)  [cm/s^2 / rad]
    delay      same pulses  -> command to first acceleration

Every experiment aborts (beam level, MODE RUN restored) if the ball
leaves a window around its start, the link or the ball goes missing, or
the firmware reports a fault. The firmware also levels the beam by
itself if THETA stops arriving for 500 ms.

Run standalone for the measurement report:

    python autotune.py --skip-calibration
"""

import argparse
import math
import statistics
import sys
import time

import protocol as P


class Aborted(Exception):
    pass


class Measurement:
    """Results of one identification run."""

    def __init__(self):
        self.sigma_cm = None        # camera noise, 1 sigma
        self.drift_cm_s = None      # ball creep at theta = 0
        self.dead_band_deg = None   # smallest tilt that moves the ball
        self.level_offset_deg = None  # tilt at which the ball does not accelerate
        self.a_pos = None           # measured acceleration at +theta (cm/s^2)
        self.a_neg = None           # ... at -theta
        self.k_pos = None           # cm/s^2 per rad, tilt toward p2
        self.k_neg = None           # ... toward p1
        self.delay_s = None
        self.pulses = []            # per-pulse detail for the report

    @property
    def k(self):
        ks = [k for k in (self.k_pos, self.k_neg) if k]
        return sum(ks) / len(ks) if ks else None

    def asymmetry(self):
        if self.k_pos and self.k_neg:
            return abs(self.k_pos - self.k_neg) / self.k
        return None


class Autotuner:
    """Drives the experiments through a link object (EspLink or LinkProxy).

    The caller pumps: every call to feed() hands over the frames drained
    from the link; step() returns a status string or None when done.
    Standalone use just calls run().
    """

    THETA_PULSE_DEG = 1.0
    PULSE_MS = 420          # crank needs ~60 ms to reach the angle; fit after that
    FIT_SKIP_S = 0.14       # ignore while the crank is still travelling
    VEL_WINDOW_S = 0.06     # sliding window for the velocity estimate
    SAFE_WINDOW_CM = 9.0
    CENTRE_CM = 22.5
    CENTRE_TOL_CM = 1.5
    CENTRE_VEL_CM_S = 0.6   # the ball must really be at rest, not just near
    CENTRE_SETTLE_S = 0.8
    CENTRE_TIMEOUT_S = 14.0
    D_NOISE_BUDGET_DEG = 0.05   # how much beam-angle jitter the D term may add
    D_NOISE_CONST = 2.16        # measured on this rig (see gains())
    CAMERA_DELAY_S = 0.02       # exposure + tracking + link (Faz D)

    def __init__(self, link, on_status=print, pump_cb=None):
        self.link = link
        self.on_status = on_status
        # Called continuously while an angle is held: the host uses it to
        # read a camera frame and send the position packet (the firmware
        # levels the beam if those stop).
        self.pump_cb = pump_cb or (lambda: time.sleep(0.003))
        self.m = Measurement()
        self.samples = []   # (t, x_cm, valid, state, fault, theta_deg, phi_deg)
        self.t0 = time.perf_counter()
        self.theta = 0.0
        self.x_start = None
        self._next_theta = 0.0
        self._next_pos = 0.0

    # --- plumbing ---------------------------------------------------------
    def _drain(self):
        for ptype, msg in self.link.drain():
            if ptype == P.T_TELEM:
                t = time.perf_counter() - self.t0
                valid = bool(msg.flags & P.TF_BALL_VALID)
                self.samples.append((t, msg.x_0p1mm / 100.0, valid, msg.state, msg.fault,
                                     msg.theta / 100.0, msg.phi_0p1deg / 10.0))
                if msg.fault:
                    raise Aborted(f"firmware faulted: {P.FAULT_NAMES.get(msg.fault, msg.fault)}")
                if msg.state not in (2, 3, 6):  # LEVEL, RUN, OPEN
                    raise Aborted(f"beklenmeyen durum {P.STATE_NAMES.get(msg.state, msg.state)}")
            elif ptype == P.T_ACK and msg.cmd_type != P.T_PING and msg.result != P.ACK_OK:
                raise Aborted(f"command 0x{msg.cmd_type:02X} rejected: {P.ACK_NAMES.get(msg.result)}")

    def _hold(self, seconds, theta=None, guard=True):
        """Keep THETA alive at 20 Hz for `seconds`, collecting telemetry."""
        if theta is not None:
            self.theta = theta
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            now = time.perf_counter()
            if now >= self._next_theta:
                self.link.set_theta(self.theta)
                self._next_theta = now + 0.05
            self._drain()
            if guard:
                self._check_window()
            self.pump_cb()

    def _check_window(self):
        x = self.last_x()
        if x is None:
            return
        if self.x_start is not None and abs(x - self.x_start) > self.SAFE_WINDOW_CM:
            raise Aborted(f"ball left the safe window ({x:.1f} cm, started {self.x_start:.1f})")
        if not 2.0 < x < 43.0:
            raise Aborted(f"ball near the end of the beam ({x:.1f} cm)")

    def last_x(self):
        for t, x, valid, *_ in reversed(self.samples[-40:]):
            if valid:
                return x
        return None

    def wait_ready(self, timeout=25.0):
        """Wait until the firmware has engaged and is holding the beam
        (LEVEL or RUN). Straight after a reset it spends 3 s in WAIT."""
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            for ptype, msg in self.link.drain():
                if ptype == P.T_TELEM:
                    t = time.perf_counter() - self.t0
                    self.samples.append((t, msg.x_0p1mm / 100.0, bool(msg.flags & P.TF_BALL_VALID),
                                         msg.state, msg.fault, msg.theta / 100.0, msg.phi_0p1deg / 10.0))
                    if msg.fault:
                        raise Aborted(f"firmware arızada: {P.FAULT_NAMES.get(msg.fault, msg.fault)}")
                    if msg.state in (2, 3):
                        return
            self.pump_cb()
        raise Aborted("firmware hazır duruma gelmedi (LEVEL/RUN)")

    def _require_ball(self, seconds=1.0):
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            self._drain()
            self.pump_cb()
        if self.last_x() is None:
            raise Aborted("no ball on the beam (the camera sees nothing)")

    def _window(self, t_from, t_to):
        return [(t, x) for t, x, valid, *_ in self.samples if valid and t_from <= t <= t_to]

    @staticmethod
    def _velocity(points):
        """Least-squares slope (cm/s) of x(t) over a short window."""
        if len(points) < 4:
            return None
        ts = [p[0] for p in points]
        xs = [p[1] for p in points]
        tm = sum(ts) / len(ts)
        xm = sum(xs) / len(xs)
        den = sum((t - tm) ** 2 for t in ts)
        if den < 1e-9:
            return None
        return sum((t - tm) * (x - xm) for t, x in zip(ts, xs)) / den

    def recentre(self, quiet=False):
        """Hand the ball back to the PID and wait until it rests mid-beam.

        Open-loop experiments leave it rolling; every measurement starts
        from the same place so the safe window means the same thing.
        """
        if not quiet:
            self.on_status("top ortaya alınıyor (PID)")
        self.link.set_mode(P.MODE_RUN)
        self.link.set_setpoint(self.CENTRE_CM)
        t_start = time.perf_counter()
        settled_since = None
        while time.perf_counter() - t_start < self.CENTRE_TIMEOUT_S:
            self._drain()
            self.pump_cb()
            t_now = time.perf_counter() - self.t0
            pts = self._window(t_now - 0.3, t_now)
            x = self.last_x()
            if x is None or len(pts) < 8:
                settled_since = None
                continue
            v = self._velocity(pts) or 0.0
            if abs(x - self.CENTRE_CM) < self.CENTRE_TOL_CM and abs(v) < self.CENTRE_VEL_CM_S:
                settled_since = settled_since or time.perf_counter()
                if time.perf_counter() - settled_since > self.CENTRE_SETTLE_S:
                    break
            else:
                settled_since = None
        self.link.set_mode(P.MODE_OPENLOOP)
        self.theta = 0.0
        self._hold(0.3, 0.0, guard=False)
        self.x_start = self.last_x()
        if self.x_start is None:
            raise Aborted("top görünmüyor")
        if not quiet:
            self.on_status(f"  başlangıç {self.x_start:.1f} cm")

    def _velocity_series(self, t_from, t_to):
        """(t, v) pairs from a sliding least-squares fit of x(t)."""
        pts = self._window(t_from - self.VEL_WINDOW_S, t_to)
        out = []
        j = 0
        for i, (t, _) in enumerate(pts):
            if t < t_from:
                continue
            while pts[j][0] < t - self.VEL_WINDOW_S:
                j += 1
            v = self._velocity(pts[j:i + 1])
            if v is not None:
                out.append((t, v))
        return out

    def _acceleration(self, t_from, t_to):
        """cm/s^2 over a window, from the slope of the velocity series."""
        vs = self._velocity_series(t_from, t_to)
        if len(vs) < 8:
            return None
        return self._velocity(vs)  # slope of v(t) is the acceleration

    # --- experiments ------------------------------------------------------
    def measure_noise(self):
        self.on_status("gürültü ölçümü (beam yatay, 1.5 s)")
        self._hold(1.5, 0.0, guard=False)
        t_end = time.perf_counter() - self.t0
        pts = self._window(t_end - 1.3, t_end)
        if len(pts) < 40:
            raise Aborted("çok az kamera örneği -- top görünüyor mu?")
        # A ball at rest with a quiet camera lands on one or two quantisation
        # steps (0.01 cm), so only a completely constant reading means the
        # stream itself is frozen (seen right after re-plugging the PS3 Eye).
        if len({round(x, 3) for _, x in pts}) < 2:
            raise Aborted("kamera donmuş görünüyor (konum hiç değişmiyor) -- USB'yi çıkarıp takın")
        # The ball is rarely perfectly still, so fit a constant acceleration
        # (x = a0 + a1 t + a2 t^2) and call the residual the camera noise;
        # a plain mean would count the ball's own motion as noise.
        t_mid = (pts[0][0] + pts[-1][0]) / 2
        ts = [t - t_mid for t, _ in pts]
        xs = [x for _, x in pts]
        n = len(ts)
        s1 = sum(ts); s2 = sum(t * t for t in ts); s3 = sum(t ** 3 for t in ts); s4 = sum(t ** 4 for t in ts)
        b0 = sum(xs); b1 = sum(t * x for t, x in zip(ts, xs)); b2 = sum(t * t * x for t, x in zip(ts, xs))
        # 3x3 normal equations, solved by elimination (no numpy in this module)
        A = [[n, s1, s2, b0], [s1, s2, s3, b1], [s2, s3, s4, b2]]
        for i in range(3):
            piv = A[i][i]
            if abs(piv) < 1e-12:
                piv = 1e-12
            A[i] = [v / piv for v in A[i]]
            for j in range(3):
                if j != i:
                    f = A[j][i]
                    A[j] = [vj - f * vi for vj, vi in zip(A[j], A[i])]
        a0, a1, a2 = A[0][3], A[1][3], A[2][3]
        resid = [x - (a0 + a1 * t + a2 * t * t) for t, x in zip(ts, xs)]
        self.m.sigma_cm = statistics.pstdev(resid)
        self.m.drift_cm_s = a1
        self.x_start = a0
        self.on_status(f"  gürültü σ = {self.m.sigma_cm * 10:.3f} mm, kayma = {a1:+.2f} cm/s, "
                       f"ivme {2 * a2:+.1f} cm/s²")

    def measure_dead_band(self):
        self.on_status("ölü bant taraması (her adım ortadan başlar)")
        for th in self.DEAD_BAND_STEPS:
            self.recentre(quiet=True)
            start_x = self.x_start
            self._hold(self.DEAD_BAND_MS / 1000.0, th)
            self._hold(0.25, 0.0)
            moved = abs((self.last_x() or start_x) - start_x)
            self.on_status(f"  θ = {th:+.2f}° · {self.DEAD_BAND_MS} ms -> {moved:.2f} cm")
            if moved > 0.4:
                self.m.dead_band_deg = th
                self.on_status(f"  ölü bant ≈ {th:.2f}°")
                return
        self.m.dead_band_deg = self.DEAD_BAND_STEPS[-1]
        self.on_status(f"  ölü bant > {self.DEAD_BAND_STEPS[-1]:.2f}° (yüksek sürtünme)")

    def _pulse(self, theta_deg):
        """Tilt from rest and measure the ball's acceleration.

        Returns (a_cm_s2, delay_s). The first FIT_SKIP_S is dropped: the
        crank takes ~60 ms to reach the angle, and the fit must see the
        steady tilt only.
        """
        self.recentre(quiet=True)
        t_cmd = time.perf_counter() - self.t0
        self._hold(self.PULSE_MS / 1000.0, theta_deg)
        t_end = time.perf_counter() - self.t0
        # brake with the opposite tilt so the ball does not run away; the
        # next recentre hands it back to the PID anyway
        self._hold(self.PULSE_MS / 1000.0 * 0.8, -theta_deg)

        a = self._acceleration(t_cmd + self.FIT_SKIP_S, t_end)
        # Delay from the crank's own telemetry (clean) rather than from the
        # ball's acceleration (buried in camera noise for the first 100 ms):
        # time for the crank to cover 63% of its travel, plus the camera
        # pipeline (exposure + tracking + link, measured in Faz D).
        crank = [(t, phi) for t, _, _, _, _, _, phi in self.samples if t_cmd <= t <= t_end]
        delay = None
        if len(crank) > 5:
            phi0 = crank[0][1]
            phi_end = crank[-1][1]
            if abs(phi_end - phi0) > 3.0:
                target = phi0 + 0.63 * (phi_end - phi0)
                for t, phi in crank:
                    if (phi_end > phi0 and phi >= target) or (phi_end < phi0 and phi <= target):
                        delay = (t - t_cmd) + self.CAMERA_DELAY_S
                        break
        self.m.pulses.append(dict(theta=theta_deg, a=a, delay=delay))
        self.on_status(f"  θ = {theta_deg:+.2f}° -> ivme {a:+.1f} cm/s²"
                       + (f", gecikme {1000 * delay:.0f} ms" if delay else ""))
        return a, delay

    def measure_gain(self):
        """Symmetric pulses give the gain and the true level tilt at once.

        With a = K (theta - theta0):
            K      = (a_plus - a_minus) / (2 theta)
            theta0 = -(a_plus + a_minus) / (2 K)
        so a rail that is not quite level (seen as a drift at theta = 0)
        cannot bias the gain, and its size comes out as a by-product.
        """
        th = self.THETA_PULSE_DEG
        self.on_status(f"kazanç ve yatay ofset ölçümü (±{th:.1f}°, {self.PULSE_MS} ms, her darbe ortadan)")
        a_pos, a_neg, delays = [], [], []
        for theta in (+th, -th, +th, -th, +th, -th):
            a, d = self._pulse(theta)
            if a is None:
                continue
            (a_pos if theta > 0 else a_neg).append(a)
            if d:
                delays.append(d)
        if not (a_pos and a_neg):
            raise Aborted("darbelerden ivme okunamadı")
        # Median: one pulse in six tends to catch the ball still rolling.
        self.m.a_pos = statistics.median(a_pos)
        self.m.a_neg = statistics.median(a_neg)
        th_rad = math.radians(th)
        k = (self.m.a_pos - self.m.a_neg) / (2 * th_rad)
        if k <= 0:
            raise Aborted(f"kazanç negatif çıktı ({k:.0f}) -- yön uzlaşımı ters olabilir")
        self.m.k_pos = self.m.k_neg = k
        self.m.level_offset_deg = math.degrees(-(self.m.a_pos + self.m.a_neg) / (2 * k))
        self.m.delay_s = statistics.median(delays) if delays else 0.04
        self.on_status(f"  ivme +{self.m.a_pos:+.1f} / {self.m.a_neg:+.1f} cm/s²  ->  "
                       f"K = {k:.0f} cm/s²/rad, yatay ofset {self.m.level_offset_deg:+.2f}°")

    def measure_dead_band(self):
        """Tiny tilts around the measured level: does the ball move at all?"""
        if self.m.level_offset_deg is None:
            return
        self.on_status("ölü bant kontrolü (ölçülen yatayın etrafında)")
        for extra in (0.15, 0.3, 0.5):
            self.recentre(quiet=True)
            t_cmd = time.perf_counter() - self.t0
            self._hold(0.5, self.m.level_offset_deg + extra)
            t_end = time.perf_counter() - self.t0
            a = self._acceleration(t_cmd + self.FIT_SKIP_S, t_end)
            self._hold(0.3, self.m.level_offset_deg - extra)
            self.on_status(f"  +{extra:.2f}° -> ivme {a:+.1f} cm/s²" if a is not None else f"  +{extra:.2f}° -> ölçülemedi")
            if a is not None and a > 0.5:
                self.m.dead_band_deg = extra
                return
        self.m.dead_band_deg = 0.5

    # --- gains from the measurement --------------------------------------
    PROFILES = {          # name: (wn rad/s, zeta)
        "yumuşak": (2.0, 0.85),
        "normal": (3.0, 0.75),
        "agresif": (4.0, 0.70),
    }

    def gains(self, profile="normal"):
        """Pole placement on the measured plant.

        x'' = K theta, so a PD loop gives  s^2 + K*Kd s + K*Kp = 0:
            Kp = wn^2 / K,  Kd = 2 zeta wn / K       (rad -> deg here)
        The bandwidth is capped by the measured delay (wn * T_d <= 0.18
        keeps roughly 45 deg of phase margin), Ki closes the residual
        offset from the rail tilt, and tau_D is the smallest filter that
        keeps the derivative's noise contribution under D_NOISE_BUDGET.
        """
        if not self.m.k:
            raise Aborted("kazanç ölçülmedi")
        wn, zeta = self.PROFILES[profile]
        note = ""
        td = self.m.delay_s or 0.03
        wn_max = 0.18 / td
        if wn > wn_max:
            note = f"gecikme {td * 1000:.0f} ms yüzünden ωn {wn:.1f} -> {wn_max:.1f} rad/s"
            wn = wn_max

        k = self.m.k                       # cm/s^2 per rad
        k_deg = math.radians(1.0) * k      # cm/s^2 per degree of beam angle
        kp = wn * wn / k_deg               # deg per cm
        kd = 2 * zeta * wn / k_deg         # deg per (cm/s)

        # Ki: only as much as the measured rail tilt needs. The steady tilt
        # the loop must hold is |level_offset|, reached by Ki * e * t; with a
        # 3 s budget and the tolerance below, this stays far from windup.
        offset = abs(self.m.level_offset_deg or 0.0)
        ki = 0.0
        if offset > 0.05:
            ki = min(kp * wn / 8.0, offset / (3.0 * self.CENTRE_TOL_CM))

        # tau_D from the noise the derivative actually produces. A white-noise
        # formula overestimates it threefold here (camera noise is
        # correlated), so the constant is calibrated on this rig: with
        # Kd 0.34, tau 0.15 and sigma 0.2 mm the measured theta jitter from
        # the D term was 0.038 deg (K-025).  jitter = C * Kd * sigma / sqrt(tau)
        sigma = max(self.m.sigma_cm or 0.02, 0.008)
        tau = (self.D_NOISE_CONST * kd * sigma / self.D_NOISE_BUDGET_DEG) ** 2
        tau = max(0.06, min(0.25, tau))

        return dict(kp=round(kp, 3), ki=round(ki, 3), kd=round(kd, 3), tau=round(tau, 3),
                    wn=wn, zeta=zeta, note=note)

    # --- verification ------------------------------------------------------
    STEP_CM = 5.0
    STEP_SETTLE_BAND_CM = 0.7
    STEP_TIMEOUT_S = 9.0

    def score_step(self, label):
        """One setpoint step under the closed loop, scored.

        Overshoot, settling time, residual error and the beam-angle
        jitter once settled -- the four numbers the user is shown.
        """
        self.recentre(quiet=True)
        self.link.set_mode(P.MODE_RUN)
        target = self.CENTRE_CM + self.STEP_CM
        x0 = self.last_x() or self.CENTRE_CM
        t_step = time.perf_counter() - self.t0
        self.link.set_setpoint(target)
        end = time.perf_counter() + self.STEP_TIMEOUT_S
        while time.perf_counter() < end:
            self._drain()
            self.pump_cb()
        pts = self._window(t_step, t_step + self.STEP_TIMEOUT_S)
        if len(pts) < 50:
            raise Aborted("basamak ölçümü için yeterli veri yok")
        step = target - x0
        peak = max(x for _, x in pts) if step > 0 else min(x for _, x in pts)
        overshoot = (peak - target) / step * 100 if step else 0.0
        settle = 0.0
        for t, x in pts:
            if abs(x - target) > self.STEP_SETTLE_BAND_CM:
                settle = t - t_step
        tail_t = t_step + max(settle, 3.0)
        tail_x = [x for t, x in pts if t > tail_t]
        tail_th = [th for t, _, valid, _, _, th, _ in self.samples if t > tail_t and valid]
        res = dict(label=label,
                   overshoot=overshoot,
                   settle=settle,
                   err=(statistics.mean(tail_x) - target) if tail_x else float("nan"),
                   jitter=statistics.pstdev(tail_th) if len(tail_th) > 20 else 0.0)
        self.on_status(f"  {label}: aşım %{overshoot:.0f}, oturma {settle:.2f} s, "
                       f"kalıcı hata {res['err']:+.2f} cm, θ titreşimi {res['jitter']:.3f}°")
        self.link.set_setpoint(self.CENTRE_CM)
        return res

    @staticmethod
    def better(new, old):
        """Is the new gain set actually an improvement?

        Settling time dominates, overshoot is capped, and a set that
        makes the beam noticeably noisier is rejected even if it is
        marginally faster.
        """
        def cost(r):
            return (r["settle"]
                    + 0.05 * max(0.0, r["overshoot"] - 10.0)
                    + 2.0 * abs(r["err"])
                    + 4.0 * r["jitter"])
        return cost(new) < cost(old) * 0.95

    def apply(self, g):
        self.link.set_gains(g["kp"], g["ki"], g["kd"], g["tau"])
        self._hold(0.3, 0.0, guard=False)

    # --- entry point ------------------------------------------------------
    def run(self, profile="normal", old_gains=None, verify=True):
        """Identify, compute, verify, and keep the better gain set.

        old_gains: (kp, ki, kd, tau) currently in the firmware; taken from
        CONFIG if omitted. Returns a result dict for the report.
        """
        if old_gains is None:
            cfg = getattr(self.link, "config", None)
            if cfg is None:
                raise Aborted("CONFIG alınamadı -- kazançlar bilinmiyor")
            old_gains = (cfg.kp, cfg.ki, cfg.kd, cfg.d_tau_s)
        old = dict(kp=old_gains[0], ki=old_gains[1], kd=old_gains[2], tau=old_gains[3])

        self.identify()
        new = self.gains(profile)
        self.on_status(f"öneri: Kp {new['kp']:.2f}  Ki {new['ki']:.2f}  Kd {new['kd']:.2f}  "
                       f"τ_D {new['tau']:.2f}   (ωn {new['wn']:.1f}, ζ {new['zeta']:.2f})"
                       + (f"  [{new['note']}]" if new["note"] else ""))
        result = dict(measurement=self.m, old=old, new=new, profile=profile,
                      before=None, after=None, applied=False)
        if not verify:
            self.apply(new)
            result["applied"] = True
            return result

        self.on_status("doğrulama: mevcut kazançlarla basamak")
        self.apply(old)
        result["before"] = self.score_step("mevcut")
        self.on_status("doğrulama: önerilen kazançlarla basamak")
        self.apply(new)
        result["after"] = self.score_step("öneri")

        if self.better(result["after"], result["before"]):
            result["applied"] = True
            self.on_status("öneri daha iyi -- uygulandı")
        else:
            self.apply(old)
            self.on_status("öneri daha iyi değil -- eski kazançlara dönüldü")
        return result

    def identify(self):
        """Runs every experiment; leaves the beam level and the mode as found."""
        self.wait_ready()
        self.on_status("OPENLOOP moduna geçiliyor")
        self.link.set_mode(P.MODE_OPENLOOP)
        self._require_ball()
        try:
            self.recentre()
            self.measure_noise()
            self.measure_gain()
            self.measure_dead_band()
        finally:
            self.theta = 0.0
            try:
                self.link.set_theta(0.0)
                self.link.set_mode(P.MODE_RUN)
            except Exception:
                pass
        return self.m

    # --- report -----------------------------------------------------------
    def report(self):
        m = self.m
        out = []
        out.append(f"kamera gürültüsü σ   : {m.sigma_cm * 10:.2f} mm" if m.sigma_cm is not None else "")
        out.append(f"yatayda kayma        : {m.drift_cm_s:+.2f} cm/s" if m.drift_cm_s is not None else "")
        out.append(f"ölü bant             : {m.dead_band_deg:.2f}°" if m.dead_band_deg else "")
        if m.level_offset_deg is not None:
            out.append(f"yatay ofset          : {m.level_offset_deg:+.2f}°  (θ=0'da ivme sıfır değil)")
        if m.a_pos is not None:
            out.append(f"ivmeler              : {m.a_pos:+.1f} / {m.a_neg:+.1f} cm/s²")
        if m.k:
            asym = m.asymmetry()
            out.append(f"tesis kazancı K      : {m.k:.0f} cm/s²/rad")
            out.append(f"model karşılaştırma  : içi boş top teorisi 600, içi dolu 700")
        if m.delay_s:
            out.append(f"gecikme              : {m.delay_s * 1000:.0f} ms")
        return "\n".join(x for x in out if x)

    @staticmethod
    def format_result(r):
        """Human-readable summary of a full run (measurement + verdict)."""
        o, n = r["old"], r["new"]
        out = [f"mevcut : Kp {o['kp']:.2f}  Ki {o['ki']:.2f}  Kd {o['kd']:.2f}  τ_D {o['tau']:.2f}",
               f"öneri  : Kp {n['kp']:.2f}  Ki {n['ki']:.2f}  Kd {n['kd']:.2f}  τ_D {n['tau']:.2f}"
               f"   (profil {r['profile']}, ωn {n['wn']:.1f} rad/s, ζ {n['zeta']:.2f})"]
        if n["note"]:
            out.append(f"         {n['note']}")
        for key, label in (("before", "mevcut"), ("after", "öneri ")):
            v = r.get(key)
            if v:
                out.append(f"{label} basamak: aşım %{v['overshoot']:.0f}  oturma {v['settle']:.2f} s  "
                           f"kalıcı hata {v['err']:+.2f} cm  θ titreşimi {v['jitter']:.3f}°")
        out.append("SONUÇ: yeni kazançlar uygulandı" if r["applied"] else "SONUÇ: eski kazançlar korundu")
        return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    from esp_link import EspLink
    link = EspLink(port=args.port)
    link.start()
    t0 = time.perf_counter()
    while not link.connected and time.perf_counter() - t0 < 10:
        time.sleep(0.05)
    if not link.connected:
        print("ESP32 bulunamadı")
        return 1
    print(f"bağlandı: {link.port}")
    print("NOT: bu araç kamerayı açmaz -- konum paketlerini app.py veya link.py göndermeli.")
    tuner = Autotuner(link)
    try:
        result = tuner.run()
    except Aborted as e:
        print(f"\nDURDURULDU: {e}")
        link.set_theta(0.0)
        link.set_mode(P.MODE_RUN)
        time.sleep(0.5)
        link.stop()
        return 2
    print("\n--- ölçüm ---")
    print(tuner.report())
    print("\n--- ayar ---")
    print(tuner.format_result(result))
    link.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
