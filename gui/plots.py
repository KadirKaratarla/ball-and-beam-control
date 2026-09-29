"""Scrolling plots on ring buffers (pyqtgraph)."""

import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import QWidget, QVBoxLayout

WINDOW_S = 20.0
RATE_HZ = 125
N = int(WINDOW_S * RATE_HZ)


class Ring:
    def __init__(self, n=N):
        self.t = np.full(n, np.nan)
        self.v = np.full(n, np.nan)
        self.i = 0
        self.n = n

    def push(self, t, v):
        self.t[self.i] = t
        self.v[self.i] = v
        self.i = (self.i + 1) % self.n

    def ordered(self):
        return np.roll(self.t, -self.i), np.roll(self.v, -self.i)


class Plots(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        pg.setConfigOptions(antialias=False, background=(24, 28, 33), foreground=(200, 205, 214))
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        self.p_x = pg.PlotWidget(title="Ball position (cm)")
        self.p_x.addLegend(offset=(10, 5))
        self.p_x.setYRange(0, 45)
        self.c_x = self.p_x.plot(pen=pg.mkPen((255, 140, 0), width=2), name="x")
        self.c_xs = self.p_x.plot(pen=pg.mkPen((255, 200, 120), width=1, style=pg.QtCore.Qt.DashLine), name="setpoint")

        self.p_th = pg.PlotWidget(title="Beam angle command and PID terms (°)")
        self.p_th.addLegend(offset=(10, 5))
        self.p_th.setYRange(-3.5, 3.5)
        self.c_th = self.p_th.plot(pen=pg.mkPen((235, 235, 235), width=2), name="θ")
        self.c_p = self.p_th.plot(pen=pg.mkPen((90, 160, 255), width=1), name="P")
        self.c_i = self.p_th.plot(pen=pg.mkPen((110, 220, 120), width=1), name="I")
        self.c_d = self.p_th.plot(pen=pg.mkPen((200, 130, 255), width=1), name="D")

        self.p_enc = pg.PlotWidget(title="Crank (encoder, ° from level) and follow error (counts)")
        self.p_enc.addLegend(offset=(10, 5))
        self.c_enc = self.p_enc.plot(pen=pg.mkPen((235, 235, 235), width=1), name="crank °")
        self.c_fol = self.p_enc.plot(pen=pg.mkPen((255, 110, 110), width=1), name="follow")

        for p in (self.p_x, self.p_th, self.p_enc):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("bottom", "s")
            lay.addWidget(p)
        self.p_th.setXLink(self.p_x)
        self.p_enc.setXLink(self.p_x)
        self.autoscroll = True
        # any manual pan/zoom switches autoscroll off until "Ortala"
        for p in (self.p_x, self.p_th, self.p_enc):
            p.getViewBox().sigRangeChangedManually.connect(self._manual)
        self.t_last = 0.0

        self.r_x, self.r_xs, self.r_th, self.r_p, self.r_i, self.r_d, self.r_enc, self.r_fol = (Ring() for _ in range(8))
        self.level_counts = 1101

    def _manual(self, *a):
        self.autoscroll = False

    def recenter(self):
        """Back to the live window and the default Y ranges."""
        self.autoscroll = True
        self.p_x.setYRange(0, 45)
        self.p_th.setYRange(-3.5, 3.5)
        self.p_enc.enableAutoRange(axis="y")
        self._scroll()

    def _scroll(self):
        self.p_x.setXRange(max(0.0, self.t_last - WINDOW_S), max(WINDOW_S, self.t_last), padding=0)

    def push_telem(self, t_s, telem):
        self.t_last = t_s
        self.r_x.push(t_s, telem.x_0p1mm / 100 if telem.flags & 1 else np.nan)
        self.r_xs.push(t_s, telem.x_set_0p1mm / 100)
        self.r_th.push(t_s, telem.theta / 100)
        self.r_p.push(t_s, telem.p / 100)
        self.r_i.push(t_s, telem.i / 100)
        self.r_d.push(t_s, telem.d / 100)
        self.r_enc.push(t_s, (telem.enc_counts - self.level_counts) * 360 / 4096)
        self.r_fol.push(t_s, telem.follow_0p1 / 10)

    def redraw(self):
        for ring, curve in ((self.r_x, self.c_x), (self.r_xs, self.c_xs), (self.r_th, self.c_th),
                            (self.r_p, self.c_p), (self.r_i, self.c_i), (self.r_d, self.c_d),
                            (self.r_enc, self.c_enc), (self.r_fol, self.c_fol)):
            t, v = ring.ordered()
            curve.setData(t, v, connect="finite")
        if self.autoscroll:
            self._scroll()
