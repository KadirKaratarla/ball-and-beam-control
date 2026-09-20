"""Beam + ball picture. Click on the beam to set the setpoint."""

import math

from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QBrush, QFont
from PySide6.QtWidgets import QWidget

ANGLE_EXAGGERATION = 6.0  # the real tilt is +-3 deg; drawn x6 so it can be seen


class BeamView(QWidget):
    setpoint_clicked = Signal(float)

    def __init__(self, beam_cm=45.0, parent=None):
        super().__init__(parent)
        self.beam_cm = beam_cm
        self.x_cm = None
        self.x_set_cm = 22.5
        self.theta_deg = 0.0
        self.ball_valid = False
        self.state = "?"
        self.setMinimumHeight(170)
        self.setToolTip("Setpoint için beam üzerine tıklayın")

    def update_state(self, x_cm, valid, x_set_cm, theta_deg, state):
        self.x_cm, self.ball_valid, self.x_set_cm, self.theta_deg, self.state = x_cm, valid, x_set_cm, theta_deg, state
        self.update()

    # geometry: the hinge (p2, 45 cm) is on the right, the crank end (p1) left
    def _beam_ends(self):
        w, h = self.width(), self.height()
        margin = 40
        x1, x2 = margin, w - margin
        yc = h * 0.55
        # positive theta lifts the crank end (p1) -- ball rolls toward p2
        dy = (x2 - x1) * math.tan(math.radians(self.theta_deg * ANGLE_EXAGGERATION)) / 2
        return QPointF(x1, yc + dy), QPointF(x2, yc - dy)

    def _cm_to_point(self, cm):
        p1, p2 = self._beam_ends()
        f = max(0.0, min(1.0, cm / self.beam_cm))
        return QPointF(p1.x() + (p2.x() - p1.x()) * f, p1.y() + (p2.y() - p1.y()) * f)

    def mousePressEvent(self, ev):
        p1, p2 = self._beam_ends()
        if p2.x() > p1.x():
            f = (ev.position().x() - p1.x()) / (p2.x() - p1.x())
            cm = max(0.0, min(self.beam_cm, f * self.beam_cm))
            self.setpoint_clicked.emit(cm)

    def paintEvent(self, ev):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        qp.fillRect(self.rect(), QColor(250, 250, 250))
        p1, p2 = self._beam_ends()

        # hinge tower (right) and crank side marker (left)
        qp.setPen(QPen(QColor(120, 120, 120), 2))
        qp.drawLine(QPointF(p2.x(), p2.y()), QPointF(p2.x(), self.height() - 15))
        qp.drawLine(QPointF(p1.x(), p1.y()), QPointF(p1.x(), self.height() - 15))

        # beam
        qp.setPen(QPen(QColor(40, 40, 40), 5, Qt.SolidLine, Qt.RoundCap))
        qp.drawLine(p1, p2)

        # cm ticks
        qp.setPen(QPen(QColor(150, 150, 150), 1))
        qp.setFont(QFont("Segoe UI", 8))
        for cm in range(0, int(self.beam_cm) + 1, 5):
            pt = self._cm_to_point(cm)
            qp.drawLine(QPointF(pt.x(), pt.y() + 6), QPointF(pt.x(), pt.y() + 12))
            qp.drawText(QRectF(pt.x() - 15, pt.y() + 12, 30, 14), Qt.AlignCenter, str(cm))

        # setpoint marker
        sp = self._cm_to_point(self.x_set_cm)
        qp.setPen(QPen(QColor(200, 30, 30), 2))
        qp.drawLine(QPointF(sp.x(), sp.y() - 26), QPointF(sp.x(), sp.y() - 8))
        qp.drawText(QRectF(sp.x() - 30, sp.y() - 44, 60, 16), Qt.AlignCenter, f"{self.x_set_cm:.1f}")

        # ball
        if self.x_cm is not None:
            b = self._cm_to_point(self.x_cm)
            r = 11
            qp.setPen(QPen(QColor(30, 30, 30), 1))
            qp.setBrush(QBrush(QColor(255, 140, 0) if self.ball_valid else QColor(200, 200, 200)))
            qp.drawEllipse(QPointF(b.x(), b.y() - r - 2), r, r)

        # labels
        qp.setPen(QPen(QColor(90, 90, 90)))
        qp.setFont(QFont("Segoe UI", 9))
        qp.drawText(QRectF(5, 5, 200, 16), Qt.AlignLeft, f"θ {self.theta_deg:+.2f}°   {self.state}")
        qp.drawText(QRectF(p1.x() - 20, self.height() - 14, 60, 14), Qt.AlignLeft, "p1 motor")
        qp.drawText(QRectF(p2.x() - 40, self.height() - 14, 60, 14), Qt.AlignRight, "p2 mafsal")
