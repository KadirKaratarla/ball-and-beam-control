"""Schematic of the rig: the beam pivots on the hinge tower (p2, right,
fixed) and the motor end (p1, left) rises and falls with the crank. Ball
in orange, setpoint as an orange ring; a small gauge shows the crank
angle from level. Click on the beam to set the setpoint."""

import math

from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QBrush, QFont, QRadialGradient, QPainterPath
from PySide6.QtWidgets import QWidget

THETA_FULL_DEG = 3.13  # the firmware's limit; the drawing scales the tilt so
                      # that this maximum exactly fills the space available
# The displayed angle follows theta through a first-order filter: the command
# itself dithers by ~0.1 deg from camera noise, which the exaggeration turns
# into visible shake even though the real beam barely moves.
SMOOTH = 0.25

BG = QColor(30, 34, 40)
BEAM = QColor(200, 204, 210)
TOWER = QColor(110, 116, 126)
TEXT = QColor(190, 195, 205)
ORANGE = QColor(255, 140, 0)
CYAN = QColor(80, 200, 220)


class BeamView(QWidget):
    setpoint_clicked = Signal(float)

    def __init__(self, beam_cm=45.0, parent=None):
        super().__init__(parent)
        self.beam_cm = beam_cm
        self.x_cm = None
        self.x_set_cm = 22.5
        self.theta_deg = 0.0      # raw command (shown as a number)
        self.theta_draw = 0.0     # filtered, used for the drawing
        self.phi_deg = 0.0
        self.phi_draw = 0.0
        self.ball_valid = False
        self.state = "?"
        self.setMinimumHeight(0)
        self.setToolTip("Setpoint için beam üzerine tıklayın")

    def update_state(self, x_cm, valid, x_set_cm, theta_deg, phi_deg, state):
        self.x_cm, self.ball_valid, self.x_set_cm = x_cm, valid, x_set_cm
        self.theta_deg, self.phi_deg, self.state = theta_deg, phi_deg, state
        self.theta_draw += SMOOTH * (theta_deg - self.theta_draw)
        self.phi_draw += SMOOTH * (phi_deg - self.phi_draw)
        self.update()

    # --- geometry: hinge (p2) fixed on the right, p1 swings about it ---
    # The beam is laid out so that even at the full drawn tilt nothing leaves
    # the widget: the pivot sits low enough for the lifted end plus the ball.
    def _margins(self):
        return 46, 30, 26  # left, right, floor height

    def _hinge(self):
        ml, mr, fh = self._margins()
        floor = self.height() - fh
        headroom = 38  # ball + setpoint label above the beam
        return QPointF(self.width() - mr, (headroom + floor - 6) / 2)

    def _rise_px(self, theta_deg):
        # Vertical travel of the motor end, scaled so +-THETA_FULL_DEG just
        # fits between the label headroom and the floor: the drawing can
        # never spill out of the widget, whatever the window size.
        p2 = self._hinge()
        ml, mr, fh = self._margins()
        floor = self.height() - fh
        room = min(p2.y() - 38, floor - 6 - p2.y())
        t = max(-1.0, min(1.0, theta_deg / THETA_FULL_DEG))
        return room * t

    def _beam_ends(self):
        p2 = self._hinge()
        ml, mr, fh = self._margins()
        length = self.width() - ml - mr
        # positive theta lifts the motor end (p1), so the ball rolls toward p2.
        # p1 keeps its x (the motor tower is fixed to the base) and only rises.
        p1 = QPointF(p2.x() - length, p2.y() - self._rise_px(self.theta_draw))
        return p1, p2

    def _cm_to_point(self, cm):
        p1, p2 = self._beam_ends()
        f = max(0.0, min(1.0, cm / self.beam_cm))
        return QPointF(p1.x() + (p2.x() - p1.x()) * f, p1.y() + (p2.y() - p1.y()) * f)

    def mousePressEvent(self, ev):
        p1, p2 = self._beam_ends()
        if p2.x() > p1.x():
            f = (ev.position().x() - p1.x()) / (p2.x() - p1.x())
            self.setpoint_clicked.emit(max(0.0, min(self.beam_cm, f * self.beam_cm)))

    def _ball(self, qp, centre, r, solid):
        if solid:
            g = QRadialGradient(centre.x() - r * 0.35, centre.y() - r * 0.35, r * 1.4)
            g.setColorAt(0, QColor(255, 200, 120))
            g.setColorAt(1, QColor(210, 100, 0))
            qp.setBrush(QBrush(g))
            qp.setPen(QPen(QColor(120, 60, 0), 1))
        else:
            qp.setBrush(Qt.NoBrush)
            qp.setPen(QPen(ORANGE, 2, Qt.DashLine))
        qp.drawEllipse(centre, r, r)

    def _gauge(self, qp):
        # crank angle from level, -70..+70 deg, upper-left corner
        cx, cy, R = self.width() - 52, 40, 26
        qp.setPen(QPen(TOWER, 3))
        rect = QRectF(cx - R, cy - R, 2 * R, 2 * R)
        qp.drawArc(rect, 20 * 16, 140 * 16)
        qp.setPen(QPen(CYAN, 1))
        qp.setFont(QFont("Segoe UI", 7))
        for a in (-60, -30, 0, 30, 60):
            t = math.radians(90 - a)  # 0 at the top, positive to the right
            qp.drawLine(QPointF(cx + (R - 6) * math.cos(t), cy - (R - 6) * math.sin(t)),
                        QPointF(cx + R * math.cos(t), cy - R * math.sin(t)))
        phi = max(-70.0, min(70.0, self.phi_draw))
        t = math.radians(90 - phi)
        qp.setPen(QPen(ORANGE, 3, Qt.SolidLine, Qt.RoundCap))
        qp.drawLine(QPointF(cx, cy), QPointF(cx + (R - 10) * math.cos(t), cy - (R - 10) * math.sin(t)))
        qp.setBrush(QBrush(TOWER))
        qp.setPen(Qt.NoPen)
        qp.drawEllipse(QPointF(cx, cy), 4, 4)
        qp.setPen(QPen(TEXT))
        qp.setFont(QFont("Segoe UI", 8))
        qp.drawText(QRectF(cx - 50, cy + 4, 100, 13), Qt.AlignCenter, f"krank {self.phi_deg:+.1f}°")

    def paintEvent(self, ev):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        qp.fillRect(self.rect(), BG)
        p1, p2 = self._beam_ends()
        ml, mr, fh = self._margins()
        floor = self.height() - fh

        # hinge tower (fixed) and motor tower with the rod
        qp.setPen(QPen(TOWER, 6, Qt.SolidLine, Qt.RoundCap))
        qp.drawLine(QPointF(p2.x(), p2.y()), QPointF(p2.x(), floor))
        qp.setPen(QPen(TOWER, 4, Qt.SolidLine, Qt.RoundCap))
        tower_x = p2.x() - (self.width() - ml - mr) + 14  # fixed: does not follow the beam end
        qp.drawLine(QPointF(tower_x, floor - 40), QPointF(tower_x, floor))  # motor tower
        qp.setPen(QPen(QColor(150, 156, 166), 2))
        qp.drawLine(QPointF(p1.x() + 6, p1.y() + 2), QPointF(tower_x, floor - 40))  # rod
        qp.setPen(QPen(TOWER, 2))
        qp.drawLine(QPointF(ml - 20, floor), QPointF(self.width() - 14, floor))  # base

        # beam
        qp.setPen(QPen(BEAM, 6, Qt.SolidLine, Qt.RoundCap))
        qp.drawLine(p1, p2)
        qp.setBrush(QBrush(TOWER))
        qp.setPen(Qt.NoPen)
        qp.drawEllipse(p2, 5, 5)

        # cm ticks under the beam
        qp.setPen(QPen(QColor(120, 126, 136), 1))
        qp.setFont(QFont("Segoe UI", 7))
        for cm in range(0, int(self.beam_cm) + 1, 5):
            pt = self._cm_to_point(cm)
            qp.drawLine(QPointF(pt.x(), pt.y() + 6), QPointF(pt.x(), pt.y() + 11))
            qp.drawText(QRectF(pt.x() - 15, pt.y() + 12, 30, 12), Qt.AlignCenter, str(cm))

        # setpoint ring and ball
        r = 9
        sp = self._cm_to_point(self.x_set_cm)
        self._ball(qp, QPointF(sp.x(), sp.y() - r - 3), r, solid=False)
        qp.setPen(QPen(ORANGE))
        qp.setFont(QFont("Segoe UI", 8))
        qp.drawText(QRectF(sp.x() - 30, max(0.0, sp.y() - r * 2 - 20), 60, 13), Qt.AlignCenter, f"{self.x_set_cm:.1f}")
        if self.x_cm is not None:
            b = self._cm_to_point(self.x_cm)
            if self.ball_valid:
                self._ball(qp, QPointF(b.x(), b.y() - r - 3), r, solid=True)
            else:
                qp.setBrush(QBrush(QColor(90, 90, 90)))
                qp.setPen(QPen(QColor(60, 60, 60), 1))
                qp.drawEllipse(QPointF(b.x(), b.y() - r - 3), r, r)

        self._gauge(qp)

        qp.setPen(QPen(TEXT))
        qp.setFont(QFont("Segoe UI", 9))
        qp.drawText(QRectF(8, 4, 200, 15), Qt.AlignLeft, f"θ {self.theta_deg:+.2f}°   {self.state}")
        qp.setFont(QFont("Segoe UI", 8))
        qp.drawText(QRectF(tower_x - 22, floor + 3, 70, 13), Qt.AlignLeft, "p1 motor")
        qp.drawText(QRectF(p2.x() - 62, floor + 3, 70, 13), Qt.AlignRight, "p2 mafsal")
