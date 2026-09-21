"""Main window: device strips, warning strip, schematic/camera, setpoint,
gains, modes with state lights, tabbed health panel, plots, recorder."""

import time

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QFont, QImage, QPixmap
from PySide6.QtWidgets import (QMainWindow, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QSlider,
                               QDoubleSpinBox, QPushButton, QGroupBox, QSizePolicy, QStackedWidget, QTabWidget,
                               QFrame)

import protocol as P
from beam_view import BeamView
from plots import Plots
from recorder import Recorder

# DRV_STATUS bits (TMC2208 datasheet)
DRV_OTPW, DRV_OT, DRV_S2GA, DRV_S2GB, DRV_S2VSA, DRV_S2VSB, DRV_OLA, DRV_OLB = (1 << i for i in range(8))
DRV_T120, DRV_T143, DRV_T150, DRV_T157 = (1 << i for i in range(8, 12))

FAULT_TEXT = {
    1: "ENCODER ARALIK DIŞI: mil çalışma yayının dışında -- mekanizmayı kontrol edin",
    2: "TAKİP HATASI: komut ile encoder uyuşmuyor (adım kaybı / kablo / mekanik takılma)",
    3: "ENCODER CEVAP VERMİYOR: AS5600 kablosu / I2C hattı",
    4: "TMC2208 RESET: sürücü registerları kayboldu (besleme / VM)",
    5: "TMC2208 UART CEVAP VERMİYOR: sürücü kablosu / PDN_UART",
    6: "BEAM 6'DA DEĞİL: beam'i elle indirip RESET_FAULT / RUN verin",
}

STYLE = """
QMainWindow, QWidget { background: #1e2228; color: #d0d4dc; font-family: 'Segoe UI'; font-size: 10pt; }
QGroupBox { border: 1px solid #3a4048; border-radius: 6px; margin-top: 10px; padding: 6px 6px 4px 6px; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #9aa3b0; }
QPushButton { background: #2c323a; border: 1px solid #3f4650; border-radius: 4px; padding: 5px 10px; }
QPushButton:hover { background: #363d47; }
QPushButton:checked { background: #1f5a3a; border-color: #2e7d32; }
QDoubleSpinBox, QLineEdit { background: #151a1f; border: 1px solid #3a4048; border-radius: 3px; padding: 2px 4px; }
QSlider::groove:horizontal { height: 6px; background: #3a4048; border-radius: 3px; }
QSlider::handle:horizontal { width: 14px; margin: -5px 0; background: #ff8c00; border-radius: 7px; }
QTabWidget::pane { border: 1px solid #3a4048; border-radius: 4px; }
QTabBar::tab { background: #262b32; padding: 4px 10px; border: 1px solid #3a4048; border-bottom: none;
               border-top-left-radius: 4px; border-top-right-radius: 4px; }
QTabBar::tab:selected { background: #333a44; }
QLabel#cell { background: #151a1f; border: 1px solid #2f353d; border-radius: 4px; padding: 3px 6px;
              font-family: Consolas; font-size: 9pt; }
QLabel#cellTitle { color: #8d96a3; font-size: 8pt; }
"""

GREEN, RED, ORANGE_C, GREY = "#2e7d32", "#c62828", "#ef6c00", "#4a5058"


def strip_label(text, color):
    lb = QLabel(text)
    lb.setStyleSheet(f"background:{color}; color:white; padding:6px; font-weight:bold; border-radius:3px;")
    lb.setAlignment(Qt.AlignCenter)
    return lb


class Cell(QWidget):
    """A titled value box for the health panel."""

    def __init__(self, title):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(1)
        t = QLabel(title)
        t.setObjectName("cellTitle")
        self.value = QLabel("--")
        self.value.setObjectName("cell")
        lay.addWidget(t)
        lay.addWidget(self.value)

    def set(self, text, color=None):
        self.value.setText(text)
        self.value.setStyleSheet(f"color:{color};" if color else "")


class ModeButton(QWidget):
    """Button with a state light above it."""

    def __init__(self, text, on_click):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        self.light = QLabel("●")
        self.light.setAlignment(Qt.AlignCenter)
        self.light.setStyleSheet(f"color:{GREY}; font-size:12pt;")
        self.btn = QPushButton(text)
        self.btn.clicked.connect(on_click)
        lay.addWidget(self.light)
        lay.addWidget(self.btn)

    def lit(self, color):
        self.light.setStyleSheet(f"color:{color or GREY}; font-size:12pt;")


class MainWindow(QMainWindow):
    def __init__(self, link, camera, beam_cm=45.0):
        super().__init__()
        self.link = link
        self.camera = camera
        self.beam_cm = beam_cm
        self.recorder = Recorder()
        self.config = None
        self.defaults = None
        self.t0 = time.perf_counter()
        self.last_telem = None
        self.last_health = None
        self.prev_health = None
        self.telem_count = 0
        self.telem_rate = 0
        self.frames_prev = 0
        self.cam_rate = 0
        self.warnings = {}   # key -> (level, text); level 'fault' | 'warn'
        self.cam_state = "starting"
        self.cam_detail = ""

        self.setWindowTitle("Ball & Beam")
        self.setStyleSheet(STYLE)
        root = QWidget()
        self.setCentralWidget(root)
        vbox = QVBoxLayout(root)

        # --- device strips + warning strip ---
        top = QHBoxLayout()
        self.esp_strip = strip_label("ESP32 aranıyor ...", GREY)
        self.cam_strip = strip_label("Kamera aranıyor ...", GREY)
        self.btn_rescan = QPushButton("Yeniden tara")
        self.btn_rescan.setToolTip("Kamerayı yeniden ara (ESP32 otomatik bulunur)")
        self.btn_rescan.clicked.connect(self.camera.rescan)
        top.addWidget(self.esp_strip, 1)
        top.addWidget(self.cam_strip, 1)
        top.addWidget(self.btn_rescan, 0)
        vbox.addLayout(top)
        self.warn_strip = strip_label("", RED)
        self.warn_strip.hide()
        vbox.addWidget(self.warn_strip)

        # --- left column ---
        mid = QHBoxLayout()
        left = QVBoxLayout()
        left.setSpacing(6)

        # schematic / camera in one fixed slot
        self.stack = QStackedWidget()
        self.stack.setFixedHeight(320)
        self.beam = BeamView(beam_cm)
        self.beam.setpoint_clicked.connect(self.send_setpoint)
        self.cam_view = QLabel("kamera görüntüsü bekleniyor")
        self.cam_view.setAlignment(Qt.AlignCenter)
        self.cam_view.setStyleSheet("background:#0e1114; color:#8d96a3;")
        self.stack.addWidget(self.beam)
        self.stack.addWidget(self.cam_view)
        left.addWidget(self.stack)

        # setpoint
        sp_box = QGroupBox("Setpoint (cm)")
        sp_lay = QHBoxLayout(sp_box)
        self.sp_slider = QSlider(Qt.Horizontal)
        self.sp_slider.setRange(30, 420)  # 0.1 cm, matches the ESP limits 3..42
        self.sp_slider.setValue(225)
        self.sp_spin = QDoubleSpinBox()
        self.sp_spin.setRange(3.0, 42.0)
        self.sp_spin.setSingleStep(0.5)
        self.sp_spin.setValue(22.5)
        self.sp_spin.setSuffix(" cm")
        self.sp_slider.valueChanged.connect(lambda v: self.sp_spin.setValue(v / 10))
        self.sp_spin.valueChanged.connect(lambda v: self.sp_slider.setValue(int(v * 10)))
        self.sp_slider.sliderReleased.connect(lambda: self.send_setpoint(self.sp_spin.value()))
        self.sp_spin.editingFinished.connect(lambda: self.send_setpoint(self.sp_spin.value()))
        sp_lay.addWidget(self.sp_slider, 1)
        sp_lay.addWidget(self.sp_spin, 0)
        left.addWidget(sp_box)

        # gains
        g_box = QGroupBox("PID kazançları")
        g_lay = QGridLayout(g_box)
        self.gain_spins = {}
        for col, (key, label, rng, step) in enumerate((("kp", "Kp  °/cm", (0, 3), 0.05), ("ki", "Ki  °/(cm·s)", (0, 1), 0.05),
                                                        ("kd", "Kd  °·s/cm", (0, 2), 0.02), ("tau", "τ_D  s", (0, 1), 0.01))):
            t = QLabel(label)
            t.setObjectName("cellTitle")
            g_lay.addWidget(t, 0, col)
            sb = QDoubleSpinBox()
            sb.setRange(*rng)
            sb.setSingleStep(step)
            sb.setDecimals(3)
            g_lay.addWidget(sb, 1, col)
            self.gain_spins[key] = sb
        self.gain_status = QLabel("CONFIG bekleniyor")
        self.gain_status.setObjectName("cellTitle")
        self.btn_apply = QPushButton("Uygula")
        self.btn_apply.clicked.connect(self.send_gains)
        self.btn_defaults = QPushButton("Varsayılana dön")
        self.btn_defaults.clicked.connect(self.restore_defaults)
        g_lay.addWidget(self.gain_status, 2, 0, 1, 2)
        g_lay.addWidget(self.btn_apply, 2, 2)
        g_lay.addWidget(self.btn_defaults, 2, 3)
        left.addWidget(g_box)

        # modes with state lights
        m_box = QGroupBox("Mod")
        m_lay = QHBoxLayout(m_box)
        self.mode_btns = {}
        for text, mode in (("RUN", P.MODE_RUN), ("LEVEL", P.MODE_LEVEL), ("STOP", P.MODE_STOP), ("RESET_FAULT", P.MODE_RESET_FAULT)):
            mb = ModeButton(text, lambda _=False, m=mode: self.send_mode(m))
            self.mode_btns[mode] = mb
            m_lay.addWidget(mb)
        left.addWidget(m_box)

        # health tabs
        self.tabs = QTabWidget()
        self.cells = {}

        def tab(name, spec, cols=3):
            w = QWidget()
            g = QGridLayout(w)
            g.setContentsMargins(6, 6, 6, 6)
            for i, (key, title) in enumerate(spec):
                c = Cell(title)
                self.cells[key] = c
                g.addWidget(c, i // cols, i % cols)
            self.tabs.addTab(w, name)

        tab("Genel", (("state", "Durum"), ("x", "Top konumu"), ("xset", "Setpoint"),
                      ("theta", "Beam açısı θ"), ("phi", "Krank φ"), ("loop", "Döngü µs (ort/maks)"),
                      ("overrun", "Aşım / kaçırılan"), ("age", "Konum yaşı"), ("rtt", "RTT")))
        tab("Encoder", (("enc", "Mil (sayım, 6'dan)"), ("follow", "Takip hatası"), ("lag", "Encoder telafisi"),
                        ("i2c", "I2C hataları"), ("rej", "Makullük retleri"), ("dir", "DIR arızası")))
        tab("Motor", (("cs", "Akım (CS)"), ("temp", "Sıcaklık eşiği"), ("drvflags", "Bayraklar"),
                      ("tmcreset", "Reset"), ("tmcuart", "UART hataları"), ("vel", "Adım hızı"), ("drv", "DRV_STATUS")))
        tab("Link", (("pkt", "Konum paketleri"), ("crc", "CRC hataları"), ("gap", "Sıra boşlukları"),
                     ("cmd", "Komut ok / kötü"), ("txd", "ESP tx düşen"), ("recon", "Yeniden bağlanma")))
        tab("Kamera", (("camstate", "Kamera"), ("fps", "Kare hızı"), ("camdetail", "Ayrıntı")), cols=1)
        left.addWidget(self.tabs, 1)

        # bottom buttons
        r_lay = QHBoxLayout()
        self.btn_rec = QPushButton("●  Kayıt")
        self.btn_rec.setCheckable(True)
        self.btn_rec.toggled.connect(self.toggle_record)
        self.btn_view = QPushButton("Kamera")
        self.btn_view.setCheckable(True)
        self.btn_view.setToolTip("Şema yerine canlı takip görüntüsü")
        self.btn_view.toggled.connect(self.toggle_view)
        self.btn_center = QPushButton("Grafikleri ortala")
        self.btn_center.clicked.connect(lambda: self.plots.recenter())
        r_lay.addWidget(self.btn_rec)
        r_lay.addWidget(self.btn_view)
        r_lay.addWidget(self.btn_center)
        left.addLayout(r_lay)
        self.rec_label = QLabel("")
        self.rec_label.setObjectName("cellTitle")
        left.addWidget(self.rec_label)

        leftw = QWidget()
        leftw.setLayout(left)
        leftw.setFixedWidth(520)
        mid.addWidget(leftw, 0)
        self.plots = Plots()
        mid.addWidget(self.plots, 1)
        vbox.addLayout(mid, 1)

        # --- camera/link process signals ---
        self.camera.status.connect(self.on_status)
        self.camera.camera_state.connect(self.on_camera_state)
        self.camera.position.connect(self.on_position)
        self.camera.image.connect(self.on_image)

        # --- timers ---
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(40)
        self.rate_timer = QTimer(self)
        self.rate_timer.timeout.connect(self.rates)
        self.rate_timer.start(1000)

        self.resize(1450, 900)

    # --- commands ------------------------------------------------------------
    @Slot(float)
    def send_setpoint(self, cm):
        cm = max(3.0, min(42.0, cm))
        self.sp_spin.blockSignals(True)
        self.sp_spin.setValue(cm)
        self.sp_spin.blockSignals(False)
        self.sp_slider.blockSignals(True)
        self.sp_slider.setValue(int(cm * 10))
        self.sp_slider.blockSignals(False)
        self.link.set_setpoint(cm)

    def send_gains(self):
        g = self.gain_spins
        self.link.set_gains(g["kp"].value(), g["ki"].value(), g["kd"].value(), g["tau"].value())
        self.gain_status.setText("gönderildi, ACK bekleniyor")

    def restore_defaults(self):
        if self.defaults is None:
            return
        d = self.defaults
        for key, val in zip(("kp", "ki", "kd", "tau"), d):
            self.gain_spins[key].setValue(val)
        self.send_gains()

    def send_mode(self, mode):
        self.link.set_mode(mode)

    def toggle_record(self, on):
        if on:
            path = self.recorder.start()
            self.btn_rec.setText("■  Kaydı durdur")
            self.rec_label.setText(path)
        else:
            self.recorder.stop()
            self.btn_rec.setText("●  Kayıt")
            self.rec_label.setText(f"{self.recorder.rows} satır -> {self.recorder.path}")

    def toggle_view(self, on):
        self.camera.set_view(on)
        self.stack.setCurrentIndex(1 if on else 0)
        if not on:
            self.cam_view.clear()
            self.cam_view.setText("kamera görüntüsü bekleniyor")

    # --- incoming ------------------------------------------------------------
    @Slot(str)
    def on_status(self, text):
        self.rec_label.setText(text) if not self.recorder.active else None

    @Slot(str, str)
    def on_camera_state(self, state, detail):
        self.cam_state, self.cam_detail = state, detail
        if state == "tracking" and self.warnings.get("cam_missing"):
            self.warnings.pop("cam_missing", None)

    @Slot(bytes)
    def on_image(self, jpeg):
        if self.stack.currentIndex() != 1:
            return
        img = QImage.fromData(jpeg, "JPG")
        if not img.isNull():
            self.cam_view.setPixmap(QPixmap.fromImage(img).scaled(
                self.stack.width() - 4, self.stack.height() - 4, Qt.KeepAspectRatio, Qt.SmoothTransformation))

    @Slot(float, bool, bool, float)
    def on_position(self, pos_cm, valid, warning, t_send):
        if self.recorder.active:
            self.recorder.sent(pos_cm, valid, t_send)
        if warning:
            self.warnings["cam"] = ("warn", "Kamera: takip sağlığı uyarısı (alan / tespit oranı)")
        else:
            self.warnings.pop("cam", None)

    def on_config(self, c):
        first = self.config is None
        self.config = c
        self.defaults = (c.kp_def, c.ki_def, c.kd_def, c.d_tau_def)
        for key, val in zip(("kp", "ki", "kd", "tau"), (c.kp, c.ki, c.kd, c.d_tau_s)):
            self.gain_spins[key].setValue(val)
        self.gain_status.setText("varsayılan (ESP)" if c.defaults else "değiştirilmiş")
        self.plots.level_counts = c.level_counts
        if first:
            self.send_setpoint(c.x_set_0p1mm / 100)

    def on_ack(self, a):
        name = {P.T_SETPOINT: "SETPOINT", P.T_GAINS: "GAINS", P.T_MODE: "MODE", P.T_GET_CONFIG: "GET_CONFIG"}.get(a.cmd_type, f"0x{a.cmd_type:02X}")
        res = P.ACK_NAMES.get(a.result, str(a.result))
        if a.cmd_type == P.T_GAINS:
            self.gain_status.setText("uygulandı" if a.result == P.ACK_OK else f"REDDEDİLDİ: {res}")
            if a.result == P.ACK_OK:
                self.link.request_config()
        if a.result != P.ACK_OK:
            self.warnings["ack"] = ("warn", f"{name} komutu reddedildi: {res}")
        else:
            self.warnings.pop("ack", None)

    def poll(self):
        for ptype, msg in self.link.drain():
            if ptype == P.T_TELEM:
                self.last_telem = msg
                self.telem_count += 1
                self.plots.push_telem(time.perf_counter() - self.t0, msg)
                if self.recorder.active:
                    self.recorder.telem(msg)
            elif ptype == P.T_HEALTH:
                self.prev_health, self.last_health = self.last_health, msg
            elif ptype == P.T_CONFIG:
                self.on_config(msg)
            elif ptype == P.T_ACK and msg.cmd_type != P.T_PING:
                self.on_ack(msg)
        self.refresh()

    def rates(self):
        self.telem_rate, self.telem_count = self.telem_count, 0
        self.cam_rate, self.frames_prev = self.camera.frames - self.frames_prev, self.camera.frames
        self.link.ping()

    # --- display -------------------------------------------------------------
    def _strip(self, lb, text, color):
        lb.setText(text)
        lb.setStyleSheet(f"background:{color}; color:white; padding:6px; font-weight:bold; border-radius:3px;")

    def refresh(self):
        cells = self.cells
        # device strips
        if self.link.connected:
            rtt = self.link.stats.get("rtt_ms")
            self._strip(self.esp_strip, f"ESP32 bağlı  {self.link.port}   RTT {rtt:.1f} ms   telemetri {self.telem_rate}/s"
                        if rtt is not None else f"ESP32 bağlı  {self.link.port}", GREEN)
            self.warnings.pop("esp_missing", None)
        else:
            self._strip(self.esp_strip, "ESP32 TAKILI DEĞİL -- USB (COM12) bekleniyor, otomatik algılanır", RED)
            self.warnings["esp_missing"] = ("fault", "ESP32 bağlı değil: kontrolör yok")
        cs = self.cam_state
        if cs == "tracking":
            self._strip(self.cam_strip, f"Kamera takipte   {self.cam_rate} fps", GREEN)
            self.warnings.pop("cam_missing", None)
        elif cs in ("opening", "calibrating"):
            self._strip(self.cam_strip, f"Kamera: {self.cam_detail}", ORANGE_C)
            self.warnings.pop("cam_missing", None)
        elif cs == "missing":
            self._strip(self.cam_strip, "KAMERA TAKILI DEĞİL -- takıp 'Yeniden tara'", RED)
            self.warnings["cam_missing"] = ("fault", "Kamera yok: top konumu gelmiyor, ESP yatayda bekler")
        elif cs == "error":
            self._strip(self.cam_strip, f"Kamera hatası: {self.cam_detail}", RED)
            self.warnings["cam_missing"] = ("fault", f"Kamera: {self.cam_detail}")
        else:
            self._strip(self.cam_strip, "Kamera aranıyor ...", GREY)
        cells["camstate"].set(cs)
        cells["camdetail"].set(self.cam_detail)
        cells["fps"].set(f"{self.cam_rate} fps")
        cells["rtt"].set(f"{self.link.stats.get('rtt_ms') or 0:.1f} ms")
        cells["recon"].set(str(self.link.stats.get("reconnects", 0)))

        t = self.last_telem
        if t and self.link.connected:
            valid = bool(t.flags & P.TF_BALL_VALID)
            state = P.STATE_NAMES.get(t.state, str(t.state))
            self.beam.update_state(t.x_0p1mm / 100 if valid else None, valid, t.x_set_0p1mm / 100,
                                   t.theta / 100, t.phi_0p1deg / 10, state)
            fault = P.FAULT_NAMES.get(t.fault, str(t.fault))
            cells["state"].set(state + (f"  ({fault})" if t.fault else ""),
                               {"RUN": "#7ee787", "FAULT": "#ff7b72", "STOP": "#ffa657"}.get(state))
            cells["x"].set(f"{t.x_0p1mm / 100:6.2f} cm" if valid else "top yok")
            cells["xset"].set(f"{t.x_set_0p1mm / 100:5.1f} cm")
            cells["theta"].set(f"{t.theta / 100:+.2f}°   P {t.p / 100:+.2f}  I {t.i / 100:+.2f}  D {t.d / 100:+.2f}")
            cells["phi"].set(f"{t.phi_0p1deg / 10:+.1f}°")
            cells["enc"].set(f"{t.enc_counts}  ({t.enc_counts * 360 / 4096:.1f}°)")
            cells["follow"].set(f"{t.follow_0p1 / 10:+.1f} sayım", "#ffa657" if abs(t.follow_0p1) > 500 else None)
            cells["lag"].set(f"{t.lag_counts:+d} sayım")
            cells["vel"].set(f"{t.cmd_vel_10 * 10} µadım/s")
            cells["age"].set(f"seq {t.last_seq}" + ("  BAYAT" if t.flags & P.TF_LINK_STALE else ""))
            # mode lights
            lit = {P.MODE_RUN: None, P.MODE_LEVEL: None, P.MODE_STOP: None, P.MODE_RESET_FAULT: None}
            if t.state == 3:
                lit[P.MODE_RUN] = "#7ee787"
            elif t.state in (1, 2):
                lit[P.MODE_LEVEL] = "#7ee787"
            elif t.state == 5:
                lit[P.MODE_STOP] = "#ffa657"
            elif t.state == 4:
                lit[P.MODE_RESET_FAULT] = "#ff7b72"
            for m, col in lit.items():
                self.mode_btns[m].lit(col)
            # warnings from telemetry
            if t.state == 4:
                self.warnings["fault"] = ("fault", "ARIZA -- " + FAULT_TEXT.get(t.fault, fault))
            else:
                self.warnings.pop("fault", None)
            if t.state == 5:
                self.warnings["stop"] = ("warn", "STOP: sürücü kapalı (RUN ile devam)")
            else:
                self.warnings.pop("stop", None)
            if abs(t.follow_0p1) > 1000:
                self.warnings["follow"] = ("warn", f"Takip hatası yüksek: {t.follow_0p1 / 10:.0f} sayım (limit 150)")
            else:
                self.warnings.pop("follow", None)
        elif not self.link.connected:
            for m in self.mode_btns.values():
                m.lit(None)

        h = self.last_health
        if h and self.link.connected:
            cells["loop"].set(f"{h.loop_us_mean} / {h.loop_us_max}")
            cells["overrun"].set(f"{h.overruns} / {h.missed}", "#ff7b72" if h.overruns else None)
            cells["pkt"].set(str(h.link_packets))
            cells["crc"].set(str(h.link_crc_errors), "#ffa657" if h.link_crc_errors else None)
            cells["gap"].set(str(h.link_seq_gaps))
            cells["cmd"].set(f"{h.cmd_rx} / {h.cmd_bad}")
            cells["txd"].set(str(h.tx_dropped))
            cells["i2c"].set(str(h.enc_i2c_errors), "#ffa657" if h.enc_i2c_errors else None)
            cells["rej"].set(str(h.enc_rejects))
            cells["dir"].set(str(h.enc_dir_faults), "#ff7b72" if h.enc_dir_faults else None)
            d = h.drv_status
            cs_ = (d >> 16) & 0x1F
            flags = []
            if d & DRV_OTPW: flags.append("OTPW")
            if d & DRV_OT: flags.append("OT")
            if d & (DRV_S2GA | DRV_S2GB): flags.append("KISA DEVRE")
            if d & (DRV_S2VSA | DRV_S2VSB): flags.append("S2VS")
            if d & (DRV_OLA | DRV_OLB): flags.append("açık faz")
            temp = ">157°" if d & DRV_T157 else ">150°" if d & DRV_T150 else ">143°" if d & DRV_T143 else ">120°" if d & DRV_T120 else "<120°"
            cells["cs"].set(f"{cs_} / 31")
            cells["temp"].set(temp, "#ff7b72" if d & DRV_T120 else None)
            cells["drvflags"].set(" ".join(flags) if flags else "yok", "#ff7b72" if flags else None)
            cells["tmcreset"].set(str(h.tmc_resets), "#ff7b72" if h.tmc_resets else None)
            cells["tmcuart"].set(str(h.tmc_uart_errors), "#ffa657" if h.tmc_uart_errors else None)
            cells["drv"].set(f"0x{d:08X}")
            # warnings from health (encoder / driver)
            if d & DRV_OT:
                self.warnings["tmc_ot"] = ("fault", "TMC2208 AŞIRI SICAKLIK: sürücü kapandı")
            elif d & DRV_OTPW:
                self.warnings["tmc_ot"] = ("warn", "TMC2208 sıcaklık ön uyarısı (120 °C) -- akımı düşürün / soğutun")
            else:
                self.warnings.pop("tmc_ot", None)
            if d & (DRV_S2GA | DRV_S2GB | DRV_S2VSA | DRV_S2VSB):
                self.warnings["tmc_short"] = ("fault", "TMC2208 KISA DEVRE bayrağı: motor kablosunu kontrol edin")
            else:
                self.warnings.pop("tmc_short", None)
            if (d & (DRV_OLA | DRV_OLB)) and self.last_telem and abs(self.last_telem.cmd_vel_10) > 50:
                self.warnings["tmc_ol"] = ("warn", "TMC2208 açık faz bayrağı hareket halinde: motor kablosu")
            else:
                self.warnings.pop("tmc_ol", None)
            p = self.prev_health
            if p:
                for key, a, b, text in (("enc_i2c", h.enc_i2c_errors, p.enc_i2c_errors, "Encoder I2C hataları artıyor"),
                                        ("enc_rej", h.enc_rejects, p.enc_rejects, "Encoder makullük retleri (gürültü / DIR pini)"),
                                        ("tmc_uart", h.tmc_uart_errors, p.tmc_uart_errors, "TMC2208 UART hataları artıyor")):
                    if a > b:
                        self.warnings[key] = ("warn", f"{text} (+{a - b}/s)")
                    else:
                        self.warnings.pop(key, None)
            if h.overruns:
                self.warnings["overrun"] = ("warn", f"Kontrol döngüsü aşımı: {h.overruns}/s")
            else:
                self.warnings.pop("overrun", None)

        if not self.link.connected:
            for k in ("fault", "stop", "follow", "tmc_ot", "tmc_short", "tmc_ol", "enc_i2c", "enc_rej", "tmc_uart", "overrun"):
                self.warnings.pop(k, None)

        # warning strip: faults first, then warnings
        if self.warnings:
            faults = [txt for lvl, txt in self.warnings.values() if lvl == "fault"]
            warns = [txt for lvl, txt in self.warnings.values() if lvl == "warn"]
            self._strip(self.warn_strip, "  |  ".join(faults + warns), RED if faults else ORANGE_C)
            self.warn_strip.show()
        else:
            self.warn_strip.hide()

        self.plots.redraw()
        if self.recorder.active:
            self.rec_label.setText(f"{self.recorder.rows} satır  {self.recorder.path}")

    def closeEvent(self, ev):
        self.recorder.stop()
        self.timer.stop()
        self.rate_timer.stop()
        self.link.stop()
        super().closeEvent(ev)
