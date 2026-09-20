"""Main window: connection strip, warnings, beam view, plots, setpoint, gains, health."""

import time

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QMainWindow, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QSlider,
                               QDoubleSpinBox, QPushButton, QGroupBox, QFrame, QSizePolicy)

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
    6: "BEAM 6'DA DEĞİL: beam'i elle indirip RESET_FAULT verin",
}


def strip_label(text, color):
    lb = QLabel(text)
    lb.setStyleSheet(f"background:{color}; color:white; padding:6px; font-weight:bold;")
    lb.setAlignment(Qt.AlignCenter)
    return lb


class MainWindow(QMainWindow):
    def __init__(self, link, camera, beam_cm=45.0):
        super().__init__()
        self.link = link
        self.camera = camera
        self.beam_cm = beam_cm
        self.recorder = Recorder()
        self.config = None
        self.t0 = time.perf_counter()
        self.last_telem = None
        self.last_health = None
        self.prev_health = None
        self.telem_count = 0
        self.telem_rate = 0
        self.frames_prev = 0
        self.cam_rate = 0
        self.warnings = {}   # key -> (level, text); level 'fault' | 'warn'
        self.pending_acks = {}

        self.setWindowTitle("Ball & Beam")
        root = QWidget()
        self.setCentralWidget(root)
        vbox = QVBoxLayout(root)

        # --- connection / warning strips ---
        self.conn_strip = strip_label("ESP32 aranıyor ...", "#888")
        self.warn_strip = strip_label("", "#c62828")
        self.warn_strip.hide()
        vbox.addWidget(self.conn_strip)
        vbox.addWidget(self.warn_strip)

        # --- middle: beam view + plots ---
        mid = QHBoxLayout()
        left = QVBoxLayout()
        self.beam = BeamView(beam_cm)
        self.beam.setpoint_clicked.connect(self.send_setpoint)
        left.addWidget(self.beam)

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
        self.sp_slider.valueChanged.connect(lambda v: self.sp_spin.setValue(v / 10))
        self.sp_spin.valueChanged.connect(lambda v: self.sp_slider.setValue(int(v * 10)))
        self.sp_slider.sliderReleased.connect(lambda: self.send_setpoint(self.sp_spin.value()))
        self.sp_spin.editingFinished.connect(lambda: self.send_setpoint(self.sp_spin.value()))
        sp_lay.addWidget(self.sp_slider)
        sp_lay.addWidget(self.sp_spin)
        left.addWidget(sp_box)

        # gains
        g_box = QGroupBox("PID kazançları")
        g_lay = QGridLayout(g_box)
        self.gain_spins = {}
        for col, (key, label, rng, step) in enumerate((("kp", "Kp °/cm", (0, 3), 0.05), ("ki", "Ki", (0, 1), 0.05),
                                                        ("kd", "Kd °·s/cm", (0, 2), 0.02), ("tau", "τ_D s", (0, 1), 0.01))):
            g_lay.addWidget(QLabel(label), 0, col)
            sb = QDoubleSpinBox()
            sb.setRange(*rng)
            sb.setSingleStep(step)
            sb.setDecimals(3)
            g_lay.addWidget(sb, 1, col)
            self.gain_spins[key] = sb
        self.gain_status = QLabel("CONFIG bekleniyor")
        self.btn_apply = QPushButton("Uygula")
        self.btn_apply.clicked.connect(self.send_gains)
        self.btn_defaults = QPushButton("Varsayılana dön")
        self.btn_defaults.clicked.connect(self.restore_defaults)
        g_lay.addWidget(self.gain_status, 2, 0, 1, 2)
        g_lay.addWidget(self.btn_apply, 2, 2)
        g_lay.addWidget(self.btn_defaults, 2, 3)
        left.addWidget(g_box)

        # modes
        m_box = QGroupBox("Mod")
        m_lay = QHBoxLayout(m_box)
        for text, mode in (("RUN", P.MODE_RUN), ("LEVEL", P.MODE_LEVEL), ("STOP", P.MODE_STOP), ("RESET_FAULT", P.MODE_RESET_FAULT)):
            b = QPushButton(text)
            b.clicked.connect(lambda _=False, m=mode: self.send_mode(m))
            m_lay.addWidget(b)
        left.addWidget(m_box)

        # health
        h_box = QGroupBox("Durum / sağlık")
        h_lay = QGridLayout(h_box)
        self.h_labels = {}
        for row, (key, title) in enumerate((("state", "Durum"), ("loop", "Döngü"), ("link", "Link"),
                                             ("enc", "Encoder"), ("tmc", "TMC2208"), ("cam", "Kamera"))):
            h_lay.addWidget(QLabel(title + ":"), row, 0)
            lb = QLabel("--")
            lb.setFont(QFont("Consolas", 9))
            h_lay.addWidget(lb, row, 1)
            self.h_labels[key] = lb
        left.addWidget(h_box)

        # record / view / plots
        r_lay = QHBoxLayout()
        self.btn_rec = QPushButton("● Kayıt başlat")
        self.btn_rec.setCheckable(True)
        self.btn_rec.toggled.connect(self.toggle_record)
        self.btn_view = QPushButton("Kamera görüntüsü")
        self.btn_view.setCheckable(True)
        self.btn_view.setToolTip("Takip penceresini aç/kapat (bant, tespit, konum)")
        self.btn_view.toggled.connect(lambda on: self.camera.set_view(on))
        self.btn_center = QPushButton("Grafikleri ortala")
        self.btn_center.setToolTip("Grafikleri canlı pencereye ve varsayılan ölçeğe döndür")
        self.btn_center.clicked.connect(lambda: self.plots.recenter())
        r_lay.addWidget(self.btn_rec)
        r_lay.addWidget(self.btn_view)
        r_lay.addWidget(self.btn_center)
        left.addLayout(r_lay)
        self.rec_label = QLabel("")
        left.addWidget(self.rec_label)
        left.addStretch(1)

        leftw = QWidget()
        leftw.setLayout(left)
        leftw.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        leftw.setMaximumWidth(520)
        mid.addWidget(leftw, 0)
        self.plots = Plots()
        mid.addWidget(self.plots, 1)
        vbox.addLayout(mid, 1)

        # --- camera/link process signals ---
        self.camera.status.connect(self.on_camera_status)
        self.camera.position.connect(self.on_position)
        self.cam_status = "başlatılıyor"

        # --- timers ---
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(40)
        self.rate_timer = QTimer(self)
        self.rate_timer.timeout.connect(self.rates)
        self.rate_timer.start(1000)

        self.resize(1400, 850)

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
        if self.config is None:
            return
        d = self.defaults
        self.gain_spins["kp"].setValue(d[0])
        self.gain_spins["ki"].setValue(d[1])
        self.gain_spins["kd"].setValue(d[2])
        self.gain_spins["tau"].setValue(d[3])
        self.send_gains()

    def send_mode(self, mode):
        self.link.set_mode(mode)

    def toggle_record(self, on):
        if on:
            path = self.recorder.start()
            self.btn_rec.setText("■ Kaydı durdur")
            self.rec_label.setText(path)
        else:
            self.recorder.stop()
            self.btn_rec.setText("● Kayıt başlat")
            self.rec_label.setText(f"{self.recorder.rows} satır -> {self.recorder.path}")

    # --- incoming ------------------------------------------------------------
    @Slot(str)
    def on_camera_status(self, text):
        self.cam_status = text
        self.h_labels["cam"].setText(text)

    @Slot(float, bool, bool, float)
    def on_position(self, pos_cm, valid, warning, t_send):
        if self.recorder.active:
            self.recorder.sent(pos_cm, valid, t_send)
        if warning:
            self.warnings["cam"] = ("warn", "Kamera: takip sağlığı uyarısı (alan/oran)")
        else:
            self.warnings.pop("cam", None)

    def on_config(self, c):
        first = self.config is None
        self.config = c
        self.defaults = (c.kp_def, c.ki_def, c.kd_def, c.d_tau_def)
        self.gain_spins["kp"].setValue(c.kp)
        self.gain_spins["ki"].setValue(c.ki)
        self.gain_spins["kd"].setValue(c.kd)
        self.gain_spins["tau"].setValue(c.d_tau_s)
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
    def refresh(self):
        if self.link.connected:
            rtt = self.link.stats["rtt_ms"]
            self.conn_strip.setText(f"ESP32 bağlı ({self.link.port})   RTT {rtt:.1f} ms   telemetri {self.telem_rate}/s   "
                                    f"kamera {self.cam_rate} fps" if rtt is not None else f"ESP32 bağlı ({self.link.port})")
            self.conn_strip.setStyleSheet("background:#2e7d32; color:white; padding:6px; font-weight:bold;")
        else:
            self.conn_strip.setText(f"ESP32 BAĞLI DEĞİL -- yeniden aranıyor (gönderilemeyen {self.link.stats['tx_dropped']})")
            self.conn_strip.setStyleSheet("background:#c62828; color:white; padding:6px; font-weight:bold;")

        t = self.last_telem
        if t:
            valid = bool(t.flags & P.TF_BALL_VALID)
            state = P.STATE_NAMES.get(t.state, str(t.state))
            self.beam.update_state(t.x_0p1mm / 100 if valid else None, valid, t.x_set_0p1mm / 100, t.theta / 100, state)
            self.h_labels["state"].setText(f"{state}   x {t.x_0p1mm / 100:6.2f} cm   θ {t.theta / 100:+5.2f}°   "
                                           f"krank {t.enc_counts:4d}   takip {t.follow_0p1 / 10:+5.1f}")
            # --- warnings from telemetry ---
            if t.state == 4:
                self.warnings["fault"] = ("fault", "ARIZA -- " + FAULT_TEXT.get(t.fault, P.FAULT_NAMES.get(t.fault, str(t.fault))))
            else:
                self.warnings.pop("fault", None)
            if t.state == 5:
                self.warnings["stop"] = ("warn", "STOP: sürücü kapalı (RUN / RESET_FAULT ile devam)")
            else:
                self.warnings.pop("stop", None)
            if abs(t.follow_0p1) > 1000:
                self.warnings["follow"] = ("warn", f"Takip hatası yüksek: {t.follow_0p1 / 10:.0f} sayım (limit 150)")
            else:
                self.warnings.pop("follow", None)
            if t.flags & P.TF_LINK_STALE and t.state == 3:
                self.warnings["stale"] = ("warn", "Konum paketi bayat")
            else:
                self.warnings.pop("stale", None)

        h = self.last_health
        if h:
            self.h_labels["loop"].setText(f"{h.loop_us_mean}/{h.loop_us_max} µs   aşım {h.overruns}   kaçırılan {h.missed}")
            self.h_labels["link"].setText(f"paket {h.link_packets}  crc {h.link_crc_errors}  boşluk {h.link_seq_gaps}  "
                                          f"komut ok/kötü {h.cmd_rx}/{h.cmd_bad}  tx düşen {h.tx_dropped}")
            self.h_labels["enc"].setText(f"i2c hata {h.enc_i2c_errors}  ret {h.enc_rejects}  DIR arıza {h.enc_dir_faults}")
            d = h.drv_status
            cs = (d >> 16) & 0x1F
            flags = []
            if d & DRV_OTPW: flags.append("OTPW")
            if d & DRV_OT: flags.append("OT")
            if d & (DRV_S2GA | DRV_S2GB): flags.append("KISA DEVRE")
            if d & (DRV_S2VSA | DRV_S2VSB): flags.append("S2VS")
            if d & (DRV_OLA | DRV_OLB): flags.append("açık faz")
            temp = "157°" if d & DRV_T157 else "150°" if d & DRV_T150 else "143°" if d & DRV_T143 else "120°" if d & DRV_T120 else "<120°"
            self.h_labels["tmc"].setText(f"CS {cs}  {temp}  reset {h.tmc_resets}  uart hata {h.tmc_uart_errors}  "
                                         f"{' '.join(flags) if flags else 'bayrak yok'}   drv 0x{d:08X}")
            # --- warnings from health (encoder / driver, as requested) ---
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
                self.warnings["tmc_ol"] = ("warn", "TMC2208 açık faz bayrağı hareket halinde: motor kablosu / bağlantı")
            else:
                self.warnings.pop("tmc_ol", None)
            p = self.prev_health
            if p and h.enc_i2c_errors > p.enc_i2c_errors:
                self.warnings["enc_i2c"] = ("warn", f"Encoder I2C hataları artıyor (+{h.enc_i2c_errors - p.enc_i2c_errors}/s)")
            elif p and h.enc_i2c_errors == p.enc_i2c_errors:
                self.warnings.pop("enc_i2c", None)
            if p and h.enc_rejects > p.enc_rejects:
                self.warnings["enc_rej"] = ("warn", f"Encoder makullük retleri (+{h.enc_rejects - p.enc_rejects}/s): gürültü / DIR pini")
            elif p and h.enc_rejects == p.enc_rejects:
                self.warnings.pop("enc_rej", None)
            if p and h.tmc_uart_errors > p.tmc_uart_errors:
                self.warnings["tmc_uart"] = ("warn", "TMC2208 UART hataları artıyor")
            elif p and h.tmc_uart_errors == p.tmc_uart_errors:
                self.warnings.pop("tmc_uart", None)
            if h.overruns:
                self.warnings["overrun"] = ("warn", f"Kontrol döngüsü aşımı: {h.overruns}/s")
            else:
                self.warnings.pop("overrun", None)

        if not self.link.connected:
            self.warnings.pop("fault", None)

        # warning strip: faults first, then warnings
        if self.warnings:
            faults = [txt for lvl, txt in self.warnings.values() if lvl == "fault"]
            warns = [txt for lvl, txt in self.warnings.values() if lvl == "warn"]
            self.warn_strip.setText("  |  ".join(faults + warns))
            self.warn_strip.setStyleSheet("background:%s; color:white; padding:6px; font-weight:bold;" % ("#c62828" if faults else "#ef6c00"))
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
