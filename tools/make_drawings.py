"""Generate the mechanical drawings in hardware/drawings/ from the same
geometry the firmware uses (config.h). Run after changing any linkage
dimension:  python tools/make_drawings.py
"""
import math
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, "bb_esp32s3", "src", "config.h")
if not os.path.exists(CFG):                      # after the repo merge
    CFG = os.path.join(ROOT, "firmware", "src", "config.h")
OUT = os.path.join(ROOT, "hardware", "drawings")


def const(name):
    """Read a float #define out of config.h so the drawing cannot drift."""
    src = open(CFG, encoding="utf-8").read()
    m = re.search(r"#define\s+%s\s+\(?([-0-9.]+)f?\)?" % name, src)
    if not m:
        raise SystemExit("config.h icinde bulunamadi: " + name)
    return float(m.group(1))


R = const("LINK_CRANK_R_MM")
L = const("LINK_ROD_L_MM")
D = const("LINK_BEAM_D_MM")
DY = const("LINK_PIVOT_DY_MM")
DX = D - R
TH_MAX = const("BEAM_THETA_MAX_DEG")
CMD_MAX = const("CRANK_CMD_MAX_COUNTS")
COUNTS_REV = const("ENC_COUNTS_PER_REV")
PHI_LIMIT = CMD_MAX * 360.0 / COUNTS_REV

# --- kinematics, identical to firmware/src/linkage.c ------------------------


def pin(phi_deg):
    p = math.radians(phi_deg)
    return -R * math.cos(p), R * math.sin(p)


def beam_end(th_deg):
    t = math.radians(th_deg)
    return DX - D * math.cos(t), DY + D * math.sin(t)


def theta_from_phi(phi_deg):
    cx, cy = pin(phi_deg)
    s = max(-1.0, min(1.0, R * math.sin(math.radians(phi_deg)) / D))
    th = math.asin(s)
    for _ in range(6):
        bx, by = beam_end(math.degrees(th))
        ex, ey = bx - cx, by - cy
        f = ex * ex + ey * ey - L * L
        df = 2.0 * (ex * D * math.sin(th) + ey * D * math.cos(th))
        if abs(df) < 1e-9:
            break
        th -= f / df
    return math.degrees(th)


# --- common SVG helpers ----------------------------------------------------

NL = chr(10)
INK, MUTED, ACCENT, WARN = "#1f2328", "#6e7781", "#0969da", "#bc4c00"
FONT = "font-family='Segoe UI, Helvetica, Arial, sans-serif'"


def head(w, h, title):
    return [
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 %d %d' width='%d' height='%d' role='img'>"
        % (w, h, w, h),
        "<title>%s</title>" % title,
        "<rect x='0' y='0' width='%d' height='%d' rx='8' fill='#ffffff' stroke='#d0d7de'/>" % (w, h),
    ]


def txt(x, y, s, size=13, fill=INK, anchor="start", weight="400"):
    return ("<text x='%.1f' y='%.1f' %s font-size='%d' fill='%s' "
            "text-anchor='%s' font-weight='%s'>%s</text>"
            % (x, y, FONT, size, fill, anchor, weight, s))


def line(x1, y1, x2, y2, stroke=INK, w=1.6, dash=None):
    d = " stroke-dasharray='%s'" % dash if dash else ""
    return ("<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' "
            "stroke-width='%.1f' stroke-linecap='round'%s/>" % (x1, y1, x2, y2, stroke, w, d))


def dot(x, y, r=4.2, fill="#ffffff", stroke=INK, w=1.8):
    return ("<circle cx='%.1f' cy='%.1f' r='%.1f' fill='%s' stroke='%s' stroke-width='%.1f'/>"
            % (x, y, r, fill, stroke, w))


# --- drawing 1: dimensioned side view -------------------------------------

def linkage_svg():
    W, H = 960, 430
    k, ox, oy = 1.45, 150.0, 300.0          # mm -> px, shaft at (ox, oy)
    GND = -46.0                             # drawn ground line, mm below the shaft

    def P(x, y):
        return ox + k * x, oy - k * y

    s = head(W, H, "Ball and beam baglama geometrisi")
    s.append("<defs><marker id='a' viewBox='0 0 10 10' refX='9' refY='5' markerWidth='7' "
             "markerHeight='7' orient='auto'><path d='M0,1 L9,5 L0,9' fill='none' stroke='%s' "
             "stroke-width='1.4'/></marker></defs>" % MUTED)
    s.append(txt(24, 32, "Bağlama geometrisi — yan görünüş, ölçekli", 15, INK, weight="600"))
    s.append(txt(24, 52, "Krank–biyel–beam. Ölçüler firmware/src/config.h ile aynı kaynaktan üretildi.",
                 12, MUTED))

    # base line and the two towers
    bx0, by0 = P(-62, GND)
    bx1, _ = P(DX + 52, GND)
    s.append(line(bx0, by0, bx1, by0, MUTED, 2.0))
    for x0, x1, top in ((-24, 24, -6.0), (DX - 22, DX + 22, DY - 8)):
        ax, ay = P(x0, top)
        bxx, byy = P(x1, GND)
        s.append("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#f6f8fa' "
                 "stroke='%s' stroke-width='1.3'/>" % (ax, ay, bxx - ax, byy - ay, MUTED))
    s.append(txt(P(0, 0)[0], P(0, GND)[1] + 18, "motor kulesi (sabit)", 11, MUTED, "middle"))
    s.append(txt(P(DX, 0)[0], P(0, GND)[1] + 18, "mafsal kulesi (sabit)", 11, MUTED, "middle"))

    # crank circle
    cx0, cy0 = P(0, 0)
    s.append("<circle cx='%.1f' cy='%.1f' r='%.1f' fill='none' stroke='%s' stroke-width='1.1' "
             "stroke-dasharray='4 4'/>" % (cx0, cy0, R * k, MUTED))

    # beam at +/- theta_max (dashed) and level (solid)
    px, py = P(DX, DY)
    for sign in (1, -1):
        bxm, bym = beam_end(sign * TH_MAX)
        x2, y2 = P(bxm, bym)
        s.append(line(px, py, x2, y2, ACCENT, 1.2, dash="5 5"))
    b0x, b0y = beam_end(0.0)
    bpx, bpy = P(b0x, b0y)
    s.append(line(bpx, bpy, px, py, INK, 5.0))
    s.append(txt((bpx + px) / 2, bpy + 48, "beam — izlenen yol ≈ 45 cm", 12, MUTED, "middle"))

    # ball on the beam
    ballx, bally = P(b0x + 0.62 * D, DY)
    s.append("<circle cx='%.1f' cy='%.1f' r='8' fill='#fb8500' stroke='%s' stroke-width='1.2'/>"
             % (ballx, bally - 10, INK))
    s.append(txt(ballx, bally - 24, "top", 11, WARN, "middle", "600"))

    # crank arm + rod at the level position (phi = 0, pin at 9 o'clock)
    cpx, cpy = pin(0.0)
    cx, cy = P(cpx, cpy)
    s.append(line(cx0, cy0, cx, cy, WARN, 4.0))
    s.append(line(cx, cy, bpx, bpy, WARN, 3.0))
    for x, y, lab, dxx, dyy, anc in ((cx0, cy0, "motor mili", 52, 5, "start"),
                                     (cx, cy, "krank pimi", -10, 6, "end"),
                                     (bpx, bpy, "p1 — beam ucu (hareketli)", 10, -14, "start"),
                                     (px, py, "p2 — mafsal (sabit)", -10, -14, "end")):
        s.append(dot(x, y))
        s.append(txt(x + dxx, y + dyy, lab, 11, MUTED, anc))

    # dimension r
    s.append(line(cx0, cy0 + 14, cx, cy + 14, MUTED, 1.1))
    s.append(txt((cx0 + cx) / 2, cy0 + 30, "r = %.0f mm" % R, 11, WARN, "middle", "600"))
    # dimension l (rod)
    s.append(txt((cx + bpx) / 2 + 12, (cy + bpy) / 2, "l = %.0f mm" % L, 11, WARN, "start", "600"))
    # dimension d
    dyl = P(0, GND + 12)[1]
    s.append("<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-width='1.1' "
             "marker-end='url(#a)'/>" % (bpx + 4, dyl, px - 4, dyl, MUTED))
    s.append(txt((bpx + px) / 2, dyl - 8, "d = %.0f mm  (mafsal pimi ↔ beam ucu pimi)" % D,
                 11, INK, "middle", "600"))
    # dimension pivot height
    hx = P(DX + 40, 0)[0]
    s.append(line(hx, P(0, 0)[1], hx, P(0, DY)[1], MUTED, 1.1))
    s.append(txt(hx + 8, (P(0, 0)[1] + P(0, DY)[1]) / 2, "%.0f mm" % DY, 11, INK, "start", "600"))
    s.append(txt(hx + 8, (P(0, 0)[1] + P(0, DY)[1]) / 2 + 15, "mafsal", 10, MUTED, "start"))
    s.append(txt(hx + 8, (P(0, 0)[1] + P(0, DY)[1]) / 2 + 27, "yüksekliği", 10, MUTED, "start"))

    s.append(txt(bpx + 0.62 * (px - bpx), bpy - 56, "θ = ±%.2f°  (komut sınırı)" % TH_MAX,
                 12, ACCENT, "middle", "600"))
    s.append(txt(24, H - 26, "Fiziksel düzenekte mafsal kulesi ve motor kulesi sabittir; hareket eden "
                 "tek nokta beam'in krank tarafındaki ucudur.", 11, MUTED))
    s.append(txt(24, H - 10, "θ(φ) dönüşümü doğrusal değil ve simetrik değil — theta_vs_phi.svg.",
                 11, MUTED))
    s.append("</svg>")
    return "\n".join(s)


# --- drawing 2: theta(phi) characteristic ---------------------------------

def characteristic_svg():
    W, H = 720, 450
    l_, r_, t_, b_ = 78.0, 34.0, 92.0, 74.0
    phi = [i * 0.5 for i in range(-220, 221)]
    th = [theta_from_phi(p) for p in phi]
    peak_i = max(range(len(th)), key=lambda i: th[i])
    xr = (-110.0, 110.0)
    yr = (-4.2, 4.2)

    def X(v):
        return l_ + (v - xr[0]) / (xr[1] - xr[0]) * (W - l_ - r_)

    def Y(v):
        return H - b_ - (v - yr[0]) / (yr[1] - yr[0]) * (H - t_ - b_)

    s = head(W, H, "theta(phi) karakteristigi")
    s.append(txt(24, 32, "Beam açısı θ'nın krank açısı φ'ye bağımlılığı", 15, INK, weight="600"))
    s.append(txt(24, 50, "Aynı Newton çözümü firmware'de linkage.c içinde çalışıyor.", 12, MUTED))

    # working band
    s.append("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#0969da' opacity='0.07'/>"
             % (X(-PHI_LIMIT), Y(TH_MAX), X(PHI_LIMIT) - X(-PHI_LIMIT), Y(-TH_MAX) - Y(TH_MAX)))
    for v in (TH_MAX, -TH_MAX):
        s.append(line(X(xr[0]), Y(v), X(xr[1]), Y(v), ACCENT, 1.1, dash="5 4"))
    s.append(txt(X(xr[0]) + 8, Y(TH_MAX) - 7, "θ sınırı ±%.2f°" % TH_MAX, 11, ACCENT, "start", "600"))

    # axes + ticks
    s.append(line(l_, Y(0), W - r_, Y(0), MUTED, 1.2))
    s.append(line(l_, t_ - 8, l_, H - b_, MUTED, 1.2))
    for v in range(-100, 101, 25):
        s.append(line(X(v), Y(0) - 4, X(v), Y(0) + 4, MUTED, 1.0))
        s.append(txt(X(v), H - b_ + 20, "%d" % v, 11, MUTED, "middle"))
    for v in (-4, -3, -2, -1, 1, 2, 3, 4):
        s.append(line(l_ - 4, Y(v), l_ + 4, Y(v), MUTED, 1.0))
        s.append(txt(l_ - 9, Y(v) + 4, "%d" % v, 11, MUTED, "end"))
    s.append(txt((l_ + W - r_) / 2, H - b_ + 42, "φ  krank açısı [°]   (0 = saat 9, pim kuleden uzakta)",
                 11, MUTED, "middle"))
    s.append("<text x='22' y='%.1f' %s font-size='11' fill='%s' text-anchor='middle' "
             "transform='rotate(-90 22 %.1f)'>θ  beam açısı [°]</text>"
             % ((t_ + H - b_) / 2, FONT, MUTED, (t_ + H - b_) / 2))

    # curve
    pts = " ".join("%.1f,%.1f" % (X(p), Y(t)) for p, t in zip(phi, th))
    s.append("<polyline points='%s' fill='none' stroke='%s' stroke-width='2.4'/>" % (pts, INK))

    # peaks
    for i in (peak_i, len(th) - 1 - peak_i):
        s.append("<circle cx='%.1f' cy='%.1f' r='4' fill='%s'/>" % (X(phi[i]), Y(th[i]), WARN))
    s.append(txt(X(phi[peak_i]) - 14, Y(th[peak_i]) - 24,
                 "tepe: φ = %.1f°, θ = %.3f°" % (phi[peak_i], th[peak_i]), 11, WARN, "end", "600"))
    s.append(txt(X(phi[peak_i]) - 14, Y(th[peak_i]) - 10,
                 "sonrasında θ geri döner → tekillik", 11, WARN, "end"))

    # command limit markers
    for v in (-PHI_LIMIT, PHI_LIMIT):
        s.append(line(X(v), Y(yr[0]), X(v), Y(yr[1]), ACCENT, 1.1, dash="3 4"))
    s.append(txt(X(0.0), t_ - 12, "komut aralığı  φ = ±%.1f°  (±%.0f sayım)" % (PHI_LIMIT, CMD_MAX),
                 11, ACCENT, "middle", "600"))
    s.append(txt(24, H - 16, "%%4 emniyet payı: komut sınırı tepe değerinin altında kalıyor, mekanizma "
                 "tekilliğe hiç girmiyor." % (), 11, MUTED))
    s.append("</svg>")
    return "\n".join(s)



# --- drawing 3: signal flow ------------------------------------------------

def architecture_svg():
    W, H = 940, 496
    s = head(W, H, "Sinyal akisi")
    s.append(txt(24, 32, "Sinyal akışı", 15, INK, weight="600"))
    s.append(txt(24, 52, "Konum PC'de görüntüden çıkıyor, denetim ESP32-S3'te 250 Hz'de koşuyor.",
                 12, MUTED))

    def box(x, y, w, h, title, lines, accent=INK, fill="#f6f8fa"):
        out = ["<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' rx='7' fill='%s' "
               "stroke='%s' stroke-width='1.6'/>" % (x, y, w, h, fill, accent)]
        out.append(txt(x + 12, y + 22, title, 12, accent, weight="600"))
        for i, ln in enumerate(lines):
            out.append(txt(x + 12, y + 41 + i * 15, ln, 10.5, MUTED))
        return out

    def arrow(x1, y1, x2, y2, label, above=True, color=ACCENT):
        out = ["<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-width='1.8' "
               "marker-end='url(#ar)'/>" % (x1, y1, x2, y2, color)]
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        out.append(txt(mx, my - 7 if above else my + 15, label, 10.5, color, "middle", "600"))
        return out

    s.append("<defs><marker id='ar' viewBox='0 0 10 10' refX='9' refY='5' markerWidth='8' "
             "markerHeight='8' orient='auto'><path d='M0,1 L9,5 L0,9' fill='%s'/></marker></defs>"
             % ACCENT)

    s += box(24, 92, 196, 92, "PS3 Eye kamera",
             ["320x240 @ 100 fps", "libusbK + pseyepy", "pozlama/kazanç sabit"])
    s += box(24, 232, 196, 118, "PC — Python",
             ["tracker.py: HSV eşiği,", "ışın üzerine izdüşüm", "ui_main.py: PySide6 arayüz",
              "kamera ve link ayrı süreçte"])
    s += box(372, 232, 210, 118, "ESP32-S3", ["4 ms GPTimer ISR (250 Hz)",
             "pid.c: ölçüm türevli PID", "linkage.c: θ → krank açısı",
             "korkuluklar: θ, takip, WDT"], ACCENT, "#ddf4ff")
    s += box(716, 92, 200, 92, "AS5600 encoder",
             ["4096 sayım/tur, 400 kHz", "LUT düzeltmesi", "makullük filtresi"])
    s += box(716, 232, 200, 118, "TMC2208 + NEMA 17",
             ["1/256 mikro adım, 0,61 A", "LEDC darbe + PCNT sayım",
              "krank r=31 → biyel l=90", "→ beam θ = ±3,13°"])
    s += box(372, 392, 210, 56, "top — 45 cm ray", ["kapalı çevrim buradan kapanıyor"], WARN, "#fff1e5")

    s += arrow(122, 184, 122, 232, "kare", True, MUTED)
    s += arrow(220, 268, 372, 268, "USB ikili çerçeve", True)
    s += arrow(372, 314, 220, 314, "TELEM 125 Hz", False)
    s += arrow(582, 268, 716, 268, "adım / yön", True)
    s += arrow(816, 232, 816, 184, "açı", True, MUTED)
    s += arrow(716, 152, 582, 240, "krank φ", True, MUTED)
    s += arrow(716, 330, 582, 410, "beam açısı", False, WARN)
    s += arrow(372, 420, 122, 350, "topun görüntüsü", False, WARN)

    s.append(txt(24, H - 20, "POS 100 Hz (PC→ESP) · TELEM 125 Hz + HEALTH 1 Hz (ESP→PC) · "
                 "AA 55 | tip | uzunluk | yük ≤64 | crc8", 10.5, MUTED))
    s.append("</svg>")
    return NL.join(s)


os.makedirs(OUT, exist_ok=True)
for name, svg in (("linkage.svg", linkage_svg()),
                  ("theta_vs_phi.svg", characteristic_svg()),
                  ("architecture.svg", architecture_svg())):
    open(os.path.join(OUT, name), "w", encoding="utf-8", newline="\n").write(svg)
    print("yazildi: hardware/drawings/%s (%d bayt)" % (name, len(svg)))
print("r=%.0f l=%.0f d=%.0f dy=%.0f  theta_max=%.2f  phi_limit=%.1f"
      % (R, L, D, DY, TH_MAX, PHI_LIMIT))
print("tepe: phi=75.5 -> theta=%.3f" % theta_from_phi(75.5))
