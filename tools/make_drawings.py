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
    tower_mid = (P(-24, 0)[0] + P(24, 0)[0]) / 2
    s.append(txt(tower_mid, cy0 + 40, "r = %.0f mm" % R, 11, WARN, "middle", "600"))
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
    yr = (-4.8, 4.8)

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
    s.append(txt(X(xr[1]) - 6, Y(th[peak_i]) + 58,
                 "tepe  φ %.1f°  θ %.3f°" % (phi[peak_i], th[peak_i]), 11, WARN, "end", "600"))
    s.append(txt(X(xr[1]) - 6, Y(th[peak_i]) + 73, "sonrası tekillik", 11, WARN, "end"))

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
    s = head(W, H, "Signal flow")
    s.append(txt(24, 32, "Signal flow", 15, INK, weight="600"))
    s.append(txt(24, 52, "The ball's position is measured on the PC from the camera image; "
                 "the control loop runs on the ESP32-S3 at 250 Hz.", 12, MUTED))

    def box(x, y, w, h, title, lines, accent=INK, fill="#f6f8fa"):
        out = ["<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' rx='7' fill='%s' "
               "stroke='%s' stroke-width='1.6'/>" % (x, y, w, h, fill, accent)]
        out.append(txt(x + 12, y + 22, title, 12, accent, weight="600"))
        for i, ln in enumerate(lines):
            out.append(txt(x + 12, y + 41 + i * 15, ln, 10.5, MUTED))
        return out

    def arrow(x1, y1, x2, y2, label, above=True, color=ACCENT):
        """The label is pushed off the arrow along its normal, so it clears the
        line whatever direction the arrow runs in."""
        out = ["<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-width='1.8' "
               "marker-end='url(#ar)'/>" % (x1, y1, x2, y2, color)]
        dx, dy = x2 - x1, y2 - y1
        ln = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / ln, dx / ln          # unit normal
        # the normal of a left-to-right arrow points down, so "above" is -n
        side = -1.0 if above else 1.0
        half = 0.27 * 10.5 * len(label)     # rough half-width of the label
        if abs(ny) > abs(nx):               # mostly horizontal arrow
            # A slanted arrow would otherwise cut through the ends of a wide
            # label, so the clearance grows with the slope and the width.
            slope = abs(dy / dx) if abs(dx) > 1e-6 else 0.0
            anchor = "middle"
            mx = (x1 + x2) / 2
            my = (y1 + y2) / 2 + side * (13.0 + half * slope) + 4
        else:                               # vertical or steep: push sideways
            off, anchor = 10.0, ("start" if nx * side > 0 else "end")
            mx = (x1 + x2) / 2 + nx * side * off
            my = (y1 + y2) / 2 + 4
        out.append(txt(mx, my, label, 10.5, color, anchor, "600"))
        return out

    s.append("<defs><marker id='ar' viewBox='0 0 10 10' refX='9' refY='5' markerWidth='8' "
             "markerHeight='8' orient='auto'><path d='M0,1 L9,5 L0,9' fill='%s'/></marker></defs>"
             % ACCENT)

    s += box(24, 92, 196, 92, "PS3 Eye camera",
             ["320x240 @ 100 fps", "libusbK + pseyepy", "fixed exposure and gain"])
    s += box(24, 232, 196, 118, "PC — Python",
             ["tracker.py: HSV threshold,", "projection onto the rail axis",
              "ui_main.py: PySide6 GUI", "camera and link: own processes"])
    s += box(372, 232, 210, 118, "ESP32-S3", ["4 ms GPTimer ISR (250 Hz)",
             "pid.c: measurement-derivative PID", "linkage.c: θ → crank angle",
             "guards: θ, follow error, WDT"], ACCENT, "#ddf4ff")
    s += box(716, 92, 200, 92, "AS5600 encoder",
             ["4096 counts/rev, 400 kHz", "look-up table correction", "plausibility filter"])
    s += box(716, 232, 200, 118, "TMC2208 + NEMA 17",
             ["1/256 microstep, 0.61 A rms", "LEDC pulses + PCNT count",
              "crank r=31 → rod l=90", "→ beam θ = ±3.13°"])
    s += box(372, 392, 210, 56, "ball — 45 cm rail", ["the loop closes here"], WARN, "#fff1e5")

    s += arrow(122, 184, 122, 232, "frames", True, MUTED)
    s += arrow(220, 268, 372, 268, "USB binary frames", True)
    s += arrow(372, 314, 220, 314, "TELEM 125 Hz", False)
    s += arrow(582, 268, 716, 268, "step / dir", True)
    s += arrow(816, 232, 816, 184, "shaft angle", True, MUTED)
    s += arrow(716, 152, 582, 240, "crank φ", True, MUTED)
    s += arrow(716, 330, 582, 410, "beam angle", False, WARN)
    s += arrow(372, 420, 122, 350, "ball in view", False, WARN)

    s.append(txt(24, H - 20, "POS 100 Hz (PC→ESP) · TELEM 125 Hz + HEALTH 1 Hz (ESP→PC) · "
                 "frame: AA 55 | type | len | payload ≤64 B | crc8", 10.5, MUTED))
    s.append("</svg>")
    return NL.join(s)


# --- drawing 4: wiring ------------------------------------------------------

def wiring_svg():
    """Connection diagram: which pin goes where, and the details that cost the
    most time to get right (single-wire UART, AS5600 DIR, common ground).

    Pins are stacked by the layout code rather than placed by hand, so a
    label can never land on the block header or on its neighbour.
    """
    W, H = 1180, 840
    RAIL = 636                      # common ground rail
    COLL = 916                      # ground collector, right of the blocks
    PITCH, GAP, HEAD = 23.0, 11.0, 52.0
    s = head(W, H, "Wiring")
    s.append(txt(24, 32, "Wiring", 15, INK, weight="600"))
    s.append(txt(24, 52, "Verified against the running rig. Pin numbers come from "
                 "firmware/src/board_pins.h, the same file the firmware builds against.",
                 12, MUTED))

    out = []
    pins = {}

    def block(name, x, y, w, title, sub, left=(), right=(), accent=INK, fill="#f6f8fa"):
        """Draw a block and remember where each of its pins sits."""
        def stack(items):
            ys, cur = {}, y + HEAD
            for it in items:
                if it is None:
                    cur += GAP
                    continue
                ys[it] = cur
                cur += PITCH
            return ys, cur

        ly, lend = stack(left)
        ry, rend = stack(right)
        h = max(lend, rend) - y + 6
        out.append("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' rx='7' fill='%s' "
                   "stroke='%s' stroke-width='1.6'/>" % (x, y, w, h, fill, accent))
        out.append(txt(x + 14, y + 24, title, 12.5, accent, weight="600"))
        if sub:
            out.append(txt(x + 14, y + 41, sub, 10, MUTED))
        for label, yy in ly.items():
            out.append(txt(x + 12, yy + 4, label, 10.5, INK))
            pins[(name, label)] = (x, yy)
        for label, yy in ry.items():
            colour = WARN if label.startswith("DIR") else INK
            out.append(txt(x + w - 12, yy + 4, label, 10.5, colour, "end"))
            pins[(name, label)] = (x + w, yy)
        return x, y, w, h

    def P(block_name, label):
        return pins[(block_name, label)]

    def wire(pts, color=INK, w=1.5):
        p = " ".join("%.1f,%.1f" % (x, y) for x, y in pts)
        out.append("<polyline points='%s' fill='none' stroke='%s' stroke-width='%.1f' "
                   "stroke-linejoin='round' stroke-linecap='round'/>" % (p, color, w))

    def junction(x, y, color=INK):
        out.append("<circle cx='%.1f' cy='%.1f' r='3.4' fill='%s'/>" % (x, y, color))

    def resistor(x, y, label, color=INK, vertical=False, below=False):
        if vertical:
            out.append("<rect x='%.1f' y='%.1f' width='14' height='32' rx='2' fill='#ffffff' "
                       "stroke='%s' stroke-width='1.4'/>" % (x - 7, y - 16, color))
            out.append(txt(x + 13, y + 4, label, 10, color, "start", "600"))
        else:
            out.append("<rect x='%.1f' y='%.1f' width='36' height='13' rx='2' fill='#ffffff' "
                       "stroke='%s' stroke-width='1.4'/>" % (x - 18, y - 6.5, color))
            out.append(txt(x, y + 20 if below else y - 11, label, 10, color, "middle", "600"))

    def note(x, y, text, color=MUTED, anchor="start"):
        out.append(txt(x, y, text, 10.5, color, anchor, "600"))

    # ---- blocks ------------------------------------------------------------
    block("esp", 60, 96, 244, "ESP32-S3 DevKitC-1", "N8 · USB-Serial-JTAG + CH343 bridge",
          right=("3V3", None, "GPIO4  STEP", "GPIO5  DIR", "GPIO15  EN", None,
                 "GPIO6  UART TX", "GPIO7  UART RX", None, "GPIO8  SDA", "GPIO9  SCL",
                 None, "GPIO10  TRIG", "GPIO11  ECHO", "5V  (USB)", None,
                 "GPIO16  loop probe", "GND"))
    block("tmc", 636, 96, 250, "TMC2208", "UART mode · 1/256 · IRUN 10 → 0.61 A rms",
          left=("VIO 3V3", "STEP", "DIR", "EN", None, "PDN_UART", "CLK"),
          right=("VM +12 V", None, "M1A M1B M2A M2B", None, "GND"))
    block("enc", 636, 348, 250, "AS5600", "I²C 400 kHz · 4096 counts/rev",
          left=("VCC 3V3", "SDA", "SCL"), right=("GND", "DIR"))
    block("son", 636, 500, 250, "HC-SR04", "backup sensor, not in use",
          left=("VCC 5V", "TRIG", "ECHO"), right=("GND",))
    block("mot", 950, 96, 170, "NEMA 17", "200 steps/rev · 1/256 microstep")
    block("psu", 950, 180, 170, "12 V supply", "motor rail only", left=("+", "−"))

    # ---- 3V3 to both 3.3 V loads -------------------------------------------
    x3 = 332
    wire([P("esp", "3V3"), (x3, P("esp", "3V3")[1]), (x3, P("enc", "VCC 3V3")[1]),
          P("enc", "VCC 3V3")])
    wire([(x3, P("tmc", "VIO 3V3")[1]), P("tmc", "VIO 3V3")])
    junction(x3, P("tmc", "VIO 3V3")[1])

    # ---- step, direction, enable -------------------------------------------
    for name, tgt, mx in (("GPIO4  STEP", "STEP", 358), ("GPIO5  DIR", "DIR", 374),
                          ("GPIO15  EN", "EN", 390)):
        ex, ey = P("esp", name)
        tx, ty = P("tmc", tgt)
        wire([(ex, ey), (mx, ey), (mx, ty), (tx, ty)], ACCENT)

    # ---- single-wire UART ---------------------------------------------------
    node = 574
    tx_x, tx_y = P("esp", "GPIO6  UART TX")
    rx_x, rx_y = P("esp", "GPIO7  UART RX")
    px, py = P("tmc", "PDN_UART")
    wire([(tx_x, tx_y), (node, tx_y), (node, py), (px, py)], WARN, 1.8)
    resistor(460, tx_y, "1 kΩ", WARN)
    wire([(rx_x, rx_y), (node, rx_y), (node, py)], WARN, 1.8)
    junction(node, py, WARN)
    note(562, (tx_y + P("tmc", "EN")[1]) / 2 - 2, "single-wire half-duplex", WARN, "middle")

    # ---- I2C ----------------------------------------------------------------
    for name, tgt, mx in (("GPIO8  SDA", "SDA", 408), ("GPIO9  SCL", "SCL", 424)):
        ex, ey = P("esp", name)
        tx, ty = P("enc", tgt)
        wire([(ex, ey), (mx, ey), (mx, ty), (tx, ty)], ACCENT)

    # ---- ultrasonic: supply, trigger, divided echo --------------------------
    wire([P("esp", "5V  (USB)"), (556, P("esp", "5V  (USB)")[1]),
          (556, P("son", "VCC 5V")[1]), P("son", "VCC 5V")], MUTED)
    wire([P("esp", "GPIO10  TRIG"), (528, P("esp", "GPIO10  TRIG")[1]),
          (528, P("son", "TRIG")[1]), P("son", "TRIG")], MUTED)
    ex, ey = P("esp", "GPIO11  ECHO")
    sx, sy = P("son", "ECHO")
    div = 440
    wire([(sx, sy), (div, sy), (div, ey), (ex, ey)], MUTED)
    resistor(500, sy, "1 kΩ", MUTED, below=True)
    junction(div, sy, MUTED)
    wire([(div, sy), (div, RAIL)], MUTED)
    resistor(div, (sy + RAIL) / 2, "2 kΩ", MUTED, vertical=True)
    note(div - 14, (sy + RAIL) / 2 + 4, "5 V → 3.3 V", MUTED, "end")

    # ---- grounds ------------------------------------------------------------
    gx, gy = P("esp", "GND")
    wire([(gx, gy), (350, gy), (350, RAIL)])
    wire([P("tmc", "GND"), (COLL, P("tmc", "GND")[1]), (COLL, RAIL)])
    for blk in ("enc", "son"):
        wire([P(blk, "GND"), (COLL, P(blk, "GND")[1])])
        junction(COLL, P(blk, "GND")[1])
    wire([P("psu", "−"), (COLL, P("psu", "−")[1]), (COLL, P("tmc", "GND")[1])])
    cx, cy = P("tmc", "CLK")
    wire([(cx, cy), (610, cy), (610, RAIL)])
    junction(610, RAIL)
    wire([(120, RAIL), (COLL, RAIL)], INK, 2.4)
    note(124, RAIL + 20, "common ground", MUTED)

    # ---- the one that bites -------------------------------------------------
    dx, dy = P("enc", "DIR")
    wire([(dx, dy), (COLL - 20, dy), (COLL - 20, RAIL - 26), (COLL, RAIL - 26)], WARN, 2.0)
    junction(COLL, RAIL - 26, WARN)
    for k, line in enumerate(["DIR gets its own wire", "to ground — see note 2"]):
        note(COLL + 16, dy + 24 + k * 15, line, WARN)

    # ---- power and motor ----------------------------------------------------
    wire([P("psu", "+"), (906, P("psu", "+")[1]), (906, P("tmc", "VM +12 V")[1]),
          P("tmc", "VM +12 V")], WARN, 2.0)
    mx, my = P("tmc", "M1A M1B M2A M2B")
    wire([(mx, my), (932, my), (932, 125), (950, 125)], INK, 2.4)

    s += out
    notes = [
        "1.  PDN_UART is a single-wire half-duplex pin. TX reaches it through 1 kΩ, RX "
        "connects straight to the same node; without the resistor the ESP32 drives over "
        "the driver's reply and every read-back returns 0xFF.",
        "2.  AS5600 DIR must be tied to ground with its own wire. Left floating it picks up "
        "the I²C lines and flips polarity between reads, reporting the same angle "
        "alternately as x and 4096 − x.",
        "3.  HC-SR04 drives ECHO at 5 V, so it reaches GPIO11 through a 1 kΩ / 2 kΩ divider. "
        "The sensor is wired but unused: the ball's position comes from the camera.",
        "4.  MS1 and MS2 float — the microstep setting arrives over UART "
        "(mstep_reg_select = 1). CLK to ground selects the driver's internal 12 MHz "
        "oscillator. EN is active low. AS5600 OUT and GPO stay unconnected.",
        "5.  The AS5600 breakout carries its own 10 kΩ pull-ups, so the ESP32's internal "
        "ones are disabled. Bus runs at 400 kHz.",
        "6.  GPIO16 toggles once per control tick so a logic analyser can read the loop "
        "period independently of the firmware's own measurement.",
    ]
    box_h = 46 + len(notes) * 16 + 10
    ly = H - box_h - 18
    s.append("<rect x='40' y='%d' width='%d' height='%d' rx='7' fill='#f6f8fa' "
             "stroke='#d0d7de' stroke-width='1.2'/>" % (ly, W - 80, box_h))
    s.append(txt(58, ly + 24, "Notes", 12, INK, weight="600"))
    for k, line in enumerate(notes):
        s.append(txt(58, ly + 46 + k * 16, line, 10.5, MUTED))
    s.append("</svg>")
    return NL.join(s)


os.makedirs(OUT, exist_ok=True)
for name, svg in (("linkage.svg", linkage_svg()),
                  ("theta_vs_phi.svg", characteristic_svg()),
                  ("architecture.svg", architecture_svg()),
                  ("wiring.svg", wiring_svg())):
    open(os.path.join(OUT, name), "w", encoding="utf-8", newline="\n").write(svg)
    print("yazildi: hardware/drawings/%s (%d bayt)" % (name, len(svg)))
print("r=%.0f l=%.0f d=%.0f dy=%.0f  theta_max=%.2f  phi_limit=%.1f"
      % (R, L, D, DY, TH_MAX, PHI_LIMIT))
print("tepe: phi=75.5 -> theta=%.3f" % theta_from_phi(75.5))
