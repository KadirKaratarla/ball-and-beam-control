"""Check the generated drawings for overlaps before they reach the README.

Reading a diagram in a browser at half size hides collisions that are
obvious at full size, so this measures them instead: every label's box is
computed from the real font the SVG asks for, then tested against the other
labels, against the wires, and against the blocks.

    python tools/check_drawings.py            # all of hardware/drawings
    python tools/check_drawings.py a.svg b.svg

Exit code is the number of problems found, so it can gate a commit.
"""
import re
import sys
import glob
import os
from PIL import ImageFont

FONTS = {
    "400": r"C:\Windows\Fonts\segoeui.ttf",
    "600": r"C:\Windows\Fonts\seguisb.ttf",
}
_cache = {}


def font(size, weight):
    key = (round(size), weight)
    if key not in _cache:
        path = FONTS.get(weight, FONTS["400"])
        if not os.path.exists(path):
            path = FONTS["400"]
        _cache[key] = ImageFont.truetype(path, max(1, round(size)))
    return _cache[key]


TEXT_RE = re.compile(
    r"<text x='([-\d.]+)' y='([-\d.]+)'[^>]*?font-size='([\d.]+)'[^>]*?"
    r"text-anchor='(\w+)' font-weight='(\w+)'[^>]*?>(.*?)</text>", re.S)
LINE_RE = re.compile(r"<line x1='([-\d.]+)' y1='([-\d.]+)' x2='([-\d.]+)' y2='([-\d.]+)'")
POLY_RE = re.compile(r"<polyline points='([^']+)'")
RECT_RE = re.compile(r"<rect x='([-\d.]+)' y='([-\d.]+)' width='([-\d.]+)' height='([-\d.]+)'")
VIEW_RE = re.compile(r"viewBox='0 0 (\d+) (\d+)'")


def unescape(s):
    return s.replace("&amp;", "&").replace("&lt;", "<").replace("&gt;", ">")


def text_boxes(src):
    out = []
    for m in TEXT_RE.finditer(src):
        x, y, size, anchor, weight, body = m.groups()
        x, y, size = float(x), float(y), float(size)
        body = unescape(re.sub(r"<[^>]+>", "", body))
        f = font(size, weight)
        w = f.getlength(body)
        asc, desc = f.getmetrics()
        if anchor == "middle":
            x -= w / 2
        elif anchor == "end":
            x -= w
        # the drawn glyphs sit tighter than the full em box
        top = y - asc * 0.78
        h = asc * 0.78 + desc * 0.5
        out.append(dict(s=body, x=x, y=top, w=w, h=h))
    return out


def segments(src):
    segs = []
    for m in LINE_RE.finditer(src):
        segs.append(tuple(float(v) for v in m.groups()))
    for m in POLY_RE.finditer(src):
        pts = [tuple(float(v) for v in p.split(",")) for p in m.group(1).split()]
        for a, b in zip(pts, pts[1:]):
            segs.append((a[0], a[1], b[0], b[1]))
    return segs


def rects(src, vw, vh):
    out = []
    for m in RECT_RE.finditer(src):
        x, y, w, h = (float(v) for v in m.groups())
        if w * h < 0.5 * vw * vh and w > 40 and h > 28:
            out.append((x, y, w, h))
    return out


def seg_hits_rect(seg, r, pad=1.0):
    x1, y1, x2, y2 = seg
    rx0, ry0 = r["x"] - pad, r["y"] - pad
    rx1, ry1 = r["x"] + r["w"] + pad, r["y"] + r["h"] + pad
    if max(x1, x2) < rx0 or min(x1, x2) > rx1:
        return False
    if max(y1, y2) < ry0 or min(y1, y2) > ry1:
        return False

    def inside(x, y):
        return rx0 <= x <= rx1 and ry0 <= y <= ry1

    if inside(x1, y1) or inside(x2, y2):
        return True

    def cross(ax, ay, bx, by, cx, cy, dx, dy):
        d1 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        d2 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax)
        d3 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx)
        d4 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx)
        return (d1 > 0) != (d2 > 0) and (d3 > 0) != (d4 > 0)

    return (cross(x1, y1, x2, y2, rx0, ry0, rx1, ry0)
            or cross(x1, y1, x2, y2, rx1, ry0, rx1, ry1)
            or cross(x1, y1, x2, y2, rx1, ry1, rx0, ry1)
            or cross(x1, y1, x2, y2, rx0, ry1, rx0, ry0))


def check(path):
    src = open(path, encoding="utf-8").read()
    vw, vh = (int(v) for v in VIEW_RE.search(src).groups())
    ts = text_boxes(src)
    segs = segments(src)
    bs = rects(src, vw, vh)
    print("\n=== %s  (%dx%d, %d yazi, %d tel)" % (os.path.basename(path), vw, vh, len(ts), len(segs)))

    pad = 1.5
    n = 0
    for i in range(len(ts)):
        for j in range(i + 1, len(ts)):
            a, b = ts[i], ts[j]
            if (a["x"] - pad < b["x"] + b["w"] and b["x"] - pad < a["x"] + a["w"]
                    and a["y"] - pad < b["y"] + b["h"] and b["y"] - pad < a["y"] + a["h"]):
                print("  YAZI-YAZI  %-44r  <>  %r" % (a["s"][:44], b["s"][:44]))
                n += 1

    for t in ts:
        hits = [s for s in segs if seg_hits_rect(s, t)]
        if hits:
            print("  TEL-YAZI   %-44r  (%d tel, x=%d y=%d)"
                  % (t["s"][:44], len(hits), round(t["x"]), round(t["y"])))
            n += 1

    for t in ts:
        for (bx, by, bw, bh) in bs:
            over = (t["x"] < bx + bw and bx < t["x"] + t["w"]
                    and t["y"] < by + bh and by < t["y"] + t["h"])
            full = (t["x"] >= bx and t["x"] + t["w"] <= bx + bw
                    and t["y"] >= by and t["y"] + t["h"] <= by + bh)
            if over and not full:
                print("  YAZI-KUTU  %-44r  (kutu x=%d y=%d %dx%d)"
                      % (t["s"][:44], bx, by, bw, bh))
                n += 1

    # tel bir kutunun icinden geciyor mu -- yalnizca blok semalarinda anlamli;
    # grafiklerde egrinin tarali bandin icinden gecmesi normal
    for (bx, by, bw, bh) in (bs if 'wiring' in os.path.basename(path) else []):
        r = dict(x=bx + 3, y=by + 3, w=bw - 6, h=bh - 6)
        for seg in segs:
            x1, y1, x2, y2 = seg
            ends_on_edge = 0
            for (px_, py_) in ((x1, y1), (x2, y2)):
                if bx - 2 <= px_ <= bx + bw + 2 and by - 2 <= py_ <= by + bh + 2:
                    on_v = abs(px_ - bx) <= 2 or abs(px_ - (bx + bw)) <= 2
                    on_h = abs(py_ - by) <= 2 or abs(py_ - (by + bh)) <= 2
                    if on_v or on_h:
                        ends_on_edge += 1
            if seg_hits_rect(seg, r, pad=0) and not ends_on_edge:
                print("  TEL-KUTU   (%d,%d)-(%d,%d) kutunun icinden geciyor (x=%d y=%d %dx%d)"
                      % (x1, y1, x2, y2, bx, by, bw, bh))
                n += 1

    for t in ts:
        if t["x"] < 4 or t["x"] + t["w"] > vw - 4 or t["y"] < 2 or t["y"] + t["h"] > vh - 2:
            print("  TASMA      %-44r  (x=%d..%d, y=%d)"
                  % (t["s"][:44], round(t["x"]), round(t["x"] + t["w"]), round(t["y"])))
            n += 1

    print("  -> %d sorun" % n)
    return n


total = 0
targets = sys.argv[1:] or sorted(glob.glob(r"E:\projeler\ball_beam\hardware\drawings\*.svg"))
for p in targets:
    total += check(p)
print("\nTOPLAM: %d sorun" % total)
