"""Cut the raw recordings into one short video for sharing.

The clips were shot at different times and sizes (phone at 848x478, screen
captures at ~1906x1022), so each segment is cropped to 16:9 and scaled to
720p, captioned, and cross-faded into the next one.

    python tools/make_reel.py

Writes assets/video/ball_and_beam_reel.mp4 (silent) and, next to it, the
soundtrack from make_music.py. Muxing the two needs ffmpeg.
"""
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VIDEO = os.path.join(ROOT, "assets", "video")
OUT = os.path.join(VIDEO, "ball_and_beam_reel.mp4")
SILENT = os.path.join(VIDEO, "ball_and_beam_reel_silent.mp4")
MUSIC = os.path.join(VIDEO, "ball_and_beam_music.wav")

W, H, FPS = 1280, 720, 30
FADE = 0.4                               # cross-dissolve, seconds
BG = (17, 21, 28)

RIG_SETPOINT = "arayüzden hedef konum girdiğimde gerçek hareket.mp4"
RIG_LINKAGE = "beem in hareketi sırasında krankın davranışı.mp4"
RIG_DISTURB = "bozucu etki uygulama.mp4"
RIG_CRANK = "krankın yandan videosu.mp4"
SPLIT_RIG = "onuncu adım için çekilen fiziksel hareket videosu.mp4"
SPLIT_GUI = "onuncu adım için ekran görüntüsü.mp4"
GUI_CAP = "normal çalışma ekran görüntüsü.mp4"
SPLIT_OFFSET = 5.19                      # rig time = gui time + this (tools/make_media.py)

FONT_DIR = r"C:\Windows\Fonts"


def font(size, bold=False):
    name = "seguisb.ttf" if bold else "segoeui.ttf"
    path = os.path.join(FONT_DIR, name)
    if not os.path.exists(path):
        path = os.path.join(FONT_DIR, "segoeui.ttf")
    return ImageFont.truetype(path, size)


# --- frame helpers ---------------------------------------------------------

def fit(frame, crop=None):
    """Crop to the wanted region, then scale to fill 1280x720 centre-cropped."""
    im = frame
    if crop:
        x0, y0, x1, y1 = crop
        im = im[y0:y1, x0:x1]
    h, w = im.shape[:2]
    scale = max(W / w, H / h)
    im = cv2.resize(im, (int(round(w * scale)), int(round(h * scale))), interpolation=cv2.INTER_AREA)
    y = (im.shape[0] - H) // 2
    x = (im.shape[1] - W) // 2
    return im[y:y + H, x:x + W]


def read_segment(path, t0, t1, crop=None):
    """Frames of one clip, resampled to the output frame rate."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit("acilamadi: " + path)
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(round((t1 - t0) * FPS))
    out = []
    for k in range(n):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int((t0 + k / FPS) * src_fps))
        ok, fr = cap.read()
        if not ok:
            break
        out.append(fit(fr, crop))
    cap.release()
    return out


def read_split(t0, t1):
    """The rig above, the GUI's position plot below, on the shared clock.

    Stacked rather than side by side: both sources are far wider than they
    are tall, so two columns would leave most of the frame empty.
    """
    rig = cv2.VideoCapture(os.path.join(VIDEO, SPLIT_RIG))
    gui = cv2.VideoCapture(os.path.join(VIDEO, SPLIT_GUI))
    rf, gf = rig.get(cv2.CAP_PROP_FPS) or 30.0, gui.get(cv2.CAP_PROP_FPS) or 30.0
    top_h, bot_h = 430, 285
    out = []
    for k in range(int(round((t1 - t0) * FPS))):
        t = t0 + k / FPS
        rig.set(cv2.CAP_PROP_POS_FRAMES, int((t + SPLIT_OFFSET) * rf))
        gui.set(cv2.CAP_PROP_POS_FRAMES, int(t * gf))
        ok1, a = rig.read()
        ok2, b = gui.read()
        if not (ok1 and ok2):
            break
        a = cv2.resize(a[120:405, 0:848], (W, top_h), interpolation=cv2.INTER_AREA)
        b = cv2.resize(b[75:352, 660:1906], (W, bot_h), interpolation=cv2.INTER_AREA)
        canvas = np.full((H, W, 3), BG, np.uint8)
        canvas[0:top_h] = a
        canvas[H - bot_h:H] = b
        cv2.line(canvas, (0, top_h + 2), (W, top_h + 2), (60, 66, 76), 1)
        out.append(canvas)
    rig.release()
    gui.release()
    return out


def overlay(lines, box=True, top=False, right=False, y=None):
    """A caption rendered once and alpha-blended onto every frame.

    `right` and `y` exist because a caption pinned to one corner sooner or
    later covers the thing it is describing; the split-screen shot needs it
    over the cables on the right instead.
    """
    img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    f1, f2 = font(30, True), font(20)
    if y is None:
        y = 96 if top else H - 128
    w = max(d.textlength(lines[0], f1),
            max((d.textlength(l, f2) for l in lines[1:]), default=0)) + 40
    x = (W - 54 - w + 22) if right else 54
    if box:
        h = 52 + (len(lines) - 1) * 26
        d.rounded_rectangle([x - 22, y - 34, x - 22 + w, y - 34 + h], 10, fill=(13, 17, 23, 190))
    d.text((x, y - 24), lines[0], font=f1, fill=(255, 255, 255, 255))
    for i, ln in enumerate(lines[1:]):
        d.text((x, y + 16 + i * 26), ln, font=f2, fill=(190, 197, 208, 255))
    return np.array(img)


def blend(frames, ov):
    a = ov[:, :, 3:4].astype(np.float32) / 255.0
    rgb = ov[:, :, 2::-1].astype(np.float32)
    return [np.clip(f.astype(np.float32) * (1 - a) + rgb * a, 0, 255).astype(np.uint8)
            for f in frames]


def card(lines, seconds, big=54, small=24):
    img = Image.new("RGB", (W, H), (13, 17, 23))
    d = ImageDraw.Draw(img)
    fb, fs = font(big, True), font(small)
    total = 64 + (len(lines) - 1) * 40
    y = (H - total) // 2
    d.text((W / 2, y), lines[0], font=fb, fill=(255, 255, 255), anchor="ma")
    for i, ln in enumerate(lines[1:]):
        d.text((W / 2, y + 78 + i * 38), ln, font=fs, fill=(150, 158, 170), anchor="ma")
    d.rounded_rectangle([W / 2 - 34, y - 26, W / 2 + 34, y - 20], 3, fill=(251, 133, 0))
    frame = np.array(img)[:, :, ::-1].copy()
    return [frame] * int(seconds * FPS)


def build():
    segs = []

    segs.append(card(["Ball & Beam", "camera-feedback balancing system",
                      "ESP32-S3  ·  250 Hz PID  ·  100 fps camera"], 2.6))

    segs.append(blend(read_segment(os.path.join(VIDEO, SPLIT_RIG), 2.0, 6.0, (0, 60, 848, 460)),
                      overlay(["The rig", "45 cm rail, ping-pong ball, crank-and-rod linkage"])))

    segs.append(blend(read_segment(os.path.join(VIDEO, RIG_SETPOINT), 1.2, 7.4, (0, 30, 848, 440)),
                      overlay(["New setpoint", "a target position is entered in the interface"])))

    segs.append(blend(read_segment(os.path.join(VIDEO, RIG_LINKAGE), 3.0, 8.0, (0, 118, 640, 478)),
                      overlay(["The linkage", "the crank turns, the rod lifts the end of the beam"])))

    segs.append(blend(read_segment(os.path.join(VIDEO, RIG_CRANK), 1.0, 4.2, (130, 140, 630, 421)),
                      overlay(["Motor and crank", "NEMA 17 · TMC2208 · 1/256 microstepping"])))

    segs.append(blend(read_split(16.0, 26.0),
                      overlay(["Same moment", "the rig above, the measured position below"],
                              top=True, right=True, y=322)))

    segs.append(blend(read_segment(os.path.join(VIDEO, RIG_DISTURB), 1.0, 11.2, (0, 30, 848, 440)),
                      overlay(["Disturbance", "the ball is pushed by hand, back on target in ~1 s"])))

    segs.append(blend(read_segment(os.path.join(VIDEO, GUI_CAP), 26.0, 32.0, (0, 28, 1500, 872)),
                      overlay(["The interface", "live plots, camera tracking, health panel"])))

    segs.append(card(["1.9% overshoot  ·  1.5 s settling  ·  1.4 mm error",
                      "github.com/KadirKaratarla/ball-and-beam-control"], 3.2, big=40, small=26))

    # --- cross-dissolve and write ---
    fade = int(FADE * FPS)
    os.makedirs(VIDEO, exist_ok=True)
    vw = cv2.VideoWriter(SILENT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    prev_tail = None
    for i, seg in enumerate(segs):
        if not seg:
            continue
        if prev_tail is not None:
            for k in range(fade):
                a = k / fade
                vw.write(cv2.addWeighted(prev_tail[k], 1 - a, seg[k], a, 0))
            seg = seg[fade:]
        body = seg[:-fade] if i < len(segs) - 1 and len(seg) > fade else seg
        for f in body:
            vw.write(f)
        prev_tail = seg[-fade:] if i < len(segs) - 1 and len(seg) > fade else None
        print("  bolum %d: %4.1f s" % (i + 1, len(seg) / FPS), flush=True)
    vw.release()
    cap = cv2.VideoCapture(SILENT)
    dur = cap.get(cv2.CAP_PROP_FRAME_COUNT) / FPS
    cap.release()
    print("\ngoruntu: %.1f s" % dur)
    mux(dur)


def mux(duration):
    """Re-encode to H.264 and lay the soundtrack under the picture.

    OpenCV writes video only, and its mp4v stream is both larger and less
    widely accepted than H.264, so the last step goes through ffmpeg.
    """
    import subprocess
    try:
        import imageio_ffmpeg
        exe = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        print("ffmpeg yok (pip install imageio-ffmpeg) -- sessiz surum: " + SILENT)
        return
    if not os.path.exists(MUSIC):
        print("muzik yok, once calistir: python tools/make_music.py %.1f" % duration)
        return
    subprocess.run([exe, "-y", "-loglevel", "error",
                    "-i", SILENT, "-i", MUSIC,
                    "-c:v", "libx264", "-preset", "slow", "-crf", "20",
                    "-pix_fmt", "yuv420p",
                    "-c:a", "aac", "-b:a", "192k",
                    "-af", "volume=0.55",      # music sits under the picture
                    "-shortest", "-movflags", "+faststart", OUT], check=True)
    # Drop the silent intermediate so the folder holds one obvious answer.
    try:
        os.remove(SILENT)
    except OSError:
        pass
    print("yazildi: %s\n  %dx%d, %.1f s, %.1f MB (H.264 + AAC)"
          % (OUT, W, H, duration, os.path.getsize(OUT) / 1048576))


if __name__ == "__main__":
    build()
