"""Cut the README's images and GIFs out of the raw recordings.

The raw videos in assets/video/ are too big for git; this turns them into
the small PNGs and GIFs that the README actually references. Re-run after
dropping a new recording in:

    python tools/make_media.py                 # everything
    python tools/make_media.py setpoint_steps  # just one clip

No ffmpeg needed -- OpenCV reads the video, Pillow writes the GIF.
"""
import os
import sys

import cv2
from PIL import Image, ImageDraw, ImageOps

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VIDEO = os.path.join(ROOT, "assets", "video")
GIF = os.path.join(ROOT, "assets", "gif")
IMG = os.path.join(ROOT, "assets", "images")

GUI_CAP = "normal çalışma ekran görüntüsü.mp4"
RIG_SETPOINT = "arayüzden hedef konum girdiğimde gerçek hareket.mp4"
RIG_LINKAGE = "beem in hareketi sırasında krankın davranışı.mp4"
RIG_DISTURB = "bozucu etki uygulama.mp4"
RIG_CRANK = "krankın yandan videosu.mp4"

# 10. cekim: ayni ani hem duzenekten hem ekrandan. Iki kayit ayri ayri
# baslatildigi icin kayma turuncu topun iki videodaki yatay konumu capraz
# ilintilenerek bulundu (scratchpad/sync.py, ilinti 0.74):
#     rig zamani = ekran zamani + SPLIT_OFFSET
SPLIT_RIG = "onuncu adım için çekilen fiziksel hareket videosu.mp4"
SPLIT_GUI = "onuncu adım için ekran görüntüsü.mp4"
SPLIT_OFFSET = 5.19
SPLIT = dict(t0=15.5, t1=27.5, fps=6, width=1000, colors=72,
             rig_crop=(0, 120, 848, 400), gui_crop=(690, 88, 1906, 385))


# The GIFs keep the top strip of the window: durum bandi + sema/kamera on
# the left and the ball-position plot on the right. Dropping the rest is
# what gets a 10 s clip under a few MB.
STRIP = (0, 28, 1906, 440)
# The device-loss clips need the warning text legible, so they keep the left
# half of the window down to the health tabs instead.
PANEL = (0, 28, 1320, 650)

# name -> (source video, start s, end s, output fps, output width, crop, colors)
# Ekran kayitlari duz renkli ve durgun, telefon videolari her pikselde
# oynuyor; onlar daha dar, daha dusuk kare hizli ve daha az renkli olmali.
CLIPS = {
    "setpoint_steps":  (GUI_CAP, 103.0, 114.0, 9, 860, STRIP, 96),
    "camera_tracking": (GUI_CAP, 26.0, 35.0, 9, 860, STRIP, 96),
    "esp_recovery":    (GUI_CAP, 78.5, 88.0, 7, 1000, PANEL, 96),
    "camera_recovery": (GUI_CAP, 93.5, 103.0, 7, 1000, PANEL, 96),

    # Duzenegin kendisi (telefon, 848x478).
    "rig_disturbance": (RIG_DISTURB, 0.6, 10.6, 8, 480, None, 64),
    "rig_setpoint":    (RIG_SETPOINT, 0.3, 10.3, 8, 480, None, 64),
    "rig_linkage":     (RIG_LINKAGE, 0.3, 9.0, 8, 480, None, 64),
    "rig_crank":       (RIG_CRANK, 0.1, 4.3, 8, 480, None, 64),
}

# name -> (source video, timestamp s, width, crop)
# t < 70 s the gains were still being played with on screen; the stills that
# show the gain boxes are taken after that, where the firmware defaults are
# back (0.740 / 0.060 / 0.500 / 0.150).
HEALTH = (0, 690, 700, 1010)

STILLS = {
    "gui_overview":       (GUI_CAP, 75.0, 1600, None),
    "gui_camera_view":    (GUI_CAP, 30.0, 1600, None),
    "gui_health_motor":   (GUI_CAP, 47.0, 700, HEALTH),
    "gui_esp_missing":    (GUI_CAP, 82.0, 1600, None),
    "gui_camera_missing": (GUI_CAP, 97.5, 1600, None),
}


def grab(path, t, width, crop=None):
    """One frame at t seconds, resized to width, as a PIL image."""
    v = cv2.VideoCapture(path)
    if not v.isOpened():
        raise SystemExit("video acilamadi: " + path)
    fps = v.get(cv2.CAP_PROP_FPS)
    v.set(cv2.CAP_PROP_POS_FRAMES, int(t * fps))
    ok, fr = v.read()
    v.release()
    if not ok:
        raise SystemExit("kare okunamadi: %s @ %.1f s" % (path, t))
    im = Image.fromarray(cv2.cvtColor(fr, cv2.COLOR_BGR2RGB))
    if crop:
        im = im.crop(crop)
    return im.resize((width, round(width * im.height / im.width)), Image.LANCZOS)


def frames(path, t0, t1, out_fps, width, crop=None):
    v = cv2.VideoCapture(path)
    if not v.isOpened():
        raise SystemExit("video acilamadi: " + path)
    src_fps = v.get(cv2.CAP_PROP_FPS)
    step = max(1, round(src_fps / out_fps))
    v.set(cv2.CAP_PROP_POS_FRAMES, int(t0 * src_fps))
    out, f = [], int(t0 * src_fps)
    end = int(t1 * src_fps)
    while f < end:
        ok, fr = v.read()
        if not ok:
            break
        if (f - int(t0 * src_fps)) % step == 0:
            im = Image.fromarray(cv2.cvtColor(fr, cv2.COLOR_BGR2RGB))
            if crop:
                im = im.crop(crop)
            out.append(im.resize((width, round(width * im.height / im.width)), Image.LANCZOS))
        f += 1
    v.release()
    return out


def write_gif(name, spec):
    src, t0, t1, fps, width, crop, colors = spec
    path = os.path.join(VIDEO, src)
    if not os.path.exists(path):
        print("  atlandi (video yok): %s" % src)
        return
    ims = frames(path, t0, t1, fps, width, crop)
    if not ims:
        print("  atlandi (kare yok): %s" % name)
        return
    # One palette for the whole clip: the UI barely changes between frames,
    # so a shared palette both looks steadier and compresses much better.
    base = ims[len(ims) // 2].quantize(colors=colors, method=Image.MEDIANCUT)
    # No dithering: on a flat dark UI it sprays per-pixel noise that changes
    # every frame, which both looks worse and triples the file size.
    ims = [im.quantize(palette=base, dither=Image.NONE) for im in ims]
    out = os.path.join(GIF, name + ".gif")
    # disposal=1 leaves the previous frame in place, so Pillow can store only
    # the rectangle that actually changed.
    ims[0].save(out, save_all=True, append_images=ims[1:], loop=0,
                duration=round(1000 / fps), optimize=True, disposal=1)
    print("  %-18s %5.1f-%5.1f s  %3d kare  %5.1f MB" %
          (name + ".gif", t0, t1, len(ims), os.path.getsize(out) / 1048576))


def write_still(name, spec):
    src, t, width, crop = spec
    path = os.path.join(VIDEO, src)
    if not os.path.exists(path):
        print("  atlandi (video yok): %s" % src)
        return
    out = os.path.join(IMG, name + ".png")
    grab(path, t, width, crop).save(out, optimize=True)
    print("  %-22s t=%6.1f s  %5.2f MB" % (name + ".png", t, os.path.getsize(out) / 1048576))


# Kullanicinin ham fotograflari -> README'nin baglayacagi ASCII adlar.
PHOTOS = {
    "rig_overview":         "sistemin genel görseli.jpeg",
    "electronics_overview": "genel donanım görünümü.jpeg",
    "esp32_s3":             "esp görünümü.jpeg",
    "tmc2208":              "sürücü görünümü.jpeg",
    "encoder_as5600":       "encoder görünümü.jpeg",
    "camera_mount":         "kamera konumu.jpeg",
}
PHOTO_WIDTH = 1280


def copy_photos():
    src_dir = os.path.join(ROOT, "assets", "photos")
    for name, src in PHOTOS.items():
        path = os.path.join(src_dir, src)
        if not os.path.exists(path):
            print("  atlandi (foto yok): %s" % src)
            continue
        im = ImageOps.exif_transpose(Image.open(path)).convert("RGB")
        if im.width > PHOTO_WIDTH:
            im = im.resize((PHOTO_WIDTH, round(PHOTO_WIDTH * im.height / im.width)), Image.LANCZOS)
        out = os.path.join(IMG, name + ".jpg")
        im.save(out, quality=86, optimize=True, progressive=True)
        print("  %-24s %4dx%-4d %5.2f MB" % (name + ".jpg", im.width, im.height,
                                             os.path.getsize(out) / 1048576))


def write_split():
    """The rig and the GUI side by side, aligned on the shared clock."""
    rig_p = os.path.join(VIDEO, SPLIT_RIG)
    gui_p = os.path.join(VIDEO, SPLIT_GUI)
    if not (os.path.exists(rig_p) and os.path.exists(gui_p)):
        print("  atlandi (10. cekim videolari eksik)")
        return
    half = SPLIT["width"] // 2
    rig = frames(rig_p, SPLIT["t0"] + SPLIT_OFFSET, SPLIT["t1"] + SPLIT_OFFSET,
                 SPLIT["fps"], half, SPLIT["rig_crop"])
    gui = frames(gui_p, SPLIT["t0"], SPLIT["t1"], SPLIT["fps"], half, SPLIT["gui_crop"])
    n = min(len(rig), len(gui))
    if not n:
        print("  atlandi (kare yok): split_screen")
        return
    h = max(rig[0].height, gui[0].height) + 22
    ims = []
    for a, b in zip(rig[:n], gui[:n]):
        canvas = Image.new("RGB", (SPLIT["width"], h), "#0d1117")
        canvas.paste(a, (0, 22 + (h - 22 - a.height) // 2))
        canvas.paste(b, (half, 22 + (h - 22 - b.height) // 2))
        d = ImageDraw.Draw(canvas)
        d.text((8, 6), "duzenek", fill="#8b949e")
        d.text((half + 8, 6), "arayuz -- top konumu", fill="#8b949e")
        ims.append(canvas)
    base = ims[len(ims) // 2].quantize(colors=SPLIT["colors"], method=Image.MEDIANCUT)
    ims = [im.quantize(palette=base, dither=Image.NONE) for im in ims]
    out = os.path.join(GIF, "split_screen.gif")
    ims[0].save(out, save_all=True, append_images=ims[1:], loop=0,
                duration=round(1000 / SPLIT["fps"]), optimize=True, disposal=1)
    print("  %-18s %5.1f-%5.1f s  %3d kare  %5.1f MB" %
          ("split_screen.gif", SPLIT["t0"], SPLIT["t1"], len(ims),
           os.path.getsize(out) / 1048576))


def main():
    only = sys.argv[1] if len(sys.argv) > 1 else None
    os.makedirs(GIF, exist_ok=True)
    os.makedirs(IMG, exist_ok=True)
    print("Fotograflar:")
    if only is None:
        copy_photos()
    print("PNG:")
    for name, spec in STILLS.items():
        if only in (None, name):
            write_still(name, spec)
    print("GIF:")
    for name, spec in CLIPS.items():
        if only in (None, name):
            write_gif(name, spec)
    if only in (None, "split_screen"):
        write_split()


if __name__ == "__main__":
    main()
