"""Synthesise the reel's soundtrack.

Written from scratch rather than taken from a library so the video carries
no licence baggage and nothing gets muted when it is uploaded. Calm and
deliberately unobtrusive: a slow pad, a plucked arpeggio, a soft bass and a
brushed tick, arranged so the busy part of the video gets the fullest
sound.

    python tools/make_music.py [seconds]
"""
import math
import os
import sys
import wave

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "assets", "video", "ball_and_beam_music.wav")

SR = 44100
BPM = 81.4
BEAT = 60.0 / BPM
BAR = 4 * BEAT

# A minor - F - C - G, the plainest progression that stays out of the way.
CHORDS = [
    (57, [57, 60, 64, 69]),   # Am
    (53, [53, 57, 60, 65]),   # F
    (48, [48, 52, 55, 60]),   # C
    (55, [55, 59, 62, 67]),   # G
]


def hz(midi):
    return 440.0 * 2 ** ((midi - 69) / 12.0)


def env(n, attack, decay, sustain=1.0, release=0.0):
    """Simple AD(S)R over n samples, times in seconds."""
    a = max(1, int(attack * SR))
    d = max(1, int(decay * SR))
    r = max(1, int(release * SR)) if release else 0
    out = np.full(n, sustain, np.float32)
    a = min(a, n)
    out[:a] = np.linspace(0, 1, a, dtype=np.float32)
    d = min(d, max(0, n - a))
    if d:
        out[a:a + d] = np.linspace(1, sustain, d, dtype=np.float32)
    if r:
        r = min(r, n)
        out[n - r:] *= np.linspace(1, 0, r, dtype=np.float32)
    return out


def lowpass(x, cutoff):
    """One-pole smoothing: takes the edge off the saw partials."""
    a = math.exp(-2 * math.pi * cutoff / SR)
    y = np.empty_like(x)
    acc = 0.0
    for i in range(len(x)):
        acc = (1 - a) * x[i] + a * acc
        y[i] = acc
    return y


def pad(midi_notes, dur, detune=0.004):
    n = int(dur * SR)
    t = np.arange(n, dtype=np.float32) / SR
    sig = np.zeros(n, np.float32)
    for m in midi_notes:
        f = hz(m)
        for k, amp in ((1, 1.0), (2, 0.28), (3, 0.12), (4, 0.06)):
            sig += amp * np.sin(2 * np.pi * f * k * (1 + detune) * t).astype(np.float32)
            sig += amp * np.sin(2 * np.pi * f * k * (1 - detune) * t + 0.7).astype(np.float32)
    sig /= (len(midi_notes) * 4)
    return sig * env(n, 0.55, 0.4, 0.85, 0.7)


def pluck(midi, dur, amp=0.5):
    n = int(dur * SR)
    t = np.arange(n, dtype=np.float32) / SR
    f = hz(midi)
    sig = (np.sin(2 * np.pi * f * t)
           + 0.35 * np.sin(2 * np.pi * f * 2 * t)
           + 0.12 * np.sin(2 * np.pi * f * 3 * t)).astype(np.float32)
    return amp * sig * np.exp(-t * 5.5).astype(np.float32)


def bass(midi, dur, amp=0.55):
    n = int(dur * SR)
    t = np.arange(n, dtype=np.float32) / SR
    f = hz(midi - 12)
    sig = (np.sin(2 * np.pi * f * t) + 0.22 * np.sin(2 * np.pi * f * 2 * t)).astype(np.float32)
    return amp * sig * env(n, 0.03, 0.25, 0.7, 0.35)


def tick(dur=0.09, amp=0.10):
    n = int(dur * SR)
    rng = np.random.default_rng(7)
    noise = rng.standard_normal(n).astype(np.float32)
    return amp * lowpass(noise, 3500) * np.exp(-np.arange(n, dtype=np.float32) / SR * 38)


def add(buf, sig, at):
    i = int(at * SR)
    j = min(len(buf), i + len(sig))
    if i < len(buf):
        buf[i:j] += sig[:j - i]


def build(total):
    n = int(total * SR)
    left = np.zeros(n, np.float32)
    right = np.zeros(n, np.float32)
    bars = int(math.ceil(total / BAR))

    for b in range(bars):
        t0 = b * BAR
        root, notes = CHORDS[b % len(CHORDS)]
        p = pad(notes, BAR * 1.05)
        add(left, p * 0.55, t0)
        add(right, p * 0.55, t0 + 0.012)          # a hair of width
        add(left, bass(root, BAR * 0.9), t0)
        add(right, bass(root, BAR * 0.9), t0)

        if b >= 2:                                 # the arpeggio comes in late
            gain = min(1.0, (b - 1) / 3.0)
            seq = [notes[0], notes[2], notes[1], notes[3], notes[2], notes[1]]
            for k, m in enumerate(seq):
                at = t0 + k * (BAR / len(seq))
                if at >= total - 0.2:
                    break
                v = pluck(m + 12, 0.9, 0.34 * gain)
                add(left, v * (1.0 if k % 2 == 0 else 0.7), at)
                add(right, v * (0.7 if k % 2 == 0 else 1.0), at + 0.008)

        if 4 <= b < bars - 2:                      # brushed tick on 2 and 4
            for beat in (1, 3):
                add(left, tick(), t0 + beat * BEAT)
                add(right, tick(), t0 + beat * BEAT + 0.004)

    st = np.stack([left, right], axis=1)
    st = np.tanh(st * 1.1) * 0.92                  # soft clip instead of hard limiting
    st /= max(1e-6, np.abs(st).max())
    st *= 0.89

    fi, fo = int(1.2 * SR), int(3.0 * SR)
    st[:fi] *= np.linspace(0, 1, fi, dtype=np.float32)[:, None]
    st[-fo:] *= np.linspace(1, 0, fo, dtype=np.float32)[:, None]
    return st


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 47.2
    st = build(seconds)
    pcm = (st * 32767).astype(np.int16)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with wave.open(OUT, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(pcm.tobytes())
    print("yazildi: %s\n  %.1f s, %.1f MB, %d Hz stereo"
          % (OUT, seconds, os.path.getsize(OUT) / 1048576, SR))


if __name__ == "__main__":
    main()
