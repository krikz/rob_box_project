"""Числа приёмки ADR-0149 §7.2, которых нет в compare.py/audit_wav.py.

python accept.py file.wav [file2.wav ...]
По каждой записи: A1 самая длинная тишина (окно 0.5 с < -50 dBFS), A6 размах RMS по минутам в 6-мин окне
(max-min среди минут не тише -60 dBFS; у длинной записи - медиана по скользящим окнам),
A7 пик и число клипов, A8 |RMS L - RMS R| и Side/Mid 150-2000 Гц. Только numpy+scipy, WAV 16 бит.
"""
import sys
import wave

import numpy as np
from scipy.signal import butter, sosfilt


def load(path):
    w = wave.open(path)
    ch = w.getnchannels()
    a = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float64) / 32768
    return a.reshape(-1, ch), w.getframerate()


def db(x):
    return 20 * np.log10(x + 1e-9)


def longest_silence(m, sr, win=0.5, thr=-50.0):
    n = int(sr * win)
    k = len(m) // n
    rms = np.sqrt((m[:k * n].reshape(k, n) ** 2).mean(axis=1))
    quiet = db(rms) < thr
    best = cur = 0
    for q in quiet:
        cur = cur + 1 if q else 0
        best = max(best, cur)
    return best * win, quiet.sum() * win


def minute_range(m, sr, chunk=60.0, window=6):
    """Размах RMS по минутам в 6-мин окне; для длинных записей - медиана по скользящим окнам.
    Минуты тише -60 dBFS (тишина) из окна выбрасываются."""
    n = int(sr * chunk)
    levels = [db(np.sqrt((m[i:i + n] ** 2).mean())) for i in range(0, len(m) - n + 1, n)]
    ranges = []
    for i in range(max(1, len(levels) - window + 1)):
        win = [v for v in levels[i:i + window] if v > -60]
        if win:
            ranges.append(max(win) - min(win))
    return float(np.median(ranges)) if ranges else float("nan")


def band_rms(x, sr):
    sos = butter(4, [150, 2000], btype="band", fs=sr, output="sos")
    return db(np.sqrt((sosfilt(sos, x) ** 2).mean()))


def stereo_numbers(a, sr):
    if a.shape[1] < 2:
        return float("nan"), float("nan")
    lr = db(np.sqrt((a[:, 0] ** 2).mean())) - db(np.sqrt((a[:, 1] ** 2).mean()))
    sm = band_rms((a[:, 0] - a[:, 1]) / 2, sr) - band_rms((a[:, 0] + a[:, 1]) / 2, sr)
    return lr, sm


def report(path):
    a, sr = load(path)
    m = a.mean(axis=1)
    longest, total = longest_silence(m, sr)
    lr, sm = stereo_numbers(a, sr)
    clips = int((np.abs(a) >= 0.999).sum())
    print(f"{path}: {len(m) / sr:.0f} с | A1 тишина max {longest:.1f} с, всего {total:.1f} с | "
          f"A6 размах RMS/мин {minute_range(m, sr):.1f} дБ | "
          f"A7 пик {db(np.abs(a).max()):.1f} dBFS, клипов {clips} | A8 L-R {lr:+.1f} дБ, Side/Mid {sm:.1f} дБ")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        report(p)
