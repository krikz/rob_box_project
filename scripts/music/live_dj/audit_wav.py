"""Что реально прозвучало: разбор записи jack_rec (выход scsynth).

python audit_wav.py file.wav [--win 8] [--t0 SEC]

Печатает по окнам: уровень, тональность по хромаграмме, долю энергии вне
лучшего диатонического лада («грязь»), темп и захват сетки 16-х, сдвиг фазы
сетки относительно предыдущего окна (сбивка ритма). В конце — сводка.
"""
import sys
import wave

import numpy as np
from scipy.signal import find_peaks, stft

NOTES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]
MAJ = np.array([6.35, 2.23, 3.48, 2.33, 4.38, 4.09, 2.52, 5.19, 2.39, 3.66, 2.29, 2.88])
MIN = np.array([6.33, 2.68, 3.52, 5.38, 2.60, 3.53, 2.54, 4.75, 3.98, 2.69, 3.34, 3.17])
DIATONIC = np.array([1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1], float)  # C major mask


def load(path):
    w = wave.open(path)
    sr, ch = w.getframerate(), w.getnchannels()
    a = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(float)
    return a.reshape(-1, ch).mean(1) / 32768, sr


def chroma(seg, sr):
    f, _t, Z = stft(seg, sr, nperseg=8192, noverlap=4096)
    P = (np.abs(Z) ** 2).mean(1)
    ok = (f > 60) & (f < 2500)
    pc = np.round(12 * np.log2(f[ok] / 440.0) + 9).astype(int) % 12  # 0=C
    c = np.bincount(pc, weights=P[ok], minlength=12)
    return c / (c.sum() + 1e-12)


def key_of(c):
    best = None
    for r in range(12):
        for name, prof in (("maj", MAJ), ("min", MIN)):
            s = np.corrcoef(c, np.roll(prof, r))[0, 1]
            if best is None or s > best[0]:
                best = (s, r, name)
    return best


def out_of_scale(c):
    """Доля энергии вне ЛУЧШЕГО из 12 диатонических ладов (0 = чисто)."""
    return min(1 - (c * np.roll(DIATONIC, r)).sum() for r in range(12))


def onsets(a, sr):
    _f, _t, Z = stft(a, sr, nperseg=1024, noverlap=1024 - 128)
    M = np.log1p(np.abs(Z) * 50)
    flux = np.maximum(np.diff(M, axis=1), 0).sum(0)
    return flux, sr / 128


def grid(flux, fs, s, e):
    x = flux[int(s * fs):int(e * fs)]
    if len(x) < 10:
        return None
    p, _ = find_peaks(x, height=np.percentile(x, 85), distance=int(fs * 0.06))
    tt = p / fs + s
    if len(tt) < 12:
        return None
    best = None
    for P in np.arange(0.095, 0.160, 0.0005):  # период 16-й: 94..158 bpm
        z = np.exp(2j * np.pi * tt / P).mean()
        if best is None or abs(z) > best[0]:
            best = (abs(z), P, np.angle(z))
    return best + (len(tt),)


def phase_track(flux, fs, bpm, dur, step=2.0):
    """Фаза сетки 16-х при ИЗВЕСТНОМ темпе по окнам step с.

    Ровный бит — фаза стоит (Δ≈0); сбивка — скачок фазы; «ломаный» рисунок
    без сбивки — низкий R при стоящей фазе.
    """
    P = 60 / bpm / 4
    out = []
    for s in np.arange(0, dur - step + 0.01, step):
        x = flux[int(s * fs):int((s + step) * fs)]
        p, _ = find_peaks(x, height=np.percentile(x, 80), distance=int(fs * 0.05))
        if len(p) < 4:
            out.append((s, None, None))
            continue
        tt = p / fs + s
        w = x[p]
        z = (w * np.exp(2j * np.pi * tt / P)).sum() / w.sum()
        out.append((s, abs(z), np.angle(z) / (2 * np.pi) * P * 1000))
    print(f"--- фаза сетки при {bpm} bpm (16-я = {P*1000:.0f} мс), окна {step:g} с")
    prev, jumps = None, []
    for s, R, ms in out:
        if R is None:
            print(f"{s:6.0f}  —")
            prev = None
            continue
        d = ""
        if prev is not None:
            dd = (ms - prev + P * 500) % (P * 1000) - P * 500
            d = f"{dd:+5.0f}"
            if abs(dd) > 25 and R > 0.4:
                jumps.append((s, dd))
                d += "  <<< скачок"
        prev = ms
        print(f"{s:6.0f}  R={R:4.2f} фаза={ms:+5.0f}мс  Δ={d}")
    print(f"скачков фазы >25 мс: {len(jumps)} {[(int(s), int(d)) for s, d in jumps]}")


def main():
    path = sys.argv[1]
    if "--bpm" in sys.argv:
        a, sr = load(path)
        flux, fs = onsets(a, sr)
        phase_track(flux, fs, float(sys.argv[sys.argv.index("--bpm") + 1]), len(a) / sr)
        return
    win = float(sys.argv[sys.argv.index("--win") + 1]) if "--win" in sys.argv else 8.0
    a, sr = load(path)
    dur = len(a) / sr
    rms1 = np.array([np.sqrt(np.mean(a[int(i * sr):int((i + 1) * sr)] ** 2)) for i in range(int(dur))])
    db1 = 20 * np.log10(rms1 + 1e-9)
    loud = np.where(db1 > -50)[0]
    print(f"== {path}: {dur:.0f} с, пик {20*np.log10(np.abs(a).max()+1e-9):.1f} dBFS")
    first = loud[0] if len(loud) else '—'
    print(f"первый звук (>-50 dBFS): {first} с; тишина: {int((db1 <= -50).sum())} с из {int(dur)}")
    flux, fs = onsets(a, sr)
    rows, prev = [], None
    print(" окно      dBFS  тональность  r     вне-лада  bpm    R(сетка) Δфаза,мс")
    for s in np.arange(0, dur - win + 0.01, win):
        seg = a[int(s * sr):int((s + win) * sr)]
        lvl = 20 * np.log10(np.sqrt(np.mean(seg ** 2)) + 1e-9)
        if lvl < -50:
            print(f"{s:5.0f}-{s+win:<4.0f} {lvl:6.1f}  тишина")
            prev = None
            continue
        c = chroma(seg, sr)
        r, root, mode = key_of(c)
        oos = out_of_scale(c)
        g = grid(flux, fs, s, s + win)
        if g:
            R, P, ph, n = g
            bpm = 60 / (P * 4)
            dph = ""
            if prev and abs(prev[1] - P) < 0.002:
                d = (ph - prev[2] + np.pi) % (2 * np.pi) - np.pi
                dph = f"{d / (2 * np.pi) * P * 1000:+6.0f}"
            prev = (R, P, ph)
            gs = f"{bpm:6.1f} {R:5.2f}    {dph}"
        else:
            prev, gs, R = None, "  —", None
        rows.append((s, lvl, oos, R))
        print(f"{s:5.0f}-{s+win:<4.0f} {lvl:6.1f}  {NOTES[root]:>2} {mode}    {r:4.2f}  {oos:5.2f}    {gs}")
    if rows:
        o = np.array([x[2] for x in rows])
        Rs = np.array([x[3] for x in rows if x[3] is not None])
        print(f"СВОДКА: вне-лада медиана {np.median(o):.2f}, макс {o.max():.2f}; "
              f"окон с R<0.5: {(Rs < 0.5).sum()}/{len(Rs)}; R медиана {np.median(Rs) if len(Rs) else float('nan'):.2f}")


if __name__ == "__main__":
    main()


def kick_dev(a, sr, bpm, s, e):
    """Удары бочки (30-150 Гц) → отклонение от сетки 16-х, мс.

    Фаза сетки берётся по медиане всего отрезка; ровный бит даёт |откл| < 15 мс
    у большинства ударов, сбивка — серии ударов, уехавших на 30+ мс.
    """
    from scipy.signal import butter, sosfiltfilt
    seg = a[int(s * sr):int(e * sr)]
    lo = sosfiltfilt(butter(4, [30, 150], btype="band", fs=sr, output="sos"), seg)
    env = np.convolve(np.abs(lo), np.ones(int(sr * 0.005)) / (sr * 0.005), mode="same")[::int(sr / 1000)]
    d = np.maximum(np.diff(env), 0)
    p, _ = find_peaks(d, height=np.percentile(d, 99), distance=int(1000 * 60 / bpm / 4 * 0.8))
    tt = p / 1000.0
    P = 60 / bpm / 4
    z = np.exp(2j * np.pi * tt / P).mean()
    ph = np.angle(z) / (2 * np.pi) * P
    dev = ((tt - ph + P / 2) % P - P / 2) * 1000
    return tt + s, dev, abs(z)
