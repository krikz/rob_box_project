"""Профиль записи по кускам: эталон vs наши треки.

python compare.py file.wav [--chunk 60] [--from S --to E]
Метрики куска: bpm+R (лучший темп 100-140), вне-лада, тональность, RMS,
пик-фактор (crest, дБ), LR-корреляция (1 = моно), доли энергии
низ <150 / середина 150-2k / верх >2k, онсетов в секунду.
"""
import sys
import wave

import numpy as np
from scipy.signal import find_peaks, stft

import audit_wav as A


def read(path, s, e):
    w = wave.open(path)
    sr, ch = w.getframerate(), w.getnchannels()
    w.setpos(int(s * sr))
    x = np.frombuffer(w.readframes(int((e - s) * sr)), dtype=np.int16).astype(float) / 32768
    return x.reshape(-1, ch), sr


def bestR(flux, fs, dur):
    tt_all = None
    p, _ = find_peaks(flux, height=np.percentile(flux, 85), distance=int(fs * 0.05))
    tt_all = p / fs
    best = (0, 0)
    for bpm in np.arange(100, 140.01, 0.25):
        P = 60 / bpm / 4
        Rs = []
        for t in np.arange(0, dur - 4, 4):
            tt = tt_all[(tt_all >= t) & (tt_all < t + 4)]
            if len(tt) >= 6:
                Rs.append(abs(np.exp(2j * np.pi * tt / P).mean()))
        if Rs and np.median(Rs) > best[0]:
            best = (np.median(Rs), bpm)
    return best, len(tt_all) / dur


def profile(x, sr):
    m = x.mean(1)
    dur = len(m) / sr
    rms = np.sqrt(np.mean(m ** 2))
    crest = 20 * np.log10(np.abs(m).max() / (rms + 1e-12) + 1e-12)
    lr = np.corrcoef(x[:, 0], x[:, 1])[0, 1] if x.shape[1] > 1 else 1.0
    f, _t, Z = stft(m, sr, nperseg=4096, noverlap=2048)
    Pw = (np.abs(Z) ** 2).mean(1)
    tot = Pw.sum() + 1e-12
    low, mid, hi = Pw[f < 150].sum() / tot, Pw[(f >= 150) & (f < 2000)].sum() / tot, Pw[f >= 2000].sum() / tot
    flux, fs = A.onsets(m, sr)
    (R, bpm), dens = bestR(flux, fs, dur)
    oos = np.median([A.out_of_scale(A.chroma(m[int(s * sr):int((s + 8) * sr)], sr))
                     for s in np.arange(0, dur - 8 + 0.01, 8)])
    r, root, mode = A.key_of(A.chroma(m, sr))
    return dict(db=20 * np.log10(rms + 1e-12), crest=crest, lr=lr, low=low, mid=mid, hi=hi,
                bpm=bpm, R=R, dens=dens, oos=oos, key=f"{A.NOTES[root]}{'m' if mode == 'min' else ''}")


HDR = (f"{'кусок':>11} {'dBFS':>6} {'crest':>5} {'LR':>5} {'низ':>5} {'сер':>5} {'верх':>5} "
       f"{'bpm':>6} {'R':>5} {'онс/с':>5} {'вне-лада':>8} тон.")


def row(lab, p):
    return (f"{lab:>11} {p['db']:6.1f} {p['crest']:5.1f} {p['lr']:5.2f} {p['low']:5.2f} {p['mid']:5.2f} {p['hi']:5.2f} "
            f"{p['bpm']:6.2f} {p['R']:5.2f} {p['dens']:5.1f} {p['oos']:8.2f} {p['key']}")


def main():
    path = sys.argv[1]
    chunk = float(sys.argv[sys.argv.index("--chunk") + 1]) if "--chunk" in sys.argv else 60
    w = wave.open(path)
    total = w.getnframes() / w.getframerate()
    s0 = float(sys.argv[sys.argv.index("--from") + 1]) if "--from" in sys.argv else 0
    e0 = float(sys.argv[sys.argv.index("--to") + 1]) if "--to" in sys.argv else total
    print(path)
    print(HDR)
    rows = []
    for s in np.arange(s0, e0 - chunk / 2, chunk):
        e = min(s + chunk, e0)
        x, sr = read(path, s, e)
        p = profile(x, sr)
        rows.append(p)
        print(row(f"{s:.0f}-{e:.0f}", p))
    if len(rows) > 1:
        med = {k: (np.median([r[k] for r in rows]) if k != "key" else "") for k in rows[0]}
        print(row("МЕДИАНА", med))


if __name__ == "__main__":
    main()
