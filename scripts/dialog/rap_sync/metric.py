"""Замер A19 «попадание речи в такт» (ADR-0150 §14) для спайка PR-S1: онсеты слогов → смещение от сетки 16-х.

* **Онсет слога, два детектора.** ``syllable_onsets`` — подъём огибающей полосы 300–3000 Гц до «пик − 6 дБ» перед
  пиком; им пользуются подходы (2) и (3), когда ставят речь на сетку. **Замер A19 — другим детектором**
  (``flux_onsets``: спектральный поток librosa по мел-спектру до 2 кГц), иначе подход (3) мерил бы сам себя.
  Сверка: детекторы между собой — медиана 35–40 мс, 75–82 % ближе 60 мс; метки букв MiniMax (``subtitle_type=word``,
  шаг 42.7 мс) с обоими — медиана 55–70 мс. Точность онсета слога ~40 мс — это треть 16-й при 131 BPM: мелкие
  разницы в p50 метрикой не различимы, только крупные.
* **Смещение** — от ближайшей 16-й сетки подложки, мс со знаком; сводка p50/p95 модуля, доля 16-й (``norm``) и
  **R16** — длина среднего вектора фаз онсетов на периоде 16-й (1 — все на сетке, ~1/√N — случайно).
* **Случайный уровень:** онсеты, равномерные относительно сетки, дают ``|смещение|/16-я`` p50 = 0.25, p95 = 0.475 —
  любое число сравнивать с ним, иначе «20 мс» при 16-й 115 мс выглядит попаданием, а это случайность.
* **Старт строки** — онсет (того же независимого детектора), ближайший к известному началу строки (метка первой
  буквы строки MiniMax, пересчитанная раскладкой подхода; окно ±``LINE_WIN_S``), от ближайшей доли (четверти).
  Это прямое продолжение предварительного порога A19 для подхода (1): старт сегмента ≤ 40/80 мс.
"""

from __future__ import annotations

from typing import Dict, Sequence

import numpy as np
from scipy.signal import butter, find_peaks, sosfiltfilt

from common import SR

HOP_S, WIN_S = 0.005, 0.010
LINE_WIN_S = 0.15
CHANCE_P50, CHANCE_P95 = 0.25, 0.475


def envelope_db(x: np.ndarray, band=(300.0, 3000.0)) -> np.ndarray:
    sos = butter(4, band, btype="band", fs=SR, output="sos")
    y = sosfiltfilt(sos, x.astype(np.float64)) ** 2
    hop, win = int(HOP_S * SR), int(WIN_S * SR)
    n = max(0, (len(y) - win) // hop + 1)
    if n == 0:
        return np.full(1, -120.0)
    c = np.concatenate([[0.0], np.cumsum(y)])
    e = (c[np.arange(n) * hop + win] - c[np.arange(n) * hop]) / win
    return 10 * np.log10(e + 1e-12)


def syllable_onsets(x: np.ndarray) -> np.ndarray:
    """Онсеты слогов, секунды от начала ``x``."""
    env = envelope_db(x)
    env = np.convolve(env, np.ones(3) / 3, mode="same")
    floor = env.max() - 35.0
    peaks, _ = find_peaks(env, prominence=4.0, distance=int(0.08 / HOP_S), height=floor)
    out, prev = [], -1
    for p in peaks:
        lo = max(prev + 1, p - int(0.12 / HOP_S))
        k = p
        while k > lo and env[k - 1] > env[p] - 6.0:
            k -= 1
        out.append(k * HOP_S + WIN_S / 2)
        prev = p
    return np.asarray(out)


def flux_onsets(x: np.ndarray) -> np.ndarray:
    """Независимый детектор для замера: спектральный поток (librosa), мел до 2 кГц, шаг 5 мс."""
    import librosa

    mel = librosa.feature.melspectrogram(y=x, sr=SR, n_fft=512, hop_length=80, n_mels=40, fmax=2000)
    env = librosa.onset.onset_strength(S=librosa.power_to_db(mel), sr=SR, hop_length=80)
    return librosa.onset.onset_detect(onset_envelope=env, sr=SR, hop_length=80, units="time", delta=0.15, wait=12)


def grid_offsets_ms(onsets: Sequence[float], t0: float, step_s: float) -> np.ndarray:
    """Смещение каждого онсета от ближайшего узла сетки ``t0 + k·step``, мс (> 0 — позже узла)."""
    t = np.asarray(onsets, dtype=float) - t0
    return (t - np.round(t / step_s) * step_s) * 1000.0


def resultant(onsets: Sequence[float], t0: float, step_s: float) -> float:
    if len(onsets) == 0:
        return 0.0
    ph = 2 * np.pi * (np.asarray(onsets, dtype=float) - t0) / step_s
    return float(np.abs(np.mean(np.exp(1j * ph))))


def coherence(onsets: Sequence[float], t0: float, step_s: float) -> float:
    """Проекция среднего вектора фаз на узел сетки: 1 — на сетке, 0 — случайно, −1 — ровно между узлами.
    В отличие от R16 штрафует постоянный сдвиг (ровно, но мимо долей)."""
    if len(onsets) == 0:
        return 0.0
    ph = 2 * np.pi * (np.asarray(onsets, dtype=float) - t0) / step_s
    return float(np.mean(np.cos(ph)))


def line_onsets(onsets: Sequence[float], expected: Sequence[float]) -> np.ndarray:
    """Для каждого ожидаемого начала строки — ближайший онсет в окне ±``LINE_WIN_S`` (нет — строка пропущена)."""
    o = np.asarray(onsets, dtype=float)
    got = [o[np.argmin(np.abs(o - e))] for e in expected if len(o) and np.min(np.abs(o - e)) <= LINE_WIN_S]
    return np.asarray(got)


def a19(onsets: Sequence[float], t0: float, bpm: float, lines_at: Sequence[float] = ()) -> Dict[str, float]:
    """Сводка A19 по онсетам на сетке подложки (``t0`` — звучание первой доли, ``bpm``); ``lines_at`` — где по
    раскладке начинаются строки."""
    s16, beat = 15.0 / bpm, 60.0 / bpm
    off = np.abs(grid_offsets_ms(onsets, t0, s16))
    ls = np.abs(grid_offsets_ms(line_onsets(onsets, lines_at), t0, beat)) if len(lines_at) else np.zeros(1)
    return {
        "n": int(len(off)), "bpm": round(bpm, 2), "s16_ms": round(s16 * 1000, 1),
        "p50_ms": round(float(np.percentile(off, 50)), 1), "p95_ms": round(float(np.percentile(off, 95)), 1),
        "p50_norm": round(float(np.percentile(off, 50)) / (s16 * 1000), 3),
        "p95_norm": round(float(np.percentile(off, 95)) / (s16 * 1000), 3),
        "R16": round(resultant(onsets, t0, s16), 3), "R8": round(resultant(onsets, t0, 2 * s16), 3),
        "C16": round(coherence(onsets, t0, s16), 3),
        "bias_ms": round(float(np.median(grid_offsets_ms(onsets, t0, s16))), 1),
        "line_n": int(len(ls)), "line_expected": int(len(lines_at)), "line_p50_ms": round(float(np.percentile(ls, 50)), 1),
        "line_p95_ms": round(float(np.percentile(ls, 95)), 1),
    }


def detector_check(onsets: Sequence[float], vowel_times: Sequence[float]) -> Dict[str, float]:
    """Сверка детектора с метками гласных MiniMax: медиана |метка − ближайший онсет| и доля меток ближе 60 мс."""
    o = np.asarray(onsets, dtype=float)
    if len(o) == 0 or len(vowel_times) == 0:
        return {"vowels": len(vowel_times), "onsets": int(len(o))}
    d = np.array([np.min(np.abs(o - v)) for v in vowel_times])
    return {"vowels": len(vowel_times), "onsets": int(len(o)), "median_ms": round(float(np.median(d)) * 1000, 1),
            "within_60ms": round(float(np.mean(d < 0.06)), 3)}


def ltas_db(x: np.ndarray) -> np.ndarray:
    """Средний спектр (дБ) в 1/3-октавных полосах 200–7500 Гц — для «качества голоса» после обработки."""
    f = np.fft.rfftfreq(1024, 1 / SR)
    frames = np.lib.stride_tricks.sliding_window_view(x, 1024)[::256] * np.hanning(1024)
    p = np.mean(np.abs(np.fft.rfft(frames, axis=1)) ** 2, axis=0)
    edges = 200.0 * 2 ** (np.arange(0, 16) / 3.0)
    edges = edges[edges < 7600]
    return np.array([10 * np.log10(np.mean(p[(f >= a) & (f < b)]) + 1e-12) for a, b in zip(edges[:-1], edges[1:])])


def quality(orig: np.ndarray, proc: np.ndarray) -> Dict[str, float]:
    """Отклонение формы среднего спектра обработанного голоса от исходного (дБ, RMS по полосам, уровни выровнены)
    и спектральная плоскостность на звонких кадрах (рост — шум/«фазовость» растяжения)."""
    a, b = ltas_db(orig), ltas_db(proc)
    dev = (b - b.mean()) - (a - a.mean())

    def flat(x: np.ndarray) -> float:
        frames = np.lib.stride_tricks.sliding_window_view(x, 512)[::256] * np.hanning(512)
        p = np.abs(np.fft.rfft(frames, axis=1)) ** 2 + 1e-12
        e = p.sum(axis=1)
        voiced = e > e.max() * 10 ** (-2.5)
        g = np.exp(np.mean(np.log(p[voiced]), axis=1)) / np.mean(p[voiced], axis=1)
        return float(np.median(g))

    return {"ltas_dev_db": round(float(np.sqrt(np.mean(dev ** 2))), 2),
            "flatness_orig": round(flat(orig), 4), "flatness_proc": round(flat(proc), 4)}
