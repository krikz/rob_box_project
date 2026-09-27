"""Нотные события -> аудио-буферы (issue #2977).

Наивные (без oversampling) осцилляторы на частоте дискретизации робота
(16 kHz, ``docker/vision/scripts/supercollider/start_supercollider.sh``)
— НАМЕРЕННО: цель RC5 аудита ``docs/analysis/2026-08-30-music-quality-
audit.md`` — «пила/меандр алиасят в слышимой полосе», и наивная пила даёт
ровно этот класс искажений, не притворяясь конкретным SynthDef. Это не
претендует на тембровое сходство с ``wobblebass``/``wideblip`` и т.п. —
только на тот же физический эффект (широкий гармонический спектр выше
Найквиста при 16 kHz), см. docstring пакета.
"""

from __future__ import annotations

from typing import Dict, Sequence

import numpy as np
from scipy.signal import butter, lfilter

from .note_extract import NoteEvent, ROLE_WAVEFORM

#: Огибающая одного события: линейный attack/release, без щелчков на
#: границах. Коротка относительно типичной длительности ноты (>= 1/16
#: такта на разумном bpm), поэтому не «съедает» короткие ударные форшлаги.
_ATTACK_S = 0.004
_RELEASE_S = 0.015


def _adsr(n: int, sample_rate: int) -> np.ndarray:
    if n <= 0:
        return np.zeros(0, dtype=np.float64)
    attack = max(1, int(_ATTACK_S * sample_rate))
    release = max(1, int(_RELEASE_S * sample_rate))
    attack = min(attack, n // 2 or 1)
    release = min(release, n - attack if n > attack else 1)
    env = np.ones(n, dtype=np.float64)
    env[:attack] = np.linspace(0.0, 1.0, attack, endpoint=False)
    if release > 0:
        env[n - release:] = np.linspace(env[n - release - 1] if n - release - 1 >= 0 else 1.0, 0.0, release)
    return env


def _osc(waveform: str, freq: float, t: np.ndarray) -> np.ndarray:
    phase = freq * t
    if waveform == "sine":
        return np.sin(2.0 * np.pi * phase)
    if waveform == "saw":
        # Наивная пила: 2*frac(phase+0.5) - 1. Богата гармониками -> алиасинг
        # выше Найквиста на 16 kHz (RC5), как и реальные saw/varsaw-синты.
        frac = phase + 0.5 - np.floor(phase + 0.5)
        return 2.0 * frac - 1.0
    if waveform == "triangle":
        frac = phase - np.floor(phase)
        return 2.0 * np.abs(2.0 * frac - 1.0) - 1.0
    if waveform == "soft":
        # Пэд: 3 мягкие гармоники, спадающие по амплитуде — ближе к
        # тёплому пэду, чем чистая пила, и без резких высоких обертонов.
        return (
            np.sin(2.0 * np.pi * phase)
            + 0.35 * np.sin(2.0 * np.pi * 2.0 * phase)
            + 0.15 * np.sin(2.0 * np.pi * 3.0 * phase)
        ) / 1.5
    raise ValueError(f"неизвестный waveform={waveform!r}")


#: Коэффициенты band-pass фильтра кэшируются по (полоса, sample_rate) —
#: у одной ударной роли одна и та же полоса повторяется на КАЖДОМ ударе
#: (в 50-темовом прогоне это тысячи вызовов), а ``scipy.signal.butter``
#: пересчитывать заново на каждый вызов незачем.
_BANDPASS_CACHE: dict = {}


def _bandpass_coeffs(band_hz: Sequence[float], sample_rate: int):
    key = (float(band_hz[0]), float(band_hz[1]), sample_rate)
    coeffs = _BANDPASS_CACHE.get(key)
    if coeffs is None:
        nyquist = sample_rate * 0.5
        lo = max(1e-6, band_hz[0] / nyquist)
        hi = min(0.999, band_hz[1] / nyquist)
        coeffs = butter(2, [lo, hi], btype="band")
        _BANDPASS_CACHE[key] = coeffs
    return coeffs


def _noise_burst(n: int, band_hz: Sequence[float], sample_rate: int, rng: np.random.Generator) -> np.ndarray:
    """Белый шум, band-pass'нутый по полосе роли (кэшированный IIR, не FFT-маска —
    цена не в точности АЧХ удара, а в том, что стенд гоняет тысячи ударов
    на выборке в 50 тем, и FFT на каждый был непозволительно медленным)."""
    if n <= 0:
        return np.zeros(0, dtype=np.float64)
    white = rng.uniform(-1.0, 1.0, size=n)
    b, a = _bandpass_coeffs(band_hz, sample_rate)
    out = lfilter(b, a, white)
    peak = np.max(np.abs(out)) or 1.0
    return out / peak


def render_events(
    events: Sequence[NoteEvent],
    total_beats: float,
    bpm: float,
    sample_rate: int = 16000,
    seed: int = 0,
) -> Dict[str, np.ndarray]:
    """События -> {роль: моно-буфер}, все буферы одной длины.

    Длина = ``total_beats`` в секундах при ``bpm`` (+хвост release
    последней ноты). Роли без событий не попадают в результат.
    """
    sec_per_beat = 60.0 / max(1.0, float(bpm))
    total_s = float(total_beats) * sec_per_beat
    n_total = max(1, int(round(total_s * sample_rate)) + int(_RELEASE_S * sample_rate) + 1)
    rng = np.random.default_rng(seed)

    buffers: Dict[str, np.ndarray] = {}
    for ev in events:
        buffers.setdefault(ev.role, np.zeros(n_total, dtype=np.float64))
        start_sample = int(round(ev.start_beat * sec_per_beat * sample_rate))
        n_samples = max(1, int(round(ev.dur_beat * sec_per_beat * sample_rate)))
        if start_sample >= n_total:
            continue
        n_samples = min(n_samples, n_total - start_sample)
        env = _adsr(n_samples, sample_rate)
        if ev.freqs_hz:
            waveform = ROLE_WAVEFORM.get(ev.role, "sine")
            t = np.arange(n_samples, dtype=np.float64) / sample_rate
            tone = np.zeros(n_samples, dtype=np.float64)
            for freq in ev.freqs_hz:
                tone += _osc(waveform, freq, t)
            tone /= max(1, len(ev.freqs_hz))
            chunk = tone * env * ev.amp
        elif ev.noise_band_hz is not None:
            chunk = _noise_burst(n_samples, ev.noise_band_hz, sample_rate, rng) * env * ev.amp
        else:
            continue
        buffers[ev.role][start_sample:start_sample + n_samples] += chunk
    return buffers


def sum_buffers(buffers: Dict[str, np.ndarray]) -> np.ndarray:
    if not buffers:
        return np.zeros(1, dtype=np.float64)
    length = max(len(b) for b in buffers.values())
    total = np.zeros(length, dtype=np.float64)
    for b in buffers.values():
        total[: len(b)] += b
    return total
