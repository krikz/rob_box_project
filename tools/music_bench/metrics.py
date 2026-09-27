"""Числа из issue #2977: peak/crest/centroid/насыщение/слои-в-моменте.

``true_peak`` — приближение (4x линейный оверсэмплинг перед поиском
максимума), НЕ полный ITU-R BS.1770 true-peak (тот требует конкретного
полифазного фильтра). Для сравнения «эта конфигурация звучит громче/
резче той» этого достаточно; абсолютные значения не заявляются как
эталонные — см. ограничения в docs/music_bench.md.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np
from scipy.signal import resample_poly

_EPS = 1e-12


def pct_time_over(sig: np.ndarray, threshold: float = 1.0) -> float:
    """Доля отсчётов, где ``|sig| > threshold`` (насыщение до ``tanh``, RC1/#2963)."""
    if sig.size == 0:
        return 0.0
    return float(np.mean(np.abs(sig) > threshold))


def peak(sig: np.ndarray) -> float:
    return float(np.max(np.abs(sig))) if sig.size else 0.0


def true_peak_approx(sig: np.ndarray, oversample: int = 4) -> float:
    """Пиковое значение после линейного оверсэмплинга (приближение, не BS.1770)."""
    if sig.size < 2:
        return peak(sig)
    up = resample_poly(sig, oversample, 1)
    return float(np.max(np.abs(up)))


def crest_factor_db(sig: np.ndarray) -> float:
    """20*log10(peak/rms) в дБ. Тишина (``rms==0``) -> ``inf``."""
    if sig.size == 0:
        return 0.0
    p = peak(sig)
    rms = float(np.sqrt(np.mean(np.square(sig))))
    if rms <= _EPS:
        return float("inf") if p > _EPS else 0.0
    return float(20.0 * np.log10(p / rms))


def spectral_centroid_hz(sig: np.ndarray, sample_rate: int) -> float:
    """Взвешенная по амплитуде спектра средняя частота. Тишина -> 0.0."""
    if sig.size < 2:
        return 0.0
    spectrum = np.abs(np.fft.rfft(sig))
    freqs = np.fft.rfftfreq(sig.size, d=1.0 / sample_rate)
    total = float(np.sum(spectrum))
    if total <= _EPS:
        return 0.0
    return float(np.sum(spectrum * freqs) / total)


def simultaneous_layers_per_section(
    events, plan: Sequence[Tuple[str, int, dict]], beats_per_bar: int,
) -> List[Tuple[str, int, int]]:
    """(имя секции, тактов, число РАЗНЫХ ролей со звучащим событием в ней).

    Считает по фактическим событиям (после огибающей/паузы паттерна), а не
    по номинальному составу формы — молчащий в этой секции слой (см.
    ``arranger._is_silent_drum_layer`` и паузы паттерна) не завышает
    подсчёт.
    """
    bounds = []
    acc = 0.0
    for name, bars, _i in plan:
        start = acc
        acc += int(bars) * beats_per_bar
        bounds.append((name, int(bars), start, acc))

    out: List[Tuple[str, int, int]] = []
    for name, bars, start, end in bounds:
        roles = {
            ev.role for ev in events
            if start <= ev.start_beat < end and ev.amp > 0
        }
        out.append((name, bars, len(roles)))
    return out


@dataclass(frozen=True)
class TrackMetrics:
    """Один прогон стенда — числа для строки отчёта."""

    label: str
    pre_tanh_peak: float
    pct_time_over_1: float
    true_peak: float
    crest_factor_db: float
    role_centroid_hz: Dict[str, float]
    layers_per_section: List[Tuple[str, int, int]]
    max_simultaneous_layers: int

    def as_row(self) -> Dict[str, object]:
        return {
            "label": self.label,
            "pre_tanh_peak": round(self.pre_tanh_peak, 4),
            "pct_time_over_1": round(self.pct_time_over_1, 5),
            "true_peak": round(self.true_peak, 4),
            "crest_factor_db": round(self.crest_factor_db, 2)
            if self.crest_factor_db not in (float("inf"), float("-inf"))
            else self.crest_factor_db,
            "max_simultaneous_layers": self.max_simultaneous_layers,
            **{f"centroid_{role}_hz": round(hz, 1) for role, hz in self.role_centroid_hz.items()},
        }
