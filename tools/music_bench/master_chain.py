"""Python-модель мастер-шины ``masterfilter.scd`` (issue #2977).

Повторяет ЧЕТЫРЕ шага реального ``.scd`` (см. docstring файла в
``docker/vision/voice_assistant/custom_synthdefs/masterfilter.scd``):
чистка NaN/Inf -> HPF 35 Гц -> LPF ``min(lpf, nyquist*0.55)`` -> мягкий
потолок ``tanh(sig*drive)`` -> мастер-фейдер ``gain``.

Честное ограничение: HPF/LPF здесь — 2-полюсные Баттерворт-фильтры
(``scipy.signal.butter`` + ``lfilter``, каузально, как в реальном
времени), а не байт-в-байт тот же UGen, что в SuperCollider (``HPF``/
``LPF`` в SC — тоже 2-полюсные, но с собственной, не документированной
наружу реализацией resonant-less фильтра). Для метрик, которые считает
этот стенд (доля времени с суммой > 1 до ``tanh``, true peak, crest
factor, спектральный центроид), разница между двумя корректными
2-полюсными АЧХ на 35 Гц и ~4000-7000 Гц не меняет вывод — RC5.1/RC1 в
аудите про ПОРЯДОК величины (клиппинг есть или нет), не про доли дБ на
срезе.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import butter, lfilter


@dataclass(frozen=True)
class MasterChainResult:
    """Сигнал ДО ``tanh`` (для метрики насыщения) и финальный выход."""

    pre_tanh: np.ndarray
    output: np.ndarray


def apply_master_chain(
    sig: np.ndarray,
    sample_rate: int,
    hpf_hz: float = 35.0,
    lpf_hz: float = 7000.0,
    gain: float = 0.5,
    drive: float = 1.0,
) -> MasterChainResult:
    """Пропустить моно-сигнал через модель ``masterfilter.scd``."""
    clean = np.nan_to_num(np.asarray(sig, dtype=np.float64), nan=0.0, posinf=0.0, neginf=0.0)

    nyquist = sample_rate * 0.5
    b_hp, a_hp = butter(2, max(1e-6, hpf_hz / nyquist), btype="high")
    sig_hp = lfilter(b_hp, a_hp, clean)

    cutoff = min(lpf_hz, nyquist * 0.55)
    b_lp, a_lp = butter(2, min(0.999, cutoff / nyquist), btype="low")
    sig_lp = lfilter(b_lp, a_lp, sig_hp)

    pre_tanh = sig_lp * drive
    output = np.tanh(pre_tanh) * gain
    return MasterChainResult(pre_tanh=pre_tanh, output=output)
