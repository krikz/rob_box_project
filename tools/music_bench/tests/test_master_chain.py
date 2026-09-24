"""Свойства модели masterfilter.scd (issue #2977) — не «звучит нормально», а числа."""

from __future__ import annotations

import numpy as np

from tools.music_bench.master_chain import apply_master_chain

SR = 16000


def test_tanh_never_exceeds_gain():
    """``tanh`` в [-1, 1] по определению -> выход не может превысить ``gain``."""
    rng = np.random.default_rng(0)
    loud = rng.uniform(-5.0, 5.0, size=SR)
    result = apply_master_chain(loud, SR, gain=0.5)
    assert np.max(np.abs(result.output)) <= 0.5 + 1e-9


def test_pre_tanh_can_exceed_one_when_input_is_loud():
    """Сумма слоёв > 1 ДО tanh — то, что измеряет issue #2963."""
    t = np.arange(SR) / SR
    loud_sum = 2.0 * np.sin(2 * np.pi * 200 * t)  # заведомо выше 1.0
    result = apply_master_chain(loud_sum, SR)
    assert np.max(np.abs(result.pre_tanh)) > 1.0


def test_nan_input_does_not_propagate():
    """Правило 1 masterfilter.scd: NaN на входе не должен выйти наружу."""
    sig = np.full(1000, np.nan)
    result = apply_master_chain(sig, SR)
    assert np.all(np.isfinite(result.output))
    assert np.all(np.isfinite(result.pre_tanh))


def test_hpf_removes_dc_offset():
    """HPF 35 Гц должен убрать постоянную составляющую (issue #2977: «субниз»)."""
    sig = np.ones(SR) * 0.5
    result = apply_master_chain(sig, SR, hpf_hz=35.0, gain=1.0)
    # После переходного процесса фильтра сигнал должен осесть у нуля.
    tail = result.pre_tanh[SR // 2:]
    assert abs(np.mean(tail)) < 0.05


def test_lpf_attenuates_above_cutoff():
    """LPF должен заметно ослаблять тон значительно выше среза."""
    t = np.arange(SR) / SR
    low_tone = np.sin(2 * np.pi * 200 * t)
    high_tone = np.sin(2 * np.pi * 7900 * t)  # почти у Найквиста 8000

    low_out = apply_master_chain(low_tone, SR, gain=1.0).pre_tanh
    high_out = apply_master_chain(high_tone, SR, gain=1.0).pre_tanh

    low_rms = np.sqrt(np.mean(np.square(low_out[SR // 4:])))
    high_rms = np.sqrt(np.mean(np.square(high_out[SR // 4:])))
    assert high_rms < low_rms * 0.5
