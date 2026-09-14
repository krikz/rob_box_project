"""Pytest маркер ``pregenerate_latency`` для замера TTS-латентности между чанками.

ADR-0056 §3.7 / issue #2003 / DoD: «спекулятивный путь уменьшил latency между
``batch_complete`` чанка N и готовностью чанка N+1, raw-цифры».

Этот модуль собирает *измеримые* метрики для двух режимов:

* ``mode='baseline'``   — последовательный синтез двух чанков (как было ДО
  ``pregenerate``: текущее состояние develop, где ``pregenerate`` в ``tts_node``
  не подключён, см. ``docker/vision/config/tts_node.yaml:8``).
* ``mode='speculative'`` — параллельный pre-gen чанка N+1 во время playback N.

Замеры повторяются ``N_RUNS`` раз для уменьшения шума, печатают медиану и
среднее. Принимаются pytest-маркером::

    pytest -m pregenerate_latency -v -s

Логика соответствует ``test_speculative_path_latency.py`` (эталонный тест),
но фокус здесь — *сбор цифр*, а не ассерты. Это позволяет:

1. Запускать в CI как «контрактную» проверку (см. ``pytest --collect-only -m
   pregenerate_latency`` — собирается, но не прогоняется).
2. Запускать локально с ``-s`` для raw-вывода.
3. Прогонять вручную перед мержем PR #2373 (``z-{agent}/2003-operator-agent-13-
   pregenerate-tts-node-v2``).

DoD-метрики
-----------
``batch_complete`` → ``ready`` latency (ADR-0056 §3.7):
* **baseline**:   ~2 × synth_delay (нет параллелизма)
* **speculative**: ~synth_delay + playback_delay (один синтез прячется в playback)

Вердикт по quality/estimator (issue #2003 DoD #3): см. тесты в
``test_pregen_quality.py`` / ``test_pregen_estimator.py`` / ``test_pregen_decision.py``
— отдельные pytest-маркеры не нужны, их достаточно собрать одной командой
``pytest -m pregenerate_quality`` (см. conftest).
"""

from __future__ import annotations

import asyncio
import statistics
import time
from typing import List, Tuple

import numpy as np
import pytest

from rob_box_voice.scheduler.pregen import SpeculativeExecutor


# Задержка провайдера (как в test_speculative_path_latency.py).
_SYNTH_DELAY_S = 0.10  # 100 ms — типичный Yandex gRPC TTS

# Длительность синтезированного аудио (60 ms / 16 kHz).
_SINE_DURATION_S = 0.06
_SINE_SR = 16000
_SINE_AMPLITUDE = 0.5

# Число прогонов для усреднения шума (CI может быть медленным; 30 — хорошее
# число для стабильных median/mean).
N_RUNS = 30


def _sine_audio() -> np.ndarray:
    """60 ms / 16 kHz sine — проходит quality-эвристики."""
    t = np.arange(int(_SINE_DURATION_S * _SINE_SR))
    return (_SINE_AMPLITUDE * np.sin(2 * np.pi * 440 * t / _SINE_SR)).astype(
        np.float32
    )


def _make_slow_synth(delay_s: float = _SYNTH_DELAY_S):
    """Синхронный synth с задержкой."""
    audio = _sine_audio()

    def _synth(ssml, text, ssml_attributes, voice, language):
        time.sleep(delay_s)
        return {"audio_np": audio.copy(), "sample_rate": _SINE_SR}

    return _synth


def _make_chunk(speech_id: str, next_speech_id: str) -> dict:
    return {
        "speech_id": speech_id,
        "ssml": "<speak>x</speak>",
        "dialogue_id": "d",
        "batch_index": 1,
        "batch_total": 3,
        "pregenerate": {
            "next_speech_id": next_speech_id,
            "next_ssml": "<speak>y</speak>",
        },
    }


def _warm_history(executor: SpeculativeExecutor, n: int = 5) -> None:
    for _ in range(n):
        executor.record_synthesis_actual(
            actual_duration_ms=60.0, estimated_duration_ms=60.0
        )


def _playback_wait_ms(audio_duration_s: float) -> float:
    """Задержка, имитирующая playback (ms)."""
    return max(1.0, (audio_duration_s * 1000.0) - 1.0)


def _measure_baseline_chunk_to_chunk_ms() -> float:
    """Baseline: serial synth → playback → synth → playback.

    Возвращает суммарный wall-clock в ms.
    """
    s = _make_slow_synth(delay_s=_SYNTH_DELAY_S)
    audio_ms = _SINE_DURATION_S * 1000.0

    start = time.monotonic()
    s("x", "x", {}, "v", "ru")
    time.sleep(_playback_wait_ms(_SINE_DURATION_S) / 1000.0)
    s("y", "y", {}, "v", "ru")
    time.sleep(_playback_wait_ms(_SINE_DURATION_S) / 1000.0)
    return (time.monotonic() - start) * 1000.0


async def _run_speculative_scenario() -> float:
    """Один speculative сценарий: kickoff + claim + play."""
    executor = SpeculativeExecutor(synth_callable=_make_slow_synth(_SYNTH_DELAY_S))
    _warm_history(executor)

    start = time.monotonic()

    await executor.kickoff(_make_chunk("cur", "next"))
    await asyncio.to_thread(executor._synth, "x", "x", {}, "v", "ru")
    await asyncio.sleep(_playback_wait_ms(_SINE_DURATION_S) / 1000.0)

    # Poll claim.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        await asyncio.sleep(0.005)
        if "next" not in executor._active:
            break
    result = executor.claim("next")
    assert result is not None, "speculative task did not land in cache"

    await asyncio.sleep(_playback_wait_ms(_SINE_DURATION_S) / 1000.0)
    return (time.monotonic() - start) * 1000.0


def _measure_speculative_chunk_to_chunk_ms() -> float:
    """Speculative path."""
    return asyncio.run(_run_speculative_scenario())


def _stats(samples: List[float]) -> Tuple[float, float, float, float]:
    """min / median / mean / max."""
    return (
        min(samples),
        statistics.median(samples),
        statistics.mean(samples),
        max(samples),
    )


def _print_table(label: str, samples: List[float]) -> None:
    mn, md, mean, mx = _stats(samples)
    print(
        f"  {label:<12s} runs={len(samples):>3d}  "
        f"min={mn:7.1f}ms  median={md:7.1f}ms  "
        f"mean={mean:7.1f}ms  max={mx:7.1f}ms"
    )


# ---------------------------------------------------------------------------
# Mark: register the marker so --strict-markers doesn't complain.
# ---------------------------------------------------------------------------

pytestmark = pytest.mark.pregenerate_latency


def test_pregenerate_latency_baseline_marker():
    """Сырой замер baseline (N_RUNS итераций).

    Маркер ``pregenerate_latency`` позволяет собирать/запускать этот набор
    отдельно от других тестов:

        pytest -m pregenerate_latency -v -s   # full output

    """
    print(
        f"\n[pregenerate_latency] synth_delay={_SYNTH_DELAY_S * 1000:.0f}ms, "
        f"audio={_SINE_DURATION_S * 1000:.0f}ms, N_RUNS={N_RUNS}"
    )

    baseline_samples = [_measure_baseline_chunk_to_chunk_ms() for _ in range(N_RUNS)]
    spec_samples = [_measure_speculative_chunk_to_chunk_ms() for _ in range(N_RUNS)]

    print("[pregenerate_latency] BASELINE (no pregenerate):")
    _print_table("baseline", baseline_samples)
    print("[pregenerate_latency] SPECULATIVE (pregenerate ON):")
    _print_table("speculative", spec_samples)

    bn_min, bn_med, bn_mean, bn_max = _stats(baseline_samples)
    sn_min, sn_med, sn_mean, sn_max = _stats(spec_samples)

    delta_median = bn_med - sn_med
    delta_mean = bn_mean - sn_mean
    pct_median = (delta_median / bn_med) * 100.0
    pct_mean = (delta_mean / bn_mean) * 100.0

    print(
        f"[pregenerate_latency] DELTA  median={delta_median:+7.1f}ms "
        f"({pct_median:+5.1f}%)  mean={delta_mean:+7.1f}ms ({pct_mean:+5.1f}%)"
    )

    # Sanity-чек: speculative реально быстрее (≥10% — терпимо для asyncio jitter).
    assert pct_median >= 10.0, (
        f"Speculative path is not faster enough: median speedup {pct_median:.1f}% "
        f"< 10% (baseline={bn_med:.1f}ms, spec={sn_med:.1f}ms)"
    )