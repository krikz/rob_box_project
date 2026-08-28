"""test_observability.py — Unit tests for ``rob_box_voice.observability`` (issue #1160).

Проверяем три контракта модуля:

1. **Optional dep** — без ``prometheus_client`` все функции — no-op,
   ``start_metrics_server`` возвращает ``False``.
2. **Idempotent registration** — повторный ``get_metric`` с тем же именем
   возвращает тот же объект (не падает с ``ValueError: Duplicated
   timeseries`` при перезагрузке ноды в unit-тесте).
3. **record_* helpers** — с доступным ``prometheus_client`` пишут в
   правильные counters/histograms (проверяем через ``get_metric`` +
   ``_sample`` из прометеевского REGISTRY).

Pure Python — no ROS, no rclpy. ``prometheus_client`` не обязателен:
если пакет установлен, дополнительные тесты метрик проходят; если нет —
проверяется только no-op поведение.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/core/test_observability.py
"""

from __future__ import annotations

import threading
import time

import pytest

from rob_box_voice.observability import (
    HistogramValue,
    MetricsDisabled,
    get_metric,
    is_metrics_enabled,
    record_barge_in,
    record_session_duration,
    record_speaker_recognize,
    record_stt_recognize,
    record_telegram_message,
    record_tts_synthesize,
    record_voice_llm_request,
    start_metrics_server,
)


class TestNoopBehaviour:
    """Контракт «optional dep»: без prometheus_client всё — no-op."""

    def test_get_metric_disabled_returns_noop(self):
        if is_metrics_enabled():
            pytest.skip("prometheus_client installed — no-op path not exercised")
        metric = get_metric("counter", "test_noop_counter", "doc")
        assert isinstance(metric, MetricsDisabled)

    def test_record_helpers_noop(self):
        if is_metrics_enabled():
            pytest.skip("prometheus_client installed — no-op path not exercised")
        # Ни один из helpers не должен падать без prometheus_client.
        record_stt_recognize("yandex", success=True, duration_s=0.1)
        record_voice_llm_request("deepseek", success=True, fallback=False, duration_s=0.5)
        record_tts_synthesize("minimax", success=True, duration_s=0.2)
        record_speaker_recognize(known=True, confidence=0.9)
        record_barge_in()
        record_session_duration(12.0, result="success")
        record_telegram_message("in", message_type="text")

    def test_start_server_disabled_returns_false(self):
        if is_metrics_enabled():
            pytest.skip("prometheus_client installed — no-op path not exercised")
        assert start_metrics_server(19999) is False

    def test_histogram_value_noop(self):
        if is_metrics_enabled():
            pytest.skip("prometheus_client installed — no-op path not exercised")
        with HistogramValue("test_hist", {"provider": "x"}):
            pass  # не падает


@pytest.mark.skipif(
    not is_metrics_enabled(),
    reason="prometheus_client not installed — metric tests skipped",
)
class TestMetricsWithPrometheusClient:
    """Контракт «идемпотентность + helpers» — только с установленным пакетом."""

    def test_get_metric_idempotent(self):
        m1 = get_metric("counter", "test_idem_counter", "doc", labelnames=("x",))
        m2 = get_metric("counter", "test_idem_counter", "doc", labelnames=("x",))
        assert m1 is m2

    def test_record_stt_recognize_increments(self):
        # Считаем до и после — уникальное имя, чтобы не пересекаться с
        # другими тестами/нодами в том же процессе.
        before = _counter_value("voice_stt_recognize_total", {"provider": "yandex", "result": "success"})
        record_stt_recognize("yandex", success=True, duration_s=0.1)
        after = _counter_value("voice_stt_recognize_total", {"provider": "yandex", "result": "success"})
        assert after == before + 1

    def test_record_tts_synthesize_increments(self):
        before = _counter_value("voice_tts_synthesize_total", {"provider": "silero", "result": "fail"})
        record_tts_synthesize("silero", success=False, duration_s=None)
        after = _counter_value("voice_tts_synthesize_total", {"provider": "silero", "result": "fail"})
        assert after == before + 1

    def test_record_voice_llm_request_success_and_fallback(self):
        b_success = _counter_value("voice_llm_request_total", {"provider": "minimax", "result": "success"})
        b_fallback = _counter_value("voice_llm_request_total", {"provider": "minimax", "result": "fallback"})
        record_voice_llm_request("minimax", success=True, fallback=False, duration_s=0.3)
        record_voice_llm_request("minimax", success=True, fallback=True, duration_s=0.9)
        a_success = _counter_value("voice_llm_request_total", {"provider": "minimax", "result": "success"})
        a_fallback = _counter_value("voice_llm_request_total", {"provider": "minimax", "result": "fallback"})
        assert a_success == b_success + 1
        assert a_fallback == b_fallback + 1

    def test_record_speaker_recognize_known_unknown(self):
        b_known = _counter_value("voice_speaker_recognize_total", {"result": "known"})
        b_unknown = _counter_value("voice_speaker_recognize_total", {"result": "unknown"})
        record_speaker_recognize(known=True, confidence=0.95)
        record_speaker_recognize(known=False, confidence=None)
        a_known = _counter_value("voice_speaker_recognize_total", {"result": "known"})
        a_unknown = _counter_value("voice_speaker_recognize_total", {"result": "unknown"})
        assert a_known == b_known + 1
        assert a_unknown == b_unknown + 1

    def test_record_barge_in(self):
        before = _counter_value("voice_barge_in_total", {})
        record_barge_in()
        after = _counter_value("voice_barge_in_total", {})
        assert after == before + 1

    def test_record_telegram_message(self):
        b_in = _counter_value("telegram_message_total", {"direction": "in", "type": "text"})
        b_out = _counter_value("telegram_message_total", {"direction": "out", "type": "voice"})
        record_telegram_message("in", message_type="text")
        record_telegram_message("out", message_type="voice")
        a_in = _counter_value("telegram_message_total", {"direction": "in", "type": "text"})
        a_out = _counter_value("telegram_message_total", {"direction": "out", "type": "voice"})
        assert a_in == b_in + 1
        assert a_out == b_out + 1

    def test_session_duration_histogram(self):
        from prometheus_client import REGISTRY

        before = _hist_sum("voice_session_duration_seconds", {"result": "success"})
        record_session_duration(5.0, result="success")
        after = _hist_sum("voice_session_duration_seconds", {"result": "success"})
        assert after >= before + 5.0


def _counter_value(name: str, labels: dict) -> int:
    """Читает текущее значение счётчика из REGISTRY (0 если метка не существует).

    ВАЖНО: prometheus_client (>=0.20) автоматически добавляет суффикс
    ``_total`` к имени Counter: metric.name == "voice_stt_recognize",
    а sample.name == "voice_stt_recognize_total". Сравниваем именно
    sample.name, чтобы не зависеть от версии клиента.
    """
    from prometheus_client import REGISTRY

    for metric in REGISTRY.collect():
        for sample in metric.samples:
            if sample.name == name and all(
                sample.labels.get(k) == v for k, v in labels.items()
            ):
                return int(sample.value)
    return 0


def _hist_sum(name: str, labels: dict) -> float:
    """Читает сумму наблюдений гистограммы (sample '<name>_sum')."""
    from prometheus_client import REGISTRY

    for metric in REGISTRY.collect():
        for sample in metric.samples:
            if sample.name == f"{name}_sum" and all(
                sample.labels.get(k) == v for k, v in labels.items()
            ):
                return float(sample.value)
    return 0.0
