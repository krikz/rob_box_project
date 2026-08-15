"""test_tracing.py — Unit tests for ``rob_box_voice.observability.tracing`` (issue #1234).

Проверяем три контракта модуля (аналог test_observability.py для этапа 2):

1. **Optional dep** — без opentelemetry-пакетов все функции — no-op,
   ``init_tracing`` возвращает ``False``, ``start_span``/``start_span_handle``
   отдают объекты с совместимым интерфейсом (``set_attribute``/``close``).
2. **Идемпотентность** — повторный ``init_tracing`` не падает и не
   переинициализирует (с opentelemetry — возвращает ``True``).
3. **Span-хелперы** — с доступным OTel создают реальный span (проверяем
   через ``is_recording``/атрибуты); без OTel — no-op span.

Pure Python — no ROS, no rclpy. ``opentelemetry`` не обязателен: если пакет
установлен, дополнительные тесты проходят; если нет — проверяется только
no-op поведение (юнит-тесты в CI без OTel не ломаются).

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/core/test_tracing.py
"""

from __future__ import annotations

import pytest

from rob_box_voice.observability import (
    get_otlp_endpoint,
    init_tracing,
    is_tracing_enabled,
    start_span,
    start_span_handle,
)


class TestNoopBehaviour:
    """Контракт «optional dep»: без opentelemetry всё — no-op."""

    def test_init_tracing_disabled_returns_false(self):
        if is_tracing_enabled():
            pytest.skip("opentelemetry installed — no-op path not exercised")
        assert init_tracing("test_service") is False

    def test_start_span_noop(self):
        if is_tracing_enabled():
            pytest.skip("opentelemetry installed — no-op path not exercised")
        with start_span("test.span", {"provider": "deepseek"}) as span:
            span.set_attribute("duration_s", 1.0)
        # Не падает; после with объект no-op.
        assert span is not None

    def test_start_span_handle_noop(self):
        if is_tracing_enabled():
            pytest.skip("opentelemetry installed — no-op path not exercised")
        span = start_span_handle("test.span_handle", {"provider": "minimax"})
        span.set_attribute("voice", "male-qn-qingse")
        span.close()
        # Повторный close не падает.
        span.close()


@pytest.mark.skipif(
    not is_tracing_enabled(),
    reason="opentelemetry not installed — tracing tests skipped",
)
class TestTracingWithOtel:
    """Контракт «идемпотентность + span-хелперы» — только с OTel."""

    def test_init_tracing_idempotent(self):
        assert init_tracing("test_service") is True
        assert init_tracing("test_service_again") is True

    def test_start_span_records_attributes(self):
        from opentelemetry import trace as otel_trace

        with start_span("test.custom_span", {"provider": "deepseek"}) as span:
            assert span.is_recording()
            span.set_attribute("fallback", True)
            span.set_attribute("duration_s", 0.5)
            # Текущий span должен быть именно нашим (start_as_current_span).
            current = otel_trace.get_current_span()
            assert current is span

    def test_start_span_handle_close(self):
        span = start_span_handle("test.handle_span", {"provider": "minimax"})
        span.set_attribute("voice", "male-qn-qingse")
        span.close()
        # После close повторные set_attribute — no-op (не падает).
        span.set_attribute("voice", "female-chengshu")


def test_get_otlp_endpoint_default() -> None:
    """Default endpoint — otel-collector на мониторинг-машине (issue #1234)."""
    import os

    prev = os.environ.get("OTEL_EXPORTER_OTLP_ENDPOINT")
    try:
        os.environ.pop("OTEL_EXPORTER_OTLP_ENDPOINT", None)
        assert get_otlp_endpoint() == "http://10.1.1.249:4317"
    finally:
        if prev is not None:
            os.environ["OTEL_EXPORTER_OTLP_ENDPOINT"] = prev
