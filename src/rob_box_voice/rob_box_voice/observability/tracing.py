#!/usr/bin/env python3
"""OpenTelemetry-трейсы для диалоговой цепочки (issue #1234, этап 2).

Назначение модуля
-----------------
Дать трейсы для трёх ключевых этапов голосового ассистента:

* ``dialogue.llm_call`` — dialogue_node (обёртка ``process_input`` → LLM)
* ``tts.synthesize`` — tts_node (обёртка цепочки синтеза)
* ``stt.recognize`` — stt_node (обёртка распознавания)

Ограничение из #1160: **БЕЗ rclpy propagation** (ros2_tracing #15 не решён
с 2022) — трейсы живут в рамках процесса, не связываются между ROS-нодами.
Каждый ROS2-нода стартует в отдельном процессе и создаёт СВОИ корневые
спаны; в Tempo они видны per-process, но parent/child между нодами нет.

Дизайн
------
* ``opentelemetry`` — **optional dep** (как ``prometheus_client`` в
  :mod:`rob_box_voice.observability.metrics`). Если пакетов нет в окружении
  (минимальный CI, юнит-тесты без OTel) — все функции ниже no-op,
  ``init_tracing`` возвращает ``False``, ``start_span`` отдаёт no-op span.
  Это позволяет прод-коду и тестам делить один импорт без try/except в
  каждом вызове.
* ``init_tracing(service_name)`` — идемпотентно (один раз на процесс)
  настраивает глобальный ``TracerProvider`` + ``BatchSpanProcessor`` с
  OTLP-экспортёром и включает httpx auto-instrumentation
  (``opentelemetry-instrumentation-httpx``). Endpoint читается из
  ``OTEL_EXPORTER_OTLP_ENDPOINT`` (default ``http://10.1.1.249:4317`` —
  otel-collector на мониторинг-машине).
* ``start_span(name, attributes)`` — контекст-менеджер. Всегда можно
  использовать в прод-коде; без OTel отдаёт no-op span с теми же методами
  (``set_attribute`` и т.п.), чтобы код не ветвился.
* ``start_span_handle(name, attributes)`` — «ручной» span для больших
  блоков, где ``with`` неудобен (например, огромный ``try`` в
  ``tts_node._synthesize_and_play``). Открывается до ``try``, закрывается
  в ``finally`` через ``.close()``.
"""

from __future__ import annotations

import logging
import os
import threading
from contextlib import contextmanager
from typing import Any, Dict, Iterator, Optional

_log = logging.getLogger(__name__)

# ── OTel — optional dep. Если нет — no-op (как prometheus_client) ──────────
# ВАЖНО: ловим Exception, а не только ImportError. В юнит-тестах tts_node
# conftest мокает ``grpc`` (MagicMock без ``__version__``), и импорт OTLP-
# экспортёра падает с AttributeError — это НЕ должно ронять сбор тестов.
# OTel обязан оставаться опциональным: если он не импортируется по любой
# причине — работаем без трейсов.
try:
    from opentelemetry import trace as _trace_api
    from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
    from opentelemetry.instrumentation.httpx import HTTPXClientInstrumentor
    from opentelemetry.sdk.resources import Resource
    from opentelemetry.sdk.trace import TracerProvider
    from opentelemetry.sdk.trace.export import BatchSpanProcessor

    _OTEL_AVAILABLE = True
except Exception:  # noqa: BLE001 — exercised via unit tests
    _OTEL_AVAILABLE = False
    _trace_api = None  # type: ignore[assignment]
    OTLPSpanExporter = None  # type: ignore[assignment,misc]
    HTTPXClientInstrumentor = None  # type: ignore[assignment,misc]
    Resource = None  # type: ignore[assignment,misc]
    TracerProvider = None  # type: ignore[assignment,misc]
    BatchSpanProcessor = None  # type: ignore[assignment,misc]

# Default OTLP endpoint — otel-collector на мониторинг-машине (249).
# Переопределяется через OTEL_EXPORTER_OTLP_ENDPOINT (стандартная env OTel).
DEFAULT_OTLP_ENDPOINT: str = os.environ.get(
    "OTEL_EXPORTER_OTLP_ENDPOINT", "http://10.1.1.249:4317"
)

# Трейсер по умолчанию для всех нод voice-пакета.
_TRACER_NAME: str = "rob_box_voice"

_TRACING_INITIALIZED: bool = False
_TRACING_LOCK = threading.Lock()


# ── No-op span / tracer (без OTel) ─────────────────────────────────────────
class _NoopSpan:
    """No-op span: любой вызов (set_attribute, set_status, close) — пусто.

    Прод-код использует его через ``with start_span(...)`` или
    ``start_span_handle(...)`` — интерфейс совпадает с реальным span'ом,
    поэтому ветвления ``if is_tracing_enabled()`` в каждом месте не нужны.
    """

    __slots__ = ("name", "attributes")

    def __init__(self, name: str = "", attributes: Optional[Dict[str, Any]] = None) -> None:
        self.name = name
        self.attributes: Dict[str, Any] = dict(attributes or {})

    def set_attribute(self, key: str, value: Any) -> "_NoopSpan":
        self.attributes[key] = value
        return self

    def set_status(self, *args: Any, **kwargs: Any) -> "_NoopSpan":
        return self

    def record_exception(self, *args: Any, **kwargs: Any) -> "_NoopSpan":
        return self

    def end(self) -> None:
        return None

    def close(self) -> None:
        return None

    def __enter__(self) -> "_NoopSpan":
        return self

    def __exit__(self, *args: Any) -> None:
        return None


class _SpanHandle:
    """Ручной span-handle для блоков, где ``with`` неудобен (большой try).

    Открывается в ``start_span_handle`` (через ``__enter__`` реального
    context manager), закрывается ``close()`` — в ``finally`` вызывающего
    кода. ``set_attribute`` до ``close()`` пишет в реальный span.
    """

    __slots__ = ("_cm", "_span", "_closed")

    def __init__(self, cm: Any, span: Any) -> None:
        self._cm = cm
        self._span = span
        self._closed = False

    def set_attribute(self, key: str, value: Any) -> "_SpanHandle":
        if not self._closed:
            self._span.set_attribute(key, value)
        return self

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._cm.__exit__(None, None, None)


# ── Публичные функции ──────────────────────────────────────────────────────
def is_tracing_enabled() -> bool:
    """True, если opentelemetry-пакеты доступны в окружении."""
    return _OTEL_AVAILABLE


def get_otlp_endpoint() -> str:
    """Endpoint OTLP-экспортёра (env → default)."""
    return os.environ.get("OTEL_EXPORTER_OTLP_ENDPOINT", DEFAULT_OTLP_ENDPOINT)


def init_tracing(service_name: str) -> bool:
    """Инициализирует глобальный OTel TracerProvider + OTLP exporter + httpx.

    Идемпотентно: повторный вызов (например, из юнит-теста, где нода
    пересоздаётся в том же процессе) — no-op, возвращает ``True``.

    httpx auto-instrumentation включается ТУТ ЖЕ (``HTTPXClientInstrumentor``
    патчит классы ``httpx.Client``/``AsyncClient`` глобально), поэтому
    вызов должен быть ДО создания LLM/TTS-провайдеров (openai SDK и
    MiniMax TTS держат свои httpx-клиенты).

    :param service_name: имя сервиса в атрибуте ``service.name`` (например
        ``"dialogue_node"``, ``"tts_node"``, ``"stt_node"``).
    :return: ``True`` если трассировка включена, ``False`` если OTel
        недоступен или инициализация упала (не фатально).
    """
    global _TRACING_INITIALIZED

    if not _OTEL_AVAILABLE:
        return False

    with _TRACING_LOCK:
        if _TRACING_INITIALIZED:
            return True
        try:
            resource = Resource.create({"service.name": service_name})
            provider = TracerProvider(resource=resource)
            exporter = OTLPSpanExporter(endpoint=get_otlp_endpoint())
            provider.add_span_processor(BatchSpanProcessor(exporter))
            _trace_api.set_tracer_provider(provider)

            # httpx auto-instrumentation: все внешние HTTP-вызовы (LLM/TTS)
            # станут child-spans под текущим span'ом.
            HTTPXClientInstrumentor().instrument()

            _TRACING_INITIALIZED = True
            _log.info(
                "OpenTelemetry tracing initialized (service=%s, otlp=%s)",
                service_name, get_otlp_endpoint(),
            )
            return True
        except Exception as exc:  # noqa: BLE001
            # OTel не должен ронять ноду: если OTLP-экспортёр не поднялся
            # (нет сети до collector, битая версия пакета) — логируем и
            # работаем без трейсов.
            _log.warning(
                "OpenTelemetry init failed (service=%s): %r",
                service_name, exc,
            )
            return False


def get_tracer(name: str = _TRACER_NAME) -> Any:
    """Возвращает OTel-tracer (или no-op tracer, если OTel недоступен)."""
    if not _OTEL_AVAILABLE:
        # No-op tracer: start_span/start_as_current_span возвращают no-op span.
        return _NoopTracer()
    return _trace_api.get_tracer(name)


class _NoopTracer:
    """No-op tracer: имитирует start_span / start_as_current_span."""

    __slots__ = ()

    def start_span(self, name: str, attributes: Optional[Dict[str, Any]] = None, **kwargs: Any) -> _NoopSpan:
        return _NoopSpan(name, attributes)

    def start_as_current_span(self, name: str, attributes: Optional[Dict[str, Any]] = None, **kwargs: Any) -> _NoopSpan:
        return _NoopSpan(name, attributes)


@contextmanager
def start_span(
    name: str,
    attributes: Optional[Dict[str, Any]] = None,
    *,
    tracer_name: str = _TRACER_NAME,
) -> Iterator[Any]:
    """Контекст-менеджер span'а.

    Пример::

        with start_span("dialogue.llm_call", {"provider": "deepseek"}) as span:
            result = await core.process_input(...)
            span.set_attribute("duration_s", ...)

    Без OTel — no-op span с совместимым интерфейсом (``set_attribute`` и
    ``close`` ничего не делают). С OTel — ``start_as_current_span``: все
    httpx child-spans (LLM/TTS HTTP-вызовы) попадают под этот span.
    """
    if not _OTEL_AVAILABLE:
        yield _NoopSpan(name, attributes)
        return
    tracer = _trace_api.get_tracer(tracer_name)
    with tracer.start_as_current_span(name, attributes=attributes or {}) as span:
        yield span


def start_span_handle(
    name: str,
    attributes: Optional[Dict[str, Any]] = None,
    *,
    tracer_name: str = _TRACER_NAME,
) -> Any:
    """Открывает span вручную (для больших try-блоков).

    Возвращает объект с ``set_attribute(key, value)`` и ``close()``.
    Без OTel — no-op span (``close`` пустой). С OTel — span становится
    current (httpx child-spans попадают под него) до вызова ``close()``.

    Типичный паттерн::

        _span = start_span_handle("tts.synthesize", {"provider": "minimax"})
        try:
            ...
        finally:
            _span.set_attribute("duration_s", ...)
            _span.close()
    """
    if not _OTEL_AVAILABLE:
        return _NoopSpan(name, attributes)
    tracer = _trace_api.get_tracer(tracer_name)
    cm = tracer.start_as_current_span(name, attributes=attributes or {})
    span = cm.__enter__()
    return _SpanHandle(cm, span)


__all__ = [
    "DEFAULT_OTLP_ENDPOINT",
    "get_otlp_endpoint",
    "get_tracer",
    "init_tracing",
    "is_tracing_enabled",
    "start_span",
    "start_span_handle",
]
