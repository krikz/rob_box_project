#!/usr/bin/env python3
"""Observability — Prometheus metrics for the Telegram bot (issue #1160).

Lёгкий автономный аналог ``rob_box_voice.observability``: telegram-bot —
отдельный контейнер (см. ``docker/vision/telegram_bot/Dockerfile``), он
не должен тянуть ``rob_box_voice``. Здесь только то, что нужно для
``telegram_message_total``.

Принципы совпадают с voice-модулем:

* **Optional dep.** ``prometheus_client`` — optional; если пакета нет
  (минимальный CI, юнит-тесты), все функции — no-op и
  ``start_metrics_server()`` возвращает ``False``.
* **Idempotent registration.** Метрика создаётся один раз (singleton).
* **Lazy HTTP server.** ``start_metrics_server(port)`` запускает сервер
  ровно один раз на порт; повторный вызов — no-op (``True``).

Endpoints (per issue #1160): ``9101`` — telegram-bot.
"""

from __future__ import annotations

import logging
import threading
from typing import Any, Dict, TYPE_CHECKING

if TYPE_CHECKING:
    from prometheus_client import Counter as _Counter  # noqa: F401

try:
    from prometheus_client import Counter, start_http_server
except ImportError:  # pragma: no cover — exercised via unit tests
    Counter = None  # type: ignore[assignment,misc]
    start_http_server = None  # type: ignore[assignment]

_log = logging.getLogger(__name__)

_metric_registry: Dict[str, Any] = {}
_registry_lock = threading.Lock()
_http_server_started: set[int] = set()
_http_server_lock = threading.Lock()


class _NoopMetric:
    """No-op singleton для Counter, когда prometheus_client недоступен."""

    __slots__ = ()

    def labels(self, *args: Any, **kwargs: Any) -> '_NoopMetric':
        return self

    def inc(self, amount: float = 1.0) -> None:
        return None

    def __getattr__(self, name: str) -> Any:
        return self


def is_metrics_enabled() -> bool:
    return Counter is not None and start_http_server is not None


def start_metrics_server(port: int) -> bool:
    """Запускает ``prometheus_client.start_http_server(port)``.

    Идемпотентно: повторный вызов с тем же портом — no-op (``True``).
    """
    if not is_metrics_enabled():
        return False
    with _http_server_lock:
        if port in _http_server_started:
            return True
        try:
            start_http_server(port)  # type: ignore[misc]
        except OSError as exc:
            _log.warning(
                'prometheus_client.start_http_server(%d) failed: %s',
                port, exc,
            )
            return False
        _http_server_started.add(port)
        _log.info('Prometheus metrics server started on :%d/metrics', port)
        return True


def _get_counter(name: str, documentation: str, labelnames: tuple[str, ...]) -> Any:
    if not is_metrics_enabled():
        return _NoopMetric()
    key = f'counter:{name}'
    with _registry_lock:
        existing = _metric_registry.get(key)
        if existing is not None:
            return existing
        metric = Counter(name, documentation, labelnames=labelnames)  # type: ignore[misc]
        _metric_registry[key] = metric
        return metric


def record_telegram_message(direction: str, *, message_type: str = 'text') -> None:
    """Учёт telegram-сообщения.

    :param direction: ``"in"`` (входящее от юзера) или ``"out"``
        (исходящее от бота).
    :param message_type: ``"text"`` / ``"voice"`` / ``"command"`` /
        ``"callback"``.
    """
    counter = _get_counter(
        'telegram_message_total',
        'Telegram bot messages, labelled by direction and type.',
        ('direction', 'type'),
    )
    counter.labels(direction=direction, type=message_type).inc()


__all__ = [
    'is_metrics_enabled',
    'record_telegram_message',
    'start_metrics_server',
]
