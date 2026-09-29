"""Общий запуск Prometheus HTTP-сервера метрик.

architecture audit 2026-09-29, ADR-0145: логика ``start_metrics_server``
раньше дублировалась в ``rob_box_telegram.observability`` и
``rob_box_voice.observability.metrics``. Здесь — только общее тело;
состояние (набор уже занятых портов, lock), ``start_http_server`` и logger
передаёт вызывающий модуль, поэтому тесты по-прежнему могут патчить
``start_http_server`` / сбрасывать флаги в модуле-вызывающем.
"""

from __future__ import annotations

import logging
import threading
from typing import Any, Callable


def start_metrics_http_server(
    port: int,
    *,
    start_http_server: Callable[[int], Any],
    started: set[int],
    lock: threading.Lock,
    log: logging.Logger,
) -> bool:
    """Идемпотентно запускает ``start_http_server(port)``.

    Повторный вызов с тем же портом (он есть в ``started``) — no-op
    (``True``). ``OSError`` (порт занят) логируется как warning и даёт
    ``False``; порт при этом НЕ помечается как started.

    Проверка доступности ``prometheus_client`` (``is_metrics_enabled``)
    остаётся на стороне вызывающего.
    """
    with lock:
        if port in started:
            return True
        try:
            start_http_server(port)
        except OSError as exc:
            log.warning(
                "prometheus_client.start_http_server(%d) failed: %s",
                port,
                exc,
            )
            return False
        started.add(port)
        log.info("Prometheus metrics server started on :%d/metrics", port)
        return True
