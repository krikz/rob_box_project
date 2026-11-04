"""Метрики Prometheus хода dialog v2 (#4328): тонкий адаптер над ``observability.metrics``.

Считает код, а не модель: провайдер, исход и задержку знает ``V2Ports.ask`` (вокруг вызова цепочки LLM), «ясный
стоп» знает ``DialogNode._interrupt``. Имена метрик объявлены в ``rob_box_voice.observability.metrics``
(``voice_llm_request_total``, ``voice_llm_request_duration_seconds``, ``voice_llm_fallback_total``,
``voice_barge_in_total``); здесь только вызовы. Без ``prometheus_client`` всё — no-op.
"""

from __future__ import annotations

from typing import Any

from rob_box_voice.observability import record_barge_in, record_fallback, record_voice_llm_request


def primary_provider(llm: Any) -> str:
    """Первый провайдер цепочки (``snapshot()`` health-обёртки); у одиночного LLM — его ``name``."""
    snapshot = getattr(llm, "snapshot", None)
    if callable(snapshot):
        try:
            rows = snapshot()
            if rows:
                return str(rows[0].get("name") or "")
        except Exception:  # noqa: BLE001 — метрика не должна ронять ход
            return ""
    return str(getattr(llm, "name", "") or "")


def record_llm_ask(llm: Any, provider: str, *, ok: bool, duration_s: float) -> None:
    """Итог одного вызова цепочки LLM: ``success`` (ответил primary), ``fallback`` (ответил не primary), ``fail``."""
    provider = provider or "unknown"
    primary = primary_provider(llm)
    fallback = ok and bool(primary) and provider != primary
    record_voice_llm_request(provider, success=ok, fallback=fallback, duration_s=duration_s)
    if fallback:
        record_fallback(primary, provider, reason="primary_failed")


def record_interrupt() -> None:
    """Ясный стоп человека во время хода или речи робота (barge-in)."""
    record_barge_in()
