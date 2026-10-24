"""Статус LLM и wake-слов для экрана ТАРС 1 в v1 (issue #3253, Ш3б) — обвязка ``dialogue_node``.

Payload и дедупликация — ``rob_box_dialog.status_feed``, latched-паблишеры и модель из каталога провайдеров —
``core.dialog_v2_parity`` (#3912): одна реализация на v1 и v2. Здесь только :func:`describe_llm` — имена из
``HealthAwareFallbackLLM._providers`` v1. Модуль уходит вместе с v1 в PR-15.

* ``/voice/llm_status``  — ``{"provider", "model"?, "chain", "source"}``
* ``/voice/wake_words``  — ``{"words": [...]}``
"""

from __future__ import annotations

from typing import Any, Iterable, Optional

from rob_box_dialog.status_feed import LLM_STATUS_SOURCE, StatusFeed, describe_chain, describe_wake

from .dialog_v2_parity import catalog_model
from .dialog_v2_parity import start_status_feeds as _start_feeds


def _provider_name(provider: Any) -> Optional[str]:
    name = getattr(provider, "name", None)
    return name if isinstance(name, str) and name.strip() else None


def describe_llm(llm: Any) -> Optional[dict[str, Any]]:
    """Настроенная LLM-цепочка v1 → payload или None, если имя неизвестно."""
    providers = getattr(llm, "_providers", None)
    if not isinstance(providers, (list, tuple)):
        providers = [llm]
    return describe_chain([n for n in (_provider_name(p) for p in providers) if n], catalog_model)


def start_status_feeds(
    node: Any, string_cls: Any, llm: Any, wake_words: Iterable[Any]
) -> tuple[StatusFeed, StatusFeed]:
    """Создать latched-паблишеры и сразу отдать текущие значения."""
    return _start_feeds(node, string_cls, describe_llm(llm), describe_wake(wake_words))


__all__ = ["LLM_STATUS_SOURCE", "StatusFeed", "describe_llm", "describe_wake", "start_status_feeds"]
