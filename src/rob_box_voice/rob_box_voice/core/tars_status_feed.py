"""Статус LLM и wake-слов для экрана ТАРС 1 (issue #3253, Ш3б).

Топиков «какая LLM» и «какие wake-слова» в графе не было: модель задаётся
параметрами dialogue_node, wake-слова читаются из yaml. Здесь они
превращаются в два latched-топика (TRANSIENT_LOCAL, depth 1), чтобы поздний
подписчик (quest_node → шлем) получил значение:

* ``/voice/llm_status``  — ``{"provider", "model"?, "chain", "source"}``
* ``/voice/wake_words``  — ``{"words": [...]}``

Правило ADR-0018: ничего не выдумываем. ``HealthAwareFallbackLLM`` не
запоминает, какой провайдер ответил последним, поэтому отдаём НАСТРОЕННЫЙ
primary (``source="configured"``); модель — только если известна из
каталога провайдеров, иначе ключа ``model`` нет.
"""

from __future__ import annotations

import json
from typing import Any, Callable, Iterable, Optional

LLM_STATUS_SOURCE = "configured"


def _provider_name(provider: Any) -> Optional[str]:
    name = getattr(provider, "name", None)
    if isinstance(name, str) and name.strip():
        return name.strip().lower()
    return None


def _catalog_model(name: str) -> Optional[str]:
    """Модель, с которой ``build_provider(name)`` собирает провайдера."""
    try:
        from rob_box_harness.providers.catalog import LLM_PROVIDER_REGISTRY
    except ImportError:
        return None
    model = (LLM_PROVIDER_REGISTRY.get(name) or {}).get("default_model")
    return model if isinstance(model, str) and model else None


def describe_llm(llm: Any) -> Optional[dict[str, Any]]:
    """Настроенная LLM-цепочка → payload или None, если имя неизвестно."""
    providers = getattr(llm, "_providers", None)
    if not isinstance(providers, (list, tuple)):
        providers = [llm]
    chain = [n for n in (_provider_name(p) for p in providers) if n]
    if not chain:
        return None
    payload: dict[str, Any] = {
        "provider": chain[0],
        "chain": chain,
        "source": LLM_STATUS_SOURCE,
    }
    model = _catalog_model(chain[0])
    if model:
        payload["model"] = model
    return payload


def describe_wake(words: Iterable[Any]) -> Optional[dict[str, Any]]:
    """Активные wake-слова → payload или None, если список пуст."""
    cleaned = [w.strip() for w in words if isinstance(w, str) and w.strip()]
    return {"words": cleaned} if cleaned else None


class StatusFeed:
    """Публикует JSON в std_msgs/String, молча пропуская повтор."""

    def __init__(self, publisher: Any, make_msg: Callable[[str], Any]) -> None:
        self._pub = publisher
        self._make_msg = make_msg
        self._last: Optional[str] = None

    def publish(self, payload: Optional[dict[str, Any]]) -> bool:
        """True — опубликовали; False — нечего или без изменений."""
        if not payload:
            return False
        data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        if data == self._last:
            return False
        self._pub.publish(self._make_msg(data))
        self._last = data
        return True


def start_status_feeds(
    node: Any, string_cls: Any, llm: Any, wake_words: Iterable[Any]
) -> tuple[StatusFeed, StatusFeed]:
    """Создать latched-паблишеры и сразу отдать текущие значения."""
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def make_msg(data: str) -> Any:
        msg = string_cls()
        msg.data = data
        return msg

    llm_feed = StatusFeed(
        node.create_publisher(string_cls, "/voice/llm_status", qos), make_msg
    )
    wake_feed = StatusFeed(
        node.create_publisher(string_cls, "/voice/wake_words", qos), make_msg
    )
    llm_feed.publish(describe_llm(llm))
    wake_feed.publish(describe_wake(wake_words))
    return llm_feed, wake_feed
