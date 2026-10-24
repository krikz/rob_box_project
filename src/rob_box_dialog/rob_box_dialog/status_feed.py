"""Статус LLM и wake-слов для экрана ТАРС 1 (issue #3253, Ш3б): перенос v1 ``core/tars_status_feed.py`` (#3912).

Два latched-топика (их публикует хост: ``dialog_node`` v2 или ``dialogue_node`` v1), чтобы поздний подписчик
(quest_node → шлем) получил значение:

* ``/voice/llm_status``  — ``{"provider", "model"?, "chain", "source"}``
* ``/voice/wake_words``  — ``{"words": [...]}``

Правило ADR-0018: ничего не выдумываем. Отдаём НАСТРОЕННУЮ цепочку (``source="configured"``), а не «кто ответил
последним»; модель — только если её назвал каталог провайдеров хоста (``model_of``), иначе ключа ``model`` нет.
"""

from __future__ import annotations

import json
from typing import Any, Callable, Iterable, Optional

LLM_STATUS_SOURCE = "configured"
LLM_STATUS_TOPIC = "/voice/llm_status"
WAKE_WORDS_TOPIC = "/voice/wake_words"


def describe_chain(names: Iterable[Any], model_of: Callable[[str], Optional[str]] = lambda _name: None,
                   ) -> Optional[dict[str, Any]]:
    """Имена провайдеров цепочки (первый — основной) → payload или ``None``, если имён нет."""
    chain = [n.strip().lower() for n in names if isinstance(n, str) and n.strip()]
    if not chain:
        return None
    payload: dict[str, Any] = {"provider": chain[0], "chain": chain, "source": LLM_STATUS_SOURCE}
    model = model_of(chain[0])
    if isinstance(model, str) and model:
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


__all__ = ["LLM_STATUS_SOURCE", "LLM_STATUS_TOPIC", "StatusFeed", "WAKE_WORDS_TOPIC", "describe_chain",
           "describe_wake"]
