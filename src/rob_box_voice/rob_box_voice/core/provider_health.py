"""Состояния провайдеров TTS и STT для ``/voice/providers/state`` (ADR-0150 §8, PR-4 ч.2, #3693).

Функции без ROS, узлы отдают себя как duck-тип: ``tts_node`` - ``provider_chain``, ``_provider_is_dead``,
``_provider_dead_class``; ``stt_node`` - ``provider_chain``, ``_provider_dead_cache``. Класс отказа - не подстрока
ответа вендора, а тип исключения адаптера (``rob_box_llm.errors``, ``stt_fallback``).
"""

from __future__ import annotations

import time
from typing import Any, List, Tuple

from rob_box_dialog.degrade import ProviderState
from rob_box_llm.errors import TTSAuthError, TTSConnectionError, TTSRateLimitError, TTSTimeoutError


def tts_failure_class(error: BaseException, permanent: bool) -> str:
    """Класс отказа синтеза: quota (429 MiniMax) / auth / network / timeout / down.

    ``permanent`` - решение ``_mark_provider_dead`` (ключ, права на папку Yandex): долгий TTL = ключ не подошёл.
    """
    if isinstance(error, TTSRateLimitError):
        return "quota"
    if isinstance(error, TTSAuthError) or permanent:
        return "auth"
    if isinstance(error, TTSConnectionError):
        return "network"
    if isinstance(error, TTSTimeoutError):
        return "timeout"
    return "down"


def record_tts_death(node: Any, name: str, error: BaseException, permanent: bool) -> None:
    """Запомнить класс и время отказа (читает :func:`tts_provider_states`)."""
    node.__dict__.setdefault("_provider_dead_class", {})[name] = (tts_failure_class(error, permanent), time.time())


def tts_provider_states(node: Any) -> List[ProviderState]:
    """Все провайдеры цепочки TTS: мёртвые - с классом отказа, остальные ``ok``."""
    classes = getattr(node, "_provider_dead_class", {})
    out: List[ProviderState] = []
    for name in getattr(node, "provider_chain", ()):
        if not node._provider_is_dead(name):  # noqa: SLF001 - узел сам владеет кэшем мёртвых
            out.append(ProviderState(name, "tts", "ok"))
            continue
        cls: Tuple[str, float] = classes.get(name) or (_shared_class(node, name), 0.0)
        out.append(ProviderState(name, "tts", cls[0], cls[1]))
    return out


def _shared_class(node: Any, name: str) -> str:
    """Отказ увидела LLM-сторона (общий тариф MiniMax, issue #2702): класс - из общего health-кэша."""
    shared = getattr(node, "_shared_health_cache", None)
    return shared.get(name).health_class if shared is not None else "down"


def stt_provider_states(node: Any) -> List[ProviderState]:
    """Все провайдеры цепочки STT по кэшу «мёртвых» (класс - ``ProviderDeadCache.health_class``)."""
    cache = getattr(node, "_provider_dead_cache", None)
    if cache is None:
        return []
    return [ProviderState(name, "stt", cache.health_class(name)) for name in getattr(node, "provider_chain", ())]
