"""voice_state bridge: состояние голоса → WS-фрейм (0x1202).

Источник состояния — ``rob_box_core.voice_state`` (issue #3911): там таблицы v2-контрактов
(``TurnEvent``/``SpeechEvent``/``SessionState``) и адаптер строки v1; здесь только отправка итога в шлем.
Словарь ``state`` — ``docs/architecture/meta-quest-api.md §4`` (``idle | listening | thinking | speaking``),
``detail="silenced"`` — отдельный визуальный режим «mute».

Функция, а не метод ``QuestNode``: бюджет размера класса (ADR-0145) не даёт расти ноде.
"""

from __future__ import annotations

import time
from typing import Any

from ..protocol.topics import encode_voice_state


def send_voice_state(node: Any, state: Any) -> None:
    """Итог ``VoiceStateTracker`` → кэш ``node._last_voice_state`` + msgpack voice_state (0x1202) → WS.

    Шлём каждый переход (event-driven, drop-newest — meta-quest-api.md §4 frequency policy); повтор того же
    состояния безопасен — клиент дедуплицирует по ts_ms+state. Падение моста не валит подписку.
    """
    node._last_voice_state = {"state": state.state, "detail": state.detail}
    payload = encode_voice_state(state=state.state, ts_ms=int(time.time() * 1000), detail=state.detail)
    try:
        node.bridge.publish_frame("voice_state", payload)
    except Exception as e:  # noqa: BLE001
        node.get_logger().debug(f"voice_state publish failed: {e}")


__all__ = ["send_voice_state"]
