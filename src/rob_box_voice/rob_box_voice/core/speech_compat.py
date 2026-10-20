"""speech_compat - адаптер совместимости речи v1 (ADR-0150 §6.1, §10.2, PR-3 #3688).

Единственное место, где ``tts_node`` ещё принимает старые входы речи:

* JSON ``/voice/tts/request`` - внешние писатели вне ``src/`` (сценарный
  раннер E2E, ``tools/``, ``scripts/``; ``seam_allowlist.json`` с датой);
* JSON ``/voice/dialogue/response`` - ответы ``dialogue_node`` v1 с
  ``batch_*`` и ``tg_chat_id`` (Telegram-эхо читает тот же топик) до PR-8/PR-12a;
* строка ``STOP`` в ``/voice/tts/control`` и в avatar-control - старая отмена
  «всего, что звучит».

Старый запрос получает метаданные реплики (агент, эпоха) так, как их дал бы
``SpeechRequest``, и идёт тем же конвейером, с теми же ``SpeechEvent``.
Окна игнора STOP (#1563) больше нет: защиту подтверждения новой сессии от
barge-in STOP теперь даёт эпоха (``speech_client.cancel(epoch=...)``).

``/voice/tts/state`` и ``/voice/tts/finished`` публикуются из тех же переходов,
что и ``SpeechEvent`` (одна воронка ``tts_node._publish_tts_finished``), для
``face_node``/``stt_node``/``dialogue_node`` до PR-3b/PR-8.

Удаляется целиком, когда у старых топиков не останется писателей.
"""

from __future__ import annotations

from typing import Any, Dict

from rob_box_core.speech_client import SPEECH_AGENTS
from std_msgs.msg import String


def meta_from_legacy(chunk: Dict[str, Any], meta_cls: Any) -> Any:
    """Метаданные реплики из старого JSON: ``agent``/``epoch``, если писатель их дал."""
    agent = chunk.get("agent")
    if agent not in SPEECH_AGENTS:
        agent = "operator" if chunk.get("priority") == "operator" else "personality"
    try:
        epoch = max(0, int(chunk.get("epoch") or 0))
    except (TypeError, ValueError):
        epoch = 0
    return meta_cls(agent=agent, epoch=epoch)


def on_control(node: Any, msg: Any) -> None:
    """Старый ``/voice/tts/control``: ``STOP`` - отменить и заглушить всё."""
    command = (msg.data or "").strip().upper()
    if command != "STOP":
        node.get_logger().warn(f"⚠️ [speech_compat] команда {msg.data!r} не поддерживается")
        return
    owner = getattr(node, "_speech", None)
    if owner is None:
        node._interrupt_playback()
    else:
        owner.cancel_all("legacy_stop")
    node.publish_state("stopped")


def attach_legacy(node: Any, *, avatar_control_topic: str) -> None:
    """Подписки ``tts_node`` на старые входы речи."""
    node.create_subscription(String, "/voice/dialogue/response", node.dialogue_callback, 10)
    node.create_subscription(String, "/voice/tts/request", node.dialogue_callback, 10)
    node.create_subscription(String, "/voice/tts/control", lambda m: on_control(node, m), 10)
    node.create_subscription(String, avatar_control_topic, lambda m: on_control(node, m), 10)


__all__ = [
    "attach_legacy",
    "meta_from_legacy",
    "on_control",
]
