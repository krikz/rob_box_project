"""Стадии ТАРС-в-шлем со стороны супервизора (issue #3253 Ш2, ADR-0078 §3.6).

Супервизор знает две стадии хода оператора, которых больше никто не видит:

* ``thinking`` — ход LLM начался (``AvatarSupervisor._on_avatar_command``
  прямо перед ``_run_agent_sync``);
* ``idle`` — ход закончился БЕЗ реплики в шлем (агент выключен/недоступен,
  битый вход, текстовый источник, пустой ответ, ``speak_agent_replies=false``).

Если реплика ушла в ``/avatar/tts/request``, ``speaking``/``idle`` выводит
quest_node по чанкам и ``/voice/tts/finished`` — супервизор их не шлёт.

Вынесено из ``AvatarSupervisor`` отдельным классом: класс-бюджет ADR-0145
запрещает растить ``AvatarSupervisor`` приватными хелперами.

Контракт топика ``/avatar/tars/stage`` (константа ``AVATAR_TARS_STAGE_TOPIC``
в ``supervisor_node``; std_msgs/String JSON):
``{"stage": "thinking"|"idle", "request_id": str, "reason"?: str, "ts_ms": int}``.
Потребитель — ``rob_box_quest.quest_node`` → WS ``tars_state``.
"""

from __future__ import annotations

import json
import time
from typing import Any, Callable


class TarsStagePublisher:
    """Обёртка над ROS-паблишером ``/avatar/tars/stage``.

    Fire-and-forget: экран шлема — индикация, ход агента из-за сбоя
    публикации падать не должен (ошибка уходит в warning-лог).
    """

    def __init__(self, publisher: Any, msg_factory: Callable[[], Any], log: Any) -> None:
        self._pub = publisher
        self._msg_factory = msg_factory
        self._log = log

    def publish(self, stage: str, request_id: str, reason: str = "") -> None:
        body: dict[str, Any] = {
            "stage": stage,
            "request_id": request_id,
            "ts_ms": int(time.time() * 1000),
        }
        if reason:
            body["reason"] = reason
        try:
            msg = self._msg_factory()
            msg.data = json.dumps(body, ensure_ascii=False)
            self._pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"avatar_supervisor: tars_stage publish failed: {exc}")

    def thinking(self, request_id: str) -> None:
        """Ход LLM начался."""
        self.publish("thinking", request_id)

    def idle(self, request_id: str, reason: str) -> None:
        """Ход закончился без реплики в шлем."""
        self.publish("idle", request_id, reason)

    def finish_turn(self, request_id: str, spoke: bool) -> None:
        """Конец хода: реплика ушла в шлем — стадии ведёт quest_node; нет — idle."""
        if not spoke:
            self.idle(request_id, "no_speech")
