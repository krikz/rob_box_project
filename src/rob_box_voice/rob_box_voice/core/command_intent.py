"""Команда движения/статуса — от диалога к command_node через ``/dialog/intent`` (ADR-0150 PR-5 #3694).

Раньше command_node сам читал каждую реплику (``/voice/stt/utterance``) и разбирал её: движение шло мимо
адресации, а «Робот, хватит» давало два ответа — «Хорошо, молчу.» диалога и «Останавливаюсь» command_node
(A16). Теперь разбирает диалог после допуска (вейк, тишина, Tier-1), а command_node получает уже решённое
намерение ``Intent{turn_id, epoch, kind, args_json}`` и только исполняет. Без ROS: сообщение приходит снаружи.
"""

from __future__ import annotations

import json
from typing import Any, Optional

from rob_box_voice.core.command_parser import Command, IntentType

#: ``Intent.kind`` по намерению разборщика; ``stop_motion``/``move`` — имена ADR-0150 §10.1.
KIND_BY_INTENT = {
    IntentType.STOP: "stop_motion",
    IntentType.NAVIGATE: "move",
    IntentType.STATUS: "status",
    IntentType.MAP: "map",
    IntentType.VISION: "vision",
    IntentType.FOLLOW: "follow",
}
_INTENT_BY_KIND = {kind: intent for intent, kind in KIND_BY_INTENT.items()}


def fill_intent(msg: Any, command: Command, *, epoch: int, turn_id: str = "") -> Any:
    """Записать разобранную команду в ``Intent``; текст фразы едет в ``args_json`` для лога исполнителя."""
    msg.turn_id = turn_id or ""
    msg.epoch = max(0, int(epoch))
    msg.kind = KIND_BY_INTENT[command.intent]
    msg.args_json = json.dumps({"entities": command.entities, "text": command.text}, ensure_ascii=False)
    return msg


def command_from_intent(msg: Any) -> Optional[Command]:
    """``Intent`` → ``Command`` для исполнителя; неизвестный вид или битый JSON — ``None``."""
    intent = _INTENT_BY_KIND.get(getattr(msg, "kind", ""))
    if intent is None:
        return None
    try:
        args = json.loads(getattr(msg, "args_json", "") or "{}")
    except (TypeError, ValueError):
        return None
    if not isinstance(args, dict):
        return None
    entities = args.get("entities") if isinstance(args.get("entities"), dict) else {}
    return Command(intent=intent, text=str(args.get("text") or ""), entities=entities, confidence=1.0)
