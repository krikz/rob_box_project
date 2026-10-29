"""expression_client - единственный клиент экспрессии и событий хода (ADR-0150 §5.5.2, PR-3b #3690).

Решение «что показать и сыграть» принимает агент-владелец хода (``rob_box_dialog.expression``); сюда оно приходит
готовым. ``express()`` публикует ``ExpressionRequest`` в ``/voice/expression/request`` - его слушают исполнители лица
(``face_node``) и earcon'а (``sound_node``). Кольцо LED (``led_node``) берёт режим по стадии из ``turn_event()`` -
``/dialog/turn_event``. Каждый исполнитель отвечает событием, ``watch_events()`` их принимает.

Один публикатор каждого вида на ноду (кэш в ``node.__dict__``), ``rob_box_dialog_msgs`` импортируется лениво: модуль
живёт и в образах без сообщений диалога, пока они не зовут клиент.
"""

from __future__ import annotations

import json
from typing import Any, Callable, Dict, Mapping, Optional

from rob_box_core.speech_client import SPEECH_AGENTS

EXPRESSION_REQUEST_TOPIC = "/voice/expression/request"
EXPRESSION_EVENT_TOPIC = "/voice/expression/event"
SOUND_EVENT_TOPIC = "/voice/sound/event"
TURN_EVENT_TOPIC = "/dialog/turn_event"

#: Статусы ``TurnEvent.status``: ``skipped`` - стадия не состоялась (кольцо и лицо её не показывают);
#: ``cancelled`` - ход перебит стопом или устарел по эпохе (ADR-0150 §6.2, PR-9).
TURN_STATUSES = ("started", "finished", "failed", "skipped", "cancelled")
#: Потолок ``ExpressionRequest.ttl_ms`` (uint32), мс; исполнители режут свой предел сами.
TTL_MAX_MS = 600_000

_QOS_DEPTH = 10
_PUBS_ATTR = "_expression_client_pubs"


def _msgs() -> Any:
    from rob_box_dialog_msgs import msg  # noqa: PLC0415 - см. докстринг модуля

    return msg


def _publisher(node: Any, kind: str) -> Any:
    if kind not in ("request", "turn_event"):
        raise ValueError(f"publisher kind {kind!r} не из ('request', 'turn_event')")
    pubs: Dict[str, Any] = node.__dict__.setdefault(_PUBS_ATTR, {})
    pub = pubs.get(kind)
    if pub is None:
        msg = _msgs()
        if kind == "request":
            pub = node.create_publisher(msg.ExpressionRequest, EXPRESSION_REQUEST_TOPIC, _QOS_DEPTH)
        else:
            pub = node.create_publisher(msg.TurnEvent, TURN_EVENT_TOPIC, _QOS_DEPTH)
        pubs[kind] = pub
    return pub


def prepare(node: Any, *kinds: str) -> None:
    """Создать публикаторы заранее (в ``__init__`` ноды): первое сообщение после старта не должно теряться."""
    for kind in kinds or ("request", "turn_event"):
        _publisher(node, kind)


def _check_agent(agent: str) -> None:
    if agent not in SPEECH_AGENTS:
        raise ValueError(f"expression agent {agent!r} не из {SPEECH_AGENTS}")


def build_request(
    node: Any,
    *,
    turn_id: str,
    epoch: int,
    agent: str,
    emotion: Optional[str] = None,
    gesture: Optional[str] = None,
    earcon: Optional[str] = None,
    face: Optional[Mapping[str, Any]] = None,
    ttl_ms: int = 0,
) -> Any:
    """``ExpressionRequest`` из полей плана; ``None`` - «не менять» (пустая строка в сообщении)."""
    _check_agent(agent)
    req = _msgs().ExpressionRequest()
    req.turn_id = turn_id or ""
    req.epoch = max(0, int(epoch))
    req.agent = agent
    req.emotion = emotion or ""
    req.gesture = gesture or ""
    req.earcon = earcon or ""
    req.ttl_ms = min(max(0, int(ttl_ms)), TTL_MAX_MS)
    req.face_json = json.dumps(dict(face), ensure_ascii=False, sort_keys=True) if face else ""
    return req


def express(node: Any, *, turn_id: str, epoch: int, agent: str, emotion: Optional[str] = None,
            gesture: Optional[str] = None, earcon: Optional[str] = None, face: Optional[Mapping[str, Any]] = None,
            ttl_ms: int = 0) -> Any:
    """Попросить исполнителей показать и сыграть; вернуть отправленный ``ExpressionRequest``."""
    req = build_request(node, turn_id=turn_id, epoch=epoch, agent=agent, emotion=emotion, gesture=gesture,
                        earcon=earcon, face=face, ttl_ms=ttl_ms)
    _publisher(node, "request").publish(req)
    return req


def turn_event(node: Any, *, turn_id: str, epoch: int, agent: str, stage: str, status: str, reason: str = "",
               llm_calls: int = 0, llm_provider: str = "", latency_ms: int = 0, source: str = "") -> Any:
    """Событие стадии хода в ``/dialog/turn_event``: рефлекс кольца, метрики хода, журнал e2e."""
    _check_agent(agent)
    if status not in TURN_STATUSES:
        raise ValueError(f"turn status {status!r} не из {TURN_STATUSES}")
    ev = _msgs().TurnEvent()
    ev.turn_id, ev.epoch, ev.agent = turn_id or "", max(0, int(epoch)), agent
    ev.stage, ev.status, ev.reason = stage, status, reason
    ev.llm_calls, ev.llm_provider = min(max(0, int(llm_calls)), 255), llm_provider
    ev.latency_ms, ev.source = max(0, int(latency_ms)), source
    ev.stamp = node.get_clock().now().to_msg()
    _publisher(node, "turn_event").publish(ev)
    return ev


def watch_events(node: Any, on_expression: Optional[Callable[[Any], None]] = None,
                 on_sound: Optional[Callable[[Any], None]] = None) -> None:
    """Подписаться на ответы исполнителей: ``ExpressionEvent`` (лицо, кольцо, звук) и ``SoundEvent``."""
    msg = _msgs()
    if on_expression is not None:
        node.create_subscription(msg.ExpressionEvent, EXPRESSION_EVENT_TOPIC, on_expression, _QOS_DEPTH)
    if on_sound is not None:
        node.create_subscription(msg.SoundEvent, SOUND_EVENT_TOPIC, on_sound, _QOS_DEPTH)


__all__ = [
    "EXPRESSION_EVENT_TOPIC",
    "EXPRESSION_REQUEST_TOPIC",
    "SOUND_EVENT_TOPIC",
    "TTL_MAX_MS",
    "TURN_EVENT_TOPIC",
    "TURN_STATUSES",
    "build_request",
    "express",
    "prepare",
    "turn_event",
    "watch_events",
]
