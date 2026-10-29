"""События плана ``/dialog/plan_event`` (ADR-0150 §5.6.1, §10.1 ``PlanEvent.msg``; BML ``blockProgress``/``predictionFeedback``).

Чистые данные и фразы; публикует хост (поле ``kind`` → константа ``KIND_*`` сообщения по :data:`PLAN_EVENT_KINDS`).
Вместо обещания «в очереди» от планировщика v1 действие, которое ещё не началось, получает событие ``queued{after,
eta_ms}``: из него код строит фразу «Поставлю после …» (:func:`queued_phrase`) и данные для LLM «в очереди после X,
старт ≈ через N с» (:func:`pending_data`). Начатое и законченное — ``block_start``/``block_end`` по событию владельца.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.plan.composition import Decision
from rob_box_dialog.plan.resources import ResourceState

#: Виды события в порядке констант ``KIND_*`` ``rob_box_dialog_msgs/msg/PlanEvent.msg`` (держит тест).
PLAN_EVENT_KINDS: Tuple[str, ...] = (
    "block_start", "block_end", "segment_started", "segment_finished", "segment_replaced", "queued", "prediction",
    "rejected",
)


@dataclass(frozen=True)
class PlanEvent:
    """Событие плана хода; ``after_block`` и ``eta_ms`` — у ``queued``/``prediction``, ``reason`` — у ``rejected``.

    ``after_resource`` — столбец матрицы, после которого (для фразы и LLM); в ``PlanEvent.msg`` его нет — фразу
    строит агент, который это событие и создал.

    ``spoken_chars``/``text_len`` (``segment_finished``: сколько символов сегмента прозвучало из скольких; A20) и
    ``offset_ms`` (``segment_started``: на сколько мс старт сегмента отстоит от ближайшей доли подложки, со знаком; A19,
    ``None`` — сетки нет) тоже не едут в ``PlanEvent.msg``: их читает только строка лога (:func:`event_line`)."""

    kind: str
    block_id: str
    turn_id: str = ""
    epoch: int = 0
    agent: str = kn.AGENTS[0]
    segment_no: int = 0
    after_block: str = ""
    after_resource: str = ""
    eta_ms: int = 0
    synced: bool = False
    reason: str = ""
    spoken_chars: int = 0
    text_len: int = 0
    offset_ms: Optional[int] = None

    def __post_init__(self) -> None:
        if self.kind not in PLAN_EVENT_KINDS:
            raise ValueError(f"PlanEvent.kind={self.kind!r}: не из {PLAN_EVENT_KINDS}")

    @property
    def code(self) -> int:
        """Константа ``KIND_*`` сообщения."""
        return PLAN_EVENT_KINDS.index(self.kind)


def event_line(event: PlanEvent) -> str:
    """Строка лога события (после метки ``🗓️ [plan]``): всё, что нужно приёмке по ``docker logs`` — вид, блок,
    номер сегмента, ``synced`` (A20/A22), прозвучало/длина (A20), ETA, смещение старта от доли (A19), причина."""
    offset = "-" if event.offset_ms is None else str(event.offset_ms)
    return (f"PlanEvent {event.kind} block={event.block_id[:12]} seg={event.segment_no} "
            f"synced={str(event.synced).lower()} spoken_chars={event.spoken_chars} text_len={event.text_len} "
            f"offset_ms={offset} after={event.after_resource or '-'}/{event.after_block[:8] or '-'} "
            f"eta_ms={event.eta_ms} reason={event.reason or '-'}")


def queued(block_id: str, decision: Decision, state: ResourceState, **ids: Any) -> Optional[PlanEvent]:
    """``queued`` для действия, которое ждёт занятых ресурсов (``decision.waits_for``); ждать нечего — ``None``.

    «После чего» — ресурс, который освободится последним (ETA неизвестна — он и есть последний); ``eta_ms`` — его
    оценка (0 — неизвестно). ``ids`` — ``turn_id``/``epoch``/``agent``."""
    busy = state.busy()
    waits = [col for col in decision.waits_for if col in busy]
    if not waits:
        return None
    etas = {col: state.eta_s(col) for col in waits}
    last = max(waits, key=lambda col: float("inf") if etas[col] is None else etas[col])
    eta = etas[last]
    return PlanEvent("queued", block_id, after_block=busy[last].block_id, after_resource=last,
                     eta_ms=0 if eta is None else int(eta * 1000), **ids)


def rejected(block_id: str, decision: Decision, **ids: Any) -> PlanEvent:
    """``rejected`` по клетке REJECT: причина — ключ вопроса (``perform.ask_replace_music``) или ``busy``."""
    return PlanEvent("rejected", block_id, reason=decision.ask or "busy", **ids)


def queued_phrase(event: PlanEvent, new_kind: str) -> str:
    """«Поставлю после того, как договорю — примерно через 8 с.»: глагол — по строке матрицы нового действия,
    «после чего» — по ресурсу ``after_resource``; ETA называется от ``PLAN_SAY_ETA_MIN_S``."""
    verb = phrases.text(f"plan.verb.{new_kind}")
    after = phrases.text(f"plan.after.{event.after_resource}")
    eta_s = round(event.eta_ms / 1000)
    if eta_s >= kn.PLAN_SAY_ETA_MIN_S:
        return phrases.text("plan.queued.eta", verb=verb, after=after, eta=eta_s)
    return phrases.text("plan.queued", verb=verb, after=after)


def pending_data(event: PlanEvent) -> Dict[str, Any]:
    """Структурный блок для LLM о не начатом действии (BML ``predictionFeedback``) — вместо обещания «в очереди»."""
    data: Dict[str, Any] = {"status": "pending", "after": event.after_resource, "after_block": event.after_block}
    if event.eta_ms:
        data["start_in_s"] = round(event.eta_ms / 1000, 1)
    return data


__all__ = ["PLAN_EVENT_KINDS", "PlanEvent", "event_line", "pending_data", "queued", "queued_phrase", "rejected"]
