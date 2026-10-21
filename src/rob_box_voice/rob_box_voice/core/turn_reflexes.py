"""Ход диалога и выход рефлексов: ``TurnEvent`` + один ``ExpressionRequest`` на стадию (ADR-0150 §5.5.3, PR-3b #3690).

:class:`Turn` - состояние одного хода личности; :class:`ReflexOut` публикует стадию: событие хода (его читает кольцо LED,
метрики, e2e) и план экспрессии лицу/звуку из ``rob_box_dialog.expression``. Сбой публикации диалог не роняет - он в логе.
Без ROS: публикаторы, часы и эпоха приходят снаружи (:mod:`.turn_stages`).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Callable, Dict, Optional

from rob_box_dialog import expression as ex

#: Источник реплики, которому положен звук «услышал» (Telegram и оператор не слышат earcon у робота).
HEARD_SOURCE = "respeaker"
#: Ранний сигнал и допуск без общего id фразы считаются одной фразой, пока ход не ушёл дальше ``addressed``.
SAME_PHRASE_S = 3.0


@dataclass
class Turn:
    """Один ход личности: id, стадия, что обещано (речь), что накопилось (эмоция, ошибки тулов)."""

    turn_id: str
    uid: Optional[str]
    source: str
    began: float
    no: int
    stage: str = ""
    cls: Optional[str] = None
    terminal: bool = False
    emotion: str = ""
    gesture: str = ""
    llm_calls: int = 0
    llm_provider: str = ""
    expects_speech: bool = False
    speech_started: bool = False
    body_done_at: Optional[float] = None
    settle_until: float = 0.0
    failure: str = ""
    errors: Dict[str, str] = field(default_factory=dict)

    def same_phrase(self, uid: Optional[str], source: str, now: float) -> bool:
        """Тот же ли это фраза: по id; без id - тот же источник, пока ход не ушёл дальше ``addressed`` (``SAME_PHRASE_S``)."""
        if uid and self.uid:
            return self.uid == uid
        return not self.terminal and self.stage == "addressed" and self.source == source and now - self.began < SAME_PHRASE_S


class ReflexOut:
    """Стадия хода -> ``TurnEvent`` и ``ExpressionRequest`` по рефлексу; пропущенная публикация видна в логе."""

    def __init__(self, express: Callable[..., Any], turn_event: Callable[..., Any], agent: str,
                 epoch: Callable[[], int], clock: Callable[[], float], log: Callable[[str], None]) -> None:
        self._express, self._turn_event, self._agent = express, turn_event, agent
        self._epoch, self._clock, self._log = epoch, clock, log

    def _emit(self, turn: Turn, stage: str, status: str, reason: str) -> None:
        try:
            self._turn_event(
                turn_id=turn.turn_id, epoch=self._epoch(), agent=self._agent, stage=stage, status=status,
                reason=reason, llm_calls=turn.llm_calls, llm_provider=turn.llm_provider,
                latency_ms=int((self._clock() - turn.began) * 1000), source=turn.source)
        except Exception as exc:  # noqa: BLE001 - рефлекс не должен ронять диалог
            self._log(f"[reflex] TurnEvent {stage}/{status} не ушёл: {exc!r}")

    def send(self, turn: Turn, plan: Optional[ex.ExpressionPlan]) -> None:
        """Отдать исполнителям готовый план (рефлекс, ``play_animation``, успокоение лица)."""
        if plan is None:
            return
        try:
            self._express(turn_id=turn.turn_id, epoch=self._epoch(), agent=self._agent, emotion=plan.emotion,
                          gesture=plan.gesture, earcon=plan.earcon, ttl_ms=plan.ttl_ms)
        except Exception as exc:  # noqa: BLE001
            self._log(f"[reflex] ExpressionRequest не ушёл: {exc!r}")

    def enter(self, turn: Turn, stage: str, snapshot: Optional[Dict[str, Any]] = None, status: str = "started",
              reason: str = "") -> None:
        """Войти в стадию: событие хода + рефлекс, окрашенный эмоцией команды (на ``failed`` рефлекс побеждает)."""
        if stage == turn.stage and stage != "executing":
            return
        turn.stage = stage
        self._emit(turn, stage, status, reason)
        plan = ex.reflex(stage, turn.cls if stage == "executing" else None, {"turn_no": turn.no, **(snapshot or {})})
        plan = ex.merge(plan, stage, turn.emotion, turn.gesture)
        if plan is not None and stage == "addressed" and turn.source != HEARD_SOURCE:
            plan = replace(plan, earcon="")
        self.send(turn, plan)

    def answer(self, turn: Optional[Turn], executor: str, turn_id: str, started: bool, reason: str) -> str:
        """Ответ исполнителя рефлекса: пропуск виден в логе (молчаливых пропусков нет); вернуть строку лога."""
        age = f"+{(self._clock() - turn.began) * 1000:.0f}мс" if turn is not None and turn.turn_id == turn_id else "?"
        line = f"[reflex] executor={executor} turn={turn_id[:8] or '-'} " + (
            f"started {age}" if started else f"skipped reason={reason}")
        self._log(line)
        return line

    def close(self, turn: Turn, status: str, reason: str) -> None:
        """Закрыть ход без рефлекса (отменён, перебит новой фразой, фраза не для робота): событие ``skipped``."""
        turn.terminal = True
        self._emit(turn, turn.stage or "addressed", status, reason)
