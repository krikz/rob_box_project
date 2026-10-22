"""Исполнитель тулов хода v1 под планировщиком ресурсов (ADR-0150 §5.6, PR-7b #3699) — вместо ``SchedulerToolExecutor``.

Каждый вызов тула (свободный цикл ``AgentCore`` и режим команд ``command_turn``) сначала проходит допуск
:meth:`rob_box_dialog.plan.runner.PlanRunner.admit` по таблице ``knowledge.COMPOSITION``: совместимое — сразу,
APPEND/MERGE на границе — после события конца занятого блока от владельца (с потолком), REJECT — отказ. Модель видит
настоящий исход: ответ владельца, ``rejected``/``timeout`` плана с причиной; обещания «в очереди» без события
(V28) больше нет. Каналов VOICE/MUSIC/ANIM нет: порядок реплик держит владелец речи (tts_node), звуков —
sound_node.

Временно остаётся здесь же (уходит в PR-7c): «один трек за ход» (#2859, ``TrackStartGuard``) и речь о музыке хода
(``MusicTurn``, ADR-0148).
"""

from __future__ import annotations

import json
import logging
from typing import Any, Callable, List, Mapping, Optional, Sequence

from rob_box_dialog import knowledge as kn
from rob_box_dialog.plan.composition import compose, kind_of
from rob_box_dialog.plan.plan_events import PlanEvent
from rob_box_dialog.plan.runner import PlanRunner
from rob_box_llm.provider import ToolCall, ToolResult

from rob_box_voice.core.music_turn import MusicTurn, speech_refusal_content
from rob_box_voice.core.track_start_guard import TrackStartGuard, refusal_content

_LOG = logging.getLogger(__name__)
SPEAK_TOOL = kn.SPEECH_CHANNEL_TOOLS[0]
#: Тул вне таблицы классов (служебный, транспорт) идёт без допуска: ресурсов он не занимает.
_NO_PLAN = "query"


def _plan_kind(tool: str) -> str:
    return kind_of(tool) if tool in kn.TOOL_CLASSES or tool in kn.COMPOSITION_KIND_BY_TOOL else _NO_PLAN


def _plan_refusal(call: ToolCall, status: str, reason: str, event: Optional[PlanEvent]) -> ToolResult:
    """Действие не исполнено планом: честный исход для модели и для исполнителя команды (``data.status``)."""
    data = {"status": status, "reason": reason, "tool": call.name}
    if event is not None and event.after_resource:
        data["after"] = event.after_resource
    text = {"timeout": f"Не исполнено: {data.get('after', 'занятый ресурс')} не освободился вовремя.",
            "rejected": "Не исполнено: ресурс занят и это действие его не вытесняет."}[status]
    return ToolResult(tool_call_id=call.id, content=json.dumps({**data, "message": text}, ensure_ascii=False),
                      is_error=True, data=data)


class PlanToolExecutor:
    """Порт ``discover/execute/aclose`` провайдера тулов ``AgentCore`` с допуском по плану."""

    name = "plan"

    def __init__(self, underlying: Any, planner: Optional[PlanRunner] = None, *,
                 on_tool: Optional[Callable[[str, Mapping[str, Any], Optional[ToolResult]], None]] = None) -> None:
        self._underlying = underlying
        self.planner = planner or PlanRunner()
        # ADR-0150 PR-3b (#3690) — стадия хода «executing»: наблюдатель до и после вызова; его ошибка ход не ломает.
        self._on_tool = on_tool
        self._track_guard = TrackStartGuard()  # issue #2859 — один запуск трека за ход (уходит в PR-7c)
        self.music_turn = MusicTurn()  # ADR-0148 — запуски музыки хода и речь о них
        #: Хост выступления ноды (``perform_host.PerformHost``, ADR-0150 PR-7c): ``command_turn`` отдаёт ему ``perform``.
        self.performer: Any = None

    def begin_turn(self) -> None:
        """Граница хода (``AgentCore``, медиароутер): лимит «один трек» и музыка хода. Tier-1 плана сюда не входит:
        его ставит хост по словам человека до хода (``plan_host.begin_turn``), а ``AgentCore`` зовёт этот хук уже
        внутри хода — сброс здесь стирал «потом» (живой прогон 21.10 23:10Z: музыка пошла поверх анекдота)."""
        self._track_guard.reset()
        self.music_turn.reset()

    def order_batch(self, calls: Sequence[ToolCall]) -> List[ToolCall]:
        """Порядок исполнения пачки вызовов ``AgentCore`` по плану (результаты модели — в её порядке):

        * ``action.identity`` (``register_speaker``) — первым: речь хода ждёт исхода регистрации (#2913,
          ``TurnSpeechGate``; уходит с ним в PR-7c);
        * при Tier-1 «потом» — реплики раньше действий, которые план ставит после речи («расскажи анекдот, а потом
          включи что-нибудь»: модель могла позвать ``request_music`` первым);
        * остальное — в порядке модели (музыку вперёд и стоп в конец больше никто не переставляет)."""
        def key(call: ToolCall) -> int:
            kind = _plan_kind(call.name)
            if kn.TOOL_CLASSES.get(call.name) == "action.identity":
                return 0
            if kind != "speech" and "speech" in compose(kind, ["speech"], self.planner.hint).waits_for:
                return 2
            return 1

        return sorted(calls, key=key)

    async def discover(self) -> Any:
        return await self._underlying.discover()

    async def aclose(self) -> None:
        return await self._underlying.aclose()

    async def execute_owned(self, call: ToolCall) -> ToolResult:
        """Вызов кода-владельца (подложка выступления): без допуска плана, без лимита трека хода и без речи «музыки
        хода» — это не ход LLM, его ресурсы ведёт сам владелец (``PerformBlock``)."""
        return await self._underlying.execute(call)

    async def execute(self, call: ToolCall) -> ToolResult:
        """Допуск по плану → исполнение → наблюдатель стадий до и после."""
        self._notify(call, None)
        result = await self._execute(call)
        self._notify(call, result)
        return result

    async def _execute(self, call: ToolCall) -> ToolResult:
        if call.name == SPEAK_TOOL and not self.music_turn.speech_allowed():
            # ADR-0148: запуск музыки хода не удался или заиграла пьеса — о музыке говорит код (MusicTurn.phrase()).
            piece, library = self.music_turn.piece_playing(), self.music_turn.library_answered()
            why = "library phrase by code" if library else "piece phrase by code" if piece else "launch failed"
            _LOG.warning(f"music turn: speak_text refused — {why}")
            return ToolResult(tool_call_id=call.id, content=speech_refusal_content(piece, library), is_error=True)
        if _plan_kind(call.name) != _NO_PLAN:
            admission = await self.planner.admit(call.name, call.id)
            if admission.status != "go":
                rejected = admission.status == "rejected" and admission.event is not None
                reason = admission.event.reason if rejected else kn.PLAN_WAIT_REASON
                return _plan_refusal(call, admission.status, reason, admission.event)
        result = await self._execute_direct(call)
        self._after(call, result)
        return result

    def _after(self, call: ToolCall, result: ToolResult) -> None:
        """Речь хода — занятость голоса до ``queued`` владельца; начатый блок — ``PlanEvent block_start``."""
        if result.is_error:
            return
        data = result.data or {}
        if call.name == SPEAK_TOOL:
            self.planner.state.reserve(str(i) for i in data.get("speech_ids") or ())
            self.planner.changed()
        if _plan_kind(call.name) != _NO_PLAN:
            self.planner.emit(PlanEvent("block_start", call.id, reason=call.name))

    async def _execute_direct(self, call: ToolCall) -> ToolResult:
        """Исполнение с лимитом «один трек за ход» (#2859): повторный запуск трека в ходе до провайдера не доходит —
        каждый такой вызов начинается с ``Clock.clear()`` (живой DJ-переход #23: 8 смен трека за 25 с). Отказ — не
        ошибка тула: трек играет."""
        guard = self._track_guard
        if guard.should_refuse(call.name):
            _LOG.warning(f"issue #2859: refusing {call.name} — track already started this turn by {guard.started_tool}")
            return ToolResult(tool_call_id=call.id, content=refusal_content(call.name, guard.started_tool),
                              is_error=False)
        result = await self._underlying.execute(call)
        guard.record(call.name, is_error=bool(result.is_error))
        self.music_turn.record(call.name, call.arguments, is_error=bool(result.is_error),
                               content=str(result.content or ""))
        return result

    def _notify(self, call: ToolCall, result: Optional[ToolResult]) -> None:
        if self._on_tool is None:
            return
        try:
            self._on_tool(call.name, dict(call.arguments or {}), result)
        except Exception:  # noqa: BLE001 — наблюдатель рефлексов не ломает ход
            _LOG.warning("on_tool observer failed for %s", call.name, exc_info=True)


__all__ = ["PlanToolExecutor"]
