"""Режим команд в v1 терминален (ADR-0150 §4.1, §5.2–5.4; PR-7 ч.7c #3698): действия — через Executor по событию
владельца, об исходе говорит код, ``speech`` при действиях не звучит, ответ по сути (``perform``) не теряется."""

from __future__ import annotations

import asyncio
from typing import Any, Dict, Iterable, List, Mapping, Optional

from rob_box_core.tool_catalog import get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.execute import ActionRequest
from rob_box_dialog.respond import ActionResult
from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import DialogueEvent, DialogueStateMachine
from rob_box_harness.core.tool_loop.command_turn import CommandTurn, outcome_phrase, owner_event
from rob_box_harness.memory.base import InMemoryStore
from rob_box_harness.tools import ToolSpec
from rob_box_llm.provider import LLMMessage, LLMResponse, LLMSettings, ToolCall, ToolResult

HAPPY = {"sound": "robot_happy"}


def _request(tool: str, args: Optional[Mapping[str, Any]] = None) -> ActionRequest:
    return ActionRequest("t/0", 0, "personality", tool, dict(args or {}), kn.TOOL_CLASSES[tool], 3.0)


def _result(tool: str, status: str, reason: Optional[str] = None, **data: Any) -> ActionResult:
    return ActionResult("t/0", 0, "personality", tool, kn.TOOL_CLASSES[tool], status, reason, data)


# ── исход тула → событие владельца ─────────────────────────────────────────────────────────────────────────


def test_owner_event_takes_status_and_reason_from_result_data() -> None:
    busy = ToolResult("c", "Звук не сыграл", is_error=True, data={"status": "rejected", "reason": "busy"})
    event = owner_event(_request("play_sound", HAPPY), busy)
    assert (event.ref, event.status, event.reason) == ("t/0", "rejected", "busy")
    assert event.data["sound"] == "robot_happy"  # аргумент вызова — слот фразы


def test_owner_event_keeps_mcp_refusal_text_as_reason() -> None:
    refused = ToolResult("c", "sender 'x' не в списке разрешённых", is_error=True)
    assert owner_event(_request("get_current_time"), refused).reason == "sender 'x' не в списке разрешённых"


def test_plan_outcome_and_missing_status_follows_is_error() -> None:
    waited = ToolResult("c", "{}", is_error=True, data={"status": "timeout", "reason": "plan_wait"})
    assert (owner_event(_request("request_music"), waited).status,
            owner_event(_request("request_music"), waited).reason) == ("timeout", "plan_wait")
    assert owner_event(_request("memory_save"), ToolResult("c", "ok")).status == "done"
    assert owner_event(_request("memory_save"), ToolResult("c", "нет", is_error=True)).status == "rejected"
    applied = ToolResult("c", "ok", data={"status": "applied"})  # не статус исполнителя — решает is_error
    assert owner_event(_request("set_voice", {"voice": "alena"}), applied).status == "done"


# ── фраза кода об исходе ───────────────────────────────────────────────────────────────────────────────────


def test_outcome_phrase_is_built_by_code_from_the_event() -> None:
    assert outcome_phrase([_result("memory_save", "done", fact="Саша любит чай")]) == "Запомнил: Саша любит чай."
    assert outcome_phrase([_result("play_sound", "done", **HAPPY)]) is None  # успех звука молчит
    busy = outcome_phrase([_result("play_sound", "rejected", "busy", **HAPPY)])
    assert busy == phrases.text("sound.skipped.busy", sound="robot_happy")


def test_outcome_phrase_skips_queries_and_outcomes_spoken_elsewhere() -> None:
    assert outcome_phrase([_result("get_current_time", "done")]) is None
    assert outcome_phrase([_result("set_voice", "done"), _result("request_music", "done")]) is None


def test_missing_slot_falls_back_to_a_generic_failure_not_a_crash() -> None:
    assert outcome_phrase([_result("play_sound", "rejected", "busy")]) == phrases.text("local.rejected")
    assert outcome_phrase([_result("navigate_to_waypoint", "pending")]) is None


# ── ход v1-цикла с командой ────────────────────────────────────────────────────────────────────────────────


class _Tools:
    """Исполнитель тулов v1-двойник: ответы по имени тула, журнал вызовов."""

    def __init__(self, results: Optional[Dict[str, ToolResult]] = None, offered: Iterable[str] = ()) -> None:
        self._results = dict(results or {})
        self._offered = tuple(offered) or ("speak_text", "play_sound", "memory_save", "get_current_time")
        self.calls: List[ToolCall] = []

    async def discover(self) -> tuple:
        return tuple(ToolSpec(name=n, description=n, parameters=dict(get_tool(n).parameters)) for n in self._offered)

    async def execute(self, call: ToolCall) -> ToolResult:
        self.calls.append(call)
        canned = self._results.get(call.name, ToolResult(call.id, "ok"))
        return ToolResult(call.id, canned.content, canned.is_error, canned.data)

    def said(self) -> List[tuple]:
        return [(c.arguments["text"], c.arguments.get("voice")) for c in self.calls if c.name == "speak_text"]

    def acted(self) -> List[str]:
        return [c.name for c in self.calls if c.name != "speak_text"]

    async def aclose(self) -> None:
        return None


class _LLM:
    """LLM-двойник: команды моста по очереди, запросы запоминаются."""

    name = "deepseek"

    def __init__(self, replies: List[LLMResponse]) -> None:
        self._replies = list(replies)
        self.requests: List[List[LLMMessage]] = []

    async def complete(self, messages: Iterable[LLMMessage], *, tools: Iterable[Any] = (),
                       settings: Optional[LLMSettings] = None) -> LLMResponse:
        self.requests.append(list(messages))
        return self._replies.pop(0)

    async def aclose(self) -> None:
        return None


def _command(**fields: Any) -> LLMResponse:
    return LLMResponse(tool_calls=(ToolCall(id="cmd-1", name=kn.COMMAND_TOOL, arguments=fields),))


def _turn(tools: _Tools, *replies: LLMResponse) -> tuple:
    dsm = DialogueStateMachine()
    dsm.on_event(DialogueEvent.WAKE_WORD)
    dsm.on_event(DialogueEvent.STT_RESULT)
    llm = _LLM(list(replies))
    core = AgentCore(llm=llm, tools=tools, memory=InMemoryStore(), dsm=dsm, system_prompt="ты РОББОКС")
    result = asyncio.run(core.process_input("робот, сделай", preclassified_event=DialogueEvent.STT_RESULT))
    assert result.error is None
    return result, llm


def _sound(status: str, reason: Optional[str] = None) -> ToolResult:
    return ToolResult("x", "звук", is_error=status != "done", data={**HAPPY, "status": status, "reason": reason})


def test_sound_done_robot_says_nothing_about_it_and_llm_gets_no_second_word() -> None:
    """«Включи звук радостный»: об успехе звука молчит и код, и модель (её «Вот звук» теперь нарушение, #3698)."""
    tools = _Tools({"play_sound": _sound("done")})
    result, llm = _turn(tools, _command(actions=[{"tool": "play_sound", "args": HAPPY}]))
    assert tools.acted() == ["play_sound"] and tools.said() == []
    assert result.spoken_text == "" and len(llm.requests) == 1  # ход терминален
    assert list(result.tools_called) == ["play_sound"]


def test_sound_rejected_is_told_by_code_not_by_the_model() -> None:
    tools = _Tools({"play_sound": _sound("rejected", "busy")})
    result, llm = _turn(tools, _command(actions=[{"tool": "play_sound", "args": HAPPY}]))
    assert tools.said() == [(phrases.text("sound.skipped.busy", sound="robot_happy"), None)]
    assert len(llm.requests) == 1 and result.tool_error_occurred


def test_sound_and_a_joke_the_joke_is_kept_the_sound_report_is_not() -> None:
    """«Включи звук и расскажи шутку»: шутка — ``perform`` (ответ по сути), она звучит; о звуке — ни слова."""
    tools = _Tools({"play_sound": _sound("done")})
    joke = ["Заходит робот в бар.", "А бар ему: опять ты со своими битами!"]
    result, _ = _turn(tools, _command(actions=[{"tool": "play_sound", "args": HAPPY}],
                                      perform={"kind": "tale", "segments": joke}))
    assert tools.acted() == ["play_sound"]
    assert tools.said() == [(joke[0], None), (joke[1], None)]
    assert result.speak_text_real_count == 2


def test_memory_phrase_comes_after_saved_from_code() -> None:
    tools = _Tools({"memory_save": ToolResult("x", "Факт сохранён", data={"fact_id": 7, "fact": "Саша любит чай"})})
    result, llm = _turn(tools, _command(actions=[
        {"tool": "memory_save", "args": {"fact": "Саша любит чай"}}]))
    assert tools.acted() == ["memory_save"]
    assert tools.said() == [("Запомнил: Саша любит чай.", None)]
    assert len(llm.requests) == 1 and result.spoken_text == ""


def test_tale_in_different_voices_keeps_voices_without_set_voice() -> None:
    tools = _Tools()
    tale = {"kind": "tale", "segments": ["Жила-была девочка.", "Куда идёшь?", "К бабушке."],
            "voice_per_segment": ["", "zahar", "alena"]}
    _turn(tools, _command(perform=tale))
    assert tools.said() == [("Жила-была девочка.", None), ("Куда идёшь?", "zahar"), ("К бабушке.", "alena")]
    assert tools.acted() == []


def test_speech_voice_is_the_voice_of_this_line_only() -> None:
    tools = _Tools()
    _turn(tools, _command(speech="Привет!", voice="zahar"))
    assert tools.said() == [("Привет!", "zahar")] and tools.acted() == []


def test_set_voice_out_of_the_turn_slice_is_not_executed() -> None:
    """#3744: ``set_voice`` в срезе хода только по явной голосовой просьбе; вне среза команда невалидна."""
    tools = _Tools()
    result, _ = _turn(tools, _command(actions=[{"tool": "set_voice", "args": {"voice": "zahar"}}]))
    assert tools.acted() == [] and tools.said() == [] and result.turn_failure.startswith("invalid_command")


def test_query_data_go_back_to_llm_then_its_speech_is_spoken() -> None:
    tools = _Tools({"get_current_time": ToolResult("x", "12:00")})
    result, llm = _turn(tools, _command(actions=[{"tool": "get_current_time", "args": {}}]),
                        LLMResponse(content="Сейчас полдень."))
    assert len(llm.requests) == 2 and result.spoken_text == "Сейчас полдень."
    tool_msgs = [m for m in llm.requests[1] if m.role == "tool"]
    assert len(tool_msgs) == 1 and "get_current_time: 12:00" in tool_msgs[0].content
    assert tools.said() == []


def test_queries_beyond_two_iterations_are_refused() -> None:
    ask = _command(actions=[{"tool": "get_current_time", "args": {}}])
    tools = _Tools({"get_current_time": ToolResult("x", "12:00")})
    result, llm = _turn(tools, ask, ask, ask)
    assert len(llm.requests) == kn.LLM_QUERY_ITERATIONS_MAX + 1
    assert tools.acted() == ["get_current_time"] * kn.LLM_QUERY_ITERATIONS_MAX
    assert tools.said() == [] and result.turn_failure == "nothing_to_do"


def test_invalid_command_fails_the_turn_without_speech_and_executes_nothing() -> None:
    tools = _Tools()
    result, _ = _turn(tools, _command(actions=[{"tool": "play_sound", "args": {"sound": "nope"}}]))
    assert tools.acted() == [] and tools.said() == [] and result.spoken_text == ""
    assert result.turn_failure.startswith("invalid_command")


def test_bridge_marked_invalid_command_fails_the_turn_without_speech() -> None:
    tools = _Tools()
    result, _ = _turn(tools, _command(**{kn.COMMAND_INVALID_FIELD: "bad_enum"}))
    assert tools.calls == [] and result.spoken_text == "" and result.turn_failure == "invalid_command:bad_enum"


def test_valid_turn_has_no_failure() -> None:
    result, _ = _turn(_Tools(), _command(speech="Привет!"))
    assert result.turn_failure == ""


def test_handles_only_a_single_command_call() -> None:
    assert CommandTurn.handles(_command(speech="x"))
    assert not CommandTurn.handles(LLMResponse(tool_calls=(ToolCall(id="1", name="play_sound", arguments=HAPPY),)))
    assert not CommandTurn.handles(LLMResponse(content="привет"))


# ── планировщик ресурсов (PR-7b #3699): «а потом» — действие после речи команды ──────────────────────────────


class _PlannedTools(_Tools):
    """Исполнитель тулов с планировщиком, как ``PlanToolExecutor``: Tier-1 хода — по словам человека."""

    def __init__(self, utterance: str, **kwargs: Any) -> None:
        from rob_box_dialog.plan.runner import PlanRunner

        super().__init__(**kwargs)
        self.planner = PlanRunner(log=lambda _: None)
        self.planner.begin_turn(utterance)


def test_then_music_code_says_it_will_play_after_the_joke_and_the_joke_goes_first() -> None:
    """(б) «Робот, расскажи анекдот, а потом включи что-нибудь»: фраза кода «Поставлю после…», анекдот, потом музыка."""
    tools = _PlannedTools("Робот, расскажи анекдот, а потом включи что-нибудь",
                          offered=("speak_text", "request_music"))
    joke = ["Заходит робот в бар.", "А бар ему: опять ты со своими битами!"]
    _turn(tools, _command(actions=[{"tool": "request_music", "args": {"intent": "track", "text": "включи что-нибудь"}}],
                          perform={"kind": "tale", "segments": joke}))
    said = [text for text, _ in tools.said()]
    assert said[0].startswith("Поставлю после того, как договорю")
    assert said[1:3] == joke
    names = [c.name for c in tools.calls]
    assert names.index("request_music") > max(i for i, n in enumerate(names) if n == "speak_text" and i < 3)


def test_without_then_the_action_is_not_postponed() -> None:
    tools = _PlannedTools("Робот, включи музыку и расскажи анекдот", offered=("speak_text", "request_music"))
    joke = ["Заходит робот в бар.", "Конец."]
    _turn(tools, _command(actions=[{"tool": "request_music", "args": {"intent": "track", "text": "включи что-нибудь"}}],
                          perform={"kind": "tale", "segments": joke}))
    assert [c.name for c in tools.calls][0] == "request_music"
    assert not any(t.startswith("Поставлю") for t, _ in tools.said())


def test_plan_timeout_is_spoken_by_code_even_for_music() -> None:
    """Живой робот 21.10 23:20Z: «а потом включи» не дождалась очереди — робот молчал (о музыке говорит music_turn,
    а он о запросе, не дошедшем до плеера, не знает). Отказ плана говорит код."""
    waited = _result("request_music", "timeout", kn.PLAN_WAIT_REASON)
    assert outcome_phrase([waited]) == phrases.text("plan.timeout")
    assert outcome_phrase([_result("request_music", "timeout")]) is None  # обычный таймаут музыки — у music_turn


# ── выступление отдаётся его владельцу (ADR-0150 PR-7c #3700) ──────────────────────────────────────────────


class _Performer:
    """Хост выступления-двойник: что ему отдали и какой сегмент звучит."""

    def __init__(self, active: Optional[int] = None) -> None:
        self.active_segment = active
        self.started: List[Any] = []
        self.edited: List[Any] = []

    def start(self, perf: Any, turn_id: str) -> Dict[str, Any]:
        self.started.append(perf)
        return {"status": "pending"}

    def edit(self, perf: Any) -> Dict[str, Any]:
        self.edited.append(perf)
        return {"status": "done"}


RAP = {"kind": "rap", "segments": ["Я комар, звеню всю ночь.", "Ты не спишь, а мне не в мочь."], "backing": "beat"}


def test_perform_goes_to_its_owner_not_to_speak_text() -> None:
    """Рэп под бит — одно действие: сегменты, подложку и такты ведёт владелец выступления, а не N ``speak_text``."""
    tools = _Tools()
    tools.performer = _Performer()
    result, llm = _turn(tools, _command(perform=RAP))
    (perf,) = tools.performer.started
    assert perf.segments == tuple(RAP["segments"]) and perf.backing == "beat"
    assert tools.said() == [] and tools.acted() == [] and len(llm.requests) == 1
    assert not result.turn_failure


def test_edit_pending_goes_to_the_running_performance() -> None:
    """X3 «и про енота»: правка не начатых сегментов — валидна, пока идёт выступление (ACTIVE от владельца)."""
    tools = _Tools()
    tools.performer = _Performer(active=2)
    result, _ = _turn(tools, _command(perform={**RAP, "edit_pending": True, "from_segment": 2}))
    (perf,) = tools.performer.edited  # A20 22.10: флаг внутри perform, номер = ACTIVE — всё равно правка (#3700)
    assert perf.from_segment == 0 and tools.performer.started == [] and not result.turn_failure  # номер — у хоста


def test_edit_pending_without_a_performance_is_invalid() -> None:
    tools = _Tools()
    tools.performer = _Performer(active=None)
    result, _ = _turn(tools, _command(perform=RAP, edit_pending=True))
    assert tools.performer.edited == [] and result.turn_failure.startswith("invalid_command")
