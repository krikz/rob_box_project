"""Исполнитель тулов хода v1 под планировщиком ресурсов (ADR-0150 §5.6, PR-7b #3699) вместо ``SchedulerToolExecutor``.

Поведение: порядок пачки по плану, ожидание конца занятого блока по событию владельца, честный ``timeout`` плана,
``PlanEvent`` в лог/топик, занятость голоса по ``speech_ids`` ответа ``speak_text``, корм снимка из событий ноды.
"""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

from rob_box_dialog import knowledge as kn
from rob_box_dialog.plan.runner import PlanRunner
from rob_box_llm.provider import ToolCall, ToolResult

from rob_box_voice.core import plan_host
from rob_box_voice.core.plan_tools import PlanToolExecutor

JOKE = "Робот, расскажи анекдот, а потом включи что-нибудь"


class _Underlying:
    def __init__(self) -> None:
        self.executed: list = []

    async def execute(self, call: ToolCall) -> ToolResult:
        self.executed.append(call.name)
        data = {"speech_ids": [f"sp-{call.id}"]} if call.name == "speak_text" else {"ok": True}
        return ToolResult(tool_call_id=call.id, content="ok", data=data)

    async def discover(self):
        return ()

    async def aclose(self):
        return None


def _executor(utterance: str = ""):  # хост ставит Tier-1 до хода, как plan_host.begin_turn
    events, lines = [], []
    under = _Underlying()
    executor = PlanToolExecutor(under, PlanRunner(publish=events.append, log=lines.append))
    executor.planner.begin_turn(utterance)
    return executor, under, events, lines


def _call(cid: str, name: str, **args) -> ToolCall:
    return ToolCall(id=cid, name=name, arguments=args)


def test_order_batch_identity_first_then_model_order():
    executor, *_ = _executor("Робот, включи музыку")
    calls = [_call("1", "request_music"), _call("2", "speak_text", text="Включаю"), _call("3", "register_speaker")]
    assert [c.id for c in executor.order_batch(calls)] == ["3", "1", "2"]


def test_order_batch_with_then_puts_the_reply_before_the_postponed_action():
    """Модель позвала музыку первой — при «а потом» реплика (анекдот) уходит раньше, музыка — после неё."""
    executor, *_ = _executor(JOKE)
    calls = [_call("1", "request_music"), _call("2", "play_animation"), _call("3", "speak_text", text="Анекдот")]
    assert [c.id for c in executor.order_batch(calls)] == ["2", "3", "1"]


def test_stop_music_during_speech_is_not_deferred_anymore():
    """Поведение меняется (ADR §5.6.2: музыка × реплика = MERGE): стоп музыки не ждёт конца речи."""
    executor, under, events, _ = _executor("Робот, выключи музыку")
    executor.planner.state.on_speech("started", "s1", text_len=200)
    calls = [_call("1", "speak_text", text="Выключаю"), _call("2", "stop_music")]
    assert [c.id for c in executor.order_batch(calls)] == ["1", "2"]

    async def run():
        for call in calls:
            await executor.execute(call)

    asyncio.run(run())
    assert under.executed == ["speak_text", "stop_music"]
    assert [e.kind for e in events] == ["block_start", "block_start"]


def test_then_music_waits_for_the_joke_to_finish():
    """(б): музыка стартует только после ``SpeechEvent finished`` анекдота; ``PlanEvent queued`` в логе."""
    executor, under, events, lines = _executor(JOKE)

    async def run():
        executor.begin_turn()  # AgentCore зовёт хук ВНУТРИ хода: «потом» хоста не стирается (живой прогон 21.10)
        said = await executor.execute(_call("1", "speak_text", text="Заходит робот в бар."))
        music = asyncio.ensure_future(executor.execute(_call("2", "request_music", intent="track", text="что-нибудь")))
        await asyncio.sleep(0.05)
        assert not music.done() and under.executed == ["speak_text"]
        executor.planner.state.on_speech("finished", said.data["speech_ids"][0])
        executor.planner.changed()
        return await asyncio.wait_for(music, 2.0)

    result = asyncio.run(run())
    assert result.is_error is False and under.executed == ["speak_text", "request_music"]
    assert [e.kind for e in events] == ["block_start", "queued", "block_start"]
    assert events[1].after_resource == "speech" and events[1].after_block == "sp-1"
    assert any("PlanEvent queued" in ln for ln in lines)


def test_wait_ceiling_is_an_honest_timeout_and_the_action_is_not_run(monkeypatch):
    monkeypatch.setattr(kn, "PLAN_WAIT_MAX_S", 0.05)
    executor, under, _, _ = _executor("робот, а потом включи музыку")
    executor.planner.state.on_music(True, "trk")
    result = asyncio.run(executor.execute(_call("1", "request_music", intent="track", text="x")))
    assert result.is_error is True and under.executed == []
    assert (result.data["status"], result.data["reason"], result.data["after"]) == ("timeout", "plan_wait", "track")


def test_compatible_actions_go_without_waiting():
    """(а)/X11: экспрессия, звук и движение во время речи — сразу (MERGE), ожиданий плана нет."""
    executor, under, events, _ = _executor("Робот, покажи радость, включи звук и поезжай направо")
    executor.planner.state.on_speech("started", "s1", text_len=200)

    async def run():
        await asyncio.gather(executor.execute(_call("1", "play_animation")), executor.execute(_call("2", "play_sound")),
                             executor.execute(_call("3", "move_direction")))

    asyncio.run(run())
    assert sorted(under.executed) == ["move_direction", "play_animation", "play_sound"]
    assert all(e.kind == "block_start" for e in events)


def test_untabled_tools_bypass_the_plan():
    executor, under, events, _ = _executor()
    asyncio.run(executor.execute(_call("1", "set_dj_mode")))
    assert under.executed == ["set_dj_mode"] and events == []


def test_host_feeds_owner_events_into_the_snapshot():
    executor, *_ = _executor()
    node = SimpleNamespace(_tool_executor=executor)
    speech = SimpleNamespace(kind=2, speech_id="s1", turn_id="t", epoch=1, text_len=42, duration_ms=0,
                             spoken_chars=0, seconds=0.0)  # KIND_STARTED
    plan_host.on_speech(node, speech)
    assert executor.planner.state.busy()["speech"].block_id == "s1"
    plan_host.on_speech(node, SimpleNamespace(**{**vars(speech), "kind": 4}))  # KIND_FINISHED
    plan_host.on_sound(node, SimpleNamespace(kind=0, request_id="r1"))  # KIND_STARTED
    assert set(executor.planner.state.busy()) == {"sfx"}
    plan_host.on_music(node, SimpleNamespace(track_id="trk", stops_at=None, is_playing=lambda now: True))
    assert set(executor.planner.state.busy()) == {"sfx", "track"}
    plan_host.begin_turn(node, "робот, а потом включи музыку")
    assert executor.planner.hint == "later"


class _HostNode:
    """Нода-двойник для ``make_tool_executor``: публикаторы, таймеры, часы, логгер."""

    def __init__(self):
        self.timers, self.pubs = [], {}
        self._loop = None

    def create_publisher(self, msg_type, topic, depth):
        return self.pubs.setdefault(topic, SimpleNamespace(sent=[], publish=lambda m, t=topic: self.pubs[t].sent.append(m)))

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(1000 * 1e9),
                                                           to_msg=lambda: SimpleNamespace(sec=1000, nanosec=0)))

    def get_logger(self):
        return SimpleNamespace(info=lambda *a, **k: None, warning=lambda *a, **k: None)


def test_performance_speech_goes_to_its_host_and_the_block_holds_the_perform_column():
    """PR-7c: реплики выступления (и их пред-синтез без звука) голос в снимке не занимают — их видит хост; колонку
    ``perform`` держит блок выступления от ``block_start`` до ``block_end`` («а потом …» ждёт его конца)."""
    from rob_box_dialog.command import Performance

    node = _HostNode()
    executor = plan_host.make_tool_executor(node, _Underlying(), on_tool=None)
    node._tool_executor = executor
    assert [p for p, _ in node.timers] == [0.1]  # таймер выступления
    host = executor.performer
    host.start(Performance(kind="tale", segments=("Жил-был кот.", "И ушёл.")), "t1")
    sid = node.pubs["/voice/speech/request"].sent[0].speech_id
    assert set(executor.planner.state.busy()) == {"perform"}
    plan_host.on_speech(node, SimpleNamespace(kind=0, speech_id=sid, turn_id="t1", epoch=0, text_len=12,
                                              duration_ms=0, spoken_chars=0, seconds=0.0))  # KIND_QUEUED
    assert "speech" not in executor.planner.state.busy()
    assert plan_host.perform_busy(node).active_segment == 0
    host.halt()
    assert executor.planner.state.busy() == {} and plan_host.perform_busy(node).active_segment is None
