"""Ход диалога v2 без ROS (ADR-0150 §3, §4.1, §7.3, §8; PR-8 ч.8d #3702): поведение конвейера на фейковых портах."""

import asyncio
import json
import sqlite3

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import BARE_WAKE, VOICE, Addressed
from rob_box_dialog.command import Ask, Command, Invalid, Performance, ToolCall
from rob_box_dialog.e2e_events import open_events
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import MusicStateRef, SceneFrame, Session
from rob_box_dialog.turn import RECENT_ACTIONS, Answer, TurnEngine, outcome_phrase, tier1_actions
from rob_box_dialog.grammar import Tier1Command


class Ports:
    """Записывающий транспорт: ответы LLM по очереди, исходы действий по имени тула."""

    def __init__(self, answers=(), outcomes=None, ask_delay=0.0, perform_owns_music=False):
        self.answers = list(answers)
        self.outcomes = outcomes or {}
        self.ask_delay = ask_delay
        self.perform_owns_music = perform_owns_music
        self.asked, self.executed, self.said, self.stages, self.cancels, self.intents = [], [], [], [], [], []
        self.boundaries, self.performs, self.exec_epochs = [], [], []
        self.gate = None  # asyncio.Event: ask ждёт его (LLM «думает»), пока тест не отпустит

    async def ask(self, messages, tools, timeout_s):
        self.asked.append((list(messages), [t.name for t in tools], timeout_s))
        if self.ask_delay:
            await asyncio.sleep(self.ask_delay)
        if self.gate is not None:
            await self.gate.wait()
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        self.exec_epochs.append(snapshot.epoch)
        out = []
        for n, a in enumerate(actions):
            status, data = self.outcomes.get(a.tool, ("done", {}))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                                    status, None, data))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append((text, voice, emotion))
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source=""):
        self.stages.append((stage, status, reason))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        self.cancels.append(reason)
        self.boundaries.append((epoch, boundary))

    def stop_perform(self, how):
        self.performs.append(how)
        return self.perform_owns_music

    def intent(self, turn_id, epoch, kind):
        self.intents.append(kind)


def _engine(ports, **kw):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids), **kw)


def _run(engine, text, source=VOICE):
    return asyncio.run(engine.run(Addressed(text, source)))


def test_bare_wake_is_reflex_only_zero_llm():
    ports = Ports()
    agg = _run(_engine(ports), "", BARE_WAKE)
    assert ports.asked == [] and ports.said == [] and ports.executed == []
    assert agg.llm_calls == 0 and ports.stages[0] == ("addressed", "started", "")


def test_tier1_volume_runs_tool_and_says_outcome_without_llm():
    ports = Ports()
    agg = _run(_engine(ports), "громче")
    assert ports.asked == []
    assert ports.executed == [[("set_volume", {"action": "louder"})]]
    assert ports.said == [(phrases.text("voice.volume.louder"), None, None)]
    assert agg.llm_calls == 0 and agg.stage == "done"


def test_tier1_volume_failure_is_said_honestly():
    ports = Ports(outcomes={"set_volume": ("rejected", {})})
    _run(_engine(ports), "громче")
    assert ports.said[0][0] == phrases.text("voice.volume_fail")


def test_tier1_silence_cancels_speech_and_pushes_scene():
    ports = Ports()
    engine = _engine(ports)
    _run(engine, "замолчи")
    assert ports.cancels == ["tier1 silence"] and ports.said == []
    assert engine.session.snapshot.top("Silence") is not None


def test_tier1_new_session_resets_epoch_and_window():
    ports = Ports(answers=[Answer(Command(speech="Привет!"), "deepseek")])
    engine = _engine(ports)
    _run(engine, "привет")
    engine.speech_ended("sp1", 7)
    _run(engine, "новая сессия")
    assert engine.session.snapshot.epoch == 1
    assert engine.log.window() == ()


def test_tier1_stop_all_stops_music_motion_and_speech():
    ports = Ports()
    engine = _engine(ports)
    engine.session._snap = engine.session.snapshot.__class__(  # снимок плеера: играет трек
        **{**vars(engine.session.snapshot), "music": MusicStateRef("playing")})
    _run(engine, "стоп")
    assert ports.cancels == ["tier1 stop_all"] and ports.intents == ["stop_motion"]
    assert ports.executed == [[("stop_music", {})]]
    assert ports.said == []


def _with_music(engine, **music):
    engine.session._snap = engine.session.snapshot.__class__(
        **{**vars(engine.session.snapshot), "music": MusicStateRef("playing", **music)})


def test_enough_stops_the_set_and_speech_but_does_not_silence_and_next_command_is_taken():
    """Решение Шифу 22.10 (#3700): «хватит» — остановить звуки; режим тишины не включается, следующая реплика идёт."""
    ports = Ports()
    engine = _engine(ports)
    _with_music(engine)
    engine.session.push_scene("DJ", "code")
    _run(engine, "хватит")
    assert ports.cancels == ["tier1 stop_all"]
    assert ports.executed == [[("dj_set", {"action": "stop"}), ("stop_music", {})]]
    assert engine.session.snapshot.top("Silence") is None
    assert ports.said == []
    _run(engine, "музыку громче")  # следующая команда принята, а не проглочена тишиной
    assert ports.executed[-1] == [("set_music_volume", {"action": "louder"})]
    assert ports.said == [(phrases.text("music.volume.louder"), None, None)]


def test_enough_with_nothing_playing_does_not_silence_and_says_nothing():
    ports = Ports()
    engine = _engine(ports)
    _run(engine, "Робот, хватит")
    assert engine.session.snapshot.top("Silence") is None
    assert ports.executed == [] and ports.said == [] and ports.asked == []


def test_shut_up_is_still_silence():
    ports = Ports()
    engine = _engine(ports)
    _run(engine, "замолчи")
    assert engine.session.snapshot.top("Silence") is not None
    assert ports.cancels == ["tier1 silence"]


def test_tier1_actions_dj_set_is_stopped_with_set():
    snap = Session("personality").snapshot
    dj = snap.__class__(**{**vars(snap), "music": MusicStateRef("playing"), "scenes": (SceneFrame("DJ", "code"),)})
    assert [c.tool for c in tier1_actions(Tier1Command("stop_all"), dj)] == ["dj_set", "stop_music"]
    assert tier1_actions(Tier1Command("stop_all"), snap) == ()


def test_llm_speech_is_said_and_only_spoken_part_enters_window():
    ports = Ports(answers=[Answer(Command(speech="Жил-был робот. Он пел."), "deepseek", tokens=900)])
    engine = _engine(ports)
    agg = _run(engine, "расскажи сказку")
    assert ports.said == [("Жил-был робот. Он пел.", None, "neutral")]
    assert (agg.llm_calls, agg.llm_provider, agg.tokens, agg.stage) == (1, "deepseek", 900, "done")
    engine.speech_ended("sp1", 14)
    window = [(e.role, e.content) for e in engine.log.window()]
    assert window == [("user", "расскажи сказку"), ("assistant", "Жил-был робот.")]


def test_speech_with_actions_is_not_said_outcome_phrase_is():
    cmd = Command(speech="Выключаю!", actions=(ToolCall("stop_music", {}),))
    ports = Ports(answers=[Answer(cmd, "minimax")])
    _run(_engine(ports), "выключи музыку пожалуйста сейчас же")
    assert ports.executed == [[("stop_music", {})]]
    assert [s[0] for s in ports.said] == [phrases.text("music.stop_ok")]


def test_failed_action_phrase_is_sad():
    cmd = Command(actions=(ToolCall("stop_music", {}),))
    ports = Ports(answers=[Answer(cmd, "minimax")], outcomes={"stop_music": ("rejected", {})})
    _run(_engine(ports), "выключи музыку пожалуйста сейчас же")
    assert ports.said == [(phrases.text("music.stop_fail"), None, "sad")]


def test_perform_segments_are_said_even_with_actions():
    cmd = Command(actions=(ToolCall("play_sound", {"sound": "boop"}),),
                  perform=Performance("joke", ("Раз анекдот.", "Два анекдот."), voice_per_segment=("", "x")))
    ports = Ports(answers=[Answer(cmd, "deepseek")])
    _run(_engine(ports), "звук и анекдот")
    assert [(s[0], s[1]) for s in ports.said] == [("Раз анекдот.", None), ("Два анекдот.", "x")]


def test_invalid_command_fails_turn_without_speech():
    ports = Ports(answers=[Answer(Invalid("bad_json"), "minimax", calls=2)])
    agg = _run(_engine(ports), "сделай что-нибудь")
    assert ports.said == [] and ports.executed == []
    assert (agg.stage, agg.status, agg.llm_calls) == ("failed", "failed", 2)
    assert ports.stages[-1] == ("failed", "failed", "invalid_command:bad_json")


def test_passed_is_silent():
    ports = Ports(answers=[Answer(Command(passed="not_for_me"), "deepseek")])
    agg = _run(_engine(ports), "это не тебе")
    assert ports.said == [] and agg.stage == "done"


def test_query_data_goes_back_to_llm_once_then_answer():
    first = Answer(Command(actions=(ToolCall("get_current_time", {}),)), "deepseek",
                   follow=lambda data: ({"role": "tool", "content": data},))
    second = Answer(Command(speech="Сейчас пять часов."), "deepseek")
    ports = Ports(answers=[first, second], outcomes={"get_current_time": ("done", {"time": "17:00"})})
    agg = _run(_engine(ports), "который час")
    follow = ports.asked[1][0][-1]
    assert follow["role"] == "tool" and json.loads(follow["content"])["query_results"][0]["data"] == {"time": "17:00"}
    assert ports.said == [("Сейчас пять часов.", None, "neutral")] and agg.llm_calls == 2


def test_query_beyond_limit_is_not_run_and_turn_fails():
    q = Command(actions=(ToolCall("get_current_time", {}),))
    ports = Ports(answers=[Answer(q, "deepseek") for _ in range(kn.LLM_QUERY_ITERATIONS_MAX + 1)])
    agg = _run(_engine(ports), "который час")
    assert len(ports.asked) == kn.LLM_QUERY_ITERATIONS_MAX + 1
    assert len(ports.executed) == kn.LLM_QUERY_ITERATIONS_MAX
    assert (agg.stage, agg.status) == ("failed", "failed") and ports.said == []


def test_ask_sets_pending_question():
    ports = Ports(answers=[Answer(Command(ask=Ask("yes_no", "Включить музыку?")), "deepseek")])
    engine = _engine(ports)
    _run(engine, "мне скучно")
    assert ports.said[0][0] == "Включить музыку?"
    assert engine.session.snapshot.pending_ask.text == "Включить музыку?"


def test_deadline_cancels_llm_and_says_timeout():
    ports = Ports(answers=[Answer(Command(speech="поздно"), "minimax")], ask_delay=5)
    agg = _run(_engine(ports, deadline_s=0.05), "расскажи что-нибудь")
    assert ports.said == [(phrases.text("understand.timeout"), None, None)]
    assert (agg.stage, agg.status) == ("timeout", "failed")


def test_messages_have_window_without_markers_and_actions_as_structure():
    ports = Ports(answers=[Answer(Command(actions=(ToolCall("stop_music", {}),)), "deepseek"),
                           Answer(Command(speech="Пожалуйста."), "deepseek")])
    engine = _engine(ports, sections=lambda snap, text: {"speaker": {"identified": False}})
    _run(engine, "выключи музыку пожалуйста сейчас же")
    engine.speech_ended("sp1", 100)
    _run(engine, "спасибо")
    messages = ports.asked[1][0]
    assert messages[0] == {"role": "system", "content": "Ты робот."}
    assert messages[1] == {"role": "user", "content": "выключи музыку пожалуйста сейчас же"}
    assert messages[2] == {"role": "assistant", "content": phrases.text("music.stop_ok")}
    last = messages[-1]["content"]
    assert last.endswith("\n\nспасибо") and f"<{RECENT_ACTIONS}>" in last and '"tool":"stop_music"' in last
    assert "[Spkr" not in json.dumps(messages, ensure_ascii=False)
    assert len(messages) == 4, "исход действия — разделом снимка, не отдельной репликой"


def test_e2e_events_are_written_only_when_writer_given(tmp_path):
    events = open_events(True, str(tmp_path / "e2e.db"), "s1")
    ports = Ports(answers=[Answer(Command(speech="Привет!"), "deepseek")])
    engine = _engine(ports, events=events)
    _run(engine, "привет")
    engine.speech_started("sp1")
    engine.speech_ended("sp1", 7)
    events.close()
    rows = sqlite3.connect(tmp_path / "e2e.db").execute("SELECT kind, text FROM dialog_events ORDER BY id").fetchall()
    assert [k for k, _ in rows] == ["user", "command", "speech_requested", "turn", "speech_started", "speech_spoken"]
    assert rows[0][1] == "привет" and rows[-1][1] == "Привет!"


def test_without_events_writer_nothing_is_written(tmp_path):
    ports = Ports(answers=[Answer(Command(speech="Привет!"), "deepseek")])
    _run(_engine(ports), "привет")
    assert list(tmp_path.iterdir()) == []


def test_outcome_phrase_skips_query_and_self_speaking_tools():
    r = [ActionResult("t/0", 0, "personality", "set_voice", "action.local", "done"),
         ActionResult("t/1", 0, "personality", "get_current_time", "query", "done")]
    assert outcome_phrase(r) is None


@pytest.mark.parametrize("text", ["", "   "])
def test_empty_text_turn_is_reflex_only(text):
    ports = Ports()
    agg = _run(_engine(ports), text)
    assert ports.asked == [] and agg.llm_calls == 0


# -- перебивание по эпохе (PR-9 #3703, ADR-0150 §6.2) ---------------------------------------------------------------


def test_halt_cuts_speech_now_stops_perform_motion_and_bumps_epoch():
    ports = Ports()
    engine = _engine(ports)
    _with_music(engine)
    engine.session.push_scene("DJ", "code")
    agg = _run(engine, "стой")
    assert ports.boundaries == [(0, "now")] and ports.performs == ["halt"] and ports.intents == ["stop_motion"]
    assert engine.session.snapshot.epoch == 1 and agg.epoch == 1
    assert ports.executed == [[("dj_set", {"action": "stop"}), ("stop_music", {})]]
    assert ports.exec_epochs == [1], "стоп музыки — действие новой эпохи, не отброшенное"


def test_enough_cuts_on_sentence_boundary_and_performance_owns_its_beat():
    ports = Ports(perform_owns_music=True)
    engine = _engine(ports)
    _with_music(engine)
    _run(engine, "хватит")
    assert ports.boundaries == [(0, "as_requested")] and ports.performs == ["stop"]
    assert ports.executed == [], "бит выступления снимает само выступление (outro на такте), общий стоп не нужен"
    assert engine.session.snapshot.epoch == 1 and engine.session.snapshot.top("Silence") is None


def test_cancelled_turn_is_finished_as_cancelled_and_says_nothing():
    ports = Ports(answers=[Answer(Command(speech="Жили-были дед и баба…"), "deepseek")])
    engine = _engine(ports)

    async def scenario():
        ports.gate = asyncio.Event()
        story = asyncio.ensure_future(engine.run(Addressed("расскажи сказку", VOICE)))
        await asyncio.sleep(0)  # ход дошёл до LLM и ждёт ответа
        story.cancel()  # хост: «стой» отменяет ход в полёте (TurnsInFlight)
        with pytest.raises(asyncio.CancelledError):
            await story
        return await engine.run(Addressed("стой", VOICE))

    agg = asyncio.run(scenario())
    assert len(ports.asked) == 1, "отмена пришла, когда ход ждал LLM"
    assert ports.said == [] and ports.executed == []
    assert ("done", "cancelled", "interrupt") in ports.stages
    assert agg.epoch == 1 and ports.boundaries == [(0, "now")]


def test_answer_of_old_epoch_is_dropped_without_speech_or_actions():
    dance = (ToolCall("play_animation", {"name": "happy"}),)
    ports = Ports(answers=[Answer(Command(speech="Жили-были…", actions=dance), "deepseek")])
    engine = _engine(ports)

    async def scenario():
        ports.gate = asyncio.Event()
        story = asyncio.ensure_future(engine.run(Addressed("расскажи сказку", VOICE)))
        await asyncio.sleep(0)
        await engine.run(Addressed("хватит", VOICE))  # эпоха +1, ход сказки не отменили (гонка)
        ports.gate.set()  # ответ модели старой эпохи пришёл
        return await story

    agg = asyncio.run(scenario())
    assert agg.status == "cancelled" and ports.said == [] and ports.executed == []
    assert ("done", "cancelled", "stale_epoch") in ports.stages


def test_not_a_stop_does_not_cancel_speech_or_change_epoch():
    ports = Ports(answers=[Answer(Command(speech="Сейчас три часа."), "deepseek")])
    engine = _engine(ports)
    _run(engine, "а который час")
    assert ports.cancels == [] and ports.performs == [] and engine.session.snapshot.epoch == 0
