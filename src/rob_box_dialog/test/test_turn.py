"""Ход диалога v2 без ROS (ADR-0150 §3, §4.1, §7.3, §8; PR-8 ч.8d #3702): поведение конвейера на фейковых портах."""

import asyncio
import json
import logging
import sqlite3

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import BARE_WAKE, VOICE, Addressed, decide
from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.speaker_adapter import VoiceTag
from rob_box_dialog.command import Ask, Command, Invalid, Performance, ToolCall
from rob_box_dialog.e2e_events import open_events
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.execute import Executor, OwnerEvent
from rob_box_dialog.session import MusicStateRef, SceneFrame, Session, SpeakerRef
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
        self.performed, self.perform_outcome = [], None  # None — владельца выступлений нет (сегменты — репликами)
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

    def perform(self, turn_id, epoch, perf, edit):
        self.performed.append((perf, edit))
        return self.perform_outcome


def _engine(ports, **kw):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids), **kw)


def _run(engine, text, source=VOICE):
    return asyncio.run(engine.run(Addressed(text, source)))


def _run_addressed(engine, addressed):
    return asyncio.run(engine.run(addressed))


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
    assert ports.said[-1][0] in phrases.catalog()[kn.NEW_SESSION_PHRASE]


def test_new_session_confirmation_never_repeats_twice_in_a_row():
    ports = Ports()
    engine = _engine(ports)
    for _ in range(30):
        _run(engine, "новая сессия")
    said = [t for t, _, _ in ports.said]
    assert len(said) == 30 and set(said) <= set(phrases.catalog()[kn.NEW_SESSION_PHRASE])
    assert all(a != b for a, b in zip(said, said[1:])) and len(set(said)) > 3


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


def test_enough_with_continuation_stops_then_gives_the_rest_to_the_llm():
    """Акт 4 n410 (23.10): «ладно, хватит тянуть, теперь наоборот» — стоп кодом, остаток не теряется, а идёт в LLM."""
    ports = Ports(answers=[Answer(Command(speech="Тараторю!"), "deepseek")])
    engine = _engine(ports)
    agg = _run(engine, "ладно хватит тянуть теперь наоборот")
    assert ports.cancels == ["tier1 stop_all"]
    assert len(ports.asked) == 1 and ports.asked[0][0][-1]["content"].endswith("тянуть теперь наоборот")
    assert [t for t, _, _ in ports.said] == ["Тараторю!"] and agg.llm_calls == 1
    assert engine.session.snapshot.top("Silence") is None


def test_enough_with_continuation_runs_tier1_rest_without_llm():
    ports = Ports()
    engine = _engine(ports)
    _run(engine, "хватит, громче")
    assert ports.cancels == ["tier1 stop_all"] and ports.asked == []
    assert ports.executed == [[("set_volume", {"action": "louder"})]]


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
    assert [(p.kind, edit) for p, edit in ports.performed] == [("joke", False)]  # владельца нет — читает ход


RAP = Performance("rap", ("Йо, енот на крыше.", "Хвост — его визитка."), backing="beat")


def test_3901_performance_goes_to_its_owner_and_turn_does_not_read_segments():
    """#3901: владелец выступления взял рэп — ход сегменты не читает (бит, такты, паузы — у владельца)."""
    ports = Ports(answers=[Answer(Command(perform=RAP), "deepseek")])
    ports.perform_outcome = {"status": "pending", "block_id": "perform-t1"}
    agg = _run(_engine(ports), "зачитай рэп про енота под бит")
    assert ports.performed == [(RAP, False)]
    assert ports.said == [] and (agg.stage, agg.status) == ("done", "finished")


def test_3901_edit_pending_goes_to_owner_as_edit_and_rejection_is_said():
    ports = Ports(answers=[Answer(Command(perform=RAP, edit_pending=True), "deepseek")])
    ports.perform_outcome = {"status": "rejected", "reason": "edit_without_active_perform"}
    _run(_engine(ports), "а теперь второй куплет про кота")
    assert ports.performed == [(RAP, True)]
    assert ports.said == [(phrases.text("local.rejected"), None, "sad")]


def test_invalid_command_with_nothing_to_run_is_said_by_code_not_silence():
    """#3953 (Шифу 23.10, В21): невалидная после повтора - не молчащий ``failed``, а честная фраза кода, ход ``done``."""
    ports = Ports(answers=[Answer(Invalid("bad_json"), "minimax", calls=2)])
    agg = _run(_engine(ports), "сделай что-нибудь")
    assert ports.said == [(phrases.text(kn.INVALID_COMMAND_PHRASE), None, "sad")] and ports.executed == []
    assert (agg.stage, agg.status, agg.llm_calls) == ("done", "done", 2)
    assert ports.stages[-1] == ("done", "done", "invalid_salvaged:bad_json")


def test_too_long_answer_is_said_as_too_long_not_as_not_understood():
    """#3883/#3953: модель поняла, а ответ не влез в лимит речи - «не понял» было бы неправдой."""
    ports = Ports(answers=[Answer(Invalid("too_long", "speech: 900 > 600"), "deepseek", calls=2)])
    agg = _run(_engine(ports), "перечисли все голоса")
    assert [t for t, _, _ in ports.said] == [phrases.text(kn.INVALID_COMMAND_PHRASES["too_long"])]
    assert phrases.text(kn.INVALID_COMMAND_PHRASES["too_long"]) != phrases.text(kn.INVALID_COMMAND_PHRASE)
    assert ports.stages[-1] == ("done", "done", "invalid_salvaged:too_long") and agg.status == "done"


def test_two_violations_in_a_row_run_the_valid_actions_and_code_tells_the_outcome():
    """Два нарушения подряд (громкое действие + speech, ретрай то же): действие исполнено, итог говорит код."""
    sound = ToolCall("stop_music", {})
    ports = Ports(answers=[Answer(Invalid(kn.VIOLATION_SPEECH_WITH_ACTIONS, salvage=(sound,)), "deepseek", calls=2)])
    agg = _run(_engine(ports), "выключи музыку пожалуйста сейчас же")
    assert ports.executed == [[("stop_music", {})]]
    assert ports.said == [(phrases.text("music.stop_ok"), None, None)]
    assert (agg.stage, agg.status) == ("done", "done")


def test_salvaged_action_that_failed_is_said_sadly():
    ports = Ports(answers=[Answer(Invalid("bad_args", salvage=(ToolCall("stop_music", {}),)), "deepseek")],
                  outcomes={"stop_music": ("rejected", {})})
    agg = _run(_engine(ports), "выключи музыку пожалуйста сейчас же")
    assert ports.said == [(phrases.text("music.stop_fail"), None, "sad")] and agg.status == "done"


def test_quiet_action_beside_speech_speaks_and_runs_silently_n701():
    """Акт 2b n701: speech + memory_save - слова звучат, факт сохранён, «Запомнил: …» поверх них не говорится."""
    cmd = Command(speech="Привет, Саша! Приятно познакомиться.",
                  actions=(ToolCall("memory_save", {"fact": "Пользователя зовут Саша"}),))
    ports = Ports(answers=[Answer(cmd, "deepseek")], outcomes={"memory_save": ("done", {"fact_id": "f1"})})
    agg = _run(_engine(ports), "здравствуй я Саша чиню тут технику")
    assert ports.executed == [[("memory_save", {"fact": "Пользователя зовут Саша"})]]
    assert ports.said == [("Привет, Саша! Приятно познакомиться.", None, "neutral")]
    assert (agg.stage, agg.status) == ("done", "finished")


def test_register_speaker_beside_speech_is_one_reply_the_models_n722():
    """Акт 2c n722: speech + register_speaker - одна реплика (слова модели), регистрация исполнена, код молчит."""
    cmd = Command(speech="Приятно познакомиться, Борис!", actions=(ToolCall("register_speaker", {"name": "Борис"}),))
    ports = Ports(answers=[Answer(cmd, "deepseek")], outcomes={"register_speaker": ("done", {"name": "Борис"})})
    _run(_engine(ports), "меня зовут Борис")
    assert ports.executed == [[("register_speaker", {"name": "Борис"})]]
    assert [t for t, _, _ in ports.said] == ["Приятно познакомиться, Борис!"]


def test_quiet_action_with_surprising_outcome_or_failure_is_still_said_by_code():
    """Простой успех молчит, но отказ и успех с причиной (уже знакомы) честно говорит код поверх слов модели."""
    cmd = Command(speech="Приятно познакомиться, Борис!", actions=(ToolCall("register_speaker", {"name": "Борис"}),))
    failed = Ports(answers=[Answer(cmd, "deepseek")], outcomes={"register_speaker": ("rejected", {})})
    _run(_engine(failed), "меня зовут Борис")
    assert [t for t, _, _ in failed.said] == [phrases.text("identity.rejected"), "Приятно познакомиться, Борис!"]
    saved = Command(speech="Запомнил!", actions=(ToolCall("memory_save", {"fact": "x"}),))
    lost = Ports(answers=[Answer(saved, "deepseek")], outcomes={"memory_save": ("rejected", {})})
    _run(_engine(lost), "запомни что x")
    assert [t for t, _, _ in lost.said] == [phrases.text("local.rejected"), "Запомнил!"]


def test_passed_is_not_a_failure():
    ports = Ports(answers=[Answer(Command(passed="not_for_me"), "deepseek")])
    agg = _run(_engine(ports), "это не тебе")
    assert agg.stage == "done"
    # #3859: молчание — законный исход, не провал: finished/passed
    assert (agg.status, ports.stages[-1]) == ("finished", ("done", "finished", "passed"))


def test_passed_on_addressed_phrase_is_mumbled_by_code():
    """Решение Шифу 22.10: «промолчать» на адресованную реплику — короткая фраза из каталога, не тишина."""
    ports = Ports(answers=[Answer(Command(passed="nothing_to_say"), "deepseek")])
    agg = _run(_engine(ports), "робот скажи что-нибудь")
    assert [t for t, _, _ in ports.said] and ports.said[0][0] in phrases.catalog()[kn.PASSED_ADDRESSED_PHRASE]
    assert (agg.status, ports.stages[-1]) == ("finished", ("done", "finished", "passed"))


def test_passed_from_backlog_pickup_or_silence_stays_quiet():
    ports = Ports(answers=[Answer(Command(passed="not_addressed"), "deepseek")])
    _run_addressed(_engine(ports), Addressed("фон", "backlog_pickup", True, ()))
    assert ports.said == []
    ports = Ports(answers=[Answer(Command(passed="nothing_to_say"), "deepseek")])
    engine = _engine(ports)
    engine.session.push_scene("Silence", "user")
    _run(engine, "что-то")
    assert ports.said == []


def test_query_data_goes_back_to_llm_once_then_answer():
    first = Answer(Command(actions=(ToolCall("get_current_time", {}),)), "deepseek",
                   follow=lambda data: ({"role": "tool", "content": data},))
    second = Answer(Command(speech="Сейчас пять часов."), "deepseek")
    ports = Ports(answers=[first, second], outcomes={"get_current_time": ("done", {"time": "17:00"})})
    agg = _run(_engine(ports), "который час")
    follow = ports.asked[1][0][-1]
    assert follow["role"] == "tool" and json.loads(follow["content"])["query_results"][0]["data"] == {"time": "17:00"}
    assert ports.said == [("Сейчас пять часов.", None, "neutral")] and agg.llm_calls == 2


def test_query_beyond_limit_is_not_run_and_code_speaks_instead_of_silence():
    """#3953: модель и на последнем вызове просит query (мост не отсёк) — он не исполняется, ход ``done`` с фразой кода."""
    q = Command(actions=(ToolCall("get_current_time", {}),))
    ports = Ports(answers=[Answer(q, "deepseek") for _ in range(kn.LLM_QUERY_ITERATIONS_MAX + 1)])
    agg = _run(_engine(ports), "который час")
    assert len(ports.asked) == kn.LLM_QUERY_ITERATIONS_MAX + 1
    assert len(ports.executed) == kn.LLM_QUERY_ITERATIONS_MAX
    assert (agg.stage, agg.status) == ("done", "done")
    assert [t for t, _, _ in ports.said] == [phrases.text(kn.QUERY_FALLBACK_GENERIC)]
    assert ports.stages[-1] == ("done", "done", "query_fallback:ceiling")


def test_last_llm_call_has_no_query_tools_and_turn_ends_with_speech():
    """#3953 (марафон 23.10, n209): цепочка memory_search до потолка — последний вызов без query-тулов, ход говорит."""
    search = [Answer(Command(actions=(ToolCall("memory_search", {"query": q}),)), "deepseek",
                     follow=lambda data: ({"role": "tool", "content": data},)) for q in ("Борис", "Спартак")]
    final = Answer(Command(speech="Помню: болеешь за Спартак и по пятницам ешь пиццу."), "deepseek")
    ports = Ports(answers=[*search, final], outcomes={"memory_search": ("done", {"results": []})})
    agg = _run(_engine(ports), "а про меня что помнишь")
    offered = [tools for _, tools, _ in ports.asked]
    assert len(offered) == kn.LLM_QUERY_ITERATIONS_MAX + 1
    assert all("memory_search" in tools for tools in offered[:-1])
    assert offered[-1] and all(kn.TOOL_CLASSES.get(t) != "query" for t in offered[-1])
    assert ports.said == [("Помню: болеешь за Спартак и по пятницам ешь пиццу.", None, "neutral")]
    assert (agg.stage, agg.status, agg.llm_calls) == ("done", "finished", 3)


def test_first_call_keeps_query_tools():
    ports = Ports(answers=[Answer(Command(speech="Привет."), "deepseek")])
    _run(_engine(ports), "расскажи что-нибудь")
    assert "memory_search" in ports.asked[0][1]


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


class SlowOwnerPorts(Ports):
    """Владелец действия отвечает не сразу (музыка стартует), LLM думает почти весь бюджет хода."""

    def __init__(self, *a, execute_delay=0.0, **kw):
        super().__init__(*a, **kw)
        self.execute_delay = execute_delay

    async def execute(self, actions, snapshot, turn_id):
        await asyncio.sleep(self.execute_delay)
        return await super().execute(actions, snapshot, turn_id)


def test_started_action_gets_its_own_term_not_the_rest_after_llm_3960():
    """Живой прогон 23.10, акт 5: LLM с повтором съела бюджет, трек заиграл - а ход закрылся «долго думаю».
    Действие получает срок своего класса целиком (``TurnDeadline.grant``): итог - фраза владельца, не таймаут хода."""
    music = ToolCall("request_music", {"intent": "track", "text": "ритм"})
    ports = SlowOwnerPorts(answers=[Answer(Command(actions=(music,)), "deepseek")], ask_delay=0.08,
                           execute_delay=0.1)
    agg = _run(_engine(ports, deadline_s=0.1), "робот поставь ровный ритм")
    said = [t for t, _, _ in ports.said]
    assert phrases.text("understand.timeout") not in said
    assert said == [phrases.text("media.done")] and agg.status != "failed"


def _late(tool, status):
    return ActionResult("t9/0", 0, "personality", tool, kn.TOOL_CLASSES[tool], status, None, {})


def test_late_music_outcome_is_said_by_code_other_classes_stay_silent():
    """Музыка заиграла после честного ``timeout`` хода: «Включаю музыку.» говорит код (``on_late``), а не тишина
    после «пока не заиграла»; поздние итоги других классов (память) этот путь не озвучивает."""
    ports = Ports()
    engine = _engine(ports)
    engine.late(_late("request_music", "done"))
    engine.late(_late("memory_save", "done"))
    assert ports.said == [(phrases.text("media.done"), None, None)]


def test_messages_have_window_without_markers_and_actions_as_structure():
    ports = Ports(answers=[Answer(Command(actions=(ToolCall("stop_music", {}),)), "deepseek"),
                           Answer(Command(speech="Пожалуйста."), "deepseek")])
    engine = _engine(ports, sections=lambda snap, text: {"speaker": {"identified": False}})
    _run(engine, "выключи музыку пожалуйста сейчас же")
    engine.speech_ended("sp1", 100)
    _run(engine, "спасибо")
    messages = ports.asked[1][0]
    assert messages[0] == {"role": "system", "content": "Ты робот."}
    said = f"[говорит: {kn.UNKNOWN_SPEAKER_LABEL}] выключи музыку пожалуйста сейчас же"
    assert messages[1] == {"role": "user", "content": said}
    assert messages[2] == {"role": "assistant", "content": phrases.text("music.stop_ok")}
    last = messages[-1]["content"]
    assert last.endswith("\n\nспасибо") and f"<{RECENT_ACTIONS}>" in last and '"tool":"stop_music"' in last
    assert "[Spkr" not in json.dumps(messages, ensure_ascii=False)
    assert len(messages) == 4, "исход действия — разделом снимка, не отдельной репликой"


def test_e2e_events_are_written_only_when_writer_given(tmp_path):
    events = open_events(str(tmp_path / "e2e.db"), "s1")
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


class MemoryPorts(Ports):
    """Порты с настоящим исполнителем памяти: запрос владельцу уходит только с ``person_ref`` снимка (#3881)."""

    def __init__(self, answers, on_ask=None):
        super().__init__(answers)
        self.requests = []
        self.on_ask = on_ask
        self.executor = Executor(self._owner)

    def _owner(self, request):
        self.requests.append(request)
        asyncio.get_running_loop().call_soon(
            self.executor.on_event, OwnerEvent(request.turn_id, "done", data={**request.args, "fact_id": 1}))

    async def ask(self, messages, tools, timeout_s):
        if self.on_ask:
            self.on_ask()
        return await super().ask(messages, tools, timeout_s)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        return await self.executor.run(actions, snapshot, turn_id)


_FACT = ToolCall("memory_save", {"fact": "пьёт только зелёный чай"})


def test_rejected_memory_save_is_absent_from_the_next_speakers_window_3881():
    """Ход А: собеседник не узнан -> rejected{no_person}; ход Б (Борис): в запросе к LLM нет ни memory_save, ни
    текста факта из вызова, ни висящего tool-результата."""
    ports = MemoryPorts([Answer(Command(actions=(_FACT,)), "x"), Answer(Command(speech="Привет, Борис."), "x")])
    engine = _engine(ports)
    _run(engine, "робот запомни я пью только зелёный чай")
    assert ports.requests == []  # в память не ушло
    engine.session.set_speaker(SpeakerRef(person_ref="spk-boris", name="Борис", name_state="confirmed"))
    _run(engine, "продолжаю говорить, чтобы ты привык к моему голосу")
    messages = ports.asked[1][0]
    blob = json.dumps(messages, ensure_ascii=False)
    assert "memory_save" not in blob and "no_person" not in blob and _FACT.args["fact"] not in blob
    assert f"[говорит: {kn.UNKNOWN_SPEAKER_LABEL}] робот запомни" in blob  # реплика А несёт авторство, не голая
    assert all(m["role"] in ("system", "user", "assistant") for m in messages)  # нет висящих tool без вызова
    assert not [e for e in engine.log.window() if e.role == "tool"]


def test_memory_save_is_rejected_when_the_speaker_changed_during_the_turn_3881():
    """Ход начат репликой Саши, к исполнению в снимке уже Борис: факт не уходит владельцу ни за кого."""
    box = {}
    ports = MemoryPorts([Answer(Command(actions=(_FACT, ToolCall("get_current_time", {}))), "x")],
                        on_ask=lambda: box["e"].session.set_speaker(
                            SpeakerRef(person_ref="spk-boris", name="Борис", name_state="confirmed")))
    engine = box["e"] = _engine(ports)
    engine.session.set_speaker(SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="confirmed"))
    _run(engine, "робот запомни я пью только зелёный чай")
    assert [r.tool for r in ports.requests] == ["get_current_time"]
    assert ports.executed == [[("get_current_time", {})]]
    assert not [e for e in engine.log.window() if e.role == "tool" and e.content["tool"] == "memory_save"]


def test_memory_save_goes_to_the_same_speaker_when_nothing_changed():
    ports = MemoryPorts([Answer(Command(actions=(_FACT,)), "x")])
    engine = _engine(ports)
    engine.session.set_speaker(SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="confirmed"))
    _run(engine, "робот запомни я пью только зелёный чай")
    assert [r.args[kn.PERSON_ARG] for r in ports.requests] == ["spk-sasha"]


def _background_then_bare_wake():
    """Сцена акта 3 (#3892): Борис и незнакомец говорят на фоне без вейка, потом Саша — голое «робот»."""
    backlog = Backlog()
    backlog.add(Heard("закажи пиццу на вечер", t=100.0, voice_s=2.0, utterance_id="u-b"))
    backlog.note_voice("u-b", VoiceTag("known", person_ref="spk-boris", name="Борис", score=0.86))
    backlog.add(Heard("не шумите тут", t=104.0, voice_s=1.5, utterance_id="u-x"))
    backlog.note_voice("u-x", VoiceTag("unknown_voice", candidate="Борис", candidate_score=0.41))
    return decide(Heard("робот", t=108.0, voice_s=0.6), backlog, ["робот"])


def test_backlog_pickup_reaches_llm_with_each_phrase_author_3892():
    """Голое «робот» подхватывает фразы двух авторов: в запросе модели каждая со своим ``[говорит: …]`` на момент
    самой фразы — не того, кто сказал «робот» (Саша), и гипотеза голоса незнакомца — не имя (A30)."""
    addressed = _background_then_bare_wake()
    assert addressed.pickup and len(addressed.picked) == 2
    ports = Ports(answers=[Answer(Command(passed=True), "x")])
    engine = _engine(ports)
    engine.session.set_speaker(SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="confirmed"))
    _run_addressed(engine, addressed)
    said = ports.asked[0][0][-1]["content"].split("\n\n")[-1]
    assert said == f"[говорит: Борис] закажи пиццу на вечер\n[говорит: {kn.UNKNOWN_SPEAKER_LABEL}] не шумите тут"


def test_picked_phrases_stay_in_window_with_authors_for_the_next_question_3892():
    """«Кто говорил на фоне?» следующим ходом: подхваченные фразы — прошлые реплики окна, каждая со своим автором."""
    ports = Ports(answers=[Answer(Command(passed=True), "x"), Answer(Command(speech="Борис и кто-то ещё."), "x")])
    engine = _engine(ports)
    engine.session.set_speaker(SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="confirmed"))
    _run_addressed(engine, _background_then_bare_wake())
    _run(engine, "скажи кто сейчас говорил на фоне")
    messages = ports.asked[1][0]
    assert messages[1:3] == [
        {"role": "user", "content": "[говорит: Борис] закажи пиццу на вечер"},
        {"role": "user", "content": f"[говорит: {kn.UNKNOWN_SPEAKER_LABEL}] не шумите тут"}]
    assert len(messages) == 4 and messages[-1]["content"].endswith("\n\nскажи кто сейчас говорил на фоне")


def test_backlog_pickup_logs_count_speakers_and_turn_3892(caplog):
    with caplog.at_level(logging.INFO):
        _run_addressed(_engine(Ports(answers=[Answer(Command(passed=True), "x")])), _background_then_bare_wake())
        _run(_engine(Ports()), "", BARE_WAKE)
    lines = [r.getMessage() for r in caplog.records if "[backlog] pickup" in r.getMessage()]
    assert lines == [f"🗒️ [backlog] pickup n=2 speakers=Борис,{kn.UNKNOWN_SPEAKER_LABEL} turn=t1",
                     "🗒️ [backlog] pickup n=0 speakers=- turn=t1"]


def test_host_sections_collects_owner_data_for_the_snapshot_of_the_turn():
    """#3906: разделы снимка хода Личности одним сборщиком для ROS-хоста и сухого стенда."""
    from rob_box_dialog.music_state import MusicStateMemory
    from rob_box_dialog.turn import host_sections

    session = Session("personality")
    session.set_speaker(SpeakerRef(person_ref="p1", name="Саша", name_state="confirmed"))
    sections = host_sections(session.snapshot, MusicStateMemory(), [{"kind": "llm", "health": "quota"}])
    assert sections == {"speaker": {"name": "Саша", "name_state": "confirmed"},
                        "providers": [{"kind": "llm", "health": "quota"}],
                        "music_state": {"playing": "unknown", "dj": "unknown"}}
    session.set_speaker(SpeakerRef(person_ref="p1", name="Саша", name_state="tentative"))
    assert host_sections(session.snapshot, MusicStateMemory())["speaker"] == {"name": None, "name_state": "tentative"}


VOICES_DATA = {"provider": "minimax", "default_voice": "x", "providers": [
    {"provider": "yandex", "label": "Яндекс", "voices_count": 13, "active": False, "samples": []},
    {"provider": "minimax", "label": "MiniMax", "voices_count": 10, "active": True, "samples": []}]}


def _voices_turn(second):
    q = Command(actions=(ToolCall("list_tts_voices", {}),))
    ports = Ports(answers=[Answer(q, "deepseek"), Answer(second, "deepseek")],
                  outcomes={"list_tts_voices": ("done", VOICES_DATA)})
    return ports, _run(_engine(ports), "какими голосами умеешь")


def test_invalid_after_query_is_not_silent_code_says_a_phrase_from_the_data():
    """#3883: запрос выполнен, ответ модели невалиден и после повтора - говорит код по данным, ход не failed."""
    ports, agg = _voices_turn(Invalid("too_long", "speech"))
    assert [t for t, _, _ in ports.said] == ["Голосов у меня так: Яндекс — 13, MiniMax — 10. Назвать какие-нибудь?"]
    assert (agg.stage, agg.status) == ("done", "done")


def test_invalid_after_query_of_a_tool_without_phrase_says_generic_phrase():
    q = Command(actions=(ToolCall("get_current_time", {}),))
    ports = Ports(answers=[Answer(q, "deepseek"), Answer(Invalid("too_long", "speech"), "deepseek")])
    agg = _run(_engine(ports), "который час")
    assert [t for t, _, _ in ports.said] == [phrases.text(kn.QUERY_FALLBACK_GENERIC)] and agg.status == "done"


def test_invalid_without_query_is_said_by_code_not_silence():
    ports = Ports(answers=[Answer(Invalid("bad_enum", "emotion"), "deepseek")])
    agg = _run(_engine(ports), "привет")
    assert [t for t, _, _ in ports.said] == [phrases.text(kn.INVALID_COMMAND_PHRASE)]
    assert (agg.stage, agg.status) == ("done", "done")


def test_valid_short_answer_after_query_is_said_and_fallback_stays_silent():
    ports, agg = _voices_turn(Command(speech="Три провайдера, назвать?"))
    assert [t for t, _, _ in ports.said] == ["Три провайдера, назвать?"] and agg.status != "failed"
def test_host_sections_shows_speaker_memory_only_of_the_same_interlocutor_3919():
    from rob_box_dialog.music_state import MusicStateMemory
    from rob_box_dialog.turn import host_sections

    session = Session("personality")
    session.set_speaker(SpeakerRef(person_ref="p1", name="Саша", name_state="confirmed"))
    noted = ("p1", {"dialog_count": 2, "facts": ["любит чай"]})
    assert host_sections(session.snapshot, MusicStateMemory(), (), noted)["speaker_memory"] == noted[1]
    session.set_speaker(SpeakerRef(person_ref="p2", name="Борис", name_state="confirmed"))  # сменился с начала хода
    assert "speaker_memory" not in host_sections(session.snapshot, MusicStateMemory(), (), noted)
    assert "speaker_memory" not in host_sections(session.snapshot, MusicStateMemory())


def test_3974_expression_only_command_finishes_the_turn_without_failure():
    """«Покажи, как тебе грустно»: команда из одной экспрессии валидна, ход ``finished``, не ``nothing_to_do``."""
    from rob_box_dialog.command import Expression
    ports = Ports(answers=[Answer(Command(expression=Expression(emotion="sad")), "deepseek")])
    agg = _run(_engine(ports), "покажи, как тебе грустно")
    assert (agg.stage, agg.status) == ("done", "finished")
    assert ports.stages[-1] == ("done", "finished", "") and ports.said == [] and ports.executed == []
