"""#4065 (живая проверка 27.10 06:36 UTC, dev-d2144c9): смешанная команда ``query`` + действие не молчит (ADR-0148).

«Робот, что у тебя в музыкальной библиотеке?» → валидная команда ``[play_animation, music_library list]`` без речи
(``play_animation`` - повтор прошлого хода) → оба тула ✅ → ход ``done`` с ``llm_calls=1`` и без речи. Данные
возвращались модели, только если ВСЕ действия команды - ``query``; смешанная шла в ``_speak``, а там фразы по ``query``
нет и страховки ``query_fallback`` тоже. Теперь решает код, одно правило для голого, смешанного и спасённого ``query``:

* итерации есть - данные ``query`` модели (``turn.query_more``), исход действий говорит код сразу (провал - грустно),
  модели действия не уходят (ADR-0150 §4.1: ``action.*`` терминальны);
* последняя итерация (``query`` уже не исполняется) и слов модели не слышно - код говорит по данным хода
  (``query_fallback``);
* слова, написанные рядом с ``query`` до данных, не звучат: данные уходят модели, звучит её ответ по ним.
"""

import asyncio
import json

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, Form, Invalid, Performance, ToolCall, parse, validate
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import SAD_FACE, Answer, Shown, TurnEngine

#: Живой лог 27.10 06:36:19: реплика и команда модели (``play_animation sad`` - повтор действия прошлого хода).
N_LIBRARY = "Робот, что у тебя в музыкальной библиотеке?"
ANIM_CALL = ToolCall("play_animation", {"animation": "sad"})
LIST_CALL = ToolCall("music_library", {"action": "list", "limit": 10})
TRACKS = {"tracks": [{"title": "Ночной дозор"}, {"title": "Рассвет"}], "count": 2}
ANSWER = "В библиотеке два трека: «Ночной дозор» и «Рассвет»."
LIBRARY_DONE = {"music_library": ("done", TRACKS, None)}


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход действия - аргументы вызова и данные тула поверх них (как у
    ``owner_event``)."""

    def __init__(self, answers=(), outcomes=None):
        self.answers, self.outcomes = list(answers), outcomes or {}
        self.asked, self.executed, self.said, self.stages, self.faces = [], [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append((list(messages), [t.name for t in tools]))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        out = []
        for n, a in enumerate(actions):
            status, data, reason = self.outcomes.get(a.tool, ("done", {}, None))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool,
                                    kn.TOOL_CLASSES[a.tool], status, reason, {**dict(a.args), **data}))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append((text, voice, emotion))
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="",
              expression=None):
        self.stages.append((stage, status, reason))
        if expression is not None:
            self.faces.append((stage, expression))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(ports, text=N_LIBRARY, session=None):
    engine = TurnEngine(session or Session("personality"), ports, system_prompt="Ты робот.")
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def _rows(ports, call=1):
    """Данные ``query``, которые модель получила в вызове ``call`` (последнее сообщение запроса)."""
    return json.loads(ports.asked[call][0][-1]["content"])[kn.QUERY_RESULTS_FIELD]


def _mixed(*actions):
    return Answer(Command(actions=actions or (ANIM_CALL, LIST_CALL)), "deepseek")


# -- (а) итерации есть: данные query - модели, действия - фразой кода ------------------------------------------------


def test_4065_live_mixed_command_without_speech_returns_the_data_and_the_answer_is_spoken():
    """06:36:22: ``[play_animation, music_library list]`` без речи → данные list модели, её ответ звучит (было: ход
    ``done`` после одного вызова LLM и тишина)."""
    ports = Ports([_mixed(), Answer(Command(speech=ANSWER), "deepseek")], LIBRARY_DONE)
    agg = _run(ports)
    assert ports.executed == [[("play_animation", {"animation": "sad"}), ("music_library", dict(LIST_CALL.args))]]
    assert len(ports.asked) == 2 and agg.llm_calls == 2, "второй вызов LLM получил данные"
    assert _rows(ports) == [{"tool": "music_library", "status": "done", "reason": None,
                             "data": {**LIST_CALL.args, **TRACKS}}], "модели - только данные query"
    assert ports.said == [(ANSWER, None, "neutral")], "успех анимации молчит, звучит ответ модели по данным"
    assert ports.stages[-1] == ("done", "finished", "")


def test_4065_failed_action_next_to_query_is_said_by_code_now_and_not_sent_to_the_model():
    """Провал действия смешанной команды не теряется: фразу провала говорит код сразу (грустно), модели он не уходит -
    она отвечает по данным query и не пересказывает исход действия (ADR-0150 §4.1)."""
    disco = ToolCall("play_animation", {"animation": "disco"})
    ports = Ports([_mixed(disco, LIST_CALL), Answer(Command(speech=ANSWER), "deepseek")],
                  {**LIBRARY_DONE, "play_animation": ("rejected", {}, None)})
    _run(ports)
    assert ports.said == [(phrases.text("local.rejected"), None, "sad"), (ANSWER, None, "neutral")]
    assert [r["tool"] for r in _rows(ports)] == ["music_library"]


def test_4065_salvaged_and_executed_mixed_query_go_one_way_4050():
    """Спасённые действия невалидной команды (#4050) идут тем же путём, что исполненные: звонок говорит код, модели -
    только данные list."""
    bell = ToolCall("play_sound", {"sound": "ui_bell"})
    salvaged = Answer(Invalid("speech_with_actions", salvage=(bell, LIST_CALL)), "deepseek", calls=2)
    ports = Ports([salvaged, Answer(Command(speech=ANSWER), "deepseek")], LIBRARY_DONE)
    _run(ports)
    assert [t for t, _, _ in ports.said] == [phrases.text("sound.imitation.bell"), ANSWER]
    assert [r["tool"] for r in _rows(ports)] == ["music_library"]


#: Ревью PR #4067: «что ты обо мне помнишь?» неузнанным голосом, спасённый ``memory_context`` отклонён до владельца.
N_OWN_MEMORY = "Робот, что ты обо мне помнишь?"
CONTEXT_CALL = ToolCall("memory_context", {})
BELL_CALL = ToolCall("play_sound", {"sound": "ui_bell"})


@pytest.mark.parametrize("salvage", [(CONTEXT_CALL,), (CONTEXT_CALL, BELL_CALL)], ids=["refusal_only", "with_bell"])
def test_pr4067_salvaged_refused_memory_read_before_the_last_iteration_closes_the_turn_done(salvage):
    """Блокер ревью PR #4067: спасённые ``query`` невалидной команды - только озвученный отказ чтения памяти, итерации
    ещё есть. Фраза кода звучит одна, читать модели нечего - второго вызова LLM нет, и ход закрыт ``done`` (было:
    итог ``addressed`` - рефлекс «услышал» и лицо «слушаю» после фразы отказа). Лицо итога - грусть фразы отказа, а не
    рефлекс ``happy`` (#4069 п.8, живой 27.10 08:20:52: «Не узнаю твой голос…» → ``happy -> happy (3.0 с)``)."""
    ports = Ports([Answer(Invalid("speech_with_actions", salvage=salvage), "deepseek", calls=2)],
                  {"memory_context": ("rejected", {}, kn.NO_PERSON_REASON)})
    agg = _run(ports, N_OWN_MEMORY)
    assert len(ports.asked) == 1 and len(ports.said) == 1 and ports.said[0][2] == "sad"
    assert ports.said[0][0].startswith(phrases.text("query.rejected.no_person"))
    assert (agg.stage, agg.status) == ("done", "done")
    assert ports.stages[-1] == ("done", "done", "invalid_salvaged:speech_with_actions")
    assert ports.faces == [("done", SAD_FACE)], "отказ чтения - грустное лицо итога, не happy"


def test_pr4067_bare_query_expression_does_not_paint_the_done_face():
    """Ревью PR #4067: экспрессия команды с голым ``query`` написана до данных, как и её слова, - итог она не красит
    (как до PR): данные модели, её ответ невалиден - фраза кода по данным, лицо итога - её грусть («не смог сложить»,
    #4069 п.8), а не ``happy`` команды."""
    happy = Answer(Command(actions=(LIST_CALL,), expression=Expression(emotion="happy")), "deepseek")
    ports = Ports([happy, Answer(Invalid(kn.VIOLATION_SPEECH_TOO_LONG, "speech"), "deepseek", calls=2)], LIBRARY_DONE)
    _run(ports)
    assert [t for t, _, _ in ports.said] == [phrases.text(kn.QUERY_FALLBACK_GENERIC)]
    assert ports.faces == [("done", SAD_FACE)], "экспрессия до данных итог не красит"


def test_pr4067_executed_animation_next_to_query_still_paints_the_done_face():
    """Исполненный ``play_animation`` рядом с ``query`` красит итог своей сценой (#4049), даже когда ответ модели по
    данным невалиден и говорит код."""
    ports = Ports([_mixed(), Answer(Invalid(kn.VIOLATION_SPEECH_TOO_LONG, "speech"), "deepseek", calls=2)], LIBRARY_DONE)
    _run(ports)
    assert ports.faces == [("done", Shown(emotion="sad", duration_s=kn.FACE_SHOW_DEFAULT_S))]


def test_4065_invalid_answer_after_the_mixed_data_is_said_from_the_data_3883():
    """Страховка #3883 как раньше: данные смешанной команды у модели, её ответ невалиден и после повтора - говорит
    код по данным запроса; list повторно не исполняется."""
    ports = Ports([_mixed(), Answer(Invalid(kn.VIOLATION_SPEECH_TOO_LONG, "speech"), "deepseek", calls=2)], LIBRARY_DONE)
    agg = _run(ports)
    assert [t for t, _, _ in ports.said] == [phrases.text(kn.QUERY_FALLBACK_GENERIC)]
    assert (agg.stage, agg.status) == ("done", "done") and len(ports.executed) == 1


# -- (б) последняя итерация: query не исполняется, слов нет - фраза кода по данным -----------------------------------


def test_4065_mixed_on_the_last_iteration_without_words_is_said_by_code_from_the_data():
    """Модель до потолка просит ``[play_animation, list]`` без слов: на последней итерации list не исполняется
    (читать некому), анимация исполнена один раз (#4069 п.1: повтор в том же ходе не исполняется) и красит итог, а
    говорит код по данным хода - не тишина."""
    ports = Ports([_mixed()] * (kn.LLM_QUERY_ITERATIONS_MAX + 1), LIBRARY_DONE)
    agg = _run(ports)
    assert len(ports.asked) == kn.LLM_QUERY_ITERATIONS_MAX + 1
    assert ports.executed == [[("play_animation", {"animation": "sad"}), ("music_library", dict(LIST_CALL.args))],
                              *[[("music_library", dict(LIST_CALL.args))]] * (kn.LLM_QUERY_ITERATIONS_MAX - 1)], \
        "анимация не повторена; query на последней итерации не исполнен"
    assert ports.said == [(phrases.text(kn.QUERY_FALLBACK_GENERIC), None, "sad")]
    assert (agg.stage, agg.status) == ("done", "done") and ports.stages[-1] == ("done", "done", "query_fallback:ceiling")
    assert ports.faces == [("done", Shown(emotion="sad", duration_s=kn.FACE_SHOW_DEFAULT_S))], \
        "исполненная анимация красит итог (#4049) временем показа (#4083)"


def test_4065_action_after_the_data_is_the_answer_and_no_fallback_follows():
    """«Включи что-нибудь из библиотеки»: list → данные → на последней итерации ``music_library play`` - ответ сам
    действие (query модель больше не просит): фразы «нашёл ответ, но…» нет."""
    play = ToolCall("music_library", {"action": "play", "track_id": "t1"})
    ports = Ports([_mixed(LIST_CALL), _mixed(LIST_CALL), _mixed(play)],
                  {"music_library": ("done", {**TRACKS, "name": "«Рассвет»", "duration": "3 минуты"}, None)})
    agg = _run(ports, "Робот, включи что-нибудь из библиотеки")
    assert ports.executed[-1] == [("music_library", dict(play.args))]
    assert phrases.text(kn.QUERY_FALLBACK_GENERIC) not in {t for t, _, _ in ports.said}
    assert (agg.stage, agg.status) == ("done", "finished")


# -- (в) слова рядом с query: написаны до данных - данные модели, звучит её ответ ------------------------------------


@pytest.mark.parametrize("actions", [(LIST_CALL,), (ANIM_CALL, LIST_CALL)], ids=["bare_query", "mixed"])
def test_4065_words_next_to_query_wait_for_the_data(actions):
    """Речь рядом с ``query`` законна только у текстового канала (телеграм): она написана ДО данных и ответом по ним
    быть не может. Правило одно для голого и смешанного ``query`` (ADR-0150 §4.1 query-цикл, §5.1: результат
    ``query`` - модели): слова команды не уходят, данные - модели, звучит её ответ по ним."""
    early = Answer(Command(speech="Сейчас гляну!", actions=actions), "deepseek")
    ports = Ports([early, Answer(Command(speech=ANSWER), "deepseek")], LIBRARY_DONE)
    _run(ports, session=Session(kn.TEXT_CHANNEL_AGENT))
    assert [t for t, _, _ in ports.said] == [ANSWER] and len(ports.asked) == 2
    assert [r["tool"] for r in _rows(ports)] == ["music_library"]


class PerformPorts(Ports):
    """Владелец выступления берёт его (``status: started``) и записывает, что ему отдали."""

    def __init__(self, answers=(), outcomes=None):
        super().__init__(answers, outcomes)
        self.performed = []

    def perform(self, turn_id, epoch, perf, edit):
        self.performed.append(perf.kind)
        return {"status": "started"}


def test_pr4067_perform_next_to_query_waits_for_the_data():
    """Ревью PR #4067 (негативный кейс): выступление рядом с ``query`` написано до данных - владельцу оно не уходит,
    данные - модели, звучит её ответ по ним (то же правило (в), что и для речи)."""
    rap = Answer(Command(actions=(LIST_CALL,), perform=Performance("rap", ("Йо, сейчас гляну в библиотеку!",))),
                 "deepseek")
    ports = PerformPorts([rap, Answer(Command(speech=ANSWER), "deepseek")], LIBRARY_DONE)
    _run(ports)
    assert ports.performed == [] and [t for t, _, _ in ports.said] == [ANSWER] and len(ports.asked) == 2
    assert [r["tool"] for r in _rows(ports)] == ["music_library"]


def test_4065_speech_next_to_query_is_a_violation_for_the_voice_agent():
    """У Личности такая команда до хода не доходит: ``query`` - громкое действие, речь при нём - нарушение
    ``speech_with_actions`` (повтор моста), так что (в) для голоса - только ``ask``/``perform``."""
    raw = {"speech": "Сейчас гляну!", "actions": [{"tool": "music_library", "args": {"action": "list"}}]}
    res = validate(parse(raw), ("music_library", "play_animation"), form=Form(passed=False, voice=False))
    assert isinstance(res, Invalid) and res.reason == kn.VIOLATION_SPEECH_WITH_ACTIONS
