"""#4069: хвосты #4065/#4066 — ход с ``query`` и второй итерацией модели (ADR-0148: решает код).

1. Действие, исполненное на первой итерации, вторая не повторяет: после ``[music_library play, music_library list]``
   модели уходят только данные list, и её ``music_library play`` во втором ответе перезапустил бы трек
   (``turn.unrepeated``). Модель не дала ничего сверх сделанного — говорит код по данным хода.
2. Сцена ``play_animation`` первой итерации на итоге остаётся: лицо модели к ответу по данным её не перекрывает, а
   сцены второй итерации встают в очередь за ней (механизм - ``turn.command_face(earlier=)`` из #4103 n905; здесь -
   регрессия на сценарий карточки: смешанная команда с ``music_library list``).
3. У списка библиотеки своя фраза кода по данным: число треков и 2–3 названия (``turn.query_answer``), а не общая
   «Нашёл ответ, но не смог сложить…»; это ответ, а не провал — без грусти.
4. Метка окна «[ответ действием: …]» — по данным результата: ``stop_music`` при ``was_playing=False`` — «ничего не
   играло», у ``request_music`` — что заказано (text, genre, mood).
8. Лицо итога после отказа чтения памяти и любой грустной фразы кода — грусть, а не рефлекс ``happy`` (живой 27.10
   08:20:52: «Не узнаю твой голос…» → ``face_node: happy -> happy (3.0 с)``).
"""

import asyncio
import logging

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, Invalid, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.turn import SAD_FACE, Answer, Shown, TurnEngine, query_answer, unrepeated
from rob_box_dialog.turn_log import TurnLog

N_LIBRARY = "Робот, включи что-нибудь из библиотеки и расскажи, что там есть"
PLAY_CALL = ToolCall("music_library", {"action": "play", "track_id": "t1"})
LIST_CALL = ToolCall("music_library", {"action": "list", "limit": 10})
ANIM_CALL = ToolCall("play_animation", {"animation": "sad"})
#: Данные ``music_library list`` как их отдаёт тул (``gen_library._list``): ``total`` - вся библиотека.
TITLES = ("Ровный ритм", "Ночной дозор", "Ровный ритм", "Рассвет", "Полдень")
LIST_DATA = {"tracks": [{"id": f"t{n}", "title": t} for n, t in enumerate(TITLES)], "total": 213, "shown": 5}
SUMMARY = "Треков в библиотеке: 213. Например: «Ровный ритм», «Ночной дозор», «Рассвет»."
ANSWER = "В библиотеке двести тринадцать треков."
MAX = kn.LLM_QUERY_ITERATIONS_MAX


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход действия — аргументы вызова и данные тула поверх них (как у
    ``owner_event``); данные - по ``(тул, action)`` или по тулу."""

    def __init__(self, answers=(), outcomes=None):
        self.answers, self.outcomes = list(answers), outcomes or {}
        self.asked, self.executed, self.said, self.stages, self.faces = [], [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        out = []
        for n, a in enumerate(actions):
            key = (a.tool, a.args.get("action"))
            status, data, reason = self.outcomes.get(key, self.outcomes.get(a.tool, ("done", {}, None)))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool,
                                    kn.call_class(a.tool, a.args), status, reason, {**dict(a.args), **data}))
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


LIBRARY = {("music_library", "list"): ("done", LIST_DATA, None),
           ("music_library", "play"): ("done", {"title": "Рассвет"}, None)}


def _engine(ports, session=None):
    return TurnEngine(session or Session("personality"), ports, system_prompt="Ты робот.")


def _run(ports, text=N_LIBRARY, session=None):
    return asyncio.run(_engine(ports, session).run(Addressed(text, VOICE)))


def _cmd(*actions, **kw):
    return Answer(Command(actions=actions, **kw), "deepseek")


# -- п.1: исполненное действие вторая итерация не повторяет ---------------------------------------------------------


def test_play_from_the_first_iteration_is_not_restarted_by_the_second(caplog):
    """``[play, list]`` → данные list модели; её ответ ``[play]`` - повтор: трек не перезапускается, второй вызов
    исполнителю не уходит, а говорит код по данным хода (модель ничего не добавила к сделанному)."""
    ports = Ports([_cmd(PLAY_CALL, LIST_CALL), _cmd(PLAY_CALL)], LIBRARY)
    with caplog.at_level(logging.INFO):
        agg = _run(ports)
    assert ports.executed == [[("music_library", dict(PLAY_CALL.args)), ("music_library", dict(LIST_CALL.args))]], \
        "play исполнен один раз"
    assert len(ports.asked) == 2 and agg.llm_calls == 2
    assert [t for t, _, _ in ports.said][-1] == SUMMARY, "модель не ответила по данным - код говорит по ним"
    assert "повтор исполненного в этом ходе не исполняется: ['music_library']" in caplog.text
    assert (agg.stage, agg.status) == ("done", "done")


def test_repeat_is_dropped_but_a_new_action_of_the_same_answer_runs():
    """Во втором ответе повтор и новое действие: исполняется только новое, его исход говорит код."""
    louder = ToolCall("set_music_volume", {"action": "louder"})
    ports = Ports([_cmd(PLAY_CALL, LIST_CALL), _cmd(PLAY_CALL, louder)], LIBRARY)
    _run(ports)
    assert ports.executed[-1] == [("set_music_volume", {"action": "louder"})]
    assert SUMMARY not in {t for t, _, _ in ports.said}, "модель ответила действием - фразы по данным нет"


def test_repeated_animation_next_to_words_is_not_replayed_and_the_words_sound():
    """Повтор тихого ``play_animation`` рядом с речью (законная пара): сцена не переигрывается, речь звучит, а лицо
    итога - сцена первой итерации."""
    ports = Ports([_cmd(ANIM_CALL, LIST_CALL), Answer(Command(speech=ANSWER, actions=(ANIM_CALL,)), "deepseek")],
                  LIBRARY)
    _run(ports)
    assert ports.executed == [[("play_animation", {"animation": "sad"}), ("music_library", dict(LIST_CALL.args))]]
    assert [t for t, _, _ in ports.said] == [ANSWER]
    assert ports.faces == [("done", Shown(emotion="sad", duration_s=kn.FACE_SHOW_DEFAULT_S))]


def test_failed_action_is_not_repeated_and_its_failure_is_said_once():
    """Провал первой итерации модель не видит (``action.*`` терминальны) и повторяет вызов: второго провала и второй
    фразы «Не вышло.» нет."""
    disco = ToolCall("play_animation", {"animation": "disco"})
    ports = Ports([_cmd(disco, LIST_CALL), Answer(Command(speech=ANSWER, actions=(disco,)), "deepseek")],
                  {**LIBRARY, "play_animation": ("rejected", {}, None)})
    _run(ports)
    assert len(ports.executed) == 1
    assert [t for t, _, _ in ports.said] == [phrases.text("local.rejected"), ANSWER]


def test_unrepeated_compares_tool_and_arguments():
    """Повтор - тот же тул с теми же аргументами; другая сцена или другой трек - новое действие."""

    class Turn:
        turn_id, acted = "t1", (ANIM_CALL, PLAY_CALL)

    other = ToolCall("music_library", {"action": "play", "track_id": "t2"})
    happy = ToolCall("play_animation", {"animation": "happy"})
    cmd = Command(actions=(ToolCall("play_animation", {"animation": "sad"}), other, happy))
    assert unrepeated(logging.getLogger("t"), Turn(), cmd).actions == (other, happy)
    fresh = Command(actions=(other,))
    assert unrepeated(logging.getLogger("t"), Turn(), fresh) is fresh, "повтора нет - команда та же"


# -- п.2: сцена первой итерации на итоге остаётся --------------------------------------------------------------------


def test_scene_of_the_first_iteration_is_not_overwritten_by_the_answer_face():
    """``[play_animation sad, list]`` → данные → ответ словами с ``expression happy``: на итоге - грусть, которую
    показали по просьбе, а не лицо модели к ответу по данным."""
    happy = Answer(Command(speech=ANSWER, expression=Expression(emotion="happy")), "deepseek")
    ports = Ports([_cmd(ANIM_CALL, LIST_CALL), happy], LIBRARY)
    _run(ports)
    assert ports.said == [(ANSWER, None, "happy")]
    assert ports.faces == [("done", Shown(emotion="sad", duration_s=kn.FACE_SHOW_DEFAULT_S))]


def test_scene_of_the_second_iteration_queues_after_the_first():
    """Вторая итерация показывает свою сцену: она встаёт в очередь за первой (``Shown.then``), а не вместо неё."""
    happy = ToolCall("play_animation", {"animation": "happy"})
    ports = Ports([_cmd(ANIM_CALL, LIST_CALL), Answer(Command(speech=ANSWER, actions=(happy,)), "deepseek")], LIBRARY)
    _run(ports)
    seconds = kn.FACE_SHOW_DEFAULT_S
    assert ports.faces == [("done", Shown(emotion="sad", duration_s=seconds,
                                          then=(Shown(emotion="happy", duration_s=seconds),)))]


# -- п.3: фраза кода по данным списка библиотеки ---------------------------------------------------------------------


def _list_result(**data):
    return ActionResult("t/0", 0, "personality", "music_library", "query", "done", None,
                        {"action": "list", **data})


def test_library_list_answer_is_the_count_and_a_few_titles():
    assert query_answer([_list_result(**LIST_DATA)]) == SUMMARY, "варианты одного заказа - одно имя, не больше трёх"
    assert query_answer([_list_result(tracks=[], total=0)]) == phrases.text("music.library_empty")
    assert query_answer([_list_result(tracks=[{"title": ""}], total=4)]) is None, "без названий - фразы по данным нет"
    search = ActionResult("t/0", 0, "personality", "music_library", "query", "done", None,
                          {"action": "search", "query": "ритм", **LIST_DATA})
    assert query_answer([search]) is None, "своей фразы у search нет - «Треков в библиотеке» о поиске было бы неправдой"


def test_library_list_on_the_ceiling_is_said_from_the_data_without_sadness():
    """Модель до потолка просит list без слов (живой 27.10: на последней итерации звучала общая «Нашёл ответ, но…»):
    код говорит число треков и названия, это ответ - не грустный, и лицо итога грустью не красится."""
    ports = Ports([_cmd(LIST_CALL)] * (MAX + 1), LIBRARY)
    agg = _run(ports, "Робот, что у тебя в музыкальной библиотеке?")
    assert ports.said == [(SUMMARY, None, None)]
    assert ports.stages[-1] == ("done", "done", "query_fallback:ceiling") and agg.status == "done"
    assert ports.faces == []


def test_library_list_after_an_invalid_answer_is_said_from_the_data():
    """Страховка #3883: данные list у модели, её ответ невалиден и после повтора — код говорит по данным, без грусти."""
    ports = Ports([_cmd(LIST_CALL), Answer(Invalid(kn.VIOLATION_SPEECH_TOO_LONG, "speech"), "deepseek", calls=2)], LIBRARY)
    _run(ports, "Робот, что у тебя в музыкальной библиотеке?")
    assert ports.said == [(SUMMARY, None, None)] and ports.faces == []


# -- п.4: метка окна по данным результата ----------------------------------------------------------------------------


def _label(tool, **data):
    log = TurnLog()
    log.begin("t1", 0, "робот, хватит")
    log.result("t1", ActionResult("t1/0", 0, "personality", tool, kn.call_class(tool, data), "done", None, data))
    return log.window()[-1].content


def test_stop_without_playing_music_is_not_just_done_in_the_window():
    assert _label("stop_music", was_playing=False) == "[ответ действием: stop_music — сделано, ничего не играло]"
    assert _label("stop_music", was_playing=True) == "[ответ действием: stop_music — сделано]"
    assert _label("dj_set", action="stop", was_playing=False) == \
        "[ответ действием: dj_set stop — сделано, ничего не играло]"


def test_request_music_label_says_what_was_ordered():
    assert _label("request_music", intent="track", text="ровный ритм", mood="спокойная", genre="") == \
        "[ответ действием: request_music track (text: ровный ритм; mood: спокойная) — сделано]"


def test_tier1_stop_with_a_stale_playing_snapshot_writes_the_truth_to_the_window():
    """«Хватит» молчит (Tier-1), а снимок отстал: играло «playing», тул ответил ``was_playing=False``. В окне следующего
    хода - «ничего не играло», а не «сделано» (модель не думает, что выключила музыку)."""
    ports = Ports([Answer(Command(speech="Хорошо."), "deepseek")],
                  {"stop_music": ("done", {"was_playing": False}, None)})
    engine = _engine(ports)
    engine.session.set_music(MusicStateRef("playing"))
    asyncio.run(engine.run(Addressed("Робот, хватит", VOICE)))
    asyncio.run(engine.run(Addressed("Робот, что сейчас играет?", VOICE)))
    history = [m["content"] for m in ports.asked[0] if m["role"] == "assistant"]
    assert history == ["[ответ действием: stop_music — сделано, ничего не играло]"]


def test_every_window_note_is_a_phrase_key_of_the_outcome_table():
    """Слова метки берутся по ключу фразы исхода (одна таблица ``TOOL_PHRASES`` по данным) - ключ обязан быть в ней."""
    keys = {k for entries in kn.TOOL_PHRASES.values() for entry in entries.values()
            for k in (entry.all_keys() if isinstance(entry, kn.ByData) else (entry,))}
    assert set(kn.WINDOW_OUTCOME_NOTES) <= keys
    assert set(kn.WINDOW_ACTION_DETAILS) <= set(kn.TOOL_CLASSES)


# -- п.8: лицо итога после отказа и провала - грусть, а не happy -----------------------------------------------------


def test_memory_refusal_face_is_sad_not_happy():
    """Живой 27.10 08:20:52: «что ты обо мне помнишь?» неузнанным голосом → ``memory_context`` отклонён до владельца →
    «Не узнаю твой голос…» и ``face_node: happy -> happy``. Теперь лицо итога - грусть фразы отказа."""
    ports = Ports([_cmd(ToolCall("memory_context", {}))], {"memory_context": ("rejected", {}, kn.NO_PERSON_REASON)})
    agg = _run(ports, "Робот, что ты обо мне помнишь?")
    assert ports.said == [(phrases.text("query.rejected.no_person"), None, "sad")]
    assert (agg.stage, agg.status) == ("done", "finished") and ports.faces == [("done", SAD_FACE)]


def test_answer_after_the_data_with_no_failure_keeps_the_happy_reflex():
    """Без провала и без лица команды - рефлекс ``done`` как был (грусть только по грустной фразе)."""
    ports = Ports([_cmd(LIST_CALL), Answer(Command(speech=ANSWER), "deepseek")], LIBRARY)
    _run(ports)
    assert ports.said == [(ANSWER, None, "neutral")] and ports.faces == []
