"""Фраза об исходе действия строится из РЕЗУЛЬТАТА, а не из класса тула (ADR-0148; #4020, #4022, #4031).

Ход целиком на фейковых портах: исполнитель отдаёт результат так же, как настоящий (класс ТУЛА, данные = аргументы
вызова поверх ответа тула, ``command_turn.owner_event``), а ход обязан сам выбрать класс вызова и фразу.
"""

import asyncio
import json

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine, say_late

TRACKS = [{"id": "t-1", "title": "Кот и ёж", "rating": 0}, {"id": "t-2", "title": "Дождь", "rating": 1}]


class Ports:
    """Транспорт хода: ответы LLM по очереди, исходы действий по имени тула ``(статус, причина, данные тула)``."""

    def __init__(self, answers=(), outcomes=None):
        self.answers = list(answers)
        self.outcomes = outcomes or {}
        self.asked, self.executed, self.said, self.stage_classes = [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append((list(messages), [t.name for t in tools], timeout_s))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        out = []
        for n, a in enumerate(actions):
            status, reason, data = self.outcomes.get(a.tool, ("done", None, {}))
            # как настоящий исполнитель: класс ТУЛА, а аргументы вызова лежат в данных поверх ответа тула
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                                    status, reason, {**dict(a.args), **data}))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="",
              expression=None):
        if stage == "executing":
            self.stage_classes.append(cls)

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _engine(ports):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))


def _run(engine, text):
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def _follow(data):
    return ({"role": "tool", "content": data},)


def _library(action, **args):
    return ToolCall("music_library", {"action": action, **args})


# ---------------------------------------------------------------------------------------------------------------
# #4020: music_library - класс вызова по действию; «Включаю музыку.» только у play
# ---------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("call, data", [
    (_library("list"), {"tracks": TRACKS, "total": 2, "shown": 2, "sort_by": "recent"}),
    (_library("search", query="кот"), {"query": "кот", "tracks": TRACKS[:1], "total": 1}),
    (_library("info", track_id="t-1"), {"id": "t-1", "title": "Кот и ёж", "genre": "folk", "play_count": 3}),
])
def test_library_reads_go_back_to_llm_and_robot_does_not_say_it_is_playing(call, data):
    """n705/n707 (марафон 23.10): поиск и инфо говорили «Включаю музыку.», а данные до LLM не доходили."""
    first = Answer(Command(actions=(call,)), "deepseek", follow=_follow)
    second = Answer(Command(speech="Вот что нашлось в библиотеке."), "deepseek")
    ports = Ports([first, second], {"music_library": ("done", None, data)})
    agg = _run(_engine(ports), "что у тебя в музыкальной библиотеке")
    assert ports.said == ["Вот что нашлось в библиотеке."]
    assert phrases.text("media.done") not in ports.said and "Включаю музыку." not in ports.said
    follow = json.loads(ports.asked[1][0][-1]["content"])[kn.QUERY_RESULTS_FIELD][0]
    assert follow["tool"] == "music_library" and follow["data"]["action"] == call.args["action"]
    assert {k: follow["data"][k] for k in data} == data  # треки/сведения доехали до модели как есть
    assert agg.llm_calls == 2 and ports.stage_classes == ["query"]


def test_library_read_that_failed_is_a_query_failure_not_a_music_failure():
    first = Answer(Command(actions=(_library("search", query="кот"),)), "deepseek", follow=_follow)
    second = Answer(Command(speech="В библиотеке не получилось найти."), "deepseek")
    ports = Ports([first, second], {"music_library": ("rejected", "error", {})})
    _run(_engine(ports), "найди в библиотеке кота")
    assert phrases.text("media.rejected") not in ports.said
    assert json.loads(ports.asked[1][0][-1]["content"])[kn.QUERY_RESULTS_FIELD][0]["status"] == "rejected"


@pytest.mark.parametrize("call, data, want", [
    (_library("rate", verdict="like"), {"library_id": "t-1", "title": "Кот и ёж", "rating": 1},
     "Запомнил: «Кот и ёж» нравится."),
    (_library("rate", verdict="dislike", track_id="t-2"), {"library_id": "t-2", "title": "Дождь", "rating": -1},
     "Запомнил: «Дождь» не нравится, больше не включу."),
    (_library("update", track_id="t-1", title="Кот"), {"id": "t-1", "title": "Кот"},
     "Обновил данные трека «Кот»."),
    (_library("delete", track_id="t-1"), {}, "Удалил этот трек из библиотеки."),
])
def test_library_edits_say_their_own_phrase_from_the_result(call, data, want):
    """n710: удаление говорило «Включаю музыку.»; теперь у rate/update/delete свои фразы ``music.library_*``."""
    ports = Ports([Answer(Command(actions=(call,)), "deepseek")], {"music_library": ("done", None, data)})
    agg = _run(_engine(ports), "отметь трек")
    assert ports.said == [want] and ports.asked and len(ports.asked) == 1
    assert agg.llm_calls == 1 and ports.stage_classes == ["action.local"]


def test_library_edit_that_failed_says_library_failure():
    ports = Ports([Answer(Command(actions=(_library("delete", track_id="t-9"),)), "deepseek")],
                  {"music_library": ("rejected", "нет трека", {})})
    _run(_engine(ports), "удали трек")
    assert ports.said == [phrases.text("music.library_fail")]


# ---------------------------------------------------------------------------------------------------------------
# #4025, #4026: отказ тула ПО ПРИЧИНЕ (reason в данных, PR #4056) говорит своей фразой, а не общим «не вышло»
# ---------------------------------------------------------------------------------------------------------------

def _refusal(tool, cls, reason, **data):
    return ActionResult("t1/0", 0, "personality", tool, cls, "rejected", reason, data)


@pytest.mark.parametrize("title, want", [
    ("Кот и ёж", "Удалить «Кот и ёж»? Скажи «да, удаляй»."),
    ("", "Удалить этот трек? Скажи «да, удаляй»."),
])
def test_library_delete_without_confirm_asks_with_the_track_title(title, want):
    """n710 (марафон 23.10): необратимое удаление теперь идёт в два вызова. Первый тул только спрашивает
    (``needs_confirmation``, ``data.title``) - человек слышит вопрос с названием трека, а не «С библиотекой не вышло».
    Текст вопроса - фраза кода по причине и названию (#4062: тул его в данных больше не шлёт); ``want`` - литерал, а не
    сборка из того же каталога: правка ``ru.yaml`` видна здесь, совпадение с текстом тула держит тест тула."""
    data = {"track_id": "t-1", "title": title}  # данные отказа как у тула (gen_library._delete)
    ports = Ports([Answer(Command(actions=(_library("delete", track_id="t-1"),)), "deepseek")],
                  {"music_library": ("rejected", "needs_confirmation", data)})
    agg = _run(_engine(ports), "удали эту песню про енота")
    assert ports.said == [want]
    assert phrases.text("music.library_fail") not in ports.said
    assert agg.llm_calls == 1 and ports.stage_classes == ["action.local"], "вопрос говорит код, LLM за ним не зовётся"


@pytest.mark.parametrize("call", [_library("delete", track_id="x1"), _library("update", track_id="x1", title="Кот")])
def test_library_edit_of_an_unnamed_track_says_it_does_not_know_that_track(call):
    """Тул отверг id, которого не называл (``unknown_track``): фраза о незнакомом треке и что делать дальше."""
    ports = Ports([Answer(Command(actions=(call,)), "deepseek")],
                  {"music_library": ("rejected", "unknown_track", {"track_id": "x1"})})
    _run(_engine(ports), "удали вон тот трек")
    assert len(ports.said) == 1 and ports.said[0].startswith("Не знаю такого трека в библиотеке.")
    assert "список" in ports.said[0] and ports.said != [phrases.text("music.library_fail")]


def test_library_edit_refused_for_an_unlisted_reason_keeps_the_general_failure():
    """Причина, которой у каталога нет, не ломает фразу: общая ``music.library_fail`` (как было)."""
    ports = Ports([Answer(Command(actions=(_library("delete", track_id="t-9"),)), "deepseek")],
                  {"music_library": ("rejected", "disk_on_fire", {"track_id": "t-9"})})
    _run(_engine(ports), "удали трек")
    assert ports.said == [phrases.text("music.library_fail")]


def test_library_info_of_an_unnamed_track_is_a_query_refusal_with_the_same_words():
    """``info`` - чтение (``query``): данные идут LLM, код говорит только когда ответ модели не сложился
    (``query_fallback``) - и говорит ту же фразу про незнакомый трек, а не «Не нашёл ответа.»."""
    from rob_box_dialog.respond import phrase_from_result
    from rob_box_dialog.turn import query_fallback

    refused = _refusal("music_library", "query", "unknown_track", action="info", track_id="x1")
    unknown = phrases.text("music.library_fail.unknown_track")
    assert phrase_from_result(refused).text == unknown == query_fallback([refused])
    assert unknown != phrases.text("query.rejected")


def test_search_web_that_could_not_search_says_so_and_the_model_gets_the_reason():
    """n907/n908: поиск недоступен (оба бэкенда отказали) - не «ничего не нашёл». Фраза кода по ``search_unavailable``
    (``query_fallback``); модель в данных запроса видит ту же причину."""
    from rob_box_dialog.respond import phrase_from_result
    from rob_box_dialog.turn import query_fallback

    want = "Не смог проверить в интернете: поиск сейчас не ответил."
    refused = _refusal("search_web", "query", "search_unavailable", query="погода", tried=["ddgs", "deepseek"])
    said = phrase_from_result(refused)
    assert said.text == want == query_fallback([refused]) and said.emotion == "sad"
    other = _refusal("search_web", "query", "error", query="погода")
    assert phrase_from_result(other).text == phrases.text("query.rejected"), "другая причина - прежнее «Не нашёл»"

    first = Answer(Command(actions=(ToolCall("search_web", {"query": "погода"}),)), "deepseek", follow=_follow)
    second = Answer(Command(speech="Поиск сейчас не отвечает."), "deepseek")
    ports = Ports([first, second], {"search_web": ("rejected", "search_unavailable", {"tried": ["ddgs", "deepseek"]})})
    _run(_engine(ports), "какая завтра погода")
    row = json.loads(ports.asked[1][0][-1]["content"])[kn.QUERY_RESULTS_FIELD][0]
    assert (row["tool"], row["status"], row["reason"]) == ("search_web", "rejected", "search_unavailable")
    assert ports.said == ["Поиск сейчас не отвечает."]


def test_library_play_stays_media_and_is_the_only_one_who_says_playing():
    ports = Ports([Answer(Command(actions=(_library("play", track_id="t-1"),)), "deepseek")])
    _run(_engine(ports), "включи трек из библиотеки")
    assert ports.said == [phrases.text("media.done")] and ports.stage_classes == ["action.media"]
    failed = Ports([Answer(Command(actions=(_library("play", track_id="t-1"),)), "deepseek")],
                   {"music_library": ("rejected", "error", {})})
    _run(_engine(failed), "включи трек из библиотеки")
    assert failed.said == [phrases.text("media.rejected")]


def test_late_library_read_is_not_announced_as_music_that_started():
    """Итог пришёл после дедлайна: поиск молчит, а запуск трека библиотеки по-прежнему «Включаю музыку.»."""
    ports = Ports()
    engine = _engine(ports)
    late = ActionResult("t1/0", 0, "personality", "music_library", "action.media", "done", None,
                        {"action": "search", "tracks": TRACKS})
    say_late(engine, late)
    assert ports.said == []
    say_late(engine, ActionResult("t1/0", 0, "personality", "music_library", "action.media", "done", None,
                                  {"action": "play", "track_id": "t-1"}))
    assert ports.said == [phrases.text("media.done")]


# ---------------------------------------------------------------------------------------------------------------
# #4020: stop_music - «Выключил музыку.» только если было что выключать
# ---------------------------------------------------------------------------------------------------------------

def _stop(was_playing):
    data = {} if was_playing is None else {"was_playing": was_playing}
    ports = Ports([Answer(Command(actions=(ToolCall("stop_music", {}),)), "deepseek")],
                  {"stop_music": ("done", None, data)})
    _run(_engine(ports), "выключи музыку")
    return ports.said


def test_stop_music_with_nothing_playing_says_nothing_is_playing():
    """Живой прогон 26.10 20:57:50: ``[music v2] stop track_id=None`` → «Выключил музыку.» - ложь."""
    assert _stop(False) == [phrases.text("music.stop_idle")]
    assert phrases.text("music.stop_idle") != phrases.text("music.stop_ok")


def test_stop_music_that_stopped_a_track_says_it_is_off():
    assert _stop(True) == [phrases.text("music.stop_ok")]
    assert _stop(None) == [phrases.text("music.stop_ok")]  # тул не сообщил - как раньше


# ---------------------------------------------------------------------------------------------------------------
# #4022: «говори нормально» - Tier-1, два тула, фраза из результатов
# ---------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("text", [
    "Робот, говори нормально", "Робот, говори с обычной скоростью", "Робот, говори как обычно",
    "Робот, говори нормальным темпом",
])
def test_speak_normally_resets_speed_and_pitch_by_code_without_llm(text):
    """26.10 20:58:30: LLM сказала «Возвращаю обычный темп» и не вызвала тул - скорость осталась 0.8."""
    done = ("done", None, {"provider": "minimax", "status": "done"})
    ports = Ports(outcomes={"set_speed": done, "set_pitch": done})
    agg = _run(_engine(ports), text)
    assert ports.asked == [] and agg.llm_calls == 0
    assert ports.executed == [[("set_speed", {"action": "normal"}), ("set_pitch", {"action": "normal"})]]
    assert ports.said == [phrases.text(kn.PROSODY_NORMAL_PHRASE)]
    assert ports.stage_classes == ["action.local"]


def test_speak_normally_tells_which_reset_did_not_happen():
    """Скорость у провайдера не регулируется (``speed_unsupported``), тон встал - так и говорим, а не «Готово.»."""
    ports = Ports(outcomes={
        "set_speed": ("rejected", "speed_unsupported", {"provider": "silero", "status": "rejected"}),
        "set_pitch": ("done", None, {"provider": "silero", "status": "done"}),
    })
    _run(_engine(ports), "Робот, говори нормально")
    assert ports.said == [f"{phrases.text('local.rejected.speed_unsupported')} {phrases.text('pitch.normal')}"]


def test_speak_normally_when_both_resets_are_rejected_says_both_failures():
    ports = Ports(outcomes={
        "set_speed": ("rejected", "speed_unsupported", {"status": "rejected"}),
        "set_pitch": ("rejected", "pitch_unsupported", {"status": "rejected"}),
    })
    _run(_engine(ports), "Робот, говори нормально")
    assert ports.said == [f"{phrases.text('local.rejected.speed_unsupported')} "
                          f"{phrases.text('local.rejected.pitch_unsupported')}"]


def test_ordinary_voice_phrase_is_still_set_voice_not_a_pace_reset():
    """«Обычным голосом» - голос робота (#3803), а не скорость; режим тона («басом», #4074, решение Шифу 27.10) он
    тоже снимает: к ``set_voice`` default - сброс тона, скорость не трогается."""
    ports = Ports(outcomes={"set_voice": ("done", None, {})})
    _run(_engine(ports), "Робот, говори обычным голосом")
    assert ports.executed == [[("set_voice", {"voice": kn.VOICE_DEFAULT_ARG}), ("set_pitch", {"action": "normal"})]]
    assert ports.asked == []


# ---------------------------------------------------------------------------------------------------------------
# #4022: отказ set_speed / set_pitch - своя фраза
# ---------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("tool, action, reason", [
    ("set_speed", "slower", "speed_unsupported"), ("set_pitch", "higher", "pitch_unsupported"),
])
def test_unsupported_prosody_is_said_with_its_own_phrase(tool, action, reason):
    """Путь LLM: реплика, которую грамматика шага голоса не покрывает («говори помедленнее» с #4074 - Tier-1)."""
    ports = Ports([Answer(Command(actions=(ToolCall(tool, {"action": action}),)), "deepseek")],
                  {tool: ("rejected", reason, {"status": "rejected", "provider": "silero"})})
    _run(_engine(ports), "сделай голос поинтереснее")
    assert ports.said == [phrases.text(f"local.rejected.{reason}")]
    assert ports.said != [phrases.text("local.rejected")]


# ---------------------------------------------------------------------------------------------------------------
# #4031: звукоподражание к тихим звукам
# ---------------------------------------------------------------------------------------------------------------

def _sound(sound, status="done", reason=None):
    call = ToolCall("play_sound", {"sound": sound})
    ports = Ports([Answer(Command(actions=(call,)), "deepseek")],
                  {"play_sound": (status, reason, {"sound": sound, "status": status, "duration": 2.0})})
    _run(_engine(ports), "включи звук")
    return ports.said


def test_siren_is_imitated_when_it_played():
    """«Робот, включи сирену» → звук каталога ``robot_alert`` играет, а код говорит «Виу-виу!»."""
    assert _sound("robot_alert") == ["Виу-виу!"]
    assert _sound("ui_bell") == ["Дзынь-дзынь!"]


def test_beep_stays_quiet():
    assert _sound("robot_confirm") == [] and _sound("robot_talk_beep_1") == []


def test_imitation_is_not_said_when_the_sound_did_not_play():
    assert _sound("robot_alert", "rejected", "busy") == [phrases.text("sound.skipped.busy", sound="robot_alert")]


# ---------------------------------------------------------------------------------------------------------------
# Таблицы знания: класс вызова и ключ фразы по данным
# ---------------------------------------------------------------------------------------------------------------

def test_call_class_follows_the_library_action_and_falls_back_to_the_tool_class():
    by_action = {a: kn.call_class("music_library", {"action": a})
                 for a in ("list", "search", "info", "rate", "update", "delete", "play")}
    assert by_action == {"list": "query", "search": "query", "info": "query", "rate": "action.local",
                         "update": "action.local", "delete": "action.local", "play": "action.media"}
    assert kn.call_class("music_library", {}) == kn.TOOL_CLASSES["music_library"] == "action.media"
    assert kn.call_class("stop_music", {"action": "list"}) == kn.TOOL_CLASSES["stop_music"]


def test_library_tables_name_real_catalog_actions():
    """Класс и фразы библиотеки сверены с enum ``action`` схемы тула: опечатки и новое действие без строки видны."""
    from rob_box_core.tool_catalog import get_tool

    props = get_tool("music_library").parameters["properties"]
    actions, verdicts = set(props["action"]["enum"]), set(props["verdict"]["enum"]) - {""}
    classes = kn.CLASS_BY_ARG["music_library"][1]
    assert set(classes) <= actions and actions - set(classes) == {"play"}  # не в таблице - только запуск (класс тула)
    done, rejected = (kn.TOOL_PHRASES["music_library"][s] for s in ("done", "rejected"))
    assert {k[0] for k in done.keys} <= actions and {k[1] for k in done.keys} <= verdicts | {None}
    assert {k[0] for k in rejected.keys} <= actions and "play" not in {k[0] for k in rejected.keys}
    # правки - action.local, и у каждой есть своя фраза успеха
    assert {a for a, cls in classes.items() if cls == "action.local"} == {k[0] for k in done.keys}


def test_by_data_reads_empty_value_as_missing_and_matches_false():
    table = kn.ByData(("was_playing",), {(False,): "a"}, "b")
    assert table.key({"was_playing": False}) == "a"
    assert table.key({"was_playing": True}) == table.key({}) == "b"
    none_default = kn.ByData(("action", "verdict"), {("delete", None): "c"})
    assert none_default.key({"action": "delete", "verdict": ""}) == none_default.key({"action": "delete"}) == "c"
    assert none_default.key({"action": "play"}) is None
    assert none_default.all_keys() == ("c",) and table.all_keys() == ("a", "b")


def test_sound_imitations_are_short_funny_phrases_of_the_catalog():
    assert kn.SOUND_IMITATIONS["robot_alert"] == "sound.imitation.siren"
    assert phrases.text(kn.SOUND_IMITATIONS["robot_alert"]) == "Виу-виу!"
    assert phrases.text(kn.SOUND_IMITATIONS["ui_bell"]) == "Дзынь-дзынь!"
    for key in kn.SOUND_IMITATIONS.values():
        assert key in phrases.catalog() and len(phrases.text(key)) <= 20, key
    assert kn.SOUND_TOOL in kn.SILENT_DONE_TOOLS
