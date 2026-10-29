"""Речь о действии или состоянии музыки без тула, что его исполняет (ADR-0148: фразу о состоянии строит код).

Марафон-3 28.10, акт 7, n709 («Робот, останови, борис уже уснул под нее»): ясный стоп (Tier-1 ``stop_all``) выключил
музыку кодом, остаток фразы «борис уже уснул под нее» ушёл модели, и та ответила «Раз Борис уснул — сделаю потише, чтобы
не разбудить. Енот-механик пусть дослушивает шёпотом.» без тула. Две причины: (1) остаток шёл тем же ходом, а окно
исключает ход, что спрашивает модель, - стоп-действие этого хода в снимке не было, и снимок плеера приходит позже стопа
(``music_state`` ещё «играет»); (2) валидатора речи не было. Здесь: что видит модель в остатке и что судит валидатор.
"""

import asyncio
import json

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Invalid, ToolCall, explain, parse, unkept_promises, validate
from rob_box_dialog.music_state import after_stop
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.turn import Answer, TurnEngine, music_promise_key
from rob_box_dialog.words import music_promises

#: n709 марафона-5 (29.10, акт 7), ход 3cf2a59b, дословно из лога: verdict=speech actions=[] speech_dropped=0
N709_M5 = "Тихо тогда, Саша. Не буду будить Бориса — музыка остановлена, дальше шёпотом."
N604 = "Понял, Саша: сворачиваю в рок."
N709 = "Раз Борис уснул — сделаю потише, чтобы не разбудить. Енот-механик пусть дослушивает шёпотом."
TOOLS = ("stop_music", "set_music_volume", "set_volume", "request_music", "dj_set", "music_library", "edit_music",
         "play_animation", "generate_music")


# ---------------------------------------------------------------------------------------------------------------
# слова: какие фразы - обещание действия с музыкой
# ---------------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("text, kinds", [
    (N709, ("volume",)),                                         # сам n709
    ("Сделаю потише.", ("volume",)),
    ("Сделаю музыку тише, не буду будить.", ("volume",)),
    ("Тише сделаю, не переживай.", ("volume",)),                 # порядок слов не важен
    ("Убавлю, чтобы не разбудить.", ("volume",)),
    ("Приглушу немного.", ("volume",)),
    ("Сейчас сделаю погромче!", ("volume",)),
    ("Выключу музыку, раз он спит.", ("stop",)),
    ("Останавливаю трек.", ("stop",)),
    ("Включу музыку.", ("start",)),
    ("Поставлю что-нибудь спокойное, песню про енота.", ("start",)),
    ("Включу музыку, а потом сделаю потише.", ("volume", "start")),
    (N709_M5, ("whisper",)),                                      # n709 марафона-5 дословно
    ("Пусть музыка играет дальше шёпотом.", ("whisper",)),
    ("Песня теперь шёпотом, чтобы не будить.", ("whisper",)),
    ("Борис спит, дослушаем потише.", ("volume",)),
    ("Енот дослушивает шёпотом.", ("volume",)),
    (N604, ("switch",)),                                          # n604 марафона-4 дословно
    ("Плавно уведу в сторону джаза.", ("switch",)),
    ("Переключаю на техно.", ("switch",)),
    ("Сменю стиль на что-нибудь потяжелее.", ("switch",)),
])
def test_promises_of_an_action_with_the_music_are_found_by_words(text, kinds):
    assert set(music_promises(text)) == set(kinds)


@pytest.mark.parametrize("text", [
    "Музыку выключил.",                       # прошедшее - после кодового стопа это правда
    "Выключил музыку, как просили.",
    "Борис уснул под колыбельную, хорошо.",
    "Включить музыку?",                       # вопрос, инфинитив
    "Сделаю тебе чай, если хочешь.",          # «сделаю» без громкости
    "Сделаю потяжелее, как просил.",          # правка бита - не громкость
    "Поставлю будильник на семь.",            # «поставлю» без слов о музыке
    "Останавливаю движение.",                 # «останавливаю» не о музыке
    "Музыка сейчас не играет.",
    "Музыка остановлена, скажу шёпотом.",     # шёпотом будет речь, не музыка
    "Скажу шёпотом, чтобы не разбудить.",     # шёпотом без слова о музыке
    "Музыка играет, расскажу тебе шёпотом секрет.",
    "Дослушаем песню до конца.",              # дослушать - не про громкость
    "Борис дослушал песню и уснул.",          # прошедшее
    "Свернул в рок, как просили.",            # прошедшее
    "Переключить на джаз?",                   # вопрос, инфинитив
    "Сверну налево, к кухне.",                # поворот, не музыка
    "Переключаю свет.",
    "Сменю тему, попробуем другое.",          # «поп» в «попробуем» - не стиль
    "Битва была жаркой, выключу свет.",       # «бит» в «битве» и «выключу» не про музыку
    "",
])
def test_other_phrases_are_not_promises(text):
    assert music_promises(text) == ()


def test_the_promise_words_live_in_one_table():
    """Слова-обещания - одно знание (``knowledge.MUSIC_PROMISES``): ни одного регулярного выражения в самом поиске."""
    assert set(kn.MUSIC_PROMISES) == {"volume", "whisper", "stop", "start", "switch"}
    assert set(kn.MUSIC_PROMISES["volume"].tools) == {"set_volume", "set_music_volume"}
    assert set(kn.MUSIC_PROMISES["stop"].tools) == set(kn.MUSIC_STOP_TOOLS)
    assert all(word == word.lower() and "ё" not in word and " " not in word
               for p in kn.MUSIC_PROMISES.values() for word in (*p.alone, *p.with_near))


# ---------------------------------------------------------------------------------------------------------------
# валидатор команды: обещание без тула - нарушение, с тулом - нет
# ---------------------------------------------------------------------------------------------------------------


def test_the_n709_promise_without_a_tool_is_a_violation_with_a_hint():
    result = validate(parse({"speech": N709}), TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_MUSIC_PROMISE
    text = explain(result)
    assert kn.VIOLATION_MUSIC_PROMISE in text and kn.MUSIC_PROMISE_HINT in text


@pytest.mark.parametrize("speech", ["Сделаю тише.", "Выключу музыку.", "Включу музыку.", "Убавлю."])
def test_every_kind_of_promise_without_a_tool_is_a_violation(speech):
    result = validate(parse({"speech": speech}), TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_MUSIC_PROMISE


def test_the_n709_m5_whisper_promise_without_a_tool_is_a_violation():
    result = validate(parse({"speech": N709_M5}), TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_MUSIC_PROMISE
    assert unkept_promises(parse({"speech": N709_M5})) == ("whisper",)


@pytest.mark.parametrize("tool, args", [("set_music_volume", {"action": "quieter"}), ("set_volume", {"action": "quieter"})])
def test_the_whisper_promise_with_the_volume_tool_is_kept(tool, args):
    cmd = parse({"speech": N709_M5, "actions": [{"tool": tool, "args": args}]})
    assert unkept_promises(cmd) == ()
    result = validate(cmd, TOOLS)
    assert kn.VIOLATION_MUSIC_PROMISE not in {v.reason for v in getattr(result, "all", ())}


def test_speech_whisper_without_music_is_not_a_violation():
    cmd = validate(parse({"speech": "Скажу шёпотом, Борис спит."}), TOOLS)
    assert isinstance(cmd, Command)


def test_the_promise_with_its_tool_is_kept_by_the_command():
    """«Сделаю потише» + ``set_music_volume``: обещание держит команда. Речь рядом с громким тулом ловит свой валидатор
    (``speech_with_actions``: она не звучит) - но не как пустое обещание."""
    cmd = parse({"speech": "Сделаю потише.", "actions": [{"tool": "set_music_volume", "args": {"action": "quieter"}}]})
    assert unkept_promises(cmd) == ()
    result = validate(cmd, TOOLS)
    assert isinstance(result, Invalid) and kn.VIOLATION_MUSIC_PROMISE not in {v.reason for v in result.all}


def test_the_tool_without_words_is_a_valid_command():
    cmd = validate(parse({"actions": [{"tool": "set_music_volume", "args": {"action": "quieter"}}]}), TOOLS)
    assert isinstance(cmd, Command) and [a.tool for a in cmd.actions] == ["set_music_volume"]


def test_the_n604_switch_promise_without_a_tool_is_a_violation():
    result = validate(parse({"speech": N604}), TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_MUSIC_PROMISE
    assert unkept_promises(parse({"speech": N604})) == ("switch",)


@pytest.mark.parametrize("tool, args", [
    ("dj_set", {"action": "start", "theme": "рок"}),
    ("request_music", {"intent": "track", "text": "рок", "genre": "rock"}),
    ("edit_music", {"change": "heavier"}),
])
def test_the_switch_promise_with_its_tool_is_kept(tool, args):
    cmd = parse({"speech": N604, "actions": [{"tool": tool, "args": args}]})
    assert unkept_promises(cmd) == ()
    result = validate(cmd, TOOLS)
    assert kn.VIOLATION_MUSIC_PROMISE not in {v.reason for v in getattr(result, "all", ())}


def test_a_tool_of_another_kind_does_not_keep_the_promise():
    """«Включу музыку» + ``stop_music``: стоп не исполняет включение - обещание пустое."""
    cmd = Command(speech="Включу музыку.", actions=(ToolCall("stop_music", {}),))
    assert unkept_promises(cmd) == ("start",)


def test_the_past_tense_after_a_code_stop_is_not_a_violation():
    """«Музыку выключил» после кодового стопа - правда, а не обещание: валидатор её не трогает."""
    cmd = validate(parse({"speech": "Музыку выключил, Борис спит."}), TOOLS)
    assert isinstance(cmd, Command) and cmd.speech == "Музыку выключил, Борис спит."


def test_the_phrases_after_the_exhausted_retry_are_in_the_catalog_and_are_not_promises():
    assert set(kn.MUSIC_PROMISE_PHRASES) == {"stopped", "silent", "playing"}
    for key in kn.MUSIC_PROMISE_PHRASES.values():
        assert key in kn.PHRASE_KEYS
        said = phrases.text(key)
        assert said and music_promises(said) == ()


# ---------------------------------------------------------------------------------------------------------------
# фраза после неудавшегося повтора: правда по состоянию, строит код (решение Шифу по п.1, ADR-0148)
# ---------------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("stopped, playing, said", [
    (True, False, "Музыку я уже выключил."),                                # ход сам выключил музыку (n709)
    (True, True, "Музыку я уже выключил."),                                 # снимок плеера ещё прежний - стоп хода сильнее
    (False, False, "Сейчас музыка не играет."),
    (False, True, "Сделать с музыкой ничего не получилось — скажи ещё раз, что нужно."),
])
def test_the_phrase_of_a_broken_music_promise_is_the_truth_about_the_state(stopped, playing, said):
    assert phrases.text(music_promise_key(stopped, playing)) == said


# ---------------------------------------------------------------------------------------------------------------
# снимок остатка после стопа: музыка УЖЕ остановлена - событие стопа в снимке и в разделе music_state
# ---------------------------------------------------------------------------------------------------------------


class Ports:
    """Транспорт хода: музыка играет, пока её не погасит стоп-тул; модель отвечает заготовкой, запросы - в ``asked``."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.asked = []
        self.executed = []
        self.said = []
        self.playing = True

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, call in enumerate(actions):
            data = dict(call.args)
            if call.tool in kn.MUSIC_STOP_TOOLS:
                data["was_playing"], self.playing = self.playing, False
            self.executed.append(call.tool)
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, call.tool,
                                    kn.TOOL_CLASSES[call.tool], "done", None, data))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


#: Раздел плеера, как его собирает хост из СТАРОГО снимка: стоп только что исполнен, новый снимок ещё не пришёл.
STALE_MUSIC = {"playing": "yes", "track": "Колыбельная енота", "repeat": "loop", "dj": "off"}


def _context(messages):
    """Снимок хода (``<system_context>``) - последнее сообщение запроса: разделы - JSON по строкам."""
    return messages[-1]["content"]


def _run(ports, text, music="playing"):
    ids = iter(f"t{n}" for n in range(1, 100))
    session = Session("personality")
    session.set_music(MusicStateRef(music))
    engine = TurnEngine(session, ports, system_prompt="Ты робот.", new_id=lambda: next(ids),
                        sections=lambda _snap, _text: {"music_state": dict(STALE_MUSIC)})
    asyncio.run(engine.run(Addressed(text, VOICE)))
    return engine


def test_the_remainder_after_a_clear_stop_sees_the_stop_of_its_own_turn():
    """n709: стоп музыки исполнил код этого же хода - остаток фразы идёт модели со стопом в ``recent_actions`` и с
    ``music_state`` «не играет», хотя снимок плеера ещё прежний."""
    ports = Ports([Answer(Command(speech="Спокойной ночи, Борис."), "deepseek")])
    _run(ports, "Робот, останови, борис уже уснул под нее")
    assert ports.executed == ["stop_music"] and len(ports.asked) == 1
    context = _context(ports.asked[0])
    assert '"playing":"no"' in context and '"playing":"yes"' not in context
    assert '"last_track":"Колыбельная енота"' in context and '"last_ended":"stopped"' in context
    actions = _recent_actions(context)
    assert [(a["tool"], a["status"], a.get(kn.THIS_TURN_FIELD)) for a in actions] == [("stop_music", "done", True)]
    assert actions[0]["data"]["was_playing"] is True


PROMISE = Invalid(kn.VIOLATION_MUSIC_PROMISE, "speech обещает volume без тула")


def test_n709_after_the_retry_the_code_says_the_music_is_already_off():
    """Остаток после стопа, модель и после повтора обещает «потише» (невалидная команда): звучит правда кода."""
    ports = Ports([Answer(PROMISE, "deepseek")])
    _run(ports, "Робот, останови, борис уже уснул под нее")
    assert ports.executed == ["stop_music"]
    assert ports.said[-1] == "Музыку я уже выключил." and N709 not in ports.said


def test_without_a_stop_of_the_turn_the_code_says_the_state_of_the_player():
    ports = Ports([Answer(PROMISE, "deepseek")])
    _run(ports, "Робот, борис уже уснул под нее")  # музыка играет по снимку сессии, стопа в ходе нет
    assert ports.executed == [] and ports.said == [phrases.text(kn.MUSIC_PROMISE_PHRASES["playing"])]
    quiet = Ports([Answer(PROMISE, "deepseek")])
    _run(quiet, "Робот, борис уже уснул под нее", music="idle")
    assert quiet.said == ["Сейчас музыка не играет."]


def test_a_plain_turn_without_a_stop_keeps_the_player_section_and_has_no_own_actions():
    ports = Ports([Answer(Command(speech="Ага."), "deepseek")])
    _run(ports, "Робот, борис уже уснул под нее")
    context = _context(ports.asked[0])
    assert '"playing":"yes"' in context and kn.THIS_TURN_FIELD not in context


def _recent_actions(context):
    body = context.split("<recent_actions>", 1)[1].split("</recent_actions>", 1)[0]
    return json.loads(body)


# ---------------------------------------------------------------------------------------------------------------
# music_state.after_stop: данные владельца плеера, переписанные событием стопа
# ---------------------------------------------------------------------------------------------------------------


def test_after_stop_turns_playing_into_a_just_stopped_track():
    section = {"playing": "yes", "track": "Колыбельная", "repeat": "once", "ends_in_s": "40", "dj": "on",
               "set_theme": "космос", "melody": "Луна", "bpm": "120"}
    assert after_stop(section) == {"playing": "no", "dj": "on", "last_track": "Колыбельная", "last_ended": "stopped",
                                   "last_ended_ago_s": "0"}


def test_after_stop_leaves_a_silent_or_unknown_section_as_it_is():
    silent = {"playing": "no", "dj": "off", "last_track": "X", "last_ended": "finished", "last_ended_ago_s": "30"}
    assert after_stop(silent) == silent
    assert after_stop({"playing": "unknown", "dj": "unknown"}) == {"playing": "unknown", "dj": "unknown"}
    assert after_stop(None) == {}
