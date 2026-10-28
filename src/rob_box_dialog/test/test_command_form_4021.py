"""#4021 (марафон 23.10, разбор 26.10): форма команды хода — решает код, а не правило в промпте (ADR-0148 п.2).

* n813 «Робот, тишина?» → DeepSeek ``{passed: not_addressed}`` → «Хм, я задумался…»: на адресованную реплику в схеме
  нет ``passed`` (только подхват бэклога), прислала — нарушение моста и повтор;
* n109 вопрос про звуки → DeepSeek сам добавил ``voice: female-shaonv``: голос реплики — только по просьбе о голосе;
* n105 «Остально<U+FFFD×2> в порядке» ушло в TTS: U+FFFD вырезается разбором;
* ревью #4041: одна ``expression`` с одним earcon на вопрос — тоже «ответ без слов»; фраза кода после повтора
  ``silent_answer`` — без ``sad``.
"""

import asyncio

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Busy, Command, Expression, Form, Invalid, parse, validate
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.tools_view import needs_words, voice_asked
from rob_box_dialog.turn import Answer, TurnEngine, command_form, invalid_emotion

ALL = [t.name for t in TOOL_CATALOG]
ADDRESSED = Form(passed=False, voice=False)  # адресованная реплика без просьбы о голосе
#: Форма, которую ход строит из такой реплики (``command_form``): музыку в ней не просили - подложка сказки, анекдота и
#: стиха закрыта (#4152); ``ADDRESSED`` - форма, поданная валидатору «вручную», подложку не трогает.
ADDRESSED_TURN = Form(passed=False, voice=False, backing=False)
N813 = "тишина"  # текст хода n813 после вейка (22:55:19.87 STT «робот тишина»)
N109 = "а какие звуки ты умеешь играть"  # вопрос про звуки, голос не просили
N105_SPEECH = "Остально\ufffd\ufffd в порядке"  # 21:11:01.576


# -- форма: схема хода ---------------------------------------------------------------------------------------


def test_n813_addressed_turn_schema_has_no_passed_backlog_pickup_keeps_it():
    full = kn.command_schema("json")
    addressed = cm.form_schema(full, ADDRESSED)
    assert "passed" not in addressed["properties"] and "voice" not in addressed["properties"]
    assert set(full["properties"]) - set(addressed["properties"]) == set(kn.FORM_FIELDS)
    pickup = cm.form_schema(full, Form(passed=True, voice=False))
    assert pickup["properties"]["passed"] == full["properties"]["passed"]
    assert cm.form_schema(full, Form()) == full, "вне хода (v1, тесты) форма открыта: схема как была"
    assert "passed" in full["properties"], "полную схему форма не портит (копия)"


def test_prompt_fragment_has_no_lines_about_fields_the_turn_schema_lacks():
    text = kn.prompt_fragment(cm.form_schema(kn.command_schema("json"), ADDRESSED))
    assert "- passed" not in text and "- voice" not in text and "- speech" in text and "- expression" in text
    assert kn.prompt_fragment() == kn.prompt_fragment(kn.command_schema("json"))
    assert "- passed ∈" in kn.prompt_fragment() and "- voice:" in kn.prompt_fragment()


def test_form_is_the_turn_context_and_is_reset_after_the_turn():
    assert cm.current_form() == Form()
    with cm.turn_form(ADDRESSED):
        assert cm.current_form() == ADDRESSED
        assert "passed" not in cm.form_schema(kn.command_schema("json"))["properties"]
    assert cm.current_form() == Form()


# -- форма: валидатор (мост повторяет по нарушению) -------------------------------------------------------------


def test_n813_passed_on_addressed_turn_is_a_violation_with_words_for_the_model():
    res = validate(parse({"passed": "not_addressed"}), ALL, form=ADDRESSED)
    assert isinstance(res, Invalid) and res.reason == kn.VIOLATION_PASSED_ADDRESSED
    note = cm.explain(res)
    assert kn.VIOLATION_PASSED_ADDRESSED in note and cm.HINTS[kn.VIOLATION_PASSED_ADDRESSED] in note
    with cm.turn_form(ADDRESSED):  # мост зовёт validate без формы — берётся форма хода
        assert validate(parse({"passed": "nothing_to_say"}), ALL).reason == kn.VIOLATION_PASSED_ADDRESSED


def test_passed_from_backlog_pickup_stays_honest_silence():
    for form in (Form(passed=True, voice=False), Form()):
        assert validate(parse({"passed": "not_addressed"}), ALL, form=form).passed == "not_addressed"
    assert validate(parse({"passed": "nope"}), ALL, form=Form()).reason == "bad_enum"


def test_n109_unasked_voice_is_dropped_not_spoken_and_not_a_violation():
    raw = {"speech": "Умею сирену, звонок и бульканье.", "voice": "female-shaonv"}
    res = validate(parse(raw), ALL, form=ADDRESSED)
    assert isinstance(res, Command) and res.voice is None and res.speech == raw["speech"]
    assert "voice" in res.dropped
    asked = validate(parse(raw), ALL, form=Form(passed=False, voice=True))
    assert asked.voice == "female-shaonv" and "voice" not in asked.dropped


def test_asked_voice_outside_the_sounding_catalogue_is_a_violation_with_the_list():
    busy = Busy(voices=("alena", "zahar"))
    res = validate(parse({"speech": "Привет", "voice": "female-shaonv"}), ALL, busy=busy, form=Form())
    assert isinstance(res, Invalid) and res.reason == "bad_enum" and res.all[0].allowed == busy.voices
    assert validate(parse({"speech": "Привет", "voice": "alena"}), ALL, busy=busy, form=Form()).voice == "alena"
    unasked = validate(parse({"speech": "Привет", "voice": "female-shaonv"}), ALL, busy=busy, form=ADDRESSED)
    assert isinstance(unasked, Command) and unasked.voice is None, "не просили — отброшен, а не нарушение"


def test_voice_catalogue_goes_into_the_line_voice_enum_too():
    schema = kn.with_voices(kn.command_schema("json"), ("zahar", "alena"))
    assert schema["properties"]["voice"]["enum"] == ["", "alena", "zahar"]
    assert "enum" not in kn.command_schema("json")["properties"]["voice"]


# -- голос просили? --------------------------------------------------------------------------------------------


@pytest.mark.parametrize("text", ["скажи это голосом алёны", "переключись на яндекс", "говори басом",
                                  "говори помедленнее"])
def test_voice_asked_by_the_voice_group_words(text):
    assert voice_asked(Session("personality").snapshot, text)


@pytest.mark.parametrize("text", [N109, N813, "расскажи анекдот", "какие у тебя есть звуки"])
def test_voice_not_asked(text):
    assert not voice_asked(Session("personality").snapshot, text)


def test_voice_named_from_the_registry_opens_the_voice_and_pace_under_music_does_not():
    snap = Session("personality").snapshot
    assert voice_asked(snap, "скажи как female-shaonv", ("female-shaonv",))
    assert not voice_asked(snap, "скажи как диктор", ("female-shaonv",))
    music = Session("personality")
    music.set_music(MusicStateRef("playing", track_id="t1"))
    assert not voice_asked(music.snapshot, "ускорь темп"), "под музыкой «ускорь» — музыка, а не голос"


# -- U+FFFD -----------------------------------------------------------------------------------------------------


def test_n105_replacement_chars_are_cut_from_speech_ask_and_segments():
    cmd = parse({"speech": N105_SPEECH})
    assert cmd.speech == "Остально в порядке" and cmd.dropped == ("speech.U+FFFD",)
    ask = parse({"ask": {"expects": "free", "text": "Как\ufffd дела?"}})
    assert ask.ask.text == "Как дела?" and ask.dropped == ("ask.text.U+FFFD",)
    perf = parse({"perform": {"kind": "poem", "segments": ["Раз\ufffd", "два"]}})
    assert perf.perform.segments == ("Раз", "два") and perf.dropped == ("perform.segments[0].U+FFFD",)
    assert parse({"speech": "\ufffd\ufffd"}).speech is None, "одни битые символы — речи нет"
    clean = parse({"speech": "Всё в порядке"})
    assert clean.speech == "Всё в порядке" and clean.dropped == ()


# -- ход: форма ставится на время вызова LLM ---------------------------------------------------------------------


class _FormPorts:
    """Порт хода: запоминает форму команды на момент вызова LLM (её читает мост) и отвечает по очереди."""

    def __init__(self, answers):
        self.answers, self.forms, self.said, self.stages, self.asked = list(answers), [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.forms.append(cm.current_form())
        self.asked.append(messages)
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return []

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append((text, voice, emotion))
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source=""):
        self.stages.append((stage, status, reason))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _turn(ports, addressed, voices=None):
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", voices=voices)
    return asyncio.run(engine.run(addressed))


def test_n813_addressed_turn_asks_the_model_without_passed_and_resets_the_form():
    ports = _FormPorts([Answer(Command(speech="Тишина так тишина."), "deepseek")])
    _turn(ports, Addressed(N813, VOICE))
    assert ports.forms == [ADDRESSED_TURN]
    assert cm.current_form() == Form(), "после хода форма не течёт в чужие вызовы"


def test_backlog_pickup_turn_keeps_passed_in_the_form():
    ports = _FormPorts([Answer(Command(passed="not_addressed"), "deepseek")])
    _turn(ports, Addressed("мы тут о своём", "backlog_pickup", True, ()))
    assert ports.forms == [Form(passed=True, voice=False, backing=False)] and ports.said == []


def test_n109_voice_is_open_only_when_the_voice_is_asked_in_this_reply():
    asked = _FormPorts([Answer(Command(speech="Хорошо."), "deepseek")])
    _turn(asked, Addressed("скажи это голосом алёны", VOICE))
    assert asked.forms == [Form(passed=False, voice=True, backing=False)]
    named = _FormPorts([Answer(Command(speech="Хорошо."), "deepseek")])
    _turn(named, Addressed("скажи как шаонв", VOICE), voices=lambda: (("female-shaonv", "шаонв"),))
    assert named.forms == [Form(passed=False, voice=True, backing=False)]
    plain = _FormPorts([Answer(Command(speech="Сирену, звонок."), "deepseek")])
    _turn(plain, Addressed(N109, VOICE))
    # «играть» - слово группы music (``tools_view.music_asked``, #4152): подложка выступления у такой реплики не закрыта
    assert plain.forms == [Form(passed=False, voice=False, backing=True)]


JOVIALMAN = ("English_Jovialman", "Весельчак", "Джовиалмен", "Джовиалмэн")  # (id, display_name, *алиасы) реестра


@pytest.mark.parametrize("text", [
    "Робот, скажи как Весельчак",  # русское название
    "Робот, скажи как джовиалмен",  # алиас
    "Робот, скажи как Джовиалмэн",  # второй алиас
    "Робот, скажи как English_Jovialman",  # id голоса
])
def test_command_form_sees_the_display_name_and_every_alias_of_a_voice_separately_4114(text):
    """Ревью PR #4129: имена голоса шли склейкой «Весельчак Джовиалмен Джовиалмэн», а ``voice_asked`` (``has_form``
    ищет слово ровно одной формы) не находил ни название, ни алиас: форма хода закрывала ``voice``, и модель не могла
    озвучить реплику названным голосом."""
    from rob_box_dialog.turn import _Turn  # noqa: PLC0415 — внутренняя запись хода

    snap = Session("personality").snapshot
    registry = (JOVIALMAN, ("Russian_ReliableMan", "Надёжный мужчина"))
    assert command_form(_Turn("t1", 0, text, VOICE, deadline=None), snap, registry) == Form(
        passed=False, voice=True, backing=False)


def test_command_form_keeps_the_voice_closed_when_no_name_of_the_registry_is_said():
    from rob_box_dialog.turn import _Turn  # noqa: PLC0415 — внутренняя запись хода

    snap = Session("personality").snapshot
    registry = (JOVIALMAN, ("Russian_ReliableMan", "Надёжный мужчина"))
    form = command_form(_Turn("t1", 0, "Робот, скажи как диктор", VOICE, deadline=None), snap, registry)
    assert form == ADDRESSED_TURN


def test_form_of_a_turn_is_a_pure_function_of_the_turn():
    from rob_box_dialog.turn import _Turn  # noqa: PLC0415 — внутренняя запись хода

    snap = Session("personality").snapshot
    turn = _Turn("t1", 0, N109, VOICE, deadline=None)
    assert command_form(turn, snap) == ADDRESSED
    turn.pickup = True
    assert command_form(turn, snap) == Form(passed=True, voice=False)


def test_n813_passed_insisted_after_the_bridge_retry_is_the_mumble_as_before():
    """Мост повторил с нарушением, модель снова «промолчать» — исход хода ``passed``: код мычит (решение Шифу 22.10:
    не молчать), ход ``finished/passed`` как раньше, не грустно и не «не понял» невалидной команды."""
    assert kn.PASSED_INSISTED in kn.PASSED
    ports = _FormPorts([Answer(Invalid(kn.VIOLATION_PASSED_ADDRESSED, "passed=not_addressed"), "deepseek", calls=2)])
    agg = _turn(ports, Addressed(N813, VOICE))
    assert len(ports.said) == 1 and ports.said[0][0] in phrases.catalog()[kn.PASSED_ADDRESSED_PHRASE]
    assert ports.said[0][2] is None
    assert ports.stages[-1] == ("done", "finished", "passed") and (agg.stage, agg.status) == ("done", "finished")


def test_other_invalid_commands_still_close_as_invalid():
    ports = _FormPorts([Answer(Invalid("bad_enum", "emotion=joy"), "deepseek", calls=2)])
    _turn(ports, Addressed(N109, VOICE))
    assert ports.said == [(phrases.text(kn.INVALID_COMMAND_PHRASE), None, "sad")]
    assert ports.stages[-1] == ("done", "done", "invalid_salvaged:bad_enum")


# -- ревью #4041: earcon без слов; фраза после повтора silent_answer без грусти ------------------------------------


QUESTION = "ты меня слышишь ответь коротко"


def test_lone_earcon_on_a_question_needs_words():
    earcon = Command(expression=Expression(earcon="done"))
    assert needs_words(QUESTION, earcon)
    assert not needs_words("улыбнись", earcon), "лицо просили словами — без слов можно"
    assert not needs_words(QUESTION, Command(expression=Expression())), "пустая экспрессия — пустая команда, не лицо"
    assert not needs_words(QUESTION, Command(speech="Слышу.", expression=Expression(earcon="done")))


def test_lone_earcon_is_returned_to_the_model_then_code_speaks_calmly():
    earcon = Answer(Command(expression=Expression(earcon="heard")), "deepseek")
    ports = _FormPorts([earcon, Answer(Command(expression=Expression(earcon="heard")), "deepseek")])
    _turn(ports, Addressed(QUESTION, VOICE))
    assert len(ports.asked) == 2, "первый голый earcon вернули модели нарушением silent_answer"
    assert kn.VIOLATION_SILENT_ANSWER in ports.asked[1][-1]["content"]
    assert len(ports.said) == 1 and ports.said[0][0] in phrases.catalog()[kn.SILENT_ANSWER_PHRASE]
    assert ports.said[0][2] is None, "ничего не провалилось — фраза не грустная"


def test_invalid_emotion_is_sad_only_for_failure_or_not_understood():
    from rob_box_dialog.respond import ActionResult  # noqa: PLC0415

    def res(status):
        return ActionResult("t/0", 0, "personality", "play_animation", kn.TOOL_CLASSES["play_animation"], status)

    assert invalid_emotion(Invalid(kn.VIOLATION_SILENT_ANSWER), ()) is None
    assert invalid_emotion(Invalid(kn.VIOLATION_SILENT_ANSWER), (res("done"),)) is None
    assert invalid_emotion(Invalid(kn.VIOLATION_SILENT_ANSWER), (res("rejected"),)) == "sad"
    assert invalid_emotion(Invalid("bad_enum"), ()) == "sad", "«не понял» — грустно, как было"
    assert invalid_emotion(Invalid("bad_enum"), (res("done"),)) is None
