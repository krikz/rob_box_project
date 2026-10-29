"""Длина речи человеку (#4137, решение Шифу 28.10; n907): модель просим о ``SPEECH_MAX_CHARS`` (280), речь проверяем
по ``SPEECH_ACCEPT_MAX_CHARS`` (350). Длиннее - своё нарушение ``VIOLATION_SPEECH_TOO_LONG`` (мост даёт ход
компактнее), последний запас - :func:`command.fit_speech`: целые предложения с начала, пока влезают; оборванное
посреди предложения кончает пометка каталога ``command.speech_cut`` (хвост ревью #4143)."""

from __future__ import annotations

from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Command, Invalid, parse, validate

TOOLS = ("memory_save", "play_sound", "set_volume")
N907 = ("Свежих материалов за неделю поиск дал мало. Важное: Роскосмос по просьбе NASA законсервирует "
        "переходную камеру модуля «Звезда» из-за утечек воздуха — их ищут с две тысячи девятнадцатого года. "
        "Ещё на этой неделе к станции ушёл грузовой корабль, а китайский зонд прислал новые снимки Луны.")
TAIL = (" Учёные снова обсуждали пилотируемый полёт к Марсу: старт перенесли ещё на год. "
        "А новый телескоп нашёл далёкую галактику.")


def _check(raw):
    return validate(parse(raw), TOOLS)


def test_ask_and_check_are_one_knowledge_each_and_the_check_is_wider():
    schema = kn.command_schema("json")["properties"]
    assert schema["speech"]["maxLength"] == kn.SPEECH_MAX_CHARS == schema["ask"]["properties"]["text"]["maxLength"]
    assert kn.SPEECH_MAX_CHARS < kn.SPEECH_ACCEPT_MAX_CHARS


def test_n907_answer_longer_than_asked_is_valid_as_is():
    assert kn.SPEECH_MAX_CHARS < len(N907) <= kn.SPEECH_ACCEPT_MAX_CHARS
    assert _check({"speech": N907}).speech == N907
    assert _check({"ask": {"expects": "yes_no", "text": N907}}).ask.text == N907
    assert _check({"speech": "а" * kn.SPEECH_ACCEPT_MAX_CHARS}).speech  # граница - включительно


def test_speech_and_ask_longer_than_check_are_their_own_violation_not_too_long():
    for raw in ({"speech": N907 + TAIL}, {"ask": {"expects": "free", "text": N907 + TAIL}}):
        res = _check(raw)
        assert isinstance(res, Invalid) and res.reason == kn.VIOLATION_SPEECH_TOO_LONG
        assert res.detail.endswith(f" {len(N907 + TAIL)} > {kn.SPEECH_ACCEPT_MAX_CHARS}")
        assert cm.overlong_only(res)


def test_spoken_text_has_one_length_violation_and_keeps_the_control_chars_rule():
    """Проверка речи не имеет второй, недостижимой ветки ``too_long``: длину судит только ``speech_too_long``, а
    управляющие символы - своё нарушение (ревью #4143)."""
    for raw in ({"speech": N907 + TAIL}, {"ask": {"expects": "free", "text": N907 + TAIL}}):
        assert [v.reason for v in _check(raw).all] == [kn.VIOLATION_SPEECH_TOO_LONG]
    assert _check({"speech": "Привет\x07"}).reason == "control_chars"
    assert _check({"ask": {"expects": "free", "text": "Привет\x07"}}).reason == "control_chars"
    assert _check({"speech": "Строка один.\nСтрока два."}).speech == "Строка один.\nСтрока два."


def test_overlong_with_another_violation_is_not_overlong_only():
    res = _check({"speech": N907 + TAIL, "voice": "z" * (kn.VOICE_MAX_CHARS + 1)})
    assert isinstance(res, Invalid) and not cm.overlong_only(res)
    assert [v.reason for v in res.all] == [kn.VIOLATION_SPEECH_TOO_LONG, "too_long"]  # голос - прежний too_long
    assert not cm.overlong_only(Command(speech="Привет."))


def test_explain_of_overlong_asks_for_the_asked_length():
    note = cm.explain(_check({"speech": N907 + TAIL}), after_query=True)
    assert kn.VIOLATION_SPEECH_TOO_LONG in note and f"не длиннее {kn.SPEECH_MAX_CHARS}" in note
    assert "компактнее" in note and "получены" in note


def test_fit_keeps_whole_sentences_from_the_start_while_they_fit():
    long = N907 + TAIL
    fitted = cm.fit_speech(Command(speech=long))
    assert fitted.speech == N907  # три предложения влезли, с четвёртым уже длиннее проверки
    assert len(N907) + len(TAIL.split(". ")[0]) + 1 > kn.SPEECH_ACCEPT_MAX_CHARS
    assert isinstance(validate(fitted, TOOLS), Command)


ONE_SENTENCE = "Я знаю голоса " + ", ".join(f"голос{n}" for n in range(80)) + "."
CUT_MARK = phrases.text(kn.SPEECH_CUT_PHRASE)


def test_fit_cuts_one_sentence_longer_than_check_at_a_word_and_marks_the_cut():
    """Хвост ревью #4143: перечень голосов («…Голос38») обрезался по слову и звучал как полный. Теперь обрыв посреди
    предложения кончает пометка каталога, она входит в предел; запятая обрыва не звучит."""
    fitted = cm.fit_speech(Command(speech=ONE_SENTENCE)).speech
    head = fitted[:-len(CUT_MARK)].rstrip()
    assert fitted.endswith(f" {CUT_MARK}") and len(fitted) <= kn.SPEECH_ACCEPT_MAX_CHARS
    assert ONE_SENTENCE.startswith(head) and not head.endswith(",")
    assert head.split()[-1] in ONE_SENTENCE.split() or f"{head.split()[-1]}," in ONE_SENTENCE.split()
    assert isinstance(validate(parse({"speech": fitted}), TOOLS), Command)  # мост перепроверяет ужатое


def test_fit_keeps_the_longest_possible_head_when_it_marks_the_cut():
    fitted = cm.fit_speech(Command(speech=ONE_SENTENCE)).speech
    head = fitted[:-len(CUT_MARK)].rstrip()
    next_word = ONE_SENTENCE[len(head):].lstrip(", ").split(" ")[0]
    assert len(f"{head}, {next_word} {CUT_MARK}") > kn.SPEECH_ACCEPT_MAX_CHARS, "следующее слово уже не влезало"


def test_fit_marks_ask_text_cut_mid_sentence_too_but_not_whole_sentences():
    asked = cm.fit_speech(Command(ask=cm.Ask(expects="free", text=ONE_SENTENCE)))
    assert asked.ask.text.endswith(CUT_MARK) and asked.ask.expects == "free"
    whole = cm.fit_speech(Command(speech=N907 + TAIL)).speech
    assert whole == N907 and CUT_MARK not in whole, "целые предложения - без пометки: они не оборваны"


def test_fit_cut_mark_is_one_phrase_from_the_catalog():
    assert kn.SPEECH_CUT_PHRASE in kn.PHRASE_KEYS and kn.SPEECH_CUT_PHRASE in phrases.catalog()
    assert len(CUT_MARK) < kn.SPEECH_MAX_CHARS // 4, "пометка короткая"


def test_fit_touches_only_overlong_speech_and_ask_text():
    short = Command(speech="Коротко.")
    assert cm.fit_speech(short) == short
    asked = cm.fit_speech(Command(ask=cm.Ask(expects="free", text=N907 + TAIL)))
    assert asked.ask.text == N907 and asked.ask.expects == "free"
    assert cm.fit_speech(Command()) == Command()
