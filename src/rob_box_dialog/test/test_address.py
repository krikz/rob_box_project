"""Адресация кодом (ADR-0150 §4.2, §7.3 В12, В18; приёмка A15, A31) — корпус вейков и правило «робот» + бэклог."""

import types

import pytest

from rob_box_dialog import address
from rob_box_dialog.address import (
    BACKLOG_PICKUP, BARE_WAKE, COORDINATOR, EMPTY, NO_WAKE, VOICE, Addressed, NotAddressed, decide, heard_from,
    is_bare, log_label, person_ref_from,
)
from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.doa import DoaTrack
from rob_box_dialog.knowledge import DEFAULT_WAKE_WORDS, RISKY_WAKE_WORDS

WAKES = list(DEFAULT_WAKE_WORDS)
COMMANDS = [
    "включи музыку", "покажи анимацию ускорения", "который час", "расскажи анекдот", "сделай громче",
    "как тебя зовут", "спой песню про ежа", "поехали вперёд", "что ты видишь", "запомни меня",
    "какая погода", "выключи свет", "сыграй что нибудь для души", "кто я", "помолчи минутку",
    "найди трек", "давай поиграем", "что нового", "ты меня слышишь", "повтори ещё раз",
]
#: A15: «робот / робби / роббокс» в начале и в середине фразы — 100 % (3 × 20 × 2 = 120 фраз).
CORPUS_WAKE = ([f"{w} {c}" for w in ("робот", "робби", "роббокс") for c in COMMANDS]
               + [f"слушай {w} {c}" for w in ("робот", "робби", "роббокс") for c in COMMANDS])
#: A15: 0 ложных вейков — вейк внутри слова, через дефис, речь о роботе в другой форме, рискованные вейки (В18).
CORPUS_FALSE = [
    "у меня всё работает", "системный промт для робота-диджея", "он заработал денег", "это ботинок",
    "роботы захватят мир", "робототехника — наука", "супер-робот на экране", "ботаник сидит дома",
    "заработает к утру", "работа не волк", "рома пришёл с работы", "рома покажи анимацию ускорения",
    "бот в телеграме молчит", "роб и кирилл ушли", "роботизированный склад", "ботва на грядке",
    "переработать код", "робот-пылесос сломался", "это не мой роботс", "робкий мальчик",
]


def _heard(text, t=100.0, source="respeaker", doa=None, person="", voice_s=0.0, faces=()):
    return Heard(text=text, t=t, source=source, doa_deg=doa, person_ref=person, voice_s=voice_s,
                 faces=frozenset(faces))


@pytest.mark.parametrize("text", CORPUS_WAKE)
def test_wake_corpus_is_addressed(text):
    out = decide(_heard(text), Backlog(), WAKES)
    assert isinstance(out, Addressed) and out.source == VOICE and out.text, text


@pytest.mark.parametrize("text", CORPUS_FALSE)
def test_false_wake_corpus_is_not_addressed(text):
    assert decide(_heard(text), Backlog(), WAKES) == NotAddressed(NO_WAKE), text


def test_corpus_size_meets_a15():
    assert len(CORPUS_WAKE) + len(CORPUS_FALSE) >= 100


def test_risky_wake_words_work_only_when_enabled():
    assert decide(_heard("рома покажи анимацию"), None, WAKES + list(RISKY_WAKE_WORDS)) == Addressed(
        "покажи анимацию", VOICE)
    assert decide(_heard("у меня всё работает"), None, WAKES + list(RISKY_WAKE_WORDS)) == NotAddressed(NO_WAKE)


def test_telegram_and_operator_are_always_addressed():
    assert decide(_heard("включи музыку", source="telegram"), None, WAKES) == Addressed("включи музыку", "telegram")
    assert decide(_heard("робот привет", source="telegram"), None, WAKES) == Addressed("привет", "telegram")
    assert decide(_heard("статус", source="operator"), None, WAKES) == Addressed("статус", "operator")
    assert decide(_heard("робот", source="telegram"), None, WAKES) == NotAddressed(EMPTY)


@pytest.mark.parametrize("text", ["Клод привет", "клод, не попадает в ритм", "Claude: проверь лог", "  КЛОД посмотри"])
def test_coordinator_prefix_from_telegram_is_not_addressed(text):
    assert decide(_heard(text, source="telegram"), None, WAKES) == NotAddressed(COORDINATOR)


def test_coordinator_word_elsewhere_or_from_voice_is_not_the_coordinator():
    assert decide(_heard("Клодия привет", source="telegram"), None, WAKES) == Addressed("Клодия привет", "telegram")
    assert decide(_heard("скажи клоду привет", source="telegram"), None, WAKES).source == "telegram"
    assert decide(_heard("клод привет"), None, WAKES) == NotAddressed(NO_WAKE)


@pytest.mark.parametrize("rest, bare", [("", True), ("ну", True), ("а?", True), ("эй, слушай", True),
                                        ("привет", False), ("стоп", False), ("ну давай споём", False)])
def test_bare_means_no_words_but_fillers(rest, bare):
    assert is_bare(rest) is bare


# -- В12: голое «робот» после неуслышанной просьбы ------------------------------------------------------------


def _backlog_with(text, t, doa=None, person="", voice_s=0.0, faces=()):
    backlog = Backlog()
    backlog.add(_heard(text, t=t, doa=doa, person=person, voice_s=voice_s, faces=faces))
    return backlog


def test_bare_wake_always_picks_up_fresh_phrase_whoever_said_it():
    """#3767 (Шифу 21.10): «бэклог должен уходить по любому всегда» — ни лица, ни голоса, DOA с другой стороны."""
    backlog = _backlog_with("покажи анимацию ускорения", t=95.0, doa=90.0)
    out = decide(_heard("Робот", t=100.0, doa=270.0), backlog, WAKES)
    assert out.text == "покажи анимацию ускорения" and out.source == BACKLOG_PICKUP and out.pickup
    (label,) = out.picked
    assert label["speaker"] == {"identified": False, "label": "неопознанный", "bio": "none"}
    assert label["same_as_caller"] is False and label["same_direction"] is False
    assert backlog.background_phrases(100.0) == 0, "подхваченная фраза уходит из бэклога"


def test_bare_wake_takes_every_fresh_phrase_in_order():
    backlog = _backlog_with("слушай я хотела рассказать", t=92.0, doa=58.0, voice_s=14.8)
    backlog.add(_heard("как собрать полку для книг", t=94.5, doa=54.0, voice_s=9.8))
    out = decide(_heard("робот", t=98.5, doa=299.0), backlog, WAKES)
    assert out.text == "слушай я хотела рассказать как собрать полку для книг"
    assert [p["text"] for p in out.picked] == ["слушай я хотела рассказать", "как собрать полку для книг"]


def test_bare_wake_with_empty_or_stale_backlog_is_reflex_only():
    backlog = _backlog_with("девяносто пять градусов", t=70.0, doa=90.0, voice_s=2.0)
    assert decide(_heard("робот", t=100.0, doa=90.0), backlog, WAKES) == Addressed("", BARE_WAKE)
    assert decide(_heard("робот", t=100.0), Backlog(), WAKES) == Addressed("", BARE_WAKE)
    assert decide(_heard("робот", t=100.0), None, WAKES) == Addressed("", BARE_WAKE)


def test_wake_with_a_phrase_never_takes_the_backlog():
    backlog = _backlog_with("покажи анимацию ускорения", t=95.0, doa=90.0)
    assert decide(_heard("робот который час", t=100.0, doa=90.0), backlog, WAKES) == Addressed("который час", VOICE)
    assert backlog.background_phrases(100.0) == 1


# -- Utterance → Heard ----------------------------------------------------------------------------------------


def _stamp(sec):
    return types.SimpleNamespace(sec=int(sec), nanosec=int((sec - int(sec)) * 1e9))


def test_heard_from_utterance_takes_direction_from_the_phrase_window():
    track = DoaTrack()
    track.on_vad(True, 0.0)
    track.on_direction(45.0, 10.5)
    track.on_direction(300.0, 30.0)  # следующая фраза — другое направление
    msg = types.SimpleNamespace(text=" робот ", source="respeaker", started_at=_stamp(10.0), ended_at=_stamp(12.0))
    heard = heard_from(msg, now=50.0, doa=track, person_ref="p1")
    assert (heard.text, heard.t, heard.doa_deg, heard.person_ref) == ("робот", 12.0, pytest.approx(45.0), "p1")
    unknown = types.SimpleNamespace(text="x", source="telegram", started_at=_stamp(0), ended_at=_stamp(0))
    assert heard_from(unknown, now=50.0, doa=track).t == 50.0 and heard_from(unknown, now=50.0, doa=track).doa_deg is None


def test_person_ref_only_for_known_speaker():
    reg = types.SimpleNamespace(peek=lambda uid: {"u1": {"is_known": True, "speaker_id": "abc"},
                                                  "u2": {"is_known": False, "speaker_id": "zzz"}}.get(uid))
    assert person_ref_from(reg, "u1") == "abc"
    assert person_ref_from(reg, "u2") == "" and person_ref_from(reg, None) == "" and person_ref_from(None, "u1") == ""


def test_log_label_keeps_the_coordinator_contract():
    tg = types.SimpleNamespace(text="Клод привет", chat_id=42, source="telegram")
    voice = types.SimpleNamespace(text="робот привет", chat_id=0, source="respeaker")
    assert log_label(tg) == "[TG:42] Клод привет" and log_label(voice) == "робот привет"


def test_wake_table_from_yaml_or_code():
    personality, operator = address.wake_words()
    assert personality == list(DEFAULT_WAKE_WORDS) and operator
    assert address.wake_words("", ["робот"])[0] == ["робот"]


def test_sources_match_utterance_msg():
    from rob_box_core.msg_stub import build_dialog_msgs_module

    utt = build_dialog_msgs_module().Utterance
    assert (utt.SOURCE_RESPEAKER, utt.SOURCE_TELEGRAM, utt.SOURCE_OPERATOR) == (
        address.SOURCE_RESPEAKER, address.SOURCE_TELEGRAM, address.SOURCE_OPERATOR)


# -- В17(б): ответ на вопрос робота без вейка (#3695) ---------------------------------------------------------


def _question(azimuth=90.0, asked_at=95.0, person="p1", kind="unknown"):
    from rob_box_dialog.session import SceneFrame
    from rob_box_dialog.speaker_adapter import question_frame

    return SceneFrame("Identity", "code", payload=question_frame(person, kind, None, azimuth, asked_at))


@pytest.mark.parametrize("heard, question, addressed", [
    (dict(t=100.0, doa=110.0), dict(), True),                       # 5 с, та же сторона
    (dict(t=100.0, doa=200.0), dict(), True),                       # DOA не условие: другая сторона, фраза — ответ
    (dict(t=100.0, doa=None), dict(), True),                        # направление неизвестно — тоже
    (dict(t=111.0, doa=90.0), dict(), False),                       # 16 с — кадр уже не ждёт
    (dict(t=94.0, doa=90.0), dict(), False),                        # фраза раньше вопроса
    (dict(t=94.975, doa=90.0), dict(), False),                      # в грейсе микрофона — окно имени прежнее (#3849)
    (dict(t=100.0, doa=95.0), dict(azimuth=None), True),            # азимут вопроса неизвестен — не важен
])
def test_answer_to_own_question_without_wake(heard, question, addressed):
    decision = decide(_heard("меня зовут Гриша", **heard), None, DEFAULT_WAKE_WORDS, _question(**question))
    if addressed:
        assert decision == Addressed("меня зовут Гриша", address.IDENTITY_ANSWER)
    else:
        assert decision == NotAddressed(address.NO_WAKE)


@pytest.mark.parametrize("text, doa, addressed", [
    ("не скажу", 90.0, True),                                         # отказ
    ("не скажу", 157.0, True),                                        # DOA через 67° от азимута вопроса (90°) — принят
    ("Гриша", 343.0, True),                                           # голое имя, DOA прыгнул 50° → 343°
    ("меня зовут Саша", 20.0, True),
    ("зови меня Гриша", 20.0, True),
    ("да", 20.0, True),
    ("это не я", 20.0, True),
    ("поселения происходили давайте я устрою наше время", 90.0, False),   # чужой разговор — бэклог
    ("только что основали", 90.0, False),
    ("Гриша пришёл вчера", 90.0, False),
])
def test_wakeless_phrase_is_an_answer_only_when_it_looks_like_one(text, doa, addressed):
    decision = decide(_heard(text, t=100.0, doa=doa), None, DEFAULT_WAKE_WORDS, _question())
    assert decision == (Addressed(text, address.IDENTITY_ANSWER) if addressed else NotAddressed(address.NO_WAKE))


def test_no_question_no_wake_is_not_addressed_and_wake_still_works():
    assert decide(_heard("меня зовут Гриша", doa=90.0), None, DEFAULT_WAKE_WORDS, None) == NotAddressed(address.NO_WAKE)
    with_wake = decide(_heard("робот меня зовут Гриша", doa=250.0), None, DEFAULT_WAKE_WORDS, _question())
    assert with_wake == Addressed("меня зовут Гриша", address.VOICE)


def test_flag_in_knowledge_turns_the_wakeless_answer_off(monkeypatch):
    from rob_box_dialog import knowledge as kn

    monkeypatch.setattr(kn, "IDENTITY_ANSWER_WITHOUT_WAKE", False)
    decision = decide(_heard("меня зовут Гриша", t=100.0, doa=90.0), None, DEFAULT_WAKE_WORDS, _question())
    assert decision == NotAddressed(address.NO_WAKE)


@pytest.mark.parametrize("text, t, addressed", [
    ("мы разные люди", 100.0, True),                 # ответ на вопрос склейки (#2828) без вейка — #3765
    ("да, это я", 100.0, True),
    ("ты обознался", 100.0, True),
    ("да", 100.0, True),                             # голое да/нет — ответ (на conflict непонятный, решит код)
    ("Гриша", 100.0, False),                         # голое имя — не ответ на «вы разные люди?»
    ("мы разные люди", 111.0, False),                # окно то же — REPLY_WINDOW_S (15 с)
    ("это я вчера включал тебе эту музыку", 100.0, False),  # длинная фраза с «это я» — чужой разговор
])
def test_merge_question_answer_without_wake(text, t, addressed):
    decision = decide(_heard(text, t=t, doa=90.0), None, DEFAULT_WAKE_WORDS, _question(kind="conflict"))
    assert decision == (Addressed(text, address.IDENTITY_ANSWER) if addressed else NotAddressed(address.NO_WAKE))


# --- «выключить и зачитать?» (О9′, PR-7c ч.3в #3700): «да»/«нет» без вейка в окне вопроса выступления ---------------


def _ask(asked_at=100.0):
    from rob_box_dialog.session import SceneFrame

    return SceneFrame("PerformAsk", "code", payload={"asked_at": asked_at, "title": "Still Dre"})


@pytest.mark.parametrize("text,t,addressed", [
    ("да", 105.0, True), ("нет", 110.0, True), ("Да.", 101.0, True),
    ("да", 99.975, True),  # #3849 o9y_2: за 25 мс до asked_at (в грейсе микрофона, вопрос договорён) — ответ
    ("да", 97.4, False),  # кончилась раньше конца вопроса — ещё звучал вопрос
    ("да", 100.0 + 16, False),  # окно 15 с прошло
    ("включи что-нибудь весёлое", 105.0, False),  # не ответ — чужой разговор
])
def test_perform_question_yes_no_without_wake(text, t, addressed):
    decision = decide(_heard(text, t=t), None, DEFAULT_WAKE_WORDS, ask=_ask())
    assert decision == (Addressed(text, address.PERFORM_ANSWER) if addressed else NotAddressed(address.NO_WAKE))


@pytest.mark.parametrize("t, in_grace, inside", [
    (97.4, True, False), (97.5, True, True), (99.975, True, True), (115.0, True, True), (115.1, True, False),
    (99.975, False, False), (100.0, False, True), (115.0, False, True), (115.1, False, False),
])
def test_reply_window_bounds(t, in_grace, inside):
    """#3849: окно ответа до ``asked_at`` + 15 с; с ``in_grace`` (да/нет выступления) — от конца вопроса
    (``asked_at`` − грейс микрофона 2.5 с), без него (окно имени В17, приветствие) — от ``asked_at``, как было."""
    from rob_box_dialog.speaker_adapter import in_reply_window

    assert in_reply_window(100.0, t, in_grace=in_grace) is inside
    assert in_reply_window(None, t, in_grace=in_grace) is False


def test_no_window_no_wakeless_answer():
    assert decide(_heard("да", t=105.0), None, DEFAULT_WAKE_WORDS) == NotAddressed(address.NO_WAKE)


def test_wake_plus_yes_in_the_window_is_an_answer_not_a_bare_wake():
    """«Робот, да»: «да» — слово-заполнитель голого вейка, но в окне вопроса выступления это ответ."""
    assert decide(_heard("Робот, да", t=105.0), None, DEFAULT_WAKE_WORDS, ask=_ask()) == Addressed(
        "да", address.PERFORM_ANSWER)
    assert decide(_heard("Робот, да", t=130.0), None, DEFAULT_WAKE_WORDS, ask=_ask()).source == address.BARE_WAKE


@pytest.mark.parametrize("ask", [None, _ask(asked_at=100.0)], ids=["no_window", "window_expired"])
@pytest.mark.parametrize("text", ["Робот", "Робот, да"])
def test_bare_wake_outside_the_perform_window_still_takes_the_backlog(ask, text):
    """#3772 (В12″): вне окна вопроса выступления голое «робот» (и «Робот, да» — «да» тут заполнитель) берёт
    бэклог, как без выступления."""
    backlog = _backlog_with("покажи анимацию ускорения", t=125.0, doa=90.0)
    out = decide(_heard(text, t=130.0, doa=270.0), backlog, DEFAULT_WAKE_WORDS, ask=ask)
    assert (out.text, out.source) == ("покажи анимацию ускорения", BACKLOG_PICKUP)
    assert backlog.background_phrases(130.0) == 0


def test_in_the_perform_window_yes_is_the_answer_and_bare_wake_still_takes_the_backlog():
    """В окне вопроса: «Робот, да» — ответ, бэклог не трогается; голое «Робот» — по-прежнему бэклог (#3772)."""
    backlog = _backlog_with("покажи анимацию ускорения", t=103.0, doa=90.0)
    yes = decide(_heard("Робот, да", t=105.0), backlog, DEFAULT_WAKE_WORDS, ask=_ask())
    assert yes == Addressed("да", address.PERFORM_ANSWER)
    assert backlog.background_phrases(105.0) == 1, "ответ на вопрос не съедает бэклог"
    bare = decide(_heard("Робот", t=106.0, doa=270.0), backlog, DEFAULT_WAKE_WORDS, ask=_ask())
    assert (bare.text, bare.source) == ("покажи анимацию ускорения", BACKLOG_PICKUP)
