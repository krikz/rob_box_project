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
    (dict(t=100.0, doa=110.0), dict(), True),                       # 5 с, та же сторона (20° ≤ 30°)
    (dict(t=100.0, doa=200.0), dict(), False),                      # другая сторона
    (dict(t=111.0, doa=90.0), dict(), False),                       # 16 с — кадр уже не ждёт
    (dict(t=94.0, doa=90.0), dict(), False),                        # фраза раньше вопроса
    (dict(t=100.0, doa=None), dict(), False),                       # направление фразы неизвестно
    (dict(t=100.0, doa=None, person="p1", voice_s=2.5), dict(), True),   # тот же узнанный длинный голос
    (dict(t=100.0, doa=None, person="p1", voice_s=1.0), dict(), False),  # короткому голосу не верим
    (dict(t=100.0, doa=95.0), dict(azimuth=None), False),           # азимут вопроса неизвестен
])
def test_answer_to_own_question_without_wake(heard, question, addressed):
    decision = decide(_heard("меня зовут Гриша", **heard), None, DEFAULT_WAKE_WORDS, _question(**question))
    if addressed:
        assert decision == Addressed("меня зовут Гриша", address.IDENTITY_ANSWER)
    else:
        assert decision == NotAddressed(address.NO_WAKE)


def test_no_question_no_wake_is_not_addressed_and_wake_still_works():
    assert decide(_heard("меня зовут Гриша", doa=90.0), None, DEFAULT_WAKE_WORDS, None) == NotAddressed(address.NO_WAKE)
    with_wake = decide(_heard("робот меня зовут Гриша", doa=250.0), None, DEFAULT_WAKE_WORDS, _question())
    assert with_wake == Addressed("меня зовут Гриша", address.VOICE)


def test_flag_in_knowledge_turns_the_wakeless_answer_off(monkeypatch):
    from rob_box_dialog import knowledge as kn

    monkeypatch.setattr(kn, "IDENTITY_ANSWER_WITHOUT_WAKE", False)
    decision = decide(_heard("меня зовут Гриша", t=100.0, doa=90.0), None, DEFAULT_WAKE_WORDS, _question())
    assert decision == NotAddressed(address.NO_WAKE)
