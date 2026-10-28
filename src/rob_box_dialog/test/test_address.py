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
from rob_box_dialog.session import MusicStateRef, Session

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
    "заработает к утру", "работа не волк", "бот в телеграме молчит", "роб и кирилл ушли", "роботизированный склад", "ботва на грядке",
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
    assert decide(_heard("бот покажи анимацию"), None, WAKES) == NotAddressed(NO_WAKE)
    assert decide(_heard("роб покажи анимацию"), None, WAKES) == NotAddressed(NO_WAKE)
    assert decide(_heard("бот покажи анимацию"), None, WAKES + list(RISKY_WAKE_WORDS)) == Addressed(
        "покажи анимацию", VOICE)
    assert decide(_heard("роб покажи анимацию"), None, WAKES + list(RISKY_WAKE_WORDS)) == Addressed(
        "покажи анимацию", VOICE)
    assert decide(_heard("у меня всё работает"), None, WAKES + list(RISKY_WAKE_WORDS)) == NotAddressed(NO_WAKE)


@pytest.mark.parametrize("wake", ["рома", "роман"])
def test_roma_and_roman_are_ordinary_wake_words(wake):
    """23.10 Шифу (коммиты 83fdeb913, 2dae40fee): «рома» и «роман» — обычные вейки, не рискованные (В18)."""
    assert wake in DEFAULT_WAKE_WORDS and wake not in RISKY_WAKE_WORDS
    assert decide(_heard(f"{wake} покажи анимацию"), None, WAKES) == Addressed("покажи анимацию", VOICE)
    assert decide(_heard(f"{wake}, включи музыку"), None, WAKES) == Addressed("включи музыку", VOICE)


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
    got = decide(_heard("робот который час", t=100.0, doa=90.0), backlog, WAKES)
    assert (got.text, got.source, got.pickup, got.picked) == ("который час", VOICE, False, ())
    assert [p["text"] for p in got.background] == ["покажи анимацию ускорения"]  # раздел снимка, не текст хода (В23)
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


def test_decide_in_session_reads_windows_and_hold_from_the_snapshot_frames():
    """#3906: ROS-хост и сухой стенд решают адресацию одной функцией; окна и удержание берутся из кадров снимка."""
    session = Session("personality", clock=lambda: 100.0)
    backlog = Backlog()
    plain = _heard("включи что-нибудь", t=100.0)
    assert address.decide_in_session(plain, backlog, DEFAULT_WAKE_WORDS, session.snapshot) == decide(
        plain, backlog, DEFAULT_WAKE_WORDS)
    session.push_scene("OperatorHold", "operator", ttl_s=60.0)
    held = address.decide_in_session(_heard("робот привет", t=101.0), backlog, DEFAULT_WAKE_WORDS, session.snapshot)
    assert held == NotAddressed(address.HELD)


# --- #3975, #3996: Yandex искажает первое слово «Робот, …»; допуск только для первого токена -----------------------
#: Фраза и её остаток после вейка. Из живого лога (23.10) - только помеченные «лог»; остальные - варианты по тем же
#: правилам (в логах их не было, #3975 признал это честно).
SLIPPED_WAKE = [
    ("роботу дали команду", "дали команду"),
    ("Роботу, расскажи анекдот", "расскажи анекдот"),
    ("работа тишина", "тишина"),
    ("Работа, стоп", "стоп"),
    ("работу включи музыку", "включи музыку"),
    ("рада слушай дальше", "слушай дальше"),
    ("рада, говори", "говори"),
    ("работа давай поиграем", "давай поиграем"),
    ("рубот покажи анимацию", "покажи анимацию"),      # близкая форма вне списка: Левенштейн 1
    ("ребот, что ты видишь", "что ты видишь"),
    ("роббоксу привет", "привет"),                      # роббокс + 1
    # #3996: «роботы» и «работа/рада» перед вопросом, «ещё» про музыку (ворота WAKE_SLIP_GATES)
    ("роботы еще куплет добавь про комара который еноту мешал",
     "еще куплет добавь про комара который еноту мешал"),                                    # лог: n703, 22:31
    ("работа как меня зовут", "как меня зовут"),                                             # лог: n1008, 23:17
    ("рада что сейчас крутится", "что сейчас крутится"),                                     # лог: m161509, 17:53
    ("работа говори погромче дядя гриша глуховатый стоит в коридоры",
     "говори погромче дядя гриша глуховатый стоит в коридоры"),                              # лог: n407
    ("Роботы, ещё куплет добавь", "ещё куплет добавь"),                                      # ё и запятая
    ("роботы что ты умеешь", "что ты умеешь"),
    ("работа сколько у тебя заряда", "сколько у тебя заряда"),
    ("рады почему ты молчишь", "почему ты молчишь"),
    ("работа что сейчас играет", "что сейчас играет"),
    ("роботы снова песню про енота", "снова песню про енота"),
    ("работа дальше играй ритм", "дальше играй ритм"),
    ("роботы кто сейчас говорил", "кто сейчас говорил"),
]
#: Обычная речь мастерской, начинающаяся похожими словами: вейка нет. «провод …» и «ну вот что ты видишь» из лога
#: не спасаем (слишком далеко от «робот»).
SLIPPED_NOT_WAKE = [
    "работа не волк", "работа есть?", "работа закончилась к вечеру", "работу сдал вовремя", "рада тебя видеть",
    "рады познакомиться", "роботы захватят мир", "роботов собрали вчера", "рокот моторов в мастерской",
    "ротор вращается", "ворота закрыты", "провод у тебя играет", "ну вот что ты видишь", "робость прошла",
    "работе нужен паяльник", "рабочий день кончился",
    # #3996: отрицательные из проверки #3975 и границы ворот
    "работ много накопилось", "работ на неделю", "работа дальше пойдёт", "роботе не хватает памяти",
    "роботы это сложно", "роботы такие умные", "роботы ещё не умеют летать", "работа как работа", "работа что надо",
    "роботы что-то делают", "рада что пришли", "работа какая-то странная", "работа почему-то встала",
    "роботы снова не помогут", "работа дальше будет проще",
]
#: Фоновые фразы марафона 23.10 (m210648, va_marathon.log, `[backlog] accumulated reason=no_wake`), у которых нет
#: вейка вообще: акты 2 и 3 (намеренный фон) и три настоящих потери вейка, которые допуск первого токена не спасает
#: (расстояние ≥ 3: «правда …», «стоп лучше …», «вот …» - это #3989, а не #3996). Все остаются в бэклоге.
MARATHON_NO_WAKE = [
    "я собирал тебе блок питания и переделывал левый мотор запомни мой голос как следует",
    "потому что ночью я буду говорить с тобой чаще всех остальных в этой мастерской",
    "вчера я всю ночь провел этот несчастный блок питания и он все равно гудит как трансформатор",
    "а я так и не понял земля виновата или дроссель",
    "запомни мой голос отдельно от сашиным мы очень по разному звучим",
    "я не хочу чтобы ты нас путал когда мы оба будем тебе что то говорить",
    "я болел за спартак с девяносто восьмого года и саша каждый раз надо мной смеется",
    "хотя сам вообще футбол не смотрит и путает вратаря с защитником",
    "вот мы тут сейчас немного пошумим ты просто слушай и не влезай",
    "борис блок питания гудит из за земли я вчера по ночи его слушал и знаю эту ноту наизусть",
    "саша у тебя разводка ламината надо было звездой разводить а не шлейфом через плату",
    "молодые люди спать собираетесь третий час ночи у меня обход цвет горит и железка говорит",
    "я курьер мне бы подпись получить и уехать стою уже двадцать минут слушаю ваш спор про землю",
    "слушай включи ты уже какую нибудь музыку а то тишина давит на уши",
    "только не электронщину поставь что нибудь спокойное мне еще ехать",
    "правда что у тебя вообще лежит в музыкальной библиотеке",
    "стоп лучше скажи сколько будет семью восемь",
    "вот как меня зовут",
]


@pytest.mark.parametrize("text,rest", SLIPPED_WAKE)
def test_slipped_first_token_wakes(text, rest):
    out = decide(_heard(text), Backlog(), WAKES)
    assert out == Addressed(rest, VOICE), text


def test_slipped_wake_corpus_sizes_meet_card():
    assert len(SLIPPED_WAKE) >= 8 and len(SLIPPED_NOT_WAKE) >= 8
    assert len(SLIPPED_WAKE) >= 20 and len(SLIPPED_NOT_WAKE) >= 30 and len(MARATHON_NO_WAKE) >= 15


@pytest.mark.parametrize("text", MARATHON_NO_WAKE)
def test_marathon_background_phrases_stay_unaddressed(text):
    """Реальный фон марафона 23.10 не будит робота: допуск первого токена его не задевает."""
    assert decide(_heard(text), Backlog(), WAKES) == NotAddressed(NO_WAKE), text


@pytest.mark.parametrize("text", SLIPPED_NOT_WAKE)
def test_ordinary_speech_with_similar_first_word_does_not_wake(text):
    assert decide(_heard(text), Backlog(), WAKES) == NotAddressed(NO_WAKE), text


@pytest.mark.parametrize("text", ["привет роботу дали команду", "ну работа тишина", "скажи рада слушай дальше"])
def test_only_the_first_token_is_tolerated(text):
    assert decide(_heard(text), Backlog(), WAKES) == NotAddressed(NO_WAKE), text


def test_slip_needs_a_robot_wake_word_in_the_table():
    assert decide(_heard("роботу дали команду"), None, ["тарс"]) == NotAddressed(NO_WAKE)


def test_slipped_bare_wake_is_bare_wake_only_for_unambiguous_forms():
    assert decide(_heard("роботу"), Backlog(), WAKES) == Addressed("", BARE_WAKE)
    assert decide(_heard("работа"), Backlog(), WAKES) == NotAddressed(NO_WAKE)


@pytest.mark.parametrize("wake", ["рома", "роман"])
def test_roma_and_roman_still_wake_after_slip_tolerance(wake):
    assert decide(_heard(f"{wake} покажи анимацию"), Backlog(), WAKES) == Addressed("покажи анимацию", VOICE)
    assert decide(_heard(f"привет, {wake} включи музыку"), Backlog(), WAKES) == Addressed(
        "привет, включи музыку", VOICE, command="включи музыку")  # Tier-1 — по команде после вейка (#4085)


def test_slipped_wake_does_not_lift_silence():
    """Решение Шифу: тишину вейк не снимает — искажённый вейк не исключение. «Работа тишина» адресована как обычный
    вейк, а тишину снимает только «говори» в остатке (Tier-1 ``unsilence``), «тишина» её не трогает."""
    from rob_box_dialog.grammar import match
    from rob_box_dialog.session import SceneFrame, SessionSnapshot
    silenced = SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0,
                               scenes=(SceneFrame("Silence", "user"),))
    out = decide(_heard("работа тишина"), Backlog(), WAKES)
    assert isinstance(out, Addressed) and out.text == "тишина"
    assert match(out.text, silenced) is None
    unsilence = decide(_heard("роботу, говори"), Backlog(), WAKES)
    assert match(unsilence.text, silenced).kind == "unsilence"


# --- #3975: правка играющей музыки после искажённого вейка (ворота ``under_music``) --------------------------------

def _session(music: str = "idle", dj: bool = False) -> Session:
    """Сессия Личности с музыкой плеера (``idle``/``playing``) и, если ``dj``, кадром диджея."""
    session = Session("personality", clock=lambda: 100.0)
    session.set_music(MusicStateRef(music, track_id="t1" if music == "playing" else None))
    if dj:
        session.push_scene("DJ", "code")
    return session


def _in_session(text, session):
    return address.decide_in_session(_heard(text), Backlog(), DEFAULT_WAKE_WORDS, session.snapshot)


#: Живые фразы 27.10 и их остаток. 14:49 - инъекция под диджей-сетом (dev-0d94634), 09:26 - n506_faster (dev-d68a784).
TWEAK_LIVE = [
    ("роботы ускорь темп", "ускорь темп"),
    ("роботы ускорь там потом мы тут засыпаем оба", "ускорь там потом мы тут засыпаем оба"),
]


@pytest.mark.parametrize("text,rest", TWEAK_LIVE)
@pytest.mark.parametrize("session", [_session("playing"), _session("idle", dj=True), _session("playing", dj=True)],
                         ids=["player-playing", "dj-scene", "both"])
def test_3975_tweak_under_playing_music_wakes(text, rest, session):
    assert _in_session(text, session) == Addressed(rest, VOICE)


@pytest.mark.parametrize("text,rest", TWEAK_LIVE)
def test_3975_tweak_without_music_stays_unaddressed(text, rest):
    """Без музыки ворота правки закрыты - поведение как до карточки: не вейк, фраза уходит в бэклог (``no_wake``)."""
    assert _in_session(text, _session("idle")) == NotAddressed(NO_WAKE)
    assert _in_session(text, Session("personality", clock=lambda: 100.0)) == NotAddressed(NO_WAKE)


def test_3975_music_that_stopped_closes_the_gate_again():
    session = _session("playing")
    assert _in_session("роботы громче", session) == Addressed("громче", VOICE)
    session.set_music(MusicStateRef("idle"))
    assert _in_session("роботы громче", session) == NotAddressed(NO_WAKE)


@pytest.mark.parametrize("text", [
    "роботы захватят мир", "роботы ускоряются", "роботы быстро учатся", "работа добавь соли", "рада тебя видеть",
    "работа не волк", "роботы ещё не умеют летать",
    # ревью PR #4123: слово правки в начале рассказа и голое «бас» не будят (ложные побудки под музыкой)
    "роботы темп задают", "рады скорости не боятся", "работа громче всех", "роботы бас",
])
def test_3975_ordinary_speech_does_not_wake_even_under_music(text):
    assert _in_session(text, _session("playing", dj=True)) == NotAddressed(NO_WAKE), text


@pytest.mark.parametrize("text,rest", [
    # ревью PR #4123: естественные просьбы после искажённого вейка
    ("роботы ещё быстрее", "ещё быстрее"),
    ("роботы ещё громче", "ещё громче"),
    ("роботы сделай тише", "сделай тише"),
    ("роботы бас потише", "бас потише"),
    ("работа громче", "громче"),
])
@pytest.mark.parametrize("session", [_session("playing"), _session("idle", dj=True)], ids=["player-playing", "dj-scene"])
def test_3975_natural_tweak_requests_wake_under_music_and_not_without_it(text, rest, session):
    assert _in_session(text, session) == Addressed(rest, VOICE)
    assert _in_session(text, _session("idle")) == NotAddressed(NO_WAKE)


@pytest.mark.parametrize("text", [*SLIPPED_NOT_WAKE, *MARATHON_NO_WAKE])
def test_3975_known_background_corpus_stays_unaddressed_under_music(text):
    """Корпус фона и обычной речи из #3975/#3996 под играющей музыкой не будит: ворота правки их не задевают."""
    assert decide(_heard(text), Backlog(), WAKES, music_playing=True) == NotAddressed(NO_WAKE), text


def test_3975_music_does_not_change_a_real_wake_or_other_slips():
    session = _session("playing", dj=True)
    assert _in_session("Робот, ускорь темп", session) == Addressed("ускорь темп", VOICE)
    assert _in_session("работа тишина", session) == Addressed("тишина", VOICE)
    assert _in_session("роботы еще куплет добавь", session) == Addressed("еще куплет добавь", VOICE)
    assert _in_session("роботы еще куплет добавь", _session("idle")) == Addressed("еще куплет добавь", VOICE)


def test_3975_decide_takes_music_from_the_flag_and_defaults_to_silence():
    assert decide(_heard("роботы ускорь темп"), Backlog(), WAKES) == NotAddressed(NO_WAKE)
    assert decide(_heard("роботы ускорь темп"), Backlog(), WAKES, music_playing=True) == Addressed("ускорь темп", VOICE)


@pytest.mark.parametrize("text, rest, command", [
    # n811 27.10: эхо рэпа робота перед вейком — модель видит фразу целиком, Tier-1 разбирает команду после вейка
    ("четыре недели робот хватит читать а музыку оставь", "четыре недели хватит читать а музыку оставь",
     "хватит читать а музыку оставь"),
    ("мне грустно, робот, спой что-нибудь", "мне грустно, спой что-нибудь", "спой что-нибудь"),
    ("Робот, хватит читать, а музыку оставь.", "хватит читать, а музыку оставь.", ""),  # вейк первым — команда = text
    ("хватит читать, робот", "хватит читать,", ""),  # вейк в конце
    ("роботы хватит", "хватит", ""),  # искажённый вейк первым словом (#3996)
])
def test_4085_addressed_command_is_the_part_after_the_wake(text, rest, command):
    decision = decide(_heard(text), Backlog(), DEFAULT_WAKE_WORDS)
    assert isinstance(decision, Addressed) and decision.source == VOICE
    assert (decision.text, decision.command) == (rest, command)


def test_4085_telegram_text_has_no_separate_command():
    """Эхо речи робота — склейка микрофона: у Telegram и оператора текст хода и есть команда."""
    decision = decide(_heard("скажи ему, робот, хватит", source="telegram"), None, DEFAULT_WAKE_WORDS)
    assert isinstance(decision, Addressed) and decision.command == ""
