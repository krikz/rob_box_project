"""#4509, продолжение: «Добрый вечер» в ответ на «…Добрый вечер.» — человек, а не эхо.

Шифу: «в том-то и прикол, что на „Добрый вечер“ человек отвечает „Добрый вечер“». После #4514 фраза в окне ответа
сверялась со словами речи робота строго, и ответ, повторяющий последние слова робота, выбрасывался как эхо. Теперь
``echo_window`` решает по признакам звука, по порядку: время начала голоса (``ECHO_TAIL_S`` после конца речи робота),
голос (биометрия узнала человека по имени), и только потом слова.

Живой робот 06.11 (Sound Pi, ``docker logs voice-assistant``), тайминг кейсов ниже:

    16:13:41.036 SpeechEvent finished speech_id=70e67229   ('О, Денчик! А я уж думал, ты про меня забыл. Добрый вечер.')
    16:13:41.224 VAD: речь                                 ← +0,18 с: 'Добрый вечер, нет, не забыл про тебя.' (человек)
    16:13:46.328 VAD: речь                                 ← +5,3 с: 'Я не забыл про тебя, говорю.' (человек)
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_dialog import knowledge as kn
from rob_box_dialog.words import DEFAULT_WAKE_WORDS
from rob_box_voice.core import echo_window as ew

M = install_dialog_msgs_stub()

WAKE = list(DEFAULT_WAKE_WORDS)
GRACE_S = 2.5
GREETING, GREETING_ID = "О, Денчик! А я уж думал, ты про меня забыл. Добрый вечер.", "70e67229"
MIRROR = "Добрый вечер."
ECHO_OF_GREETING = "о денчик а я уж думал ты про меня забыл"
END = 4.0  # приветствие звучит 0–4 с


class Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def at(self, offset: float) -> "Clock":
        self.t = 1000.0 + offset
        return self


@pytest.fixture
def clock() -> Clock:
    return Clock()


@pytest.fixture
def window(clock: Clock) -> ew.EchoWindow:
    return ew.EchoWindow(GRACE_S, "hardware", clock=clock)


def _play(window: ew.EchoWindow, clock: Clock, events) -> None:
    for t, kind, value in events:
        clock.at(t)
        if kind == "request":
            sid, text = value
            window.journal.on_request(M.SpeechRequest(speech_id=sid, text=text, commit=True, agent="personality"))
        elif kind == "vad":
            window.on_vad(value)
        elif kind == "reply":
            window.on_reply_window(value)
        else:
            code = M.SpeechEvent.KIND_STARTED if kind == "started" else M.SpeechEvent.KIND_FINISHED
            speaking = window.journal.on_event(M.SpeechEvent(kind=code, speech_id=value, agent="personality"))
            if speaking is not None:
                window.on_speech(speaking)


def _greeting_then_voice(window, clock, rise: float, fall: float, *, reply_open: bool = True) -> ew.Admission:
    """Приветствие 0–4 с, окно ответа открывается на его конце; голос фразы ``rise``–``fall``, доставка через
    ``speech_continuation`` (3 с) после спада VAD."""
    events = [(-0.2, "request", (GREETING_ID, GREETING)), (0.0, "started", GREETING_ID), (END, "finished", GREETING_ID)]
    if reply_open:
        events.append((END, "reply", kn.IDENTITY_MIC_GRACE_S + kn.REPLY_WINDOW_S))
    events += [(rise, "vad", True), (fall, "vad", False)]
    _play(window, clock, sorted(events, key=lambda e: e[0]))
    clock.at(fall + 3.0)
    return window.admit(duration_s=fall + 3.0 - rise)


# --- время: зеркальный ответ после конца речи — человек ----------------------------------------------------------


@pytest.mark.parametrize("delay", [0.5, 0.8, 1.0, 1.5])
def test_mirror_greeting_started_after_robot_speech_is_human(window, clock, delay):
    admission = _greeting_then_voice(window, clock, END + delay, END + delay + 0.8)  # «Добрый вечер» ≈ 0,8 с
    assert admission.verdict == ew.REPLY_AFTER and "человек" in admission.note
    assert admission.echo(MIRROR, WAKE, 4.0) is None
    assert admission.echo("Добрый вечер", WAKE, 4.0) is None


def test_after_the_echo_tail_even_matching_words_are_not_echo(window, clock):
    """Время — первый признак: голос начался позже хвоста эха — в фразу речь робота не попала, слова не сверяются."""
    admission = _greeting_then_voice(window, clock, END + 1.0, END + 2.0)
    assert admission.echo(ECHO_OF_GREETING, WAKE, 4.0) is None


def test_mirror_overlapping_playback_is_echo(window, clock):
    """Та же фраза, но голос начался, пока звучало приветствие, — кандидат; слова повторяют речь робота — эхо."""
    admission = _greeting_then_voice(window, clock, END - 0.6, END + 0.6)
    assert admission.verdict == ew.REPLY and "во время речи робота" in admission.note
    echo = admission.echo(MIRROR, WAKE, 4.2)
    assert echo is not None and echo.speech_id == GREETING_ID


def test_mirror_starting_inside_the_echo_tail_is_echo(window, clock):
    admission = _greeting_then_voice(window, clock, END + 0.2, END + 1.2)
    assert admission.verdict == ew.REPLY and "в хвосте эха" in admission.note
    assert admission.echo(MIRROR, WAKE, 4.0) is not None


def test_without_the_reply_window_time_does_not_open_the_echo_window(window, clock):
    """Вне окна ответа защита #989/#4072 прежняя: фраза только в эхо-окне — молча, кроме стопа с вейком."""
    admission = _greeting_then_voice(window, clock, END + 1.0, END + 2.0, reply_open=False)
    assert admission.verdict == ew.STOP_ONLY
    assert admission.echo(MIRROR, WAKE, 4.0) is not None


# --- голос: кандидат, узнанный биометрией по имени, — человек -------------------------------------------------------


def _speaker(utterance_id: str, *, is_known=True, name="Денчик", event=0, epithet=""):
    return M.SpeakerResult(utterance_id=utterance_id, is_known=is_known, name=name, event=event, epithet=epithet)


def test_candidate_with_a_voice_known_by_name_is_human(window, clock):
    admission = _greeting_then_voice(window, clock, END + 0.2, END + 1.2)
    window.on_speaker(_speaker("u1"))
    assert window.human_voice("u1")
    assert admission.echo(MIRROR, WAKE, 4.0, human_voice=window.human_voice("u1")) is None


@pytest.mark.parametrize(
    "result",
    [
        _speaker("u1", is_known=False, name=""),  # не узнан
        _speaker("u1", name="", epithet="Наблюдатель"),  # безымянный профиль: «Наблюдатель» вырос из эха (#4145)
        _speaker("u1", name=""),  # имя скрыто как спорное (#2809)
        _speaker("u1", event=1),  # служебный ответ, не опознание
        _speaker("", name="Денчик"),  # без фразы
    ],
)
def test_voice_not_known_by_name_decides_nothing_and_words_catch_echo(window, clock, result):
    admission = _greeting_then_voice(window, clock, END + 0.2, END + 1.2)
    window.on_speaker(result)
    assert not window.human_voice("u1")
    assert admission.echo(MIRROR, WAKE, 4.0, human_voice=window.human_voice("u1")) is not None


def test_known_voice_is_remembered_for_a_bounded_number_of_phrases(window):
    for i in range(ew.VOICES_KEPT + 1):
        window.on_speaker(_speaker(f"u{i}"))
    assert not window.human_voice("u0") and window.human_voice(f"u{ew.VOICES_KEPT}")


def test_voice_does_not_rescue_echo_outside_the_reply_window(window, clock):
    """Голос решает только в окне ответа (кандидат REPLY): эхо-окно без окна ответа по-прежнему пропускает лишь стоп."""
    admission = _greeting_then_voice(window, clock, END + 0.2, END + 1.2, reply_open=False)
    assert admission.echo(MIRROR, WAKE, 4.0, human_voice=True) is not None


def test_watch_subscribes_to_speaker_results():
    node = MagicMock()
    subs = {}
    node.create_subscription.side_effect = lambda msg_type, topic, cb, qos: subs.setdefault(topic, cb)
    window = ew.watch(node, grace_s=GRACE_S, aec_mode="hardware", vad_type=bool)
    subs[ew.SPEAKER_RESULT_TOPIC](_speaker("u7"))
    assert window.human_voice("u7")


# --- склейка из прошлого PR и кейсы из логов #4509 -----------------------------------------------------------------

LINE_A, LINE_A_ID = "Принял, не забыл — слышу тебя. Скажи, что именно сделать?", "f3fcb6bb"
LINE_B, LINE_B_ID = (
    "И я тебя не забыл, я же РОББОКС и всё помню. Только не понял, что именно ты мне сказал — повтори ещё раз.",
    "61e9a8e8",
)
GLUED_ECHO = "Скажи, что именно сделать, и я тебя не забуду."


@pytest.mark.parametrize("reply_open", [False, True])
def test_glued_echo_of_two_lines_is_still_echo(window, clock, reply_open):
    """Лог 16:14: A 0–5 с, B с 4,9 с; VAD 3,3–6,3 с (во время речи), доставка в 6,37 — B ещё звучит."""
    events = [
        (-1.4, "request", (LINE_A_ID, LINE_A)),
        (0.0, "started", LINE_A_ID),
        (2.3, "request", (LINE_B_ID, LINE_B)),
        (3.3, "vad", True),
        (4.9, "started", LINE_B_ID),
        (5.0, "finished", LINE_A_ID),
        (6.3, "vad", False),
    ]
    if reply_open:
        events.insert(0, (-2.0, "reply", 17.5))
    _play(window, clock, events)
    clock.at(6.37)
    admission = window.admit(duration_s=4.57)
    echo = admission.echo(GLUED_ECHO, WAKE, 4.57)
    assert echo is not None and echo.speech_id == f"{LINE_A_ID}+{LINE_B_ID}"


def test_glued_echo_delivered_after_both_lines_in_the_reply_window_is_echo(window, clock):
    """Тот же голос во время речи, но доставлен, когда обе реплики отзвучали, а окно ответа открыто, — кандидат, эхо."""
    _play(
        window,
        clock,
        [
            (-2.0, "reply", 30.0),
            (-1.4, "request", (LINE_A_ID, LINE_A)),
            (0.0, "started", LINE_A_ID),
            (2.3, "request", (LINE_B_ID, LINE_B)),
            (3.3, "vad", True),
            (4.9, "started", LINE_B_ID),
            (5.0, "finished", LINE_A_ID),
            (6.3, "vad", False),
            (8.0, "finished", LINE_B_ID),
        ],
    )
    clock.at(9.3)
    admission = window.admit(duration_s=6.0)
    assert admission.verdict == ew.REPLY
    assert admission.echo(GLUED_ECHO, WAKE, 6.0) is not None


def test_log_4509_reply_at_0_18s_with_new_words_is_taken(window, clock):
    """16:13:41.224: голос через 0,18 с после конца приветствия — кандидат, но слова новые: человек."""
    admission = _greeting_then_voice(window, clock, END + 0.18, END + 1.5)
    assert admission.verdict == ew.REPLY
    assert admission.echo("Добрый вечер, нет, не забыл про тебя.", WAKE, 4.35) is None
    assert admission.echo("добрый вечер робот как дела", WAKE, 4.61) is None


def test_log_4509_greeting_echo_during_playback_is_echo(window, clock):
    """Первая строка лога #4509 (верно): эхо приветствия 10/10, голос во время речи — эхо и при открытом окне."""
    admission = _greeting_then_voice(window, clock, 1.06, END + 0.1)
    echo = admission.echo(ECHO_OF_GREETING, WAKE, 4.1)
    assert echo is not None and "слов 10/10" in echo.note


def test_log_4509_reply_after_the_echo_window_passes_as_before(window, clock):
    """16:13:46.328: голос через 5,3 с после конца речи — в тишине, слова не сверяются (как до #4509)."""
    admission = _greeting_then_voice(window, clock, END + 5.3, END + 6.5)
    assert admission.verdict == ew.PASS and not admission.spoken
    assert admission.echo("Я не забыл про тебя, говорю.", WAKE, 5.12) is None


# --- decide: правило времени в одном месте -------------------------------------------------------------------------


@pytest.mark.parametrize(
    "heard, began, verdict",
    [
        ({ew.GRACE}, ew.ECHO_TAIL_S, ew.REPLY_AFTER),
        ({ew.GRACE}, 1.5, ew.REPLY_AFTER),
        ({ew.GRACE}, ew.ECHO_TAIL_S - 0.01, ew.REPLY),
        ({ew.GRACE}, None, ew.REPLY),  # начало неизвестно (кусок без фронтов VAD) — кандидат
        ({ew.SPEECH, ew.GRACE}, float("-inf"), ew.REPLY),
        ({ew.SPEECH}, float("-inf"), ew.REPLY),
        ({ew.GRACE, ew.FREE}, 1.0, ew.PASS),
    ],
)
def test_decide_by_onset_time_in_the_reply_window(heard, began, verdict):
    kw = dict(speaking=False, duration_s=3.0, aec_mode="hardware", reply_open=True, began_s=began)
    assert ew.decide(heard, **kw).verdict == verdict


def test_decide_while_robot_speaks_keeps_the_old_paths():
    kw = dict(duration_s=0.5, aec_mode="hardware", reply_open=True, began_s=float("-inf"))
    assert ew.decide({ew.SPEECH}, speaking=True, **kw).verdict == ew.SHORT_STOP_ONLY
    assert ew.decide({ew.SPEECH}, speaking=True, duration_s=0.5, aec_mode="software", reply_open=True).dropped


def test_echo_tail_constant_is_the_measured_tract_delay_plus_margin():
    """0,25 с задержки тракта (замер, см. комментарий константы) + 0,25 с запаса; тест Шифу — 0,5–1,5 с — человек."""
    assert ew.ECHO_TAIL_S == pytest.approx(0.5)
