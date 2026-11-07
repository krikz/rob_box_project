"""#4509: ответ на приветствие выбрасывало эхо-окно; эхо склейки двух реплик робота не ловилось.

Живой робот 06.11 ~16:20 UTC (Sound Pi, ``docker logs voice-assistant``):

    [dialog] 🗣 [встреча] Денчик: 'О, Денчик! А я уж думал, ты про меня забыл. Добрый вечер.'
    [dialog] 💬 [reply-window] greeting Денчик: окно открыто ttl=15s (микрофон после речи +2.5s)
    [stt] 🔁 [echo] фраза совпадает с речью робота … (слов 10/10: 'о денчик а я уж думал ты про меня забыл')  ← верно
    [stt] 🔇 [issue 989/#4072] Эхо-окно: фраза 4.61с 'добрый вечер робот как дела' … — молчу      ← ответ выброшен
    [stt] 🔇 [issue 989/#4072] Эхо-окно: фраза 4.35с 'Добрый вечер, нет, не забыл про тебя.' … — молчу  ← второй раз

    [tts] TTS: speech_id=f3fcb6bb text='Принял, не забыл — слышу тебя. Скажи, что именно сделать?'
    [tts] TTS: speech_id=61e9a8e8 text='И я тебя не забыл, я же РОББОКС и всё помню. Только не понял, …'
    [stt] 🎤 [hardware AEC] Фраза 4.57с во время TTS — обрабатываем (возможно прерывание)
    [stt] ✅ ПРИНЯТО (respeaker): Скажи, что именно сделать, и я тебя не забуду.   ← эхо склейки двух реплик

Поведение модуля ``rob_box_voice.core.echo_window`` и окна ответа ``greeting_window`` (событие владельца окна).
"""

from __future__ import annotations

import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_dialog import address
from rob_box_dialog import knowledge as kn
from rob_box_dialog.words import DEFAULT_WAKE_WORDS
from rob_box_voice.core import echo_window as ew

M = install_dialog_msgs_stub()

from rob_box_dialog.session import Session  # noqa: E402
from rob_box_voice.core import greeting_window as gw  # noqa: E402
from rob_box_voice.core.identity_question import publish  # noqa: E402

WAKE = list(DEFAULT_WAKE_WORDS)
GRACE_S = 2.5
GREETING = "О, Денчик! А я уж думал, ты про меня забыл. Добрый вечер."
GREETING_ID = "greet0001"
ECHO_OF_GREETING = "о денчик а я уж думал ты про меня забыл"
REPLY_1 = "добрый вечер робот как дела"
REPLY_2 = "Добрый вечер, нет, не забыл про тебя."
LINE_A, LINE_A_ID = "Принял, не забыл — слышу тебя. Скажи, что именно сделать?", "f3fcb6bb"
LINE_B, LINE_B_ID = (
    "И я тебя не забыл, я же РОББОКС и всё помню. Только не понял, что именно ты мне сказал — повтори ещё раз.",
    "61e9a8e8",
)
GLUED_ECHO = "Скажи, что именно сделать, и я тебя не забуду."


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


def _robot(window: ew.EchoWindow, clock: Clock, events) -> None:
    """``(t, "request", (sid, text))``; ``(t, "started"|"finished", sid)``; ``(t, "vad", bool)``;
    ``(t, "reply", seconds)`` — событие окна ответа от ``dialog_node``."""
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


def _greeting_then_voice_in_grace(window: ew.EchoWindow, clock: Clock, *, reply_open: bool) -> ew.Admission:
    """Приветствие звучит 0–4 с; окно ответа открывается на его конце (событие владельца); голос — только в эхо-окне
    (4,3–6,3 с), фраза доехала после ``speech_continuation``."""
    events = [
        (-0.2, "request", (GREETING_ID, GREETING)),
        (0.0, "started", GREETING_ID),
        (4.0, "finished", GREETING_ID),
    ]
    if reply_open:
        events.append((4.0, "reply", kn.IDENTITY_MIC_GRACE_S + kn.REPLY_WINDOW_S))
    events += [(4.3, "vad", True), (6.3, "vad", False)]
    _robot(window, clock, events)
    clock.at(9.3)
    return window.admit(duration_s=4.6)


# --- дефект 1: ответ человека в эхо-окне при открытом окне ответа -------------------------------------------------


@pytest.mark.parametrize("reply", [REPLY_1, REPLY_2])
def test_reply_with_new_words_in_the_echo_window_is_taken_while_the_reply_window_is_open(window, clock, reply):
    admission = _greeting_then_voice_in_grace(window, clock, reply_open=True)
    assert admission.verdict == ew.REPLY and "окно ответа открыто" in admission.note
    assert admission.keeps(reply, WAKE), "промежуточные результаты фразы не глушатся"
    assert admission.echo(reply, WAKE, 4.6) is None


def test_greeting_echo_in_the_reply_window_is_still_echo(window, clock):
    """Первая строка лога (верно): эхо приветствия словами 10/10 — не беру и при открытом окне."""
    admission = _greeting_then_voice_in_grace(window, clock, reply_open=True)
    echo = admission.echo(ECHO_OF_GREETING, WAKE, 4.6)
    assert echo is not None and echo.speech_id == GREETING_ID and "слов 10/10" in echo.note


def test_short_tail_of_the_greeting_in_the_reply_window_is_echo(window, clock):
    """Строгая сверка в окне ответа: короткий хвост приветствия целиком («добрый вечер») — эхо."""
    admission = _greeting_then_voice_in_grace(window, clock, reply_open=True)
    assert admission.echo("Добрый вечер.", WAKE, 1.0) is not None
    assert admission.echo("и тебе привет", WAKE, 1.0) is None


def test_without_the_reply_window_the_echo_window_still_passes_only_a_stop(window, clock):
    """Защита #989/#4072 там, где окна ответа нет: фраза только в эхо-окне — молча, кроме стопа с вейком."""
    admission = _greeting_then_voice_in_grace(window, clock, reply_open=False)
    assert admission.verdict == ew.STOP_ONLY
    echo = admission.echo(REPLY_2, WAKE, 4.35)
    assert echo is not None and "Эхо-окно" in echo.note
    assert admission.echo("Робот, хватит", WAKE, 1.0) is None


def test_closed_or_expired_reply_window_restores_the_echo_window(window, clock):
    _robot(window, clock, [(0.0, "reply", 17.5), (1.0, "reply", 0.0)])
    assert not window.reply_open()
    _robot(window, clock, [(2.0, "reply", 17.5)])
    assert window.reply_open()
    clock.at(2.0 + 17.6)
    assert not window.reply_open(), "событие закрытия потерялось — окно истекает по своему сроку"


def test_decide_keeps_reply_window_only_for_the_echo_window():
    """Окно ответа меняет только вердикт фразы из эхо-окна; software-AEC при речи робота по-прежнему режет."""
    kw = dict(duration_s=2.0, aec_mode="hardware", reply_open=True)
    assert ew.decide({ew.GRACE}, speaking=False, **kw).verdict == ew.REPLY
    assert ew.decide({ew.GRACE, ew.FREE}, speaking=False, **kw).verdict == ew.PASS
    assert ew.decide({ew.SPEECH}, speaking=True, duration_s=0.5, aec_mode="hardware", reply_open=True).verdict == (
        ew.SHORT_STOP_ONLY
    )
    assert ew.decide({ew.GRACE}, speaking=True, duration_s=2.0, aec_mode="software", reply_open=True).dropped


# --- дефект 5: эхо склейки двух реплик робота ---------------------------------------------------------------------


def _two_lines_then_glued_echo(window: ew.EchoWindow, clock: Clock) -> ew.Admission:
    """Тайминг из лога: A звучит 1643.0–1648.0, B стартует 1647.9; фраза 4,57 с доехала в 1649.37 (B ещё звучит)."""
    _robot(
        window,
        clock,
        [
            (-1.4, "request", (LINE_A_ID, LINE_A)),
            (0.0, "started", LINE_A_ID),
            (2.3, "request", (LINE_B_ID, LINE_B)),
            (3.3, "vad", True),
            (4.9, "started", LINE_B_ID),
            (5.0, "finished", LINE_A_ID),
            (6.3, "vad", False),
        ],
    )
    clock.at(6.37)
    return window.admit(duration_s=4.57)


def test_glued_echo_of_two_robot_lines_is_echo(window, clock):
    admission = _two_lines_then_glued_echo(window, clock)
    assert {s.speech_id for s in admission.spoken} == {LINE_A_ID, LINE_B_ID}
    echo = admission.echo(GLUED_ECHO, WAKE, 4.57)
    assert echo is not None and echo.speech_id == f"{LINE_A_ID}+{LINE_B_ID}" and "слов 8/9" in echo.note


def test_each_robot_line_alone_does_not_make_the_glued_phrase_an_echo():
    """Почему не ловилось: с каждой репликой по отдельности совпадает 4 из 9 слов — меньше доли ECHO_WORD_SHARE."""
    assert ew.echo_of(GLUED_ECHO, [ew.Spoken(LINE_A_ID, LINE_A)], WAKE) is None
    assert ew.echo_of(GLUED_ECHO, [ew.Spoken(LINE_B_ID, LINE_B)], WAKE) is None
    both = [ew.Spoken(LINE_A_ID, LINE_A), ew.Spoken(LINE_B_ID, LINE_B)]
    assert ew.echo_of(GLUED_ECHO, both, WAKE) is not None


def test_human_phrase_next_to_two_robot_lines_is_not_echo(window, clock):
    admission = _two_lines_then_glued_echo(window, clock)
    assert admission.echo("Включи музыку погромче пожалуйста", WAKE, 4.57) is None
    assert admission.echo("Робот, а ты меня помнишь вообще", WAKE, 4.57) is None


def test_joined_lines_are_limited_to_neighbours():
    lines = [ew.Spoken(str(i), f"слово{i}") for i in range(5)]
    joined = [s for s in ew._lines(lines) if "+" in s.speech_id]
    assert max(len(s.speech_id.split("+")) for s in joined) == ew.ECHO_JOIN_LINES
    assert all(
        [int(x) for x in s.speech_id.split("+")] == list(range(int(s.speech_id[0]), int(s.speech_id[0]) + n))
        for s in joined
        for n in [len(s.speech_id.split("+"))]
    )


# --- окно ответа шлёт событие владельца (greeting_window) ---------------------------------------------------------


class _Clock:
    t = 1000.0

    def __call__(self):
        return self.t


def _node(clock):
    node = types.SimpleNamespace(_session_pub=MagicMock(), announce_reply_window=MagicMock())
    logger = MagicMock()
    node.get_logger = lambda: logger
    node._session = Session("personality", clock=clock)
    node._session.subscribe(lambda event: publish(node, event))
    return node


def _greeted(clock):
    node = _node(clock)
    gw.open_window(node, "p1", "Денчик", "g1")
    assert not node.announce_reply_window.called, "пока приветствие звучит, окна нет"
    gw.on_speech_event(node, 4, "g1", "personality")
    return node


def test_greeting_window_announces_open_and_close_to_stt():
    clock = _Clock()
    node = _greeted(clock)
    node.announce_reply_window.assert_called_once_with(float(kn.IDENTITY_MIC_GRACE_S + kn.REPLY_WINDOW_S))
    gw.take_reply(node, address.GREETING_REPLY, types.SimpleNamespace(source="respeaker", text="привет"))
    assert node.announce_reply_window.call_args.args == (0.0,) and gw.window(node) is None


def test_bare_wake_in_the_greeting_window_reopens_it_and_announces_again():
    """Дефект 2: голое «робот» в окне не снимает окно, а открывает заново (робот сказал «Слушаю.»)."""
    clock = _Clock()
    node = _greeted(clock)
    clock.t += 10.0
    gw.take_reply(node, address.GREETING_WAKE, types.SimpleNamespace(source="respeaker", text="робот"))
    frame = gw.window(node)
    assert frame is not None and frame.payload["asked_at"] == clock.t + kn.IDENTITY_MIC_GRACE_S
    assert node.announce_reply_window.call_count == 2
    clock.t += 14.0  # старое окно уже истекло бы (10 + 14 > 17,5), новое живо
    assert gw.window(node) is not None


def test_node_without_announcer_keeps_working():
    clock = _Clock()
    node = _node(clock)
    del node.announce_reply_window
    gw.open_window(node, "p1", "Денчик", "g1")
    gw.on_speech_event(node, 4, "g1", "personality")
    assert gw.window(node) is not None
