"""Эхо словами (#4145): собственная речь робота, распознанная STT, — не реплика человека.

Сцена из живого лога 28.10 (агитация из Telegram, docker logs voice-assistant, UTC; текст и время — из карточки):

    08:58:11.463 SpeechEvent started speech_id=46fd3b27 agent=telegram  ← «Дорогие друзья! Слушайте и не говорите, …»
    08:58:11.883 VAD «речь»
    08:58:16.910 🎤 [hardware AEC] Фраза 5.38с во время TTS — обрабатываем (возможно прерывание)
    08:58:18.657 ✅ ПРИНЯТО (respeaker): дорогие друзья слушайте и не говорите что не слышали
    08:58:50.383 SpeechEvent finished speech_id=46fd3b27

Текст реплики в карточке приведён только началом — тест берёт его, остальное не выдумывает.

Поведение модуля ``rob_box_voice.core.echo_window``: журнал речи (``SpeechRequest`` — текст по ``speech_id``,
``SpeechEvent`` — когда звучала), решение :meth:`Admission.echo` по словам фразы и :class:`EchoVerdicts` — биометрия
слушает то же решение (эхо не заводит профиль, фраза человека — как раньше).
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_core import speech_client
from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_dialog.words import DEFAULT_WAKE_WORDS, words_in_order
from rob_box_voice.core import echo_window as ew

M = install_dialog_msgs_stub()
WAKE = list(DEFAULT_WAKE_WORDS)
GRACE_S = 2.5
SPEECH_ID = "46fd3b27"
#: Начало реплики агитации из карточки #4145 (полный текст — 369 символов, в карточке не приведён).
AGITATION = "Дорогие друзья! Слушайте и не говорите, что не слышали! Наше ПТУ"
#: Что распознал STT в 08:58:18.657 — слово в слово начало реплики робота.
ECHO_TEXT = "дорогие друзья слушайте и не говорите что не слышали"


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


def _request(text: str, sid: str = SPEECH_ID, commit: bool = True):
    return M.SpeechRequest(
        speech_id=sid, text=text, commit=commit, agent="telegram", sink="speakers"
    )


def _event(kind: int, sid: str = SPEECH_ID):
    return M.SpeechEvent(kind=kind, speech_id=sid, agent="telegram")


def _robot(window: ew.EchoWindow, clock: Clock, events) -> None:
    """``(t, "request", text)`` — запрос речи; ``(t, "started"|"finished", sid)``; ``(t, "vad", bool)``."""
    for t, kind, value in events:
        clock.at(t)
        if kind == "request":
            window.journal.on_request(_request(value))
        elif kind == "vad":
            window.on_vad(value)
        else:
            code = (
                M.SpeechEvent.KIND_STARTED
                if kind == "started"
                else M.SpeechEvent.KIND_FINISHED
            )
            speaking = window.journal.on_event(_event(code, value))
            if speaking is not None:
                window.on_speech(speaking)


def _agitation_until_delivery(window: ew.EchoWindow, clock: Clock) -> ew.Admission:
    """Реплика агитации звучит, голос в микрофоне с 0,42 с, фраза 5,38 с доехала на 5,45 с (робот ещё говорит)."""
    _robot(
        window,
        clock,
        [
            (-0.2, "request", AGITATION),
            (0.0, "started", SPEECH_ID),
            (0.42, "vad", True),
            (5.4, "vad", False),
        ],
    )
    clock.at(5.447)
    return window.admit(duration_s=5.38)


# --- сравнение по словам (одна нормализация — words) --------------------------------------------------------------


def test_words_in_order_counts_robot_words_said_back():
    assert words_in_order(ECHO_TEXT, AGITATION) == 9
    assert words_in_order("Дорогие, друзья!", "дорогие друзья") == 2
    assert words_in_order("друзья дорогие", "дорогие друзья") == 1  # порядок важен
    assert words_in_order("не говоришь", "не говорите") == 1  # слово — целиком
    assert words_in_order("", AGITATION) == 0 and words_in_order(ECHO_TEXT, "") == 0


# --- эхо агитации из лога ------------------------------------------------------------------------------------------


def test_agitation_echo_from_the_log_is_not_taken(window, clock):
    """Фраза во время речи пропускается к STT (может быть перебивание), но её слова — речь робота: эхо, не беру."""
    admission = _agitation_until_delivery(window, clock)
    assert admission.verdict == ew.PASS and "во время TTS" in admission.note
    assert admission.spoken == (ew.Spoken(SPEECH_ID, AGITATION),)
    echo = admission.echo(ECHO_TEXT, WAKE, 5.38)
    assert echo is not None and echo.speech_id == SPEECH_ID
    assert echo.note.startswith(
        f"🔁 [echo] фраза совпадает с речью робота speech_id={SPEECH_ID} — не беру"
    )
    assert "слов 9/9" in echo.note


def test_echo_with_stt_slips_is_still_echo(window, clock):
    """Эхо через динамик распознаётся с ошибками: 7 из 9 слов совпали — всё равно эхо (запас ECHO_WORD_SHARE)."""
    admission = _agitation_until_delivery(window, clock)
    assert (
        admission.echo("дорогие друзья слушай и не говори что не слышали", WAKE, 5.38)
        is not None
    )


def test_stop_with_wake_during_robot_speech_still_passes(window, clock):
    """«Робот, стоп» во время речи — как раньше: стоп важнее, даже если слова есть в речи робота."""
    admission = _agitation_until_delivery(window, clock)
    for stop in ("Робот, стоп", "робот хватит", "Робот, всё, хватит, замолчи."):
        assert admission.echo(stop, WAKE, 5.38) is None, stop


def test_human_phrase_during_robot_speech_that_is_not_its_words_passes(window, clock):
    """Чужая фраза во время речи (не слова робота) — как раньше: дальше в диалог (бэклог или перебивание)."""
    admission = _agitation_until_delivery(window, clock)
    for human in (
        "мама пришла давай ужинать",
        "робот который час",
        "что-то громко он говорит",
    ):
        assert admission.echo(human, WAKE, 5.38) is None, human


def test_two_robot_words_are_not_enough_for_echo(window, clock):
    """«друзья, слышали?» — оба слова есть в речи робота, но их два: коротко, совпасть может случайно — как раньше
    (ECHO_MIN_WORDS)."""
    admission = _agitation_until_delivery(window, clock)
    assert words_in_order("друзья, слышали?", AGITATION) == 2
    assert admission.echo("друзья, слышали?", WAKE, 5.38) is None


#: Реплика робота со служебными словами «а ты что» (зонд ревью #4161, ``probe_fp.py``).
ASKING = (
    "А ты что думаешь? Я вот считаю, что наше ПТУ лучшее в городе, приходите учиться"
)


def _long_speech(
    window: ew.EchoWindow, clock: Clock, text: str, sid: str = "s-ask"
) -> ew.Admission:
    """Длинная реплика ``text`` звучит, голос в микрофоне с 1,0 до 2,4 с, фраза 1,6 с доехала, робот ещё говорит."""
    clock.at(-0.2)
    window.journal.on_request(_request(text, sid=sid))
    _robot(
        window, clock, [(0.0, "started", sid), (1.0, "vad", True), (2.4, "vad", False)]
    )
    clock.at(2.5)
    return window.admit(duration_s=1.6)


def test_wake_the_robot_did_not_say_is_a_human_not_echo(window, clock):
    """Ревью #4161: «робот а ты что» во время длинной речи совпадала с ней служебными словами 3 из 4 и терялась эхом.
    Вейк, которого в реплике робота нет, — голос человека: фраза идёт как раньше (бэклог/перебивание).
    """
    admission = _long_speech(window, clock, ASKING)
    assert admission.spoken == (ew.Spoken("s-ask", ASKING),)
    assert words_in_order("робот а ты что", ASKING) == 3
    assert admission.echo("робот а ты что", WAKE, 1.6) is None
    assert (
        admission.echo("а ты что думаешь", WAKE, 1.6) is not None
    )  # без вейка — эхо, как и было


def test_wake_the_robot_said_itself_is_still_compared(window, clock):
    """Робот сам сказал «робот» («Я робот Роббокс…»): эхо этих слов с вейком — всё равно эхо."""
    said = "Я робот Роббокс, а ты что скажешь про наше ПТУ"
    admission = _long_speech(window, clock, said, sid="s-self")
    assert admission.echo("я робот роббокс а ты что скажешь", WAKE, 1.6) is not None


def test_answer_with_the_options_of_the_question_that_went_into_silence_is_a_human(
    window, clock
):
    """Ревью #4161 (``probe_fp.py``): «Рассказать сказку про колобка или про репку?» — ответ «сказку про репку» начался
    в эхо-окне и ушёл в тишину. Эхо собственного голоса за эхо-окном не звучит: в фразе человек, слова не сверяются.
    Голос с 4,5 по 6,7 с, фраза доехала через 3 с тишины: PCM 5,2 с — журнал речи вопрос робота видит.
    """
    question = "Рассказать сказку про колобка или про репку?"
    _robot(
        window,
        clock,
        [
            (-0.2, "request", question),
            (0.0, "started", SPEECH_ID),
            (3.0, "finished", SPEECH_ID),
            (4.5, "vad", True),
            (GRACE_S + 3.0 + 1.2, "vad", False),
        ],
    )
    clock.at(GRACE_S + 3.0 + 1.2 + 3.0)
    assert window.journal.sounded(5.2) == (ew.Spoken(SPEECH_ID, question),)
    admission = window.admit(duration_s=5.2)
    assert admission.verdict == ew.PASS and admission.spoken == ()
    for answer in ("сказку про репку", "расскажи сказку про колобка"):
        assert (
            ew.echo_of(answer, (ew.Spoken(SPEECH_ID, question),)) is not None
        )  # по словам — «эхо»
        assert admission.echo(answer, WAKE, 5.2) is None, answer


def test_echo_heard_during_speech_and_delivered_after_its_end_is_still_echo(
    window, clock
):
    """Голос звучал во время речи, фраза доехала после её конца (speech_continuation 3 с): реплика «только что
    сказана» — эхо по словам всё равно ловится."""
    _robot(
        window,
        clock,
        [
            (-0.2, "request", AGITATION),
            (0.0, "started", SPEECH_ID),
            (0.4, "vad", True),
            (3.8, "vad", False),
            (4.0, "finished", SPEECH_ID),
        ],
    )
    clock.at(6.8)
    admission = window.admit(duration_s=6.4)
    assert admission.verdict == ew.PASS
    assert admission.echo(ECHO_TEXT, WAKE, 6.4) is not None


def test_phrase_after_the_speech_outside_the_echo_window_passes_as_before(
    window, clock
):
    """После конца речи и эхо-окна голос звучал в тишине: даже повтор слов робота — человек (цитирует), как раньше."""
    _robot(
        window,
        clock,
        [
            (-0.2, "request", AGITATION),
            (0.0, "started", SPEECH_ID),
            (38.9, "finished", SPEECH_ID),
            (45.0, "vad", True),
            (48.0, "vad", False),
        ],
    )
    clock.at(51.0)
    admission = window.admit(duration_s=6.0)
    assert admission.verdict == ew.PASS and admission.spoken == ()
    assert admission.echo(ECHO_TEXT, WAKE, 6.0) is None


def test_prefetch_of_a_reply_is_not_speech(window, clock):
    """``commit=false`` — прогрев кэша синтеза: текста в журнале нет, сверять не с чем."""
    window.journal.on_request(_request(AGITATION, commit=False))
    _robot(
        window,
        clock,
        [(0.0, "started", SPEECH_ID), (0.4, "vad", True), (5.4, "vad", False)],
    )
    clock.at(5.45)
    admission = window.admit(duration_s=5.38)
    assert admission.spoken == (ew.Spoken(SPEECH_ID, ""),)
    assert admission.echo(ECHO_TEXT, WAKE, 5.38) is None


def test_echo_window_only_phrase_is_echo_with_its_old_line(window, clock):
    """#4072 не тронут: голос только в эхо-окне и не стоп с вейком — эхо (строка лога прежняя)."""
    _robot(
        window,
        clock,
        [
            (0.0, "started", "s1"),
            (5.0, "finished", "s1"),
            (5.2, "vad", True),
            (5.6, "vad", False),
        ],
    )
    clock.at(8.6)
    echo = window.admit(duration_s=3.4).echo("тебя зовут робот робби", WAKE, 3.4)
    assert (
        echo is not None
        and echo.speech_id == ""
        and "[issue 989/#4072] Эхо-окно" in echo.note
    )


def test_lost_finished_event_does_not_keep_the_speech_sounding_forever(window, clock):
    """Реплика без события конца дольше ``SPEAKING_STALE_S`` — отзвучала: журнал её больше не даёт."""
    _robot(window, clock, [(-0.2, "request", AGITATION), (0.0, "started", SPEECH_ID)])
    clock.at(speech_client.SPEAKING_STALE_S + GRACE_S + 10.0)
    assert window.journal.sounded(5.0) == ()


# --- биометрия слушает то же решение -------------------------------------------------------------------------------


@pytest.fixture
def verdicts(clock: Clock):
    submitted = []
    log = MagicMock()
    hold = ew.EchoVerdicts(GRACE_S, submit=submitted.append, log=log, clock=clock)
    hold.journal.on_event(_event(M.SpeechEvent.KIND_STARTED))
    clock.at(5.45)
    return SimpleNamespace(hold=hold, submitted=submitted, log=log)


def test_biometry_never_changes_profiles_from_a_phrase_found_to_be_echo(verdicts):
    """Эмбеддинг агитации готов раньше текста: профили ждут вердикта; вердикт «эхо» — run не зовётся никогда."""
    ran = []
    verdicts.hold.heard("u-echo", 5.38)
    verdicts.hold.settle("u-echo", lambda: ran.append("u-echo"))
    assert ran == [] and verdicts.submitted == []
    verdicts.hold.on_verdict("u-echo", True)
    verdicts.hold.expire()
    assert ran == [] and verdicts.submitted == []
    line = verdicts.log.info.call_args.args[0]
    assert (
        "utterance_id=u-echo — эхо речи робота, профиль не завожу и не обновляю" in line
    )


def test_biometry_of_a_human_phrase_near_robot_speech_goes_on_after_the_verdict(
    verdicts,
):
    """Человек перебил робота: фраза ушла в диалог (``/voice/stt/utterance``) — профили как раньше."""
    run = MagicMock()
    verdicts.hold.heard("u-human", 4.0)
    verdicts.hold.settle("u-human", run)
    verdicts.hold.on_verdict("u-human", False)
    assert verdicts.submitted == [run]


def test_verdict_that_came_before_the_embedding_is_kept(verdicts):
    """STT обогнал биометрию (холодный энкодер): вердикт ждёт эмбеддинга."""
    echo_run, human_run = MagicMock(), MagicMock()
    verdicts.hold.heard("u-echo", 5.38)
    verdicts.hold.heard("u-human", 4.0)
    verdicts.hold.on_verdict("u-echo", True)
    verdicts.hold.on_verdict("u-human", False)
    verdicts.hold.settle("u-echo", echo_run)
    verdicts.hold.settle("u-human", human_run)
    echo_run.assert_not_called()
    human_run.assert_called_once_with()


def test_phrase_in_silence_changes_profiles_at_once_as_before(clock):
    """Робот молчит давно (журнал пуст рядом с фразой) — биометрия ничего не ждёт."""
    hold = ew.EchoVerdicts(GRACE_S, submit=MagicMock(), log=MagicMock(), clock=clock)
    run = MagicMock()
    hold.heard("u-quiet", 4.0)
    hold.settle("u-quiet", run)
    run.assert_called_once_with()


def test_no_verdict_in_time_changes_no_profile_and_publishes_nothing_late(
    verdicts, clock
):
    """Ревью #4161: текста нет (STT пуст или короткий) — вердикта нет за VERDICT_TIMEOUT_S. Фраза рядом с речью робота
    не стала репликой человека: профили не меняются и позднего результата ``enroll`` в случайный момент нет.
    """
    run = MagicMock()
    verdicts.hold.heard("u-mute", 5.0)
    verdicts.hold.settle("u-mute", run)
    clock.t += ew.VERDICT_TIMEOUT_S - 1.0
    verdicts.hold.expire()
    assert verdicts.submitted == []
    clock.t += 2.0
    verdicts.hold.expire()
    run.assert_not_called()
    assert verdicts.submitted == []
    line = verdicts.log.info.call_args.args[0]
    assert (
        "вердикта по utterance_id=u-mute нет за 20с" in line
        and "профиль не завожу и не обновляю" in line
    )


def test_embedding_after_the_deadline_finds_the_timed_out_phrase(verdicts, clock):
    """Холодный энкодер (#2885) досчитал эмбеддинг позже срока, вердикта так и не было: профили не меняются, как и у
    фразы, досчитанной вовремя (запись срока помнится до вытеснения)."""
    run = MagicMock()
    verdicts.hold.heard("u-cold", 5.0)
    clock.t += ew.VERDICT_TIMEOUT_S + 1.0
    verdicts.hold.expire()
    assert verdicts.hold.waits("u-cold")
    verdicts.hold.settle("u-cold", run)
    run.assert_not_called()
    assert not verdicts.hold.waits("u-cold")


def test_human_verdict_after_the_deadline_changes_nothing_and_says_so(verdicts, clock):
    """STT распознал фразу позже срока: профиль по ней уже не менялся — строка в лог, изменения не исполняются."""
    run = MagicMock()
    verdicts.hold.heard("u-slow", 5.0)
    verdicts.hold.settle("u-slow", run)
    clock.t += ew.VERDICT_TIMEOUT_S + 1.0
    verdicts.hold.expire()
    verdicts.hold.on_verdict("u-slow", False)
    run.assert_not_called()
    assert "пришёл после срока" in verdicts.log.info.call_args.args[0]
    verdicts.log.warning.assert_not_called()


def test_phrase_without_embedding_is_forgotten(verdicts):
    """Эмбеддинга нет (``embed_audio`` вернул None) — менять в профилях нечего: ожидание снимается сразу, не висит до
    вытеснения (ревью #4161)."""
    verdicts.hold.heard("u-noemb", 5.0)
    assert verdicts.hold.waits("u-noemb")
    verdicts.hold.forget("u-noemb")
    assert not verdicts.hold.waits("u-noemb")


def test_held_phrases_are_bounded(verdicts):
    """Сколько бы фраз ни ждало вердикта, биометрия помнит не больше VERDICTS_KEPT — старейшие забываются."""
    for i in range(ew.VERDICTS_KEPT + 5):
        verdicts.hold.heard(f"u-{i}", 5.0)
    assert not verdicts.hold.waits("u-0") and verdicts.hold.waits(
        f"u-{ew.VERDICTS_KEPT + 4}"
    )
    assert len(verdicts.hold._held) == ew.VERDICTS_KEPT


def test_echo_verdict_for_a_phrase_biometry_did_not_hold_is_loud(clock):
    """Окна речи у нод разошлись: эхо пришло по фразе, которую биометрия не ждала — предупреждение, не молчание."""
    log = MagicMock()
    hold = ew.EchoVerdicts(GRACE_S, submit=MagicMock(), log=log, clock=clock)
    hold.on_verdict("u-late", True)
    hold.on_verdict("", False)
    assert log.warning.call_count == 1 and "u-late" in log.warning.call_args.args[0]


def test_listen_wires_biometry_to_robot_speech_and_both_verdict_topics(clock):
    """``listen``: SpeechEvent — журнал, ``/voice/stt/utterance`` — человек, ``/voice/stt/echo`` — эхо; таймер срока."""
    subs, timers = {}, []
    node = SimpleNamespace(
        get_logger=MagicMock,
        create_subscription=lambda msg_type, topic, cb, qos: subs.__setitem__(
            topic, (msg_type, cb)
        ),
        create_timer=lambda period, cb: timers.append((period, cb)),
    )
    submitted = []
    hold = ew.listen(node, submit=submitted.append, clock=clock)
    assert subs[speech_client.SPEECH_EVENT_TOPIC][0] is M.SpeechEvent
    assert (
        subs[ew.UTTERANCE_TOPIC][0] is M.Utterance
        and subs[ew.ECHO_TOPIC][0] is M.Utterance
    )
    assert timers == [(ew.VERDICT_TICK_S, hold.expire)]
    subs[speech_client.SPEECH_EVENT_TOPIC][1](_event(M.SpeechEvent.KIND_STARTED))
    clock.at(5.45)
    echo_run, human_run = MagicMock(), MagicMock()
    hold.heard("u-echo", 5.38)
    hold.heard("u-human", 4.0)
    hold.settle("u-echo", echo_run)
    hold.settle("u-human", human_run)
    subs[ew.ECHO_TOPIC][1](
        M.Utterance(utterance_id="u-echo", text=ECHO_TEXT, source="respeaker")
    )
    subs[ew.UTTERANCE_TOPIC][1](
        M.Utterance(utterance_id="u-human", text="робот стоп", source="respeaker")
    )
    assert submitted == [human_run]
