"""Эхо-окно после речи робота (#989 Fix B, #4072): решает, где звучал голос фразы, а не когда она доехала.

Поведение модуля ``rob_box_voice.core.echo_window`` на сценах с часами теста: фронты речи робота (``SpeechEvent``),
фронты ``/audio/vad`` и доставка фразы через ``speech_continuation`` (3 с на роботе) после последнего VAD.
Сцены — из живых логов: марафон 27.10 n805 (стоп в паузе между сегментами сказки), va_marathon.log 23.10
22:48:33 и 22:53:21 (фраза человека во время реплики, доставленная после её конца).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from rob_box_dialog.words import DEFAULT_WAKE_WORDS
from rob_box_voice.core import echo_window as ew

WAKE = list(DEFAULT_WAKE_WORDS)
GRACE_S = 2.5


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


def _script(window: ew.EchoWindow, clock: Clock, events) -> None:
    """События по времени: ``("speech", True|False)`` — фронт речи робота, ``("vad", True|False)`` — фронт VAD."""
    for t, kind, value in events:
        clock.at(t)
        (window.on_speech if kind == "speech" else window.on_vad)(value)


# --- сцена задачи: «Робот, хватит» в стык двух сегментов выступления -----------------------------------------------


def test_stop_in_the_pause_between_segments_passes(window, clock):
    """n805 (27.10): сегмент кончился, человек сказал стоп в паузе 1,8 с, следующий сегмент начался. Голос звучал
    только в эхо-окне — распознать и пропустить ясный стоп с вейком."""
    _script(
        window,
        clock,
        [
            (0.0, "speech", True),  # сегмент 9
            (8.0, "speech", False),  # segment_finished
            (8.3, "vad", True),  # «Робот, всё, хватит, замолчи»
            (9.6, "vad", False),
            (9.8, "speech", True),  # segment_started 10
        ],
    )
    clock.at(12.6)  # доставка: последний VAD + speech_continuation 3 с
    admission = window.admit(duration_s=1.5 + 1.3 + 3.0)
    assert admission.verdict == ew.STOP_ONLY
    assert not admission.dropped
    assert admission.keeps("Робот, всё, хватит, замолчи.", WAKE)
    assert admission.keeps("робот хватит", WAKE)


def test_echo_of_the_segment_in_the_same_pause_is_cut(window, clock):
    """Та же пауза, но в ней — хвост собственного голоса: не стоп с вейком — молча режется, как раньше."""
    _script(
        window,
        clock,
        [
            (0.0, "speech", True),
            (8.0, "speech", False),
            (8.2, "vad", True),
            (8.6, "vad", False),
        ],
    )
    clock.at(11.6)
    admission = window.admit(duration_s=3.4)
    assert admission.verdict == ew.STOP_ONLY
    for echo in (
        "а кот посмотрел на меня и сказал",
        "робот расскажи дальше",
        "хватит",
        "",
        None,
    ):
        assert not admission.keeps(echo, WAKE), echo


def test_stop_said_in_the_pause_and_running_into_the_next_segment_is_a_barge_in(
    window, clock
):
    """Голос начался в паузе и продолжился в следующем сегменте — это перебивание (как во время речи): дальше
    фраза идёт как обычно, стоп решает диалог."""
    _script(
        window,
        clock,
        [
            (0.0, "speech", True),
            (8.0, "speech", False),
            (9.0, "vad", True),
            (9.8, "speech", True),
            (10.6, "vad", False),
        ],
    )
    clock.at(13.6)
    admission = window.admit(duration_s=4.6)
    assert admission.verdict == ew.PASS
    assert admission.keeps("робот хватит замолчи", WAKE)


# --- эхо собственной речи сразу после реплики — режется как раньше ---------------------------------------------------


def test_echo_right_after_a_reply_is_cut_even_when_delivered_after_the_window(
    window, clock
):
    """Эхо 0,2–0,6 с после конца реплики доезжает через 3 с тишины — уже ПОСЛЕ окна. Раньше его не было слышно
    (audio_node глох); теперь решает место звука: только в окне — эхо."""
    _script(
        window,
        clock,
        [
            (0.0, "speech", True),
            (5.0, "speech", False),
            (5.2, "vad", True),
            (5.6, "vad", False),
        ],
    )
    clock.at(8.6)
    admission = window.admit(duration_s=3.4)
    assert admission.verdict == ew.STOP_ONLY
    assert not admission.keeps("тебя зовут робот робби", WAKE)
    assert "эхо-окне" in admission.note


# --- то, что окно больше не съедает --------------------------------------------------------------------------------


def test_phrase_spoken_during_a_reply_and_delivered_after_its_end_passes(window, clock):
    """va_marathon.log 23.10 22:53:21: человек говорил 16.24–18.45 во время реплики (конец 19.60), фраза доехала
    в 21.45 — «grace 1.90с/2.5с», выброшена как эхо. Голос звучал во время речи — это перебивание, не эхо.
    """
    _script(
        window,
        clock,
        [
            (10.3, "speech", True),
            (16.24, "vad", True),
            (18.45, "vad", False),
            (19.60, "speech", False),
        ],
    )
    clock.at(21.45)
    admission = window.admit(duration_s=6.62)
    assert admission.verdict == ew.PASS
    assert admission.keeps("да давай", WAKE)


def test_answer_started_in_the_window_and_running_past_it_passes(window, clock):
    """Быстрый ответ: начался через 1 с после вопроса и шёл дальше окна. Раньше микрофон слышал только хвост
    после окна; теперь фраза целиком, голос был и вне окна — не эхо."""
    _script(
        window,
        clock,
        [
            (0.0, "speech", True),
            (4.0, "speech", False),
            (5.0, "vad", True),
            (8.0, "vad", False),
        ],
    )
    clock.at(11.0)
    assert window.admit(duration_s=7.5).verdict == ew.PASS


def test_phrase_after_the_window_passes_silently(window, clock):
    _script(
        window,
        clock,
        [
            (0.0, "speech", True),
            (4.0, "speech", False),
            (7.0, "vad", True),
            (8.0, "vad", False),
        ],
    )
    clock.at(11.0)
    admission = window.admit(duration_s=5.5)
    assert (admission.verdict, admission.note) == (ew.PASS, "")


def test_robot_never_spoke_is_free(window, clock):
    _script(window, clock, [(0.0, "vad", True), (1.0, "vad", False)])
    clock.at(4.0)
    assert window.admit(duration_s=5.5) == ew.Admission(ew.PASS)


# --- речь сейчас и AEC: как было до #4072 ---------------------------------------------------------------------------


def test_hardware_short_blip_while_speaking_is_recognized_and_cut_unless_it_is_a_stop(
    window, clock
):
    """Короткая фраза (< 0,8 с) при звучащей речи больше не режется ДО STT (ревью #4122: «Робот, стоп» бывает короче
    порога, и в паузе между сегментами он пропадал): её распознают, а пропускают только ясный стоп с вейком — щелчок и
    эхо до диалога по-прежнему не доходят."""
    _script(
        window, clock, [(0.0, "speech", True), (1.0, "vad", True), (1.2, "vad", False)]
    )
    clock.at(1.3)
    admission = window.admit(duration_s=0.5)
    assert admission.verdict == ew.SHORT_STOP_ONLY and not admission.dropped
    for blip in ("", None, "щелчок", "робот", "хватит", "робот расскажи сказку"):
        assert not admission.keeps(blip, WAKE), blip
    assert admission.keeps("Робот, стоп.", WAKE)
    assert admission.keeps("робот хватит", WAKE)
    assert "Короткая фраза" in admission.echo_note("щелчок", 0.5)
    assert "эхо-окн" not in admission.echo_note("щелчок", 0.5)


def test_short_stop_said_in_the_pause_and_delivered_while_the_next_segment_speaks_passes(
    window, clock
):
    """Сцена ревью #4122: «Робот, стоп» 0,6 с в паузе между сегментами; фраза доехала через speech_continuation, когда
    следующий сегмент уже звучит (``speaking``) и она короче порога - раньше DROP_SHORT выбрасывал её до STT.
    """
    _script(
        window,
        clock,
        [
            (0.0, "speech", True),
            (8.0, "speech", False),
            (8.3, "vad", True),
            (8.9, "vad", False),
            (9.8, "speech", True),  # следующий сегмент
        ],
    )
    clock.at(10.0)  # доставка до конца следующего сегмента
    admission = window.admit(duration_s=0.6)
    assert not admission.dropped and admission.verdict in ew.STOP_ONLY_VERDICTS
    assert admission.keeps("Робот, стоп.", WAKE)
    assert not admission.keeps("и тогда", WAKE)


def test_short_phrase_without_vad_fronts_while_speaking_is_a_stop_candidate_too(
    window, clock
):
    _script(window, clock, [(0.0, "speech", True)])
    clock.at(2.0)
    admission = window.admit(duration_s=0.4)
    assert admission.verdict == ew.SHORT_STOP_ONLY
    assert admission.keeps("робот стоп", WAKE) and not admission.keeps("робот", WAKE)


def test_hardware_long_phrase_while_speaking_goes_on_as_barge_in(window, clock):
    _script(
        window, clock, [(0.0, "speech", True), (1.0, "vad", True), (2.0, "vad", False)]
    )
    clock.at(5.0)
    admission = window.admit(duration_s=5.5)
    assert admission.verdict == ew.PASS
    assert "во время TTS" in admission.note


def test_software_aec_drops_everything_while_speaking_and_keeps_the_window(clock):
    window = ew.EchoWindow(GRACE_S, "software", clock=clock)
    _script(
        window, clock, [(0.0, "speech", True), (1.0, "vad", True), (2.0, "vad", False)]
    )
    clock.at(5.0)
    assert window.admit(duration_s=5.5).verdict == ew.DROP_SPEAKING
    _script(
        window, clock, [(6.0, "speech", False), (6.5, "vad", True), (7.0, "vad", False)]
    )
    clock.at(10.0)
    assert window.admit(duration_s=4.0).verdict == ew.STOP_ONLY


# --- учёт фронтов ---------------------------------------------------------------------------------------------------


def test_open_burst_at_delivery_belongs_to_the_next_phrase(window, clock):
    """Фраза доезжает после тишины; открытый всплеск VAD к этому моменту — уже следующая фраза."""
    _script(
        window,
        clock,
        [
            (0.0, "speech", True),
            (4.0, "speech", False),
            (4.5, "vad", True),
            (5.0, "vad", False),
        ],
    )
    clock.at(7.0)
    window.on_vad(True)  # следующая фраза — после окна
    clock.at(8.0)
    assert window.admit(duration_s=3.5).verdict == ew.STOP_ONLY
    clock.at(9.0)
    window.on_vad(False)
    clock.at(12.0)
    assert window.admit(duration_s=5.0).verdict == ew.PASS


def test_chunks_of_one_long_phrase_share_its_places(window, clock):
    """Речь длиннее speech_max_duration приходит кусками подряд (#3762): кусок без своих фронтов — места фразы."""
    _script(
        window,
        clock,
        [
            (0.0, "speech", True),
            (4.0, "speech", False),
            (4.5, "vad", True),
            (6.4, "vad", False),
        ],
    )
    clock.at(9.4)
    assert window.admit(duration_s=15.0).verdict == ew.STOP_ONLY
    clock.at(9.45)
    assert window.admit(duration_s=2.0).verdict == ew.STOP_ONLY
    clock.at(30.0)
    assert (
        window.admit(duration_s=2.0).verdict == ew.PASS
    )  # фронтов нет и давно — место сейчас


def test_without_vad_fronts_the_place_at_delivery_decides(window, clock):
    """Фронтов VAD нет (топик молчит) — решение по моменту доставки, как до #4072, но стоп проходит."""
    _script(window, clock, [(0.0, "speech", True), (4.0, "speech", False)])
    clock.at(5.0)
    admission = window.admit(duration_s=2.0)
    assert admission.verdict == ew.STOP_ONLY
    assert admission.keeps("робот стоп", WAKE)


def test_helmet_phrase_is_open():
    assert ew.OPEN.verdict == ew.PASS and not ew.OPEN.dropped
    assert ew.OPEN.keeps("что угодно", WAKE)


@pytest.mark.parametrize(
    "text, passes",
    [
        ("Робот, хватит", True),
        ("Робот, всё, хватит, замолчи.", True),
        ("робот стоп", True),
        (
            "старой робот хватит",
            True,
        ),  # хвост речи робота, склеенный с командой (живой лог 21.10)
        ("хватит", False),  # без вейка
        ("Робот, расскажи сказку", False),  # не стоп
        ("робот спой песню про стоп", False),  # спорный стоп — не ясный
        ("у меня всё работает", False),
    ],
)
def test_only_clear_stop_with_wake_word_passes_the_window(text, passes):
    assert ew.stop_passes(text, WAKE) is passes


def _repo_root() -> Path:
    here = Path(__file__).resolve()
    return next(
        p for p in here.parents if (p / "docker").is_dir() and (p / "src").is_dir()
    )


@pytest.mark.parametrize(
    "path",
    [
        "src/rob_box_voice/config/stt_node.yaml",
        "src/rob_box_voice/config/audio_node.yaml",
        "docker/sound/config/voice_assistant/stt_node.yaml",
        "docker/sound/config/voice_assistant/audio_node.yaml",
    ],
)
def test_window_length_lives_in_knowledge_not_in_node_yaml(path):
    """Длина окна — одна константа знания (``IDENTITY_MIC_GRACE_S``: дефолт stt_node и окна ответа диалога).
    Конфиги нод её не дублируют, у audio_node окна больше нет."""
    params = next(
        iter(yaml.safe_load((_repo_root() / path).read_text(encoding="utf-8")).values())
    )
    assert "tts_grace_s" not in params["ros__parameters"]
