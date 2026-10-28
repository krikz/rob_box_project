"""Эхо-гейты микрофона по речи робота (PR-15 15-0b, #3711; эхо-окно #4072).

``audio_node`` (pre-roll на фронтах речи) и ``stt_node`` (software-AEC глушит распознавание, эхо-окно после речи)
раньше держали каждый свой список строк String-топика состояния TTS — и списки разъезжались
(``stopped``/``tts_silero_warming`` залипали). Теперь оба подписаны одним помощником
``speech_client.watch_speaking`` на ``SpeechEvent``: «звучит» решает ``SpeakingWatch`` (реплика любого агента от
``started`` до ``finished|cancelled|rejected``), нода реагирует только на фронт.

#4072: эхо-окно после речи одно — у ``stt_node`` (``core/echo_window.py``) и решает по тому, где звучал голос фразы
(фронты ``/audio/vad``), а не по моменту доставки; ``audio_node`` после речи не глохнет и начатую фразу на фронте
не сбрасывает — иначе «Робот, хватит» в паузе между сегментами выступления терялся.

Ноды тянут rclpy/PyAudio/Vosk: модули грузятся с заглушками этих зависимостей (как в корневых тестах ``test/``)
и выгружаются после теста; события идут через настоящие подписки ``watch_speaking`` и ``/audio/vad``.
"""

from __future__ import annotations

import importlib
import sys
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core import speech_client
from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
SpeechEvent = M.SpeechEvent
NODE_MODULES = ("rob_box_voice.audio_node", "rob_box_voice.stt_node")
WAKE = ["робот", "робокс", "роббокс"]


class _Node:
    """rclpy.node.Node без ROS: подписки записываются по топику."""

    def __init__(self, *a, **kw):
        self.subscriptions_by_topic = {}
        self.publishers_by_topic = {}

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions_by_topic[topic] = (msg_type, callback)

    def create_publisher(self, msg_type, topic, qos):
        self.publishers_by_topic[topic] = MagicMock()
        return self.publishers_by_topic[topic]

    def get_logger(self):
        return MagicMock()


class Clock:
    def __init__(self) -> None:
        self.t = 5000.0

    def __call__(self) -> float:
        return self.t


def _forget_node_modules() -> None:
    import rob_box_voice

    for name in NODE_MODULES:
        sys.modules.pop(name, None)
        leaf = name.rsplit(".", 1)[1]
        if hasattr(rob_box_voice, leaf):
            delattr(rob_box_voice, leaf)


@pytest.fixture
def node_modules(monkeypatch):
    """``(audio_node, stt_node)`` поверх заглушек rclpy/std_msgs/PyAudio/Vosk; после теста модули выгружаются."""
    stubs = {
        "rclpy": types.SimpleNamespace(
            init=None, shutdown=None, spin=None, ok=lambda: False
        ),
        "rclpy.node": types.SimpleNamespace(Node=_Node),
        "rclpy.qos": MagicMock(),
        "std_msgs": MagicMock(),
        "std_msgs.msg": MagicMock(),
        "audio_common_msgs": MagicMock(),
        "audio_common_msgs.msg": MagicMock(),
        "pyaudio": MagicMock(),
        "vosk": MagicMock(),
        "grpc": MagicMock(),
        "rob_box_voice.utils.audio_utils": MagicMock(),
        "rob_box_voice.utils.respeaker_interface": MagicMock(),
    }
    for name, module in stubs.items():
        monkeypatch.setitem(sys.modules, name, module)
    _forget_node_modules()
    yield tuple(importlib.import_module(name) for name in NODE_MODULES)
    _forget_node_modules()


def _event(kind: int, sid: str = "s1", agent: str = "personality"):
    return SpeechEvent(kind=kind, speech_id=sid, agent=agent)


def _audio_node(module):
    node = module.AudioNode.__new__(module.AudioNode)
    _Node.__init__(node)
    node.get_logger = lambda: MagicMock()
    node.tts_active = False
    node.music_active, node.barge_in_with_music = False, True
    node.speech_audio_buffer, node.speech_prefetch_buffer, node.is_speeching = (
        b"robot, hva",
        b"tail",
        True,
    )
    node._robot_speech = speech_client.watch_speaking(node, node._on_robot_speech)
    return node


def _stt_node(module, aec_mode: str, clock=None):
    """STTNode без ROS с той же проводкой эхо-окна, что в ``__init__`` (``echo_window.watch``)."""
    from rob_box_voice.core import echo_window

    node = module.STTNode.__new__(module.STTNode)
    _Node.__init__(node)
    node.get_logger = lambda: MagicMock()
    node.aec_mode, node.tts_grace_s = aec_mode, 2.5
    node._echo = echo_window.watch(
        node,
        grace_s=node.tts_grace_s,
        aec_mode=aec_mode,
        vad_type=bool,
        clock=clock or Clock(),
    )
    return node


def _say(node, kind: int, sid: str = "s1", agent: str = "personality") -> None:
    msg_type, callback = node.subscriptions_by_topic[speech_client.SPEECH_EVENT_TOPIC]
    assert msg_type is SpeechEvent
    callback(_event(kind, sid, agent))


def _vad(node, active: bool) -> None:
    _, callback = node.subscriptions_by_topic["/audio/vad"]
    callback(types.SimpleNamespace(data=active))


# --- audio_node: фронты речи робота -------------------------------------------------------------------------------


def test_audio_node_keeps_listening_through_robot_speech_and_after_it(node_modules):
    """started: VAD проходит (barge-in #993), pre-roll чист; начатая фраза НЕ сбрасывается — это может быть стоп
    человека, начатый в паузе между сегментами (#4072). finished: VAD тоже проходит — эхо-окно у stt_node.
    """
    node = _audio_node(node_modules[0])
    _say(node, SpeechEvent.KIND_STARTED)
    assert node.tts_active is True
    assert (
        node.speech_audio_buffer,
        node.speech_prefetch_buffer,
        node.is_speeching,
    ) == (b"robot, hva", b"", True)
    assert node._vad_gated(True) is True
    node.speech_prefetch_buffer = b"robot voice"
    _say(node, SpeechEvent.KIND_FINISHED)
    assert node.tts_active is False
    assert (
        node.speech_prefetch_buffer == b""
    )  # хвост собственного голоса не станет pre-roll'ом
    assert node._vad_gated(True) is True  # микрофон не глохнет после речи (#4072)
    assert node._vad_gated(False) is False
    assert not hasattr(node, "tts_grace_s")


@pytest.mark.parametrize("end", [SpeechEvent.KIND_CANCELLED, SpeechEvent.KIND_REJECTED])
def test_audio_node_gate_does_not_stick_on_cancel_or_reject(node_modules, end):
    node = _audio_node(node_modules[0])
    _say(node, SpeechEvent.KIND_STARTED)
    _say(node, end)
    assert node.tts_active is False


def test_audio_node_ignores_synthesis_without_sound_and_overlap_ends_with_the_last_reply(
    node_modules,
):
    """queued/synthesized (пред-синтез выступления) — не речь; оператор поверх личности — конец по последней."""
    node = _audio_node(node_modules[0])
    _say(node, SpeechEvent.KIND_QUEUED)
    _say(node, SpeechEvent.KIND_SYNTHESIZED)
    assert node.tts_active is False and node.speech_prefetch_buffer == b"tail"
    _say(node, SpeechEvent.KIND_STARTED, "a")
    _say(node, SpeechEvent.KIND_STARTED, "op", agent="operator")
    _say(node, SpeechEvent.KIND_FINISHED, "a")
    assert node.tts_active is True
    _say(node, SpeechEvent.KIND_FINISHED, "op", agent="operator")
    assert node.tts_active is False


# --- stt_node: фронты речи робота -> эхо-окно ----------------------------------------------------------------------


@pytest.mark.parametrize("aec_mode", ["software", "hardware"])
def test_stt_node_robot_speaking_follows_speech_event(node_modules, aec_mode):
    """started -> робот говорит (software-AEC глушит фразы); finished -> конец речи открывает эхо-окно."""
    clock = Clock()
    node = _stt_node(node_modules[1], aec_mode, clock)
    _say(node, SpeechEvent.KIND_STARTED)
    assert node._echo.speaking is True
    clock.t += 3.0
    _say(node, SpeechEvent.KIND_FINISHED)
    assert node._echo.speaking is False
    assert node._echo.ended_at == clock.t
    assert node._echo.where() == "grace"


def test_stt_node_does_not_stick_on_cancelled_speech(node_modules):
    """STOP/barge-in (cancelled) — конец речи: иначе hardware-AEC молча режет «робот», «стоп», «да» (< 0.8 с)."""
    node = _stt_node(node_modules[1], "hardware")
    _say(node, SpeechEvent.KIND_STARTED)
    _say(node, SpeechEvent.KIND_CANCELLED)
    assert node._echo.speaking is False


# --- stt_node: фраза ReSpeaker через _process_audio ----------------------------------------------------------------


def _pipeline(module, clock, recognized):
    """stt_node, у которого распознавание отдаёт ``recognized``, а публикации — моки."""
    node = _stt_node(module, "hardware", clock)
    node.sample_rate, node.min_text_chars, node.wake_words = 16000, 3, WAKE
    node._last_speaker_tag, node._active_source = None, "respeaker"
    node._recognize_with_fallback = MagicMock(return_value=(recognized, []))
    node.publish_result = MagicMock()
    node.publish_state = MagicMock()
    node._publish_speaker = MagicMock()
    node._maybe_speak_unclear = MagicMock()
    return node


def _phrase(node, clock, at: float, seconds: float) -> None:
    clock.t = 5000.0 + at
    node.speech_audio_callback(
        types.SimpleNamespace(data=list(b"\x00\x00" * int(16000 * seconds)))
    )


def _segments_with_voice_in_the_pause(node, clock) -> None:
    """Сегмент выступления, пауза 1,8 с с голосом человека, следующий сегмент (марафон 27.10, n805)."""
    for at, step in (
        (0.0, lambda: _say(node, SpeechEvent.KIND_STARTED, "seg9")),
        (8.0, lambda: _say(node, SpeechEvent.KIND_FINISHED, "seg9")),
        (8.3, lambda: _vad(node, True)),
        (9.6, lambda: _vad(node, False)),
        (9.8, lambda: _say(node, SpeechEvent.KIND_STARTED, "seg10")),
    ):
        clock.t = 5000.0 + at
        step()


def test_stop_between_performance_segments_reaches_the_dialog(node_modules):
    """«Робот, всё, хватит, замолчи» в стык двух сегментов: распознан и опубликован (диалог рвёт выступление)."""
    clock = Clock()
    node = _pipeline(node_modules[1], clock, "робот всё хватит замолчи")
    _segments_with_voice_in_the_pause(node, clock)
    _phrase(node, clock, at=12.6, seconds=5.8)
    node._recognize_with_fallback.assert_called_once()
    assert node.publish_result.call_args.args[0] == "робот всё хватит замолчи"


@pytest.mark.parametrize("echo", ["и тогда кот посмотрел на меня", "да", None])
def test_echo_in_the_window_after_a_reply_is_dropped_silently(node_modules, echo):
    """Хвост собственной речи сразу после реплики: не стоп с вейком — не публикуется и без «не расслышал»."""
    clock = Clock()
    node = _pipeline(node_modules[1], clock, echo)
    for at, step in (
        (0.0, lambda: _say(node, SpeechEvent.KIND_STARTED)),
        (5.0, lambda: _say(node, SpeechEvent.KIND_FINISHED)),
        (5.2, lambda: _vad(node, True)),
        (5.6, lambda: _vad(node, False)),
    ):
        clock.t = 5000.0 + at
        step()
    _phrase(node, clock, at=8.6, seconds=3.4)
    node.publish_result.assert_not_called()
    node._maybe_speak_unclear.assert_not_called()
    node.publish_state.assert_called_with("ready")


def test_phrase_from_during_the_reply_delivered_after_it_is_not_dropped(node_modules):
    """va_marathon.log 23.10 22:53:21: человек говорил во время реплики, фраза доехала через 1,9 с после её конца —
    раньше «Игнор фразы — grace 1.90с/2.5с», теперь идёт дальше."""
    clock = Clock()
    node = _pipeline(node_modules[1], clock, "да давай")
    for at, step in (
        (10.3, lambda: _say(node, SpeechEvent.KIND_STARTED)),
        (16.24, lambda: _vad(node, True)),
        (18.45, lambda: _vad(node, False)),
        (19.60, lambda: _say(node, SpeechEvent.KIND_FINISHED)),
    ):
        clock.t = 5000.0 + at
        step()
    _phrase(node, clock, at=21.45, seconds=6.62)
    assert node.publish_result.call_args.args[0] == "да давай"


@pytest.mark.parametrize("blip", ["робот", "щелчок", None])
def test_short_blip_while_robot_speaks_is_recognized_but_never_published(node_modules, blip):
    """< 0,8 с при звучащей речи: щелчок и эхо до диалога не доходят, как и раньше, но решает текст, а не длина."""
    clock = Clock()
    node = _pipeline(node_modules[1], clock, blip)
    _say(node, SpeechEvent.KIND_STARTED)
    _phrase(node, clock, at=1.0, seconds=0.5)
    node._recognize_with_fallback.assert_called_once()
    node.publish_result.assert_not_called()
    node._maybe_speak_unclear.assert_not_called()  # молча, без «не расслышал» (петля #989)


def test_short_stop_while_robot_speaks_reaches_the_dialog(node_modules):
    """Ревью #4122: «Робот, стоп» короче 0,8 с при звучащей речи раньше выбрасывалось до STT (DROP_SHORT)."""
    clock = Clock()
    node = _pipeline(node_modules[1], clock, "робот стоп")
    _say(node, SpeechEvent.KIND_STARTED)
    _phrase(node, clock, at=1.0, seconds=0.5)
    node._recognize_with_fallback.assert_called_once()
    assert node.publish_result.call_args.args[0] == "робот стоп"


# --- промежуточные результаты STT идут в диалог только через окно (ревью #4122) ----------------------------------------


def _offers(node):
    """Реальный ``InterruptOffer`` ноды; ``sent`` - тексты, ушедшие в ``/dialog/interrupt_request``."""
    from rob_box_voice.core.interrupt_request import InterruptOffer

    sent = []
    node._interrupts = InterruptOffer(
        lambda msg: sent.append(msg.text_partial),
        lambda: types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0)),
        lambda: WAKE,
        lambda: None,
    )
    node._active_utterance_id = "u1"
    return sent


def _partials(node, *texts):
    """STT, у которого Yandex присылает промежуточные результаты ``texts`` (и финал - последний из них)."""

    def recognize(_audio):
        for text in texts:
            node._offer_interrupt(text)
        return texts[-1], []

    node._recognize_with_fallback = MagicMock(side_effect=recognize)


def _echo_in_the_window(node, clock):
    for at, step in (
        (0.0, lambda: _say(node, SpeechEvent.KIND_STARTED)),
        (5.0, lambda: _say(node, SpeechEvent.KIND_FINISHED)),
        (5.2, lambda: _vad(node, True)),
        (5.6, lambda: _vad(node, False)),
    ):
        clock.t = 5000.0 + at
        step()


def test_partials_of_echo_with_a_wake_word_do_not_interrupt_the_speech(node_modules):
    """Голос только в эхо-окне, в хвосте реплики есть «робот»: partial «робот расскажи…» раньше уходил в
    ``/dialog/interrupt_request`` до проверки окна; теперь - ни partial, ни финал, пока фраза не ясный стоп."""
    clock = Clock()
    node = _pipeline(node_modules[1], clock, "")
    sent = _offers(node)
    _partials(node, "робот", "робот расскажи", "робот расскажи дальше")
    _echo_in_the_window(node, clock)
    _phrase(node, clock, at=8.6, seconds=3.4)
    assert sent == []
    node.publish_result.assert_not_called()


def test_partial_clear_stop_from_the_window_phrase_interrupts(node_modules):
    """Тот же путь, но человек сказал стоп в паузе: слова без стопа молчат, ясный стоп с вейком уходит сразу."""
    clock = Clock()
    node = _pipeline(node_modules[1], clock, "")
    sent = _offers(node)
    _partials(node, "робот", "робот хватит", "робот хватит замолчи")
    _echo_in_the_window(node, clock)
    _phrase(node, clock, at=8.6, seconds=3.4)
    assert sent == ["робот хватит", "робот хватит замолчи"]
    assert node.publish_result.call_args.args[0] == "робот хватит замолчи"


def test_partials_of_a_phrase_that_the_window_passes_are_offered_as_before(node_modules):
    """Окно фразу пропускает (робот молчит давно): partial с вейком уходит в диалог сразу, как до правки."""
    clock = Clock()
    node = _pipeline(node_modules[1], clock, "")
    sent = _offers(node)
    _partials(node, "робот", "робот расскажи", "робот расскажи сказку")
    _phrase(node, clock, at=30.0, seconds=3.0)
    assert sent == ["робот", "робот расскажи", "робот расскажи сказку"]


# --- stt_node: эхо словами речи робота (#4145) ---------------------------------------------------------------------

#: Агитация из Telegram 28.10 (карточка #4145): начало реплики робота (SSML, как его кладёт speech_client) и что из
#: её эха распознал STT в 08:58:18.657.
AGITATION_SSML = "<speak>Дорогие друзья! Слушайте и не говорите, что не слышали! Наше ПТУ</speak>"
AGITATION_ECHO = "дорогие друзья слушайте и не говорите что не слышали"


def _agitation(node, clock, voice: bool = True) -> None:
    """``SpeechRequest`` агитации и ``started``; ``voice`` — голос в микрофоне с 0,42 до 5,4 с (робот говорит)."""
    _, on_request = node.subscriptions_by_topic[speech_client.SPEECH_REQUEST_TOPIC]
    clock.t = 4999.8
    on_request(M.SpeechRequest(speech_id="46fd3b27", text=AGITATION_SSML, agent="telegram", sink="speakers",
                               commit=True))
    steps = [(0.0, lambda: _say(node, SpeechEvent.KIND_STARTED, "46fd3b27", agent="telegram"))]
    if voice:
        steps += [(0.42, lambda: _vad(node, True)), (5.4, lambda: _vad(node, False))]
    for at, step in steps:
        clock.t = 5000.0 + at
        step()


def test_agitation_echo_does_not_reach_the_dialog_and_goes_to_biometry_as_echo(node_modules):
    """Эхо агитации из лога: не в диалог (ни бэклога, ни модели), без «не расслышал»; в ``/voice/stt/echo`` — по нему
    биометрия профиль не заводит (тот же utterance_id, что у фразы)."""
    from rob_box_voice.core import echo_window

    clock = Clock()
    node = _pipeline(node_modules[1], clock, AGITATION_ECHO)
    _agitation(node, clock)
    _phrase(node, clock, at=5.447, seconds=5.38)
    node._recognize_with_fallback.assert_called_once()
    node.publish_result.assert_not_called()
    node._publish_speaker.assert_not_called()
    node._maybe_speak_unclear.assert_not_called()
    node.publish_state.assert_called_with("ready")
    (published,) = [c.args[0] for c in node.publishers_by_topic[echo_window.ECHO_TOPIC].publish.call_args_list]
    assert (published.utterance_id, published.text, published.source) == (
        node._active_utterance_id, AGITATION_ECHO, "respeaker")
    assert published.utterance_id


@pytest.mark.parametrize("text", ["робот стоп", "мама пришла давай ужинать"])
def test_stop_and_a_human_phrase_during_robot_speech_go_on_as_before(node_modules, text):
    """«Робот, стоп» во время речи и чужая фраза (не слова робота) — как раньше: в диалог; эхом не помечены."""
    from rob_box_voice.core import echo_window

    clock = Clock()
    node = _pipeline(node_modules[1], clock, text)
    _agitation(node, clock)
    _phrase(node, clock, at=5.447, seconds=5.38)
    assert node.publish_result.call_args.args[0] == text
    node.publishers_by_topic[echo_window.ECHO_TOPIC].publish.assert_not_called()


def test_robot_words_after_the_speech_outside_the_echo_window_go_on_as_before(node_modules):
    """После конца речи и эхо-окна голос звучал в тишине: даже слова робота (человек цитирует) — в диалог."""
    clock = Clock()
    node = _pipeline(node_modules[1], clock, AGITATION_ECHO)
    _agitation(node, clock, voice=False)
    for at, step in (
        (38.9, lambda: _say(node, SpeechEvent.KIND_FINISHED, "46fd3b27", agent="telegram")),
        (45.0, lambda: _vad(node, True)),
        (48.0, lambda: _vad(node, False)),
    ):
        clock.t = 5000.0 + at
        step()
    _phrase(node, clock, at=51.0, seconds=6.0)
    assert node.publish_result.call_args.args[0] == AGITATION_ECHO
