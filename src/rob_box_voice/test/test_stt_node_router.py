#!/usr/bin/env python3
"""test_stt_node_router.py — wake-роутер STTNode (issue #1990, оператор-agent 05).

Проверяет единственную точку маршрутизации речи (целевая §7.1) на уровне
``_process_audio``/``_route_wake_result``/``_publish_ptt_result``:

* respeaker (``/audio/speech_audio``) → ``/voice/stt/result``; «ТАРС» из
  ReSpeaker-канала НЕ создаёт маршрут в ``/avatar/stt/result`` (namespace
  привязан к источнику, а не только к тексту — DoD #1990).
* ptt (``/audio/quest_in``, левый грип) → ``/avatar/ptt/result`` plain text.
* wake (``/audio/quest_wake``) → в ``/avatar/stt/result`` ТОЛЬКО при
  operator-вейке («ТАРС»), сам вейк вырезан из текста; без вейка — drop.
* personality-слово («робокс») из микрофона шлема агента НЕ адресует.

Тест НЕ требует rclpy: STTNode собирается через ``__new__`` + MagicMock
(тот же приём, что в test_stt_node_fallback.py / test_stt_node_boop.py).
"""

from __future__ import annotations

import json
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest


def _node_no_op(self, *a, **kw):
    return None


def _ensure_rclpy_mock(monkeypatch):
    """Минимальный rclpy mock (как в test_stt_node_boop)."""

    class _NodeBase:
        def __init__(self, *a, **kw):
            pass

        def declare_parameter(self, *a, **kw):
            pass

        def get_parameter(self, name):
            return MagicMock(value="")

        def create_publisher(self, *a, **kw):
            return MagicMock()

        def create_subscription(self, *a, **kw):
            return MagicMock()

        def get_logger(self):
            return MagicMock(
                info=_node_no_op,
                warning=_node_no_op,
                warn=_node_no_op,
                error=_node_no_op,
                debug=_node_no_op,
            )

    class _NodeMod:
        Node = _NodeBase

    class _Rclpy:
        node = _NodeMod()

        @staticmethod
        def init(*a, **kw):
            pass

        @staticmethod
        def shutdown(*a, **kw):
            pass

        @staticmethod
        def spin(*a, **kw):
            pass

    monkeypatch.setitem(sys.modules, "rclpy", _Rclpy())
    monkeypatch.setitem(sys.modules, "rclpy.node", _Rclpy.node)

    class _QoSMod:
        QoSProfile = MagicMock()
        ReliabilityPolicy = MagicMock()
        DurabilityPolicy = MagicMock()
        HistoryPolicy = MagicMock()

    monkeypatch.setitem(sys.modules, "rclpy.qos", _QoSMod())

    class _StringMsg:
        """Реальный класс: каждый вызов String() — свежий объект.

        MagicMock() отдаёт ОДИН общий return_value → повторные publish-ы
        в _process_audio (ptt-текст, потом publish_state('ready')) мутировали
        бы один и тот же объект, и тест видел бы 'ready' вместо текста.
        """

        def __init__(self, data=""):
            self.data = data

    class _Msg:
        String = _StringMsg

    monkeypatch.setitem(sys.modules, "std_msgs", _Msg())
    monkeypatch.setitem(sys.modules, "std_msgs.msg", _Msg)

    class _AudioMsg:
        AudioData = MagicMock()

    monkeypatch.setitem(sys.modules, "audio_common_msgs", _AudioMsg())
    monkeypatch.setitem(sys.modules, "audio_common_msgs.msg", _AudioMsg)

    monkeypatch.setitem(
        sys.modules,
        "vosk",
        MagicMock(Model=MagicMock(), KaldiRecognizer=MagicMock()),
    )
    monkeypatch.setitem(sys.modules, "grpc", MagicMock())
    monkeypatch.setitem(
        sys.modules,
        "yandex.cloud.ai.stt.v3",
        MagicMock(
            stt_pb2=MagicMock(
                DefaultEouClassifier=MagicMock(HIGH=MagicMock(), DEFAULT=MagicMock()),
            ),
            stt_service_pb2_grpc=MagicMock(RecognizerStub=MagicMock()),
        ),
    )


@pytest.fixture(autouse=True)
def _ensure_optional_deps(monkeypatch):
    """autouse: rclpy/vosk/grpc/yandex замоканы для каждого теста."""
    import rob_box_voice  # noqa: F401

    for cached in ["rob_box_voice.stt_node", "rob_box_voice.dialogue_node"]:
        sys.modules.pop(cached, None)
        _leaf = cached.split(".")[-1]
        if hasattr(rob_box_voice, _leaf):
            delattr(rob_box_voice, _leaf)
    _ensure_rclpy_mock(monkeypatch)
    yield


def _make_node(**overrides):
    """STTNode-stub без rclpy: поля для _process_audio/_route_wake_result."""
    from rob_box_voice import stt_node as stt_node_module

    node = stt_node_module.STTNode.__new__(stt_node_module.STTNode)
    defaults = dict(
        sample_rate=16000,
        aec_mode="software",
        is_robot_speaking=False,
        tts_grace_s=2.5,
        _tts_ended_at=0.0,
        min_text_chars=3,
        _last_speaker_tag=None,
        _phrase_started_at=0.0,
        _boop_fired=False,
        wake_words=["робокс", "робот"],
        operator_wake_words=["тарс", "tars"],
        # публикаторы — mocks
        result_pub=MagicMock(),
        avatar_ptt_result_pub=MagicMock(),
        avatar_stt_result_pub=MagicMock(),
        state_pub=MagicMock(),
        speaker_pub=MagicMock(),
        tts_control_pub=MagicMock(),
        tts_request_pub=MagicMock(),
        boop_pub=MagicMock(),
        # поведенческие mocks (обходим реальную публикацию/LLM-сайд-эффекты)
        publish_result=MagicMock(),
        _publish_speaker=MagicMock(),
        _maybe_speak_unclear=MagicMock(),
    )
    defaults.update(overrides)
    for key, value in defaults.items():
        setattr(node, key, value)
    node.get_logger = lambda: MagicMock(
        info=lambda *a, **kw: None,
        warning=lambda *a, **kw: None,
        warn=lambda *a, **kw: None,
        error=lambda *a, **kw: None,
        debug=lambda *a, **kw: None,
    )
    return node


def _audio_msg() -> SimpleNamespace:
    # 1 секунда тишины на 16 kHz 16-bit → не короткая фраза, AEC не режет.
    return SimpleNamespace(data=list(b"\x00\x00" * 16000))


def _run_route(node, source: str, recognized_text: str):
    """Прогнать _process_audio с заданным «распознанным» текстом."""
    node._recognize_with_fallback = MagicMock(return_value=(recognized_text, []))
    msg = _audio_msg()
    if source == "respeaker":
        node.speech_audio_callback(msg)
    elif source == "ptt":
        node.quest_audio_callback(msg)
    elif source == "wake":
        node.quest_wake_audio_callback(msg)
    else:  # pragma: no cover
        raise AssertionError(source)


class TestRespeakerRoute:
    def test_respeaker_publishes_to_voice_stt_result(self):
        node = _make_node()
        _run_route(node, "respeaker", "робокс включи музыку")
        node.publish_result.assert_called_once_with("робокс включи музыку")
        node._publish_speaker.assert_called_once()
        node.avatar_stt_result_pub.publish.assert_not_called()
        node.avatar_ptt_result_pub.publish.assert_not_called()

    def test_tars_from_respeaker_does_not_route_to_avatar(self):
        """DoD #1990: «ТАРС» в ReSpeaker-канале НЕ создаёт /avatar/stt/result.

        Namespace привязан к источнику: operator-вейк проверяется только на
        wake-потоке шлема, а не по тексту из микрофона робота.
        """
        node = _make_node()
        _run_route(node, "respeaker", "тарс поверни камеру")
        # личность получает фразу как есть (дальше dialogue_node сам решит —
        # там вейк-список только personality, «тарс» не адресует)
        node.publish_result.assert_called_once_with("тарс поверни камеру")
        node.avatar_stt_result_pub.publish.assert_not_called()
        node.avatar_ptt_result_pub.publish.assert_not_called()


class TestPttRoute:
    def test_ptt_publishes_plain_text_to_avatar_ptt_result(self):
        node = _make_node()
        _run_route(node, "ptt", "всем привет с левого грипа")
        node.avatar_ptt_result_pub.publish.assert_called_once()
        published = node.avatar_ptt_result_pub.publish.call_args[0][0]
        assert published.data == "всем привет с левого грипа"
        node.publish_result.assert_not_called()
        node.avatar_stt_result_pub.publish.assert_not_called()

    def test_ptt_no_speaker_profile(self):
        node = _make_node()
        _run_route(node, "ptt", "проверка связи")
        node._publish_speaker.assert_not_called()


class TestWakeRoute:
    def test_wake_with_tars_routes_and_strips_wake_word(self):
        node = _make_node()
        _run_route(node, "wake", "тарс включи анимацию полиция")
        node.avatar_stt_result_pub.publish.assert_called_once()
        raw = node.avatar_stt_result_pub.publish.call_args[0][0].data
        payload = json.loads(raw)
        # вейк вырезан из текста перед публикацией
        assert payload["text"] == "включи анимацию полиция"
        assert "тарс" not in payload["text"].lower()
        # JSON v1 как /avatar/command (04a §3.5): обработчик супервизора тот же
        assert payload["source"] == "quest"
        assert "client_id" in payload and "ts_ms" in payload
        node.avatar_ptt_result_pub.publish.assert_not_called()
        node.publish_result.assert_not_called()

    def test_wake_case_insensitive(self):
        node = _make_node()
        _run_route(node, "wake", "ТАРС, включи свет")
        node.avatar_stt_result_pub.publish.assert_called_once()
        payload = json.loads(node.avatar_stt_result_pub.publish.call_args[0][0].data)
        assert payload["text"] == "включи свет"

    def test_wake_without_operator_wake_is_dropped(self):
        node = _make_node()
        _run_route(node, "wake", "включи свет")
        node.avatar_stt_result_pub.publish.assert_not_called()
        node.avatar_ptt_result_pub.publish.assert_not_called()
        node.publish_result.assert_not_called()

    def test_wake_personality_word_does_not_address_agent(self):
        """«робокс» из микрофона шлема НЕ адресует агента оператора."""
        node = _make_node()
        _run_route(node, "wake", "робокс включи свет")
        node.avatar_stt_result_pub.publish.assert_not_called()

    def test_route_wake_result_direct_no_wake(self):
        """Прямой вызов _route_wake_result без operator-вейка — drop."""
        node = _make_node()
        node._route_wake_result("просто фоновая речь")
        node.avatar_stt_result_pub.publish.assert_not_called()


class TestHelpers:
    def test_publish_ptt_result_plain(self):
        node = _make_node()
        node._publish_ptt_result("текст грипа")
        node.avatar_ptt_result_pub.publish.assert_called_once()
        assert node.avatar_ptt_result_pub.publish.call_args[0][0].data == "текст грипа"

    def test_boop_not_fired_on_headset_streams(self):
        """Бульк — сигнал личности; на потоках шлема (ptt/wake) не играем."""
        node = _make_node()
        # имитируем, что источник активен = wake → бульк не должен сработать
        node._active_source = "wake"
        node._maybe_fire_early_boop("тарс включи свет")
        node.boop_pub.publish.assert_not_called()
