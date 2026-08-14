#!/usr/bin/env python3
"""test_stt_node_boop.py — Unit тесты раннего «булька» (issue #1251).

Проверяет ``STTNode._maybe_fire_early_boop``:
- публикует триггер на /voice/sound/trigger, когда partial/final содержит wake word
- НЕ публикует, если wake word нет (no_wake_word)
- играет ОДИН раз за фразу (Yandex шлёт несколько partials с тем же wake word)
- уважает флаг ``early_boop_enabled=False``
- публикует ровно то значение, что задано в ``early_boop_trigger``

Тест НЕ требует rclpy: STTNode.__init__ замокан через MagicMock (как в
test_stt_node_fallback.py), тестируем только чистую логику булька.
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock

import pytest

_OPTIONAL_DEPS = {
    "rclpy": True,
    "rclpy.node": True,
    "rclpy.qos": True,
    "std_msgs": False,
    "std_msgs.msg": False,
    "audio_common_msgs": False,
    "audio_common_msgs.msg": False,
    "vosk": False,
    "grpc": False,
    "numpy": False,
    "yandex": False,
    "yandex.cloud": False,
    "yandex.cloud.ai": False,
    "yandex.cloud.ai.stt": False,
    "yandex.cloud.ai.stt.v3": False,
}


def _node_no_op(self, *a, **kw):
    return None


def _ensure_rclpy_mock(monkeypatch):
    """Минимальный rclpy mock (тот же приём, что в test_stt_node_fallback)."""

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

    class _Msg:
        String = MagicMock()

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
    monkeypatch.setitem(sys.modules, "numpy", MagicMock())
    monkeypatch.setitem(sys.modules, "yandex", MagicMock())
    monkeypatch.setitem(sys.modules, "yandex.cloud", MagicMock())
    monkeypatch.setitem(sys.modules, "yandex.cloud.ai", MagicMock())
    monkeypatch.setitem(sys.modules, "yandex.cloud.ai.stt", MagicMock())
    monkeypatch.setitem(
        sys.modules,
        "yandex.cloud.ai.stt.v3",
        MagicMock(
            stt_pb2=MagicMock(
                DefaultEouClassifier=MagicMock(HIGH=MagicMock(), DEFAULT=MagicMock()),
                EouClassifierOptions=MagicMock(),
                AudioFormatOptions=MagicMock(),
                RawAudio=MagicMock(LINEAR16_PCM=MagicMock()),
                RecognitionModelOptions=MagicMock(REAL_TIME=MagicMock()),
                LanguageRestrictionOptions=MagicMock(WHITELIST=MagicMock()),
                TextNormalizationOptions=MagicMock(
                    TEXT_NORMALIZATION_DISABLED=MagicMock()
                ),
                SpeechAnalysisOptions=MagicMock(enable_speaker_analysis=MagicMock()),
                StreamingOptions=MagicMock(),
                StreamingRequest=MagicMock(),
                AudioChunk=MagicMock(),
            ),
            stt_service_pb2_grpc=MagicMock(RecognizerStub=MagicMock()),
        ),
    )


@pytest.fixture(autouse=True)
def _ensure_optional_deps(monkeypatch):
    """autouse: rclpy/vosk/grpc/yandex замоканы для каждого теста."""
    import rob_box_voice  # noqa: F401

    for cached in [
        "rob_box_voice.stt_node",
        "rob_box_voice.dialogue_node",
    ]:
        sys.modules.pop(cached, None)
        _leaf = cached.split(".")[-1]
        if hasattr(rob_box_voice, _leaf):
            delattr(rob_box_voice, _leaf)
    _ensure_rclpy_mock(monkeypatch)
    yield


def _make_boop_node(**param_overrides):
    """STTNode-stub без rclpy: только поля, нужные _maybe_fire_early_boop."""
    # Импорт ВНУТРИ функции: autouse-фикстура _ensure_optional_deps уже
    # зарегистрировала mock-модули rclpy/grpc/vosk в sys.modules к моменту
    # вызова теста (модульный import на верхнем уровне падал бы: grpc
    # отсутствует в CI-окружении до фикстуры).
    from rob_box_voice import stt_node as stt_node_module

    defaults = dict(
        early_boop_enabled=True,
        early_boop_trigger="boop",
        wake_words=["робок", "робот", "роббокс"],
    )
    defaults.update(param_overrides)

    node = stt_node_module.STTNode.__new__(stt_node_module.STTNode)
    for k, v in defaults.items():
        setattr(node, k, v)
    node._boop_fired = False
    node._phrase_started_at = 0.0
    node.boop_pub = MagicMock()
    node.get_logger = lambda: MagicMock(
        info=lambda *a, **kw: None,
        warning=lambda *a, **kw: None,
        warn=lambda *a, **kw: None,
        error=lambda *a, **kw: None,
        debug=lambda *a, **kw: None,
    )
    return node


class TestEarlyBoop:
    """Issue #1251 — ранний «бульк» при wake word в STT."""

    def test_publishes_boop_on_wake_word(self):
        node = _make_boop_node()
        node._maybe_fire_early_boop("робот расскажи анекдот")
        node.boop_pub.publish.assert_called_once()
        published = node.boop_pub.publish.call_args[0][0]
        assert published.data == "boop"

    def test_no_boop_without_wake_word(self):
        node = _make_boop_node()
        node._maybe_fire_early_boop("расскажи анекдот")
        node.boop_pub.publish.assert_not_called()

    def test_single_boop_per_phrase(self):
        """Yandex шлёт несколько partials — бульк играем один раз."""
        node = _make_boop_node()
        node._maybe_fire_early_boop("робот расскажи")
        node._maybe_fire_early_boop("робот расскажи анекдот")
        node._maybe_fire_early_boop("робот расскажи анекдот про кота")
        assert node.boop_pub.publish.call_count == 1

    def test_empty_text_no_boop(self):
        node = _make_boop_node()
        node._maybe_fire_early_boop("")
        node._maybe_fire_early_boop("   ")
        node.boop_pub.publish.assert_not_called()

    def test_disabled_no_boop(self):
        node = _make_boop_node(early_boop_enabled=False)
        node._maybe_fire_early_boop("робот расскажи анекдот")
        node.boop_pub.publish.assert_not_called()

    def test_custom_trigger_name(self):
        node = _make_boop_node(early_boop_trigger="ui_confirm")
        node._maybe_fire_early_boop("робок включи музыку")
        published = node.boop_pub.publish.call_args[0][0]
        assert published.data == "ui_confirm"

    def test_boop_fired_flag_reset(self):
        """После сброса _boop_fired (новая фраза) бульк играется снова."""
        node = _make_boop_node()
        node._maybe_fire_early_boop("робот раз")
        assert node._boop_fired is True
        node._boop_fired = False  # speech_audio_callback сбрасывает на новую фразу
        node._maybe_fire_early_boop("робот два")
        assert node.boop_pub.publish.call_count == 2

    def test_fallback_wake_word_list(self):
        """Без импорта dialogue_text (standalone) has_wake_word работает."""
        node = _make_boop_node(wake_words=["робот"])
        node._maybe_fire_early_boop("Робот, привет")
        node.boop_pub.publish.assert_called_once()
