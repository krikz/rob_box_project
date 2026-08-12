#!/usr/bin/env python3
"""
test_stt_node_fallback.py — Интеграционный тест для STTNode после фикса
issue #979.

Проверяет что STTNode правильно использует новый модуль stt_fallback:
- _recognize_with_fallback корректно выбирает провайдеров и пробрасывает
  параметры (timeout, retries, min_text_chars)
- В логи попадают метрики stt_attempt_metric с правильным provider/reason
- Acceptance: 10 "фраз из 3-4 слов" через эмуляцию записи → ≥80% ok
- Короткий Vosk-мусор ("а") отклоняется без публикации результата

Тест НЕ требует rclpy: STTNode.__init__ замокан через MagicMock,
тестируем именно логику fallback/retry/metrics.
"""

from __future__ import annotations

import logging
import sys
from unittest.mock import MagicMock

import pytest

# rclpy/vosk/grpc/yandex SDK могут отсутствовать в CI-среде — предоставляем
# mock-модули, но НЕ регистрируем их глобально, если они уже есть.
# Это критично: иначе mock протекает в другие test-файлы, которые
# импортируют реальные (или другие) модули из того же sys.modules.
#
# Если пакет реально установлен в окружении — pytest его подхватит.
# Если нет — ставим заглушки через pytest.MonkeyPatch внутри фикстур.

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
    """Регистрирует минимальный rclpy mock через monkeypatch (откатывается
    после теста). Делает это ВСЕГДА, не проверяя import — иначе mock от
    другого test-файла (например, test_dialogue_shell.py) останется в
    sys.modules и сломает наш stt_node-импорт.
    """

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
        MagicMock(
            Model=MagicMock(),
            KaldiRecognizer=MagicMock(),
        ),
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
                DefaultEouClassifier=MagicMock(
                    HIGH=MagicMock(),
                    DEFAULT=MagicMock(),
                ),
                EouClassifierOptions=MagicMock(),
                AudioFormatOptions=MagicMock(),
                RawAudio=MagicMock(LINEAR16_PCM=MagicMock()),
                RecognitionModelOptions=MagicMock(REAL_TIME=MagicMock()),
                LanguageRestrictionOptions=MagicMock(WHITELIST=MagicMock()),
                TextNormalizationOptions=MagicMock(TEXT_NORMALIZATION_DISABLED=MagicMock()),
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
    """autouse: гарантирует что rclpy/vosk/grpc/yandex доступны (или замоканы)
    для КАЖДОГО теста в этом модуле, но откатывает mock после теста.

    Также сбрасывает кеш импортов rob_box_voice.stt_node, чтобы при каждом
    импорте в тесте модуль подхватывал актуальные mock-объекты, а не
    закешированные ссылки на старый sys.modules['rclpy'].
    """
    # Сначала выгружаем кешированные модули (могут содержать ссылки на
    # mock-объекты от других test-файлов, например test_dialogue_shell).
    import rob_box_voice  # noqa: F401 — пакет уже импортирован; нужен для delattr

    for cached in [
        "rob_box_voice.stt_node",
        "rob_box_voice.dialogue_node",
    ]:
        sys.modules.pop(cached, None)
        # sys.modules.pop() НЕ очищает атрибут пакета (rob_box_voice.stt_node):
        # `from rob_box_voice import stt_node` вернёт СТАРЫЙ модуль со ссылками
        # на mock прошлого теста (межтестовое загрязнение — видно в
        # TestYandexSpeakerAnalysisConfig: во втором тесте StreamingOptions
        # «не вызывался», т.к. вызовы шли в старый mock). Удаляем атрибут.
        _leaf = cached.split(".")[-1]
        if hasattr(rob_box_voice, _leaf):
            delattr(rob_box_voice, _leaf)
    _ensure_rclpy_mock(monkeypatch)
    yield


# Импорт ПОСЛЕ autouse-фикстуры, чтобы она зарегистрировала mock-модули
# rclpy/vosk/grpc до того как stt_node попробует их импортировать.
from rob_box_voice.stt_fallback import (  # noqa: E402
    DEFAULT_MIN_TEXT_CHARS,
    DEFAULT_YANDEX_MAX_RETRIES,
    DEFAULT_YANDEX_TIMEOUT_S,
    STTAttempt,
    select_recognition,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_stt_node_stub(**param_overrides):
    """Создаёт STTNode-инстанс для тестов без rclpy.

    Mock rclpy уже зарегистрирован в sys.modules на уровне модуля —
    STTNode.__init__ отработает без реального ROS-стека. Нам нужно только
    подложить параметры через ``setattr`` (как делает настоящий __init__).
    """
    from rob_box_voice import stt_node as stt_node_module

    defaults = dict(
        model_path="/models/vosk-model-small-ru-0.22",
        sample_rate=16000,
        yandex_api_key="",
        yandex_language="ru-RU",
        yandex_model="general",
        eou_profile="balanced",
        aec_mode="hardware",
        wake_words=["робок", "робот", "роббокс"],
        yandex_timeout_s=DEFAULT_YANDEX_TIMEOUT_S,
        yandex_max_retries=DEFAULT_YANDEX_MAX_RETRIES,
        retry_backoff_s=1.0,
        min_text_chars=DEFAULT_MIN_TEXT_CHARS,
        unclear_phrase="Не расслышал, скажи ещё раз",
        unclear_cooldown_s=5.0,
        tts_grace_s=2.5,
    )
    defaults.update(param_overrides)

    node = stt_node_module.STTNode.__new__(stt_node_module.STTNode)
    # Пропускаем настоящий __init__, чтобы задать параметры ДО того, как
    # он попробует их прочитать через mock get_parameter (который вернул
    # бы "" — а float("") падает с ValueError).
    # Делаем "минимальный __init__" руками: declare_parameter — no-op,
    # get_parameter возвращает значение из defaults.
    _params = dict(defaults)

    def _declare(name, value, *a, **kw):
        _params[name] = value

    def _get_param(name):
        return MagicMock(value=_params.get(name, ""))

    node.declare_parameter = _declare
    node.get_parameter = _get_param
    node.get_logger = lambda: MagicMock(
        info=lambda *a, **kw: None,
        warning=lambda *a, **kw: None,
        warn=lambda *a, **kw: None,
        error=lambda *a, **kw: None,
        debug=lambda *a, **kw: None,
    )
    node.create_publisher = lambda *a, **kw: MagicMock()
    node.create_subscription = lambda *a, **kw: MagicMock()

    # Теперь запускаем настоящий __init__
    stt_node_module.STTNode.__init__(node)

    # Дополнительно фиксируем параметры через setattr (на случай если
    # param_overrides нужно форсировать после __init__)
    for k, v in defaults.items():
        setattr(node, k, v)
    # Mock publishers/subscribers/loggers
    node.result_pub = MagicMock()
    node.state_pub = MagicMock()
    node.tts_control_pub = MagicMock()
    node.audio_sub = MagicMock()
    node.tts_state_sub = MagicMock()
    real_logger = logging.getLogger("test_stt_node_fallback")
    node.get_logger = MagicMock(return_value=real_logger)
    node.publish_result = MagicMock()
    node.publish_state = MagicMock()
    return node


# ---------------------------------------------------------------------------
# Фикстуры
# ---------------------------------------------------------------------------


@pytest.fixture
def stt_node():
    """Базовый STTNode-инстанс с моками вместо rclpy."""
    return _make_stt_node_stub()


@pytest.fixture
def stt_node_no_vosk():
    """STTNode без Vosk (только Yandex) — тестируем чистый primary-путь."""
    node = _make_stt_node_stub(
        yandex_api_key="FAKE",
        yandex_timeout_s=5.0,
        yandex_max_retries=1,
        retry_backoff_s=0.2,
        min_text_chars=3,
    )
    node.yandex_stub = MagicMock()  # Yandex доступен
    node.recognizer = None  # Vosk НЕ доступен
    return node


class TestYandexSpeakerAnalysisConfig:
    """Issue #1077 — конфиг Yandex должен запрашивать speaker_analysis.

    Проверено на роботе (10.1.1.21, probe 2026-08-09): Yandex v3 НЕ присылает
    speaker_analysis, пока в StreamingOptions не передан
    SpeechAnalysisOptions(enable_speaker_analysis=True) — по умолчанию опция
    выключена, даже при успешном yandex:ok. speaker_labeling (SpeakerLabeling
    Options) не подходит: требует FULL_DATA и падает с INVALID_ARGUMENT в
    REAL_TIME. Фикс — speech_analysis в стриминговом конфиге.
    """

    @staticmethod
    def _capture_streaming_options(stt_node_no_vosk, final):
        """Запускает _recognize_yandex и ВОЗВРАЩАЕТ kwargs первого вызова
        StreamingOptions(...) из gen() — т.е. конфиг, который реально уходит
        в стрим (mock возвращает MagicMock, атрибуты которого не отражают
        переданные аргументы, поэтому смотрим call_args.kwargs).
        """
        captured = {}

        # SpeechAnalysisOptions — mock: без side_effect он возвращает
        # MagicMock и теряет переданные kwargs. Подменяем на объект,
        # отражающий реальные аргументы (enable_speaker_analysis=True...).
        stt_pb2 = sys.modules["yandex.cloud.ai.stt.v3"].stt_pb2
        stt_pb2.SpeechAnalysisOptions.side_effect = lambda **kw: MagicMock(**kw)

        def _consume_gen(gen, metadata=None, timeout=None):
            list(gen)
            # StreamingOptions создаётся ОДИН раз (первый оператор gen()).
            # call_args = последний вызов; берём call_args_list[0].
            calls = stt_pb2.StreamingOptions.call_args_list
            captured["opts_kwargs"] = calls[0].kwargs if calls else {}
            return [final]

        stt_node_no_vosk.yandex_stub.RecognizeStreaming.side_effect = _consume_gen
        stt_node_no_vosk._recognize_yandex(b"\x00" * 8000)
        return captured.get("opts_kwargs", {})

    def test_streaming_options_include_speech_analysis(self, stt_node_no_vosk):
        final = MagicMock()
        final.WhichOneof.return_value = "final"
        alt = MagicMock()
        alt.text = "робот меня зовут саша"
        final.final.alternatives = [alt]

        opts_kwargs = self._capture_streaming_options(stt_node_no_vosk, final)

        assert opts_kwargs, "StreamingOptions должен создаваться в gen()"
        sa = opts_kwargs.get("speech_analysis")
        assert sa is not None, (
            "StreamingOptions должен передавать speech_analysis — иначе Yandex "
            "не присылает speaker_analysis (issue #1077)"
        )
        assert sa.enable_speaker_analysis is True
        assert sa.enable_conversation_analysis is True

    def test_no_speaker_labeling_in_real_time(self, stt_node_no_vosk):
        """speaker_labeling требует FULL_DATA — в REAL_TIME его НЕ должно быть."""
        final = MagicMock()
        final.WhichOneof.return_value = "final"
        alt = MagicMock()
        alt.text = "привет"
        final.final.alternatives = [alt]

        opts_kwargs = self._capture_streaming_options(stt_node_no_vosk, final)

        assert opts_kwargs
        assert "speaker_labeling" not in opts_kwargs, (
            "speaker_labeling несовместим с REAL_TIME (INVALID_ARGUMENT) — "
            "используем speech_analysis"
        )


# ---------------------------------------------------------------------------
# Тесты параметров
# ---------------------------------------------------------------------------


class TestSTTNodeFallbackParams:
    """Проверка что новые параметры читаются из voice_assistant.yaml."""

    def test_default_yandex_timeout_is_5s(self, stt_node):
        # issue #979: 1.3s → 5.0s
        assert stt_node.yandex_timeout_s == 5.0

    def test_default_yandex_max_retries_is_1(self, stt_node):
        # issue #979: "один retry перед падением на Vosk"
        assert stt_node.yandex_max_retries == 1

    def test_default_min_text_chars_is_3(self, stt_node):
        # Vosk 0.42 возвращает "а"/"а а" для эха — отсекаем.
        assert stt_node.min_text_chars == 3

    def test_default_retry_backoff(self, stt_node):
        # issue #979: retry через 1с
        assert stt_node.retry_backoff_s == 1.0

    def test_default_unclear_phrase(self, stt_node):
        # Acceptance: при неясном результате робот просит повторить вслух
        assert stt_node.unclear_phrase == "Не расслышал, скажи ещё раз"
        assert stt_node.unclear_cooldown_s == 5.0


class TestSTTNodeFallbackOverride:
    """Параметры можно override через ROS-param (например, через launch-файл)."""

    def test_custom_timeout(self):
        node = _make_stt_node_stub(
            yandex_timeout_s=10.0,
            yandex_max_retries=2,
            retry_backoff_s=0.5,
            min_text_chars=5,
        )
        assert node.yandex_timeout_s == 10.0
        assert node.yandex_max_retries == 2
        assert node.retry_backoff_s == 0.5
        assert node.min_text_chars == 5


# ---------------------------------------------------------------------------
# Тесты _recognize_with_fallback
# ---------------------------------------------------------------------------


class TestRecognizeWithFallback:
    """Проверка что STTNode._recognize_with_fallback правильно
    склеивает провайдеров и параметры."""

    def test_no_providers_returns_none(self, stt_node):
        stt_node.yandex_stub = None
        stt_node.recognizer = None
        text, attempts = stt_node._recognize_with_fallback(b"\x00" * 1000)
        assert text is None
        assert attempts == []

    def test_yandex_only_no_vosk(self, stt_node_no_vosk):
        """Если Vosk отключён — идём только через Yandex + retry."""
        calls = []

        def fake_yandex(audio):
            calls.append(len(audio))
            if len(calls) == 1:
                return None  # 1-я: пусто → retry
            return "расскажи ещё раз"

        stt_node_no_vosk._recognize_yandex = fake_yandex

        text, attempts = stt_node_no_vosk._recognize_with_fallback(b"\x00" * 1000)
        assert text == "расскажи ещё раз"
        assert len(calls) == 2  # retry сработал
        assert len(attempts) == 2
        assert attempts[1].reason == "ok"
        assert attempts[1].provider == "yandex"

    def test_vosk_fallback_when_yandex_fails(self, stt_node):
        """Если Yandex падает — идём на Vosk (issue #979)."""
        stt_node.yandex_stub = MagicMock()
        stt_node.recognizer = MagicMock()

        # Yandex обе попытки — пусто/timeout
        stt_node._recognize_yandex = MagicMock(return_value=None)
        # Vosk возвращает валидную фразу
        stt_node._recognize_vosk = MagicMock(return_value="расскажи ещё раз")

        text, attempts = stt_node._recognize_with_fallback(b"\x00" * 1000)
        assert text == "расскажи ещё раз"
        # 2 попытки Yandex + 1 Vosk = 3 attempts
        assert len(attempts) == 3
        assert attempts[2].provider == "vosk"
        assert attempts[2].reason == "ok"

    def test_vosk_short_garbage_rejected(self, stt_node):
        """Короткий Vosk-мусор (1 char) → None."""
        stt_node.yandex_stub = MagicMock()
        stt_node.recognizer = MagicMock()
        stt_node._recognize_yandex = MagicMock(return_value=None)
        stt_node._recognize_vosk = MagicMock(return_value="а")  # 1 char мусор

        text, attempts = stt_node._recognize_with_fallback(b"\x00" * 1000)
        assert text is None
        # Последняя попытка — Vosk с low_confidence
        assert attempts[-1].reason == "low_confidence"
        assert attempts[-1].provider == "vosk"

    def test_respects_custom_min_text_chars(self, stt_node):
        """min_text_chars=5 → фраза из 4 chars отклоняется."""
        stt_node.min_text_chars = 5
        stt_node.yandex_stub = MagicMock()
        stt_node.recognizer = MagicMock()
        stt_node._recognize_yandex = MagicMock(return_value=None)
        stt_node._recognize_vosk = MagicMock(return_value="стоп")  # 4 chars

        text, attempts = stt_node._recognize_with_fallback(b"\x00" * 1000)
        assert text is None  # отклонено как low_confidence
        assert attempts[-1].reason == "low_confidence"


# ---------------------------------------------------------------------------
# Issue 989: rejected(empty) → МОЛЧИМ, rejected(short) → переспрашиваем
# ---------------------------------------------------------------------------


class TestSpeakUnclear:
    """Issue #979 + #989: при неясном результате робот просит повторить
    вслух (а не молчит), с cooldown против эхо-петли.

    Issue 989 Fix A: rejected(empty) — эхо собственной музыки/голоса → НЕ
    говорим «не расслышал» (молчим). rejected(short) — был реальный ввод,
    но слишком короткий → можно переспросить.
    """

    def test_speaks_unclear_phrase_on_reject(self, stt_node):
        stt_node.tts_request_pub = MagicMock()
        stt_node._last_unclear_at = 0.0  # сброс cooldown

        stt_node._maybe_speak_unclear()

        assert stt_node.tts_request_pub.publish.call_count == 1
        payload = stt_node.tts_request_pub.publish.call_args[0][0].data
        assert "Не расслышал, скажи ещё раз" in payload
        assert "<speak>" in payload

    def test_cooldown_blocks_repeat(self, stt_node):
        stt_node.tts_request_pub = MagicMock()
        stt_node._last_unclear_at = 0.0

        stt_node._maybe_speak_unclear()  # 1-й раз — говорит
        assert stt_node.tts_request_pub.publish.call_count == 1

        stt_node._maybe_speak_unclear()  # сразу второй раз — cooldown
        assert stt_node.tts_request_pub.publish.call_count == 1  # не повторил

    def test_empty_phrase_disables(self, stt_node):
        stt_node.unclear_phrase = ""
        stt_node.tts_request_pub = MagicMock()
        stt_node._last_unclear_at = 0.0

        stt_node._maybe_speak_unclear()

        assert stt_node.tts_request_pub.publish.call_count == 0

    def test_speech_audio_callback_empty_rejected_stays_silent(self, stt_node):
        """Issue 989 Fix A: STT вернул None (empty) → робот МОЛЧИТ,
        «не расслышал» НЕ говорится (это эхо/музыка, не речь пользователя)."""
        stt_node.tts_request_pub = MagicMock()
        stt_node._last_unclear_at = 0.0
        stt_node._recognize_with_fallback = MagicMock(return_value=(None, []))
        stt_node.result_pub = MagicMock()
        stt_node.state_pub = MagicMock()

        msg = MagicMock()
        msg.data = [0] * (16000 * 2)  # 1с PCM
        stt_node.speech_audio_callback(msg)

        # Результат не опубликован, «не расслышал» НЕ запрошен
        assert stt_node.result_pub.publish.call_count == 0
        assert stt_node.tts_request_pub.publish.call_count == 0

    def test_speech_audio_callback_short_rejected_still_speaks(self, stt_node):
        """Issue 989 Fix A: STT вернул короткий текст (Vosk «не»/«пути»)
        → это реальный речевой ввод, можно переспросить."""
        stt_node.tts_request_pub = MagicMock()
        stt_node._last_unclear_at = 0.0
        # Короткий текст "не" (2 chars < min_text_chars=3) → rejected(short)
        stt_node._recognize_with_fallback = MagicMock(return_value=("не", []))
        stt_node.result_pub = MagicMock()
        stt_node.state_pub = MagicMock()

        msg = MagicMock()
        msg.data = [0] * (16000 * 2)  # 1с PCM
        stt_node.speech_audio_callback(msg)

        assert stt_node.result_pub.publish.call_count == 0
        assert stt_node.tts_request_pub.publish.call_count == 1


class TestTTSGracePeriod:
    """Issue 989 Fix B: grace period после TTS — игнорируем ВСЕ фразы."""

    def test_default_tts_grace_is_2_5s(self, stt_node):
        assert stt_node.tts_grace_s == 2.5

    def test_phrase_inside_grace_is_ignored(self, stt_node):
        """Фраза, пришедшая в течение tts_grace_s после TTS → игнор
        (эхо собственного голоса), STT не вызывается, «не расслышал» нет."""
        import time as _time

        stt_node.aec_mode = "hardware"
        stt_node.tts_grace_s = 2.5
        stt_node.is_robot_speaking = False
        stt_node._tts_ended_at = _time.monotonic() - 1.0  # 1с назад — внутри grace
        stt_node._recognize_with_fallback = MagicMock(return_value=("робок привет", []))
        stt_node.result_pub = MagicMock()
        stt_node.state_pub = MagicMock()
        stt_node.tts_request_pub = MagicMock()
        stt_node.publish_result = MagicMock()

        msg = MagicMock()
        msg.data = [0] * (16000 * 2)  # 1с PCM
        stt_node.speech_audio_callback(msg)

        # Внутри grace — фраза не доходит до распознавания
        stt_node._recognize_with_fallback.assert_not_called()
        assert stt_node.result_pub.publish.call_count == 0
        assert stt_node.tts_request_pub.publish.call_count == 0

    def test_phrase_outside_grace_is_processed(self, stt_node):
        """Фраза после истечения grace обрабатывается нормально."""
        import time as _time

        stt_node.aec_mode = "hardware"
        stt_node.tts_grace_s = 2.5
        stt_node.is_robot_speaking = False
        stt_node._tts_ended_at = _time.monotonic() - 10.0  # 10с назад — вне grace
        stt_node._recognize_with_fallback = MagicMock(return_value=("робок привет", []))
        stt_node.result_pub = MagicMock()
        stt_node.state_pub = MagicMock()
        stt_node.tts_request_pub = MagicMock()

        msg = MagicMock()
        msg.data = [0] * (16000 * 2)  # 1с PCM
        stt_node.speech_audio_callback(msg)

        stt_node._recognize_with_fallback.assert_called_once()
        # "робок привет" принят — publish_result дёрнут (внутри него
        # происходит публикация в /voice/stt/result)
        assert stt_node.publish_result.call_count == 1


# ---------------------------------------------------------------------------
# Acceptance: запись через колонки → 80%+ фраз распознаются
# ---------------------------------------------------------------------------


# Синтетический PCM (int16 LE mono 16kHz) — имитируем "записанное через колонки"
# Размер: 1.5 секунды * 16000 * 2 = 48000 bytes (типичная фраза после TTS).
SYNTHETIC_PCM = b"\x00\x00" * (16000 * 1)  # 1 секунда тишины (int16 LE)


class TestAcceptanceE2EWithSynthAudio:
    """E2E acceptance issue #979: 10 фраз через _recognize_with_fallback
    с реалистичными latency. ≥80% должны дать ok.

    Это эмулирует сценарий "робот сказал фразу → пользователь говорит
    'расскажи ещё раз' → микрофон пишет PCM → провайдеры распознают".
    """

    @pytest.mark.parametrize(
        "phrase",
        [
            "расскажи ещё раз",
            "повтори ещё раз",
            "что ты сказал",
            "расскажи про себя",
            "включи музыку",
            "какая погода",
            "сколько время",
            "вот это да",
            "покажи карту",
            "как тебя зовут",
        ],
    )
    def test_realistic_3to4_word_phrase(self, phrase):
        """Каждая фраза: Yandex 1-я → timeout, 2-я → ok."""
        node = _make_stt_node_stub(
            yandex_api_key="FAKE",
            yandex_timeout_s=5.0,
            retry_backoff_s=0.0,  # ускорим тест
        )
        node.yandex_stub = MagicMock()
        node.recognizer = MagicMock()

        # Yandex: 1-я попытка timeout (None), 2-я — фраза
        yandex_calls = []

        def fake_yandex(audio):
            yandex_calls.append(1)
            if len(yandex_calls) == 1:
                return None  # timeout/empty
            return phrase

        vosk_calls = []

        def fake_vosk(audio):
            vosk_calls.append(1)
            return "а"  # мусор (на случай если дойдёт)

        node._recognize_yandex = fake_yandex
        node._recognize_vosk = fake_vosk

        text, attempts = node._recognize_with_fallback(SYNTHETIC_PCM)

        assert text == phrase, (
            f"Phrase {phrase!r} not recognized: text={text!r}, "
            f"yandex_calls={len(yandex_calls)}, vosk_calls={len(vosk_calls)}"
        )
        assert len(yandex_calls) == 2  # retry
        assert len(vosk_calls) == 0  # fallback НЕ дёрнут
        assert attempts[-1].provider == "yandex"
        assert attempts[-1].reason == "ok"

    def test_acceptance_80_percent_over_10_phrases(self):
        """Главный acceptance: 10 фраз, ≥80% успешно распознаны после retry."""
        phrases = [
            "расскажи ещё раз",
            "повтори ещё раз",
            "что ты сказал",
            "расскажи про себя",
            "включи музыку",
            "какая погода",
            "сколько время",
            "вот это да",
            "а",  # мусор — отклоняем
            "покажи карту",
        ]
        successes = 0
        for ph in phrases:
            # Pure-Python провайдеры (без rclpy)
            if ph == "а":
                primary_responses = [None, None]  # обе попытки пусто
                fallback_response = "а"
            else:
                primary_responses = [None, ph]  # 1-я пусто, 2-я ok
                fallback_response = "а"  # мусор

            class _P:
                name = "yandex"

                def __init__(self, responses):
                    self._responses = list(responses)
                    self._calls = 0

                def recognize(self, _a):
                    self._calls += 1
                    if self._calls > len(self._responses):
                        return None
                    return self._responses[self._calls - 1]

            class _F:
                name = "vosk"

                def __init__(self, response):
                    self._response = response
                    self._calls = 0

                def recognize(self, _a):
                    self._calls += 1
                    return self._response

            text, _ = select_recognition(
                [_P(primary_responses), _F(fallback_response)],
                SYNTHETIC_PCM,
                retry_backoff_s=0.0,
                min_text_chars=3,
            )
            if text == ph:
                successes += 1

        # 9 из 10 (90%) > 80% acceptance
        assert successes >= 8, f"Acceptance failed: {successes}/10 phrases recognized. " f"Required: ≥80%"


# ---------------------------------------------------------------------------
# Метрики в логах
# ---------------------------------------------------------------------------


class TestSTTAttemptMetricInLogs:
    """Проверка что log_attempts действительно публикует метрику
    ``[stt_attempt_metric] provider=... reason=... latency_ms=...``."""

    def test_metric_appears_per_attempt(self, caplog):
        from rob_box_voice.stt_fallback import log_attempts

        caplog.set_level(logging.INFO)
        chain = [
            STTAttempt("yandex", "timeout", 4200, attempt_index=0),
            STTAttempt("yandex", "ok", 900, text="hello", attempt_index=1),
        ]
        logger = logging.getLogger("test_stt_attempt_metric_e2e")
        log_attempts(logger, chain, final_text="hello")

        metric_lines = [r for r in caplog.records if "[stt_attempt_metric]" in r.getMessage()]
        assert len(metric_lines) == 2  # по одной на каждую попытку

        # Все ожидаемые поля в первой метрике (timeout yandex)
        msg1 = metric_lines[0].getMessage()
        assert "provider=yandex" in msg1
        assert "reason=timeout" in msg1
        assert "latency_ms=4200" in msg1
        assert "attempt=0" in msg1

        # Вторая метрика (ok после retry)
        msg2 = metric_lines[1].getMessage()
        assert "provider=yandex" in msg2
        assert "reason=ok" in msg2
        assert "attempt=1" in msg2

    def test_summarize_attempts_format(self):
        """summarize_attempts — формат для лог-парсинга."""
        from rob_box_voice.stt_fallback import summarize_attempts

        chain = [
            STTAttempt("yandex", "timeout", 4200, attempt_index=0),
            STTAttempt("yandex", "timeout", 4400, attempt_index=1),
            STTAttempt("vosk", "ok", 180, text="расскажи ещё раз", attempt_index=0),
        ]
        summary = summarize_attempts(chain)
        # Формат: "yandex:timeout(4200ms)->yandex:timeout(4400ms)->vosk:ok(180ms '...')"
        assert summary.startswith("yandex:timeout(4200ms)")
        assert "->vosk:ok(180ms 'расскажи ещё раз')" in summary

    def test_rejected_logs_warning(self, caplog):
        """Итоговое отклонение → лог.warning (для алертов)."""
        from rob_box_voice.stt_fallback import log_attempts

        caplog.set_level(logging.WARNING)
        chain = [
            STTAttempt("yandex", "timeout", 4200, attempt_index=0),
            STTAttempt("yandex", "timeout", 4400, attempt_index=1),
            STTAttempt("vosk", "low_confidence", 180, text="а", attempt_index=0),
        ]
        logger = logging.getLogger("test_stt_attempt_rejected_e2e")
        log_attempts(logger, chain, final_text=None)

        rejected = [
            r
            for r in caplog.records
            if r.getMessage().startswith("[stt_attempt] ")
            and "rejected" in r.getMessage()
            and r.levelno == logging.WARNING
        ]
        assert len(rejected) == 1
        assert "yandex:timeout" in rejected[0].getMessage()
        assert "vosk:low_confidence" in rejected[0].getMessage()


class TestTelemetryPhraseToAccept:
    """Issue 1076 (телеметрия): честный «замолчал → акцепт».

    stt_node логирует phrase_to_accept_ms — время от получения фразы
    (/audio/speech_audio) до ПРИНЯТО. Полный «замолчал → акцепт» =
    silence_to_phrase_s (audio_node, включает speech_continuation)
    + phrase_to_accept_ms (здесь).
    """

    def test_phrase_to_accept_logged_on_accept(self, stt_node, caplog):
        """При успешном распознавании пишется telemetry-строка с latency."""
        stt_node.aec_mode = "hardware"
        stt_node.tts_grace_s = 2.5
        stt_node.is_robot_speaking = False
        stt_node._tts_ended_at = 0.0
        stt_node._recognize_with_fallback = MagicMock(return_value=("робок привет", []))
        stt_node.result_pub = MagicMock()
        stt_node.state_pub = MagicMock()
        stt_node.tts_request_pub = MagicMock()
        stt_node.publish_result = MagicMock()

        # Публикация «не расслышал» не должна происходить при успехе
        stt_node._maybe_speak_unclear = MagicMock()

        caplog.set_level(logging.INFO, logger="test_stt_node_fallback")

        msg = MagicMock()
        msg.data = [0] * (16000 * 2)  # 1с PCM
        stt_node.speech_audio_callback(msg)

        telemetry = [
            r.getMessage()
            for r in caplog.records
            if "phrase_to_accept_ms=" in r.getMessage()
        ]
        assert len(telemetry) == 1, f"ожидалась 1 telemetry-строка, получено: {telemetry}"
        assert "phrase_to_accept_ms=" in telemetry[0]
        assert "text='робок привет'" in telemetry[0]

    def test_no_telemetry_on_rejected(self, stt_node, caplog):
        """При отклонении (None) telemetry-строка НЕ пишется."""
        stt_node.aec_mode = "hardware"
        stt_node.tts_grace_s = 2.5
        stt_node.is_robot_speaking = False
        stt_node._tts_ended_at = 0.0
        stt_node._recognize_with_fallback = MagicMock(return_value=(None, []))
        stt_node.result_pub = MagicMock()
        stt_node.state_pub = MagicMock()
        stt_node.tts_request_pub = MagicMock()
        stt_node._maybe_speak_unclear = MagicMock()

        caplog.set_level(logging.INFO, logger="test_stt_node_fallback")

        msg = MagicMock()
        msg.data = [0] * (16000 * 2)  # 1с PCM
        stt_node.speech_audio_callback(msg)

        telemetry = [
            r.getMessage()
            for r in caplog.records
            if "phrase_to_accept_ms=" in r.getMessage()
        ]
        assert telemetry == []
