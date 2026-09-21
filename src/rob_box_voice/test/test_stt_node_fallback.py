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
import time
from types import SimpleNamespace
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
    STTAuthError,
    STTQuotaError,
    STTTimeoutError,
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
        # Issue #1251 — ранний «бульк»
        early_boop_enabled=True,
        early_boop_trigger="boop",
        # Issue #2365 Phase 2 — цепочка провайдеров. MiniMax по умолчанию
        # ВЫКЛЮЧЕН в тестах намеренно: иначе у разработчика с живым
        # MINIMAX_API_KEY в окружении юнит-тесты полезут в сеть и станут
        # флаки. Тесты MiniMax включают его явно.
        stt_provider_chain=["yandex", "vosk"],
        minimax_stt_enabled=False,
        minimax_stt_api_key="",
        minimax_stt_api_key_env="MINIMAX_API_KEY_UNSET_FOR_TESTS",
        minimax_stt_timeout_s=5.0,
        minimax_stt_max_retries=1,
        provider_dead_ttl_s=300.0,
        provider_dead_ttl_transient_s=30.0,
        # Пустая строка — не писать файл состояния с юнит-теста.
        provider_state_file="",
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
    # ``stt_provider_chain`` — имя ROS-параметра, а нода хранит уже
    # нормализованную цепочку в ``provider_chain``; цикл выше про это не
    # знает, поэтому пересобираем её так же, как это делает __init__.
    node.provider_chain = stt_node_module.STTNode._normalize_provider_chain(
        list(defaults["stt_provider_chain"])
    )
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

        Issue #1477: в новой двухфазной реализации speech_analysis
        устанавливается через opts.speech_analysis.CopyFrom(...) ПОСЛЕ
        конструирования StreamingOptions — kwargs его не покажут. Поэтому
        дополнительно возвращаем объект-инстанс StreamingOptions (mock)
        чтобы тест мог проверить атрибут .speech_analysis.
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
            captured["opts_instance"] = (
                stt_pb2.StreamingOptions.return_value
            )
            return [final]

        stt_node_no_vosk.yandex_stub.RecognizeStreaming.side_effect = _consume_gen
        stt_node_no_vosk._recognize_yandex(b"\x00" * 8000)
        return captured

    def test_streaming_options_include_speech_analysis(self, stt_node_no_vosk):
        final = MagicMock()
        final.WhichOneof.return_value = "final"
        alt = MagicMock()
        alt.text = "робот меня зовут саша"
        final.final.alternatives = [alt]

        captured = self._capture_streaming_options(stt_node_no_vosk, final)
        opts_instance = captured.get("opts_instance")
        assert opts_instance is not None, "StreamingOptions должен создаваться в gen()"

        # Issue #1477: speech_analysis устанавливается через CopyFrom() после
        # конструктора — проверяем, что CopyFrom был вызван с нужными опциями.
        assert opts_instance.speech_analysis.CopyFrom.called, (
            "Должен вызываться opts.speech_analysis.CopyFrom() — иначе Yandex "
            "не присылает speaker_analysis (issue #1077)"
        )
        copied = opts_instance.speech_analysis.CopyFrom.call_args.args[0]
        assert copied.enable_speaker_analysis is True
        assert copied.enable_conversation_analysis is True

    def test_no_speaker_labeling_in_real_time(self, stt_node_no_vosk):
        """speaker_labeling требует FULL_DATA — в REAL_TIME его НЕ должно быть."""
        final = MagicMock()
        final.WhichOneof.return_value = "final"
        alt = MagicMock()
        alt.text = "привет"
        final.final.alternatives = [alt]

        captured = self._capture_streaming_options(stt_node_no_vosk, final)
        opts_instance = captured.get("opts_instance")
        assert opts_instance is not None
        # speech_analysis есть, но speaker_labeling НЕ должно быть ни в
        # kwargs конструктора, ни в speech_analysis (для REAL_TIME).
        opts_kwargs = captured.get("opts_kwargs", {})
        assert "speaker_labeling" not in opts_kwargs, (
            "speaker_labeling несовместим с REAL_TIME (INVALID_ARGUMENT) — "
            "используем speech_analysis"
        )


# ---------------------------------------------------------------------------
# Тесты параметров
# ---------------------------------------------------------------------------


class TestSTTNodeFallbackParams:
    """Проверка что новые параметры читаются из voice_assistant.yaml."""

    def test_default_yandex_timeout_is_12s(self, stt_node):
        # issue #979: 1.3s → 5.0s
        # issue #1477: 5.0s → 12.0s (фразы 4-6с с pre-roll + активный TTS/музыка
        # могут выходить за 5с gRPC deadline; FULL_DATA fallback требует больше).
        assert stt_node.yandex_timeout_s == 12.0

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
# Тесты двухфазного Yandex (issue #1477)
# ---------------------------------------------------------------------------


class TestYandexTwoPhaseFallback:
    """Issue #1477: двухфазный Yandex — REAL_TIME primary, FULL_DATA fallback.

    Acceptance: если фаза REAL_TIME дала empty, фаза FULL_DATA подхватывает
    фразу. Это лечит «yandex:empty» в music_library_suite, где REAL_TIME +
    speech_analysis давал empty на 4-6 секундных фразах с pre-roll.
    """

    def test_phase1_ok_skips_phase2(self, stt_node_no_vosk):
        """Если фаза 1 (REAL_TIME) дала текст, фаза 2 (FULL_DATA) не вызывается."""
        phase1_calls = []
        phase2_calls = []

        original = stt_node_no_vosk._recognize_yandex_phase

        def fake_phase(audio, *, phase, enable_speech_analysis):
            if phase == "REAL_TIME":
                phase1_calls.append(phase)
                return "робот меня зовут саша"
            phase2_calls.append(phase)
            return "никогда не вызывается"

        stt_node_no_vosk._recognize_yandex_phase = fake_phase
        try:
            text = stt_node_no_vosk._recognize_yandex(b"\x00" * 8000)
            assert text == "робот меня зовут саша"
            assert phase1_calls == ["REAL_TIME"]
            assert phase2_calls == [], (
                "Если фаза 1 вернула текст, фаза 2 (FULL_DATA) не должна вызываться"
            )
        finally:
            stt_node_no_vosk._recognize_yandex_phase = original

    def test_phase1_empty_triggers_phase2(self, stt_node_no_vosk):
        """Если фаза 1 вернула empty/None, фаза 2 (FULL_DATA) должна вызваться."""
        phase1_calls = []
        phase2_calls = []

        original = stt_node_no_vosk._recognize_yandex_phase

        def fake_phase(audio, *, phase, enable_speech_analysis):
            if phase == "REAL_TIME":
                phase1_calls.append(phase)
                return None
            phase2_calls.append((phase, enable_speech_analysis))
            return "робот расскажи анекдот"

        stt_node_no_vosk._recognize_yandex_phase = fake_phase
        try:
            text = stt_node_no_vosk._recognize_yandex(b"\x00" * 8000)
            assert text == "робот расскажи анекдот"
            assert phase1_calls == ["REAL_TIME"]
            assert phase2_calls == [("FULL_DATA", False)], (
                "Фаза 2 должна вызываться с phase=FULL_DATA, "
                "enable_speech_analysis=False"
            )
        finally:
            stt_node_no_vosk._recognize_yandex_phase = original

    def test_both_phases_empty_returns_none(self, stt_node_no_vosk):
        """Если обе фазы вернули None — итог None (Vosk-fallback пойдёт)."""
        calls = []

        original = stt_node_no_vosk._recognize_yandex_phase

        def fake_phase(audio, *, phase, enable_speech_analysis):
            calls.append(phase)
            return None

        stt_node_no_vosk._recognize_yandex_phase = fake_phase
        try:
            text = stt_node_no_vosk._recognize_yandex(b"\x00" * 8000)
            assert text is None
            assert calls == ["REAL_TIME", "FULL_DATA"]
            assert stt_node_no_vosk._last_speaker_tag is None
        finally:
            stt_node_no_vosk._recognize_yandex_phase = original

    def test_phase2_clears_speaker_tag(self, stt_node_no_vosk):
        """Если фаза 2 (без speech_analysis) сработала — speaker_tag сбрасывается."""
        original = stt_node_no_vosk._recognize_yandex_phase

        def fake_phase(audio, *, phase, enable_speech_analysis):
            if phase == "REAL_TIME":
                return None
            return "робот спасибо"

        stt_node_no_vosk._recognize_yandex_phase = fake_phase
        try:
            # speaker_tag должен сброситься
            stt_node_no_vosk._last_speaker_tag = "tag_before"
            text = stt_node_no_vosk._recognize_yandex(b"\x00" * 8000)
            assert text == "робот спасибо"
            assert stt_node_no_vosk._last_speaker_tag is None, (
                "speaker_tag должен быть None после фазы 2 (нет speech_analysis)"
            )
        finally:
            stt_node_no_vosk._recognize_yandex_phase = original

    def test_empty_audio_short_circuit(self, stt_node_no_vosk):
        """Пустые байты → None без обращения к Yandex."""
        calls = []

        original = stt_node_no_vosk._recognize_yandex_phase

        def fake_phase(audio, *, phase, enable_speech_analysis):
            calls.append(phase)
            return "не должно вызваться"

        stt_node_no_vosk._recognize_yandex_phase = fake_phase
        try:
            assert stt_node_no_vosk._recognize_yandex(b"") is None
            assert calls == [], "Пустой audio_bytes не должен вызывать Yandex"
        finally:
            stt_node_no_vosk._recognize_yandex_phase = original


class TestAudioStatsTelemetry:
    """Issue #1477: телеметрия фразы до отправки в Yandex.

    Acceptance: лог audio_rms_dbfs/peak_dbfs/duration перед стримом. Это
    устраняет главный источник «yandex:empty» — гадание «что пришло в
    микрофон»: речь (RMS > -30dBFS), эхо TTS (RMS > -20dBFS), тишина
    (RMS < -40dBFS).
    """

    def test_silence_logs_negative_rms(self, stt_node_no_vosk, caplog):
        """Для тишины RMS должен быть очень низким (dBFS < -30)."""
        import logging

        # 1 секунда тишины (int16 LE mono 16kHz).
        silence = b"\x00\x00" * 16000
        # stub логгера = "test_stt_node_fallback" (см. _make_stt_node_stub).
        with caplog.at_level(logging.INFO, logger="test_stt_node_fallback"):
            original = stt_node_no_vosk._recognize_yandex_phase

            def no_op(audio, *, phase, enable_speech_analysis):
                return "ignored"

            stt_node_no_vosk._recognize_yandex_phase = no_op
            try:
                stt_node_no_vosk._recognize_yandex(silence)
            finally:
                stt_node_no_vosk._recognize_yandex_phase = original

        rms_logs = [r for r in caplog.records if "audio_rms_dbfs" in r.message]
        assert rms_logs, "Должен быть лог audio_rms_dbfs перед отправкой в Yandex"
        msg = rms_logs[0].message
        # Тишина → RMS очень низкий.
        assert "audio_rms_dbfs=" in msg
        assert "duration=" in msg
        rms_str = msg.split("audio_rms_dbfs=")[1].split()[0]
        rms_dbfs = float(rms_str)
        assert rms_dbfs < -30.0, (
            f"rms_dbfs={rms_dbfs} для тишины (должен быть < -30, это -90..-50 dBFS)"
        )

    def test_loud_signal_logs_peak(self, stt_node_no_vosk, caplog):
        """Для громкого сигнала peak_dbfs должен быть близок к 0."""
        import logging
        import struct

        # Громкая синусоида (амплитуда 30000 из 32768).
        sr = 16000
        n = sr  # 1 секунда
        audio_bytes = b"".join(
            struct.pack("<h", int(30000 * math.sin(2 * math.pi * 440 * i / sr)))
            for i in range(n)
        )
        with caplog.at_level(logging.INFO, logger="test_stt_node_fallback"):
            original = stt_node_no_vosk._recognize_yandex_phase

            def no_op(audio, *, phase, enable_speech_analysis):
                return "ignored"

            stt_node_no_vosk._recognize_yandex_phase = no_op
            try:
                stt_node_no_vosk._recognize_yandex(audio_bytes)
            finally:
                stt_node_no_vosk._recognize_yandex_phase = original

        rms_logs = [r for r in caplog.records if "audio_rms_dbfs" in r.message]
        assert rms_logs
        msg = rms_logs[0].message
        assert "peak_dbfs=" in msg
        # peak должен быть > -5dBFS для амплитуды 30000 (log10(30000/32768) ≈ -0.76)
        peak_str = msg.split("peak_dbfs=")[1].split()[0]
        peak_dbfs = float(peak_str)
        assert peak_dbfs > -5.0, f"peak_dbfs={peak_dbfs} для громкого сигнала"


# Импорт math нужен для телеметрии (используется внутри _recognize_yandex).
import math  # noqa: E402 — ставим после определения тестов, чтобы не путать lint


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
        """Короткий Vosk-мусор (1 char) → text=«а» (rejected(short), не None).

        Возврат непустого текста критичен: speech_audio_callback по нему
        отличает rejected(short) (был речевой ввод → переспросить) от
        rejected(empty) (эхо → молчать), issue #979 acceptance.
        """
        stt_node.yandex_stub = MagicMock()
        stt_node.recognizer = MagicMock()
        stt_node._recognize_yandex = MagicMock(return_value=None)
        stt_node._recognize_vosk = MagicMock(return_value="а")  # 1 char мусор

        text, attempts = stt_node._recognize_with_fallback(b"\x00" * 1000)
        assert text == "а"
        # Последняя попытка — Vosk с low_confidence
        assert attempts[-1].reason == "low_confidence"
        assert attempts[-1].provider == "vosk"

    def test_respects_custom_min_text_chars(self, stt_node):
        """min_text_chars=5 → фраза из 4 chars отклоняется, но текст
        возвращается (rejected(short)) — caller переспросит."""
        stt_node.min_text_chars = 5
        stt_node.yandex_stub = MagicMock()
        stt_node.recognizer = MagicMock()
        stt_node._recognize_yandex = MagicMock(return_value=None)
        stt_node._recognize_vosk = MagicMock(return_value="стоп")  # 4 chars

        text, attempts = stt_node._recognize_with_fallback(b"\x00" * 1000)
        assert text == "стоп"  # не None: rejected(short), не rejected(empty)
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

    def test_speech_audio_callback_vosk_garbage_speaks_unclear_real_path(self, stt_node):
        """Issue #979 acceptance: реальный путь Yandex empty → Vosk «не»
        (мусор) должен привести к «не расслышал», а НЕ к молчанию.

        До фикса select_recognition возвращал None при low_confidence, и
        speech_audio_callback классифицировал это как rejected(empty) →
        робот молчал (именно баг из issue #979). После фикса возвращается
        последний непустой текст («не») → rejected(short) → «не расслышал».
        """
        stt_node.yandex_stub = MagicMock()
        stt_node.recognizer = MagicMock()
        stt_node._recognize_yandex = MagicMock(return_value=None)  # empty x2
        stt_node._recognize_vosk = MagicMock(return_value="не")  # 2 chars мусор
        stt_node.tts_request_pub = MagicMock()
        stt_node.result_pub = MagicMock()
        stt_node.state_pub = MagicMock()
        stt_node._last_unclear_at = 0.0

        msg = MagicMock()
        msg.data = [0] * (16000 * 2)  # 1с PCM
        stt_node.speech_audio_callback(msg)

        # Результат НЕ опубликован (слишком короткое), но робот просит
        # повторить: «Не расслышал, скажи ещё раз» — вместо молчания.
        assert stt_node.result_pub.publish.call_count == 0
        assert stt_node.tts_request_pub.publish.call_count == 1
        payload = stt_node.tts_request_pub.publish.call_args[0][0].data
        assert "Не расслышал" in payload


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


class TestBargeInWakeWordStopTTS:
    """Issue 993: STT-уровень barge-in — wake word во время TTS → STOP TTS.

    Цепочка barge-in (после снятия VAD-гейта в audio_node, фикс 2ad5ea58):
    1. VAD пропускает речь во время TTS (audio_node._vad_gated: tts_active → pass)
    2. STT обрабатывает фразу ≥0.8s во время TTS (hardware AEC)
    3. publish_result: фраза начинается с wake word → немедленный STOP TTS

    Раньше (Fix B из #989) VAD гейтился на всё время TTS, поэтому даже
    «робот, добавь бит» не доходило до STT. Теперь гейт снят, и STT обязан
    прервать TTS при wake word — это и есть barge-in.

    Issue #1734 — этот немедленный STOP работает ТОЛЬКО при
    ``barge_in_policy="replace"`` (дефолт, ``self._barge_in_policy``
    инициализируется в "replace" в ``__init__`` — fail-safe до первого
    сообщения от dialogue_node). Тесты ниже это явно закрепляют. Отдельная
    ветка при ``barge_in_policy="classify"`` — класс ``TestBargeInClassifyPolicyDefersStop``
    ниже: STOP там НЕ публикуется, решение отдаётся dialogue_node/quick_decide
    (§2.5 SCHEDULER_DESIGN.md).
    """

    @staticmethod
    def _patch_string_factory(monkeypatch, stt_node_module):
        """std_msgs.msg.String в моках — один MagicMock: String() возвращает
        ОДИН и тот же instance, поэтому stop_msg.data перезаписывается
        msg.data (оба — один объект). Для проверки payload'а нужны РАЗНЫЕ
        instance'ы — патчим фабрикой.
        """

        class _String:
            _n = 0

            def __init__(self):
                _String._n += 1
                self.data = ""

        monkeypatch.setattr(stt_node_module, "String", _String)

    def test_publish_result_wake_word_stops_tts(self, stt_node, monkeypatch):
        """Фраза с wake word → tts_control_pub получает 'STOP' (barge-in)."""
        from rob_box_voice import stt_node as stt_node_module

        self._patch_string_factory(monkeypatch, stt_node_module)
        stt_node._barge_in_policy = "replace"  # issue #1734: явный regression-pin
        stt_node.tts_control_pub = MagicMock()
        stt_node.result_pub = MagicMock()
        # В фикстуре publish_result замокан (чтобы другие тесты не публиковали),
        # для этого теста нужен РЕАЛЬНЫЙ метод — bind через __get__.
        stt_node.publish_result = stt_node_module.STTNode.publish_result.__get__(
            stt_node, stt_node_module.STTNode
        )

        stt_node.publish_result("робот, добавь бит")

        # Немедленный STOP TTS (barge-in)
        stop_call = stt_node.tts_control_pub.publish.call_args[0][0]
        assert stop_call.data == "STOP"
        # Результат всё равно публикуется (dialogue_node обработает)
        assert stt_node.result_pub.publish.call_count == 1

    def test_publish_result_without_wake_word_no_stop(self, stt_node, monkeypatch):
        """Фраза без wake word (эхо собственного голоса) → БЕЗ STOP TTS."""
        from rob_box_voice import stt_node as stt_node_module

        self._patch_string_factory(monkeypatch, stt_node_module)
        stt_node._barge_in_policy = "replace"  # issue #1734: явный regression-pin
        stt_node.tts_control_pub = MagicMock()
        stt_node.result_pub = MagicMock()
        stt_node.publish_result = stt_node_module.STTNode.publish_result.__get__(
            stt_node, stt_node_module.STTNode
        )

        stt_node.publish_result("не расслышал скажи")

        stt_node.tts_control_pub.publish.assert_not_called()
        assert stt_node.result_pub.publish.call_count == 1

    def test_full_chain_barge_in_stop_tts(self, stt_node, monkeypatch):
        """Полная цепочка: фраза с wake word во время TTS → STOP TTS.

        Симулируем: TTS активен (is_robot_speaking=True), hardware AEC,
        фраза 2с (≥0.8s) распознана как «робот добавь бит» →
        speech_audio_callback → publish_result (реальный) → STOP TTS.
        Это ровно acceptance #993: «робот, добавь бит» во время пения.
        """
        from rob_box_voice import stt_node as stt_node_module

        self._patch_string_factory(monkeypatch, stt_node_module)
        stt_node._barge_in_policy = "replace"  # issue #1734: явный regression-pin
        stt_node.aec_mode = "hardware"
        stt_node.tts_grace_s = 2.5
        stt_node.is_robot_speaking = True  # TTS активен
        stt_node._tts_ended_at = 0.0  # «давно» — не в grace (это НЕ хвост TTS)
        stt_node._recognize_with_fallback = MagicMock(
            return_value=("робот добавь бит", [])
        )
        stt_node.tts_control_pub = MagicMock()
        stt_node.result_pub = MagicMock()
        stt_node.state_pub = MagicMock()
        stt_node.tts_request_pub = MagicMock()
        stt_node.publish_result = stt_node_module.STTNode.publish_result.__get__(
            stt_node, stt_node_module.STTNode
        )

        msg = MagicMock()
        msg.data = [0] * (16000 * 2 * 2)  # 2с PCM (≥0.8s — «возможно прерывание»)
        stt_node.speech_audio_callback(msg)

        stt_node._recognize_with_fallback.assert_called_once()
        stop_call = stt_node.tts_control_pub.publish.call_args[0][0]
        assert stop_call.data == "STOP"
        assert stt_node.result_pub.publish.call_count == 1

    def test_short_phrase_during_tts_ignored(self, stt_node):
        """Hardware AEC: короткая фраза <0.8s во время TTS → игнор (эхо).

        Не путать с barge-in: короткие всплески собственного голоса
        (эхо) режутся, длинная команда пользователя (≥0.8s) проходит.
        """
        stt_node.aec_mode = "hardware"
        stt_node.tts_grace_s = 2.5
        stt_node.is_robot_speaking = True
        stt_node._tts_ended_at = 0.0
        stt_node._recognize_with_fallback = MagicMock(return_value=("робот", []))
        stt_node.result_pub = MagicMock()
        stt_node.state_pub = MagicMock()
        stt_node.tts_request_pub = MagicMock()
        stt_node.publish_result = MagicMock()

        msg = MagicMock()
        msg.data = [0] * (16000 * 2 // 2)  # 0.5с PCM (<0.8s)
        stt_node.speech_audio_callback(msg)

        stt_node._recognize_with_fallback.assert_not_called()
        assert stt_node.publish_result.call_count == 0


class TestBargeInClassifyPolicyDefersStop:
    """Issue #1734: при ``barge_in_policy="classify"`` stt_node НЕ публикует
    немедленный STOP на wake-word — решение (STOP/MERGE/PENDING_LLM/IGNORE)
    отдаётся dialogue_node/quick_decide, чтобы «правка на лету без
    замолкания» (§2.5 SCHEDULER_DESIGN.md) реально работала.

    Regression pin: до фикса этот код публиковал STOP безусловно, что и
    ломало сценарий из raw evidence issue #1734 («комар» обрывался на
    «и ещё про енота», хотя quick_decide должен был смёржить сегменты) —
    см. также test_barge_in_policy.py::TestQuickDecideDispatch на стороне
    dialogue_node (там уже проверено, что _cancel_run сам публикует STOP
    для REPLACE-вердикта — stt_node дублировать это не должен).
    """

    @staticmethod
    def _patch_string_factory(monkeypatch, stt_node_module):
        TestBargeInWakeWordStopTTS._patch_string_factory(monkeypatch, stt_node_module)

    def test_classify_wake_word_does_not_stop_tts(self, stt_node, monkeypatch):
        """policy=classify — wake-word НЕ шлёт STOP (в отличие от replace)."""
        from rob_box_voice import stt_node as stt_node_module

        self._patch_string_factory(monkeypatch, stt_node_module)
        stt_node._barge_in_policy = "classify"
        stt_node.tts_control_pub = MagicMock()
        stt_node.result_pub = MagicMock()
        stt_node.publish_result = stt_node_module.STTNode.publish_result.__get__(
            stt_node, stt_node_module.STTNode
        )

        stt_node.publish_result("робот и ещё про енота")

        stt_node.tts_control_pub.publish.assert_not_called()
        # Результат всё равно публикуется — dialogue_node должен его увидеть,
        # чтобы вообще смочь прогнать quick_decide.
        assert stt_node.result_pub.publish.call_count == 1

    def test_classify_without_wake_word_no_stop(self, stt_node, monkeypatch):
        """policy=classify без wake word — тоже без STOP (как и раньше)."""
        from rob_box_voice import stt_node as stt_node_module

        self._patch_string_factory(monkeypatch, stt_node_module)
        stt_node._barge_in_policy = "classify"
        stt_node.tts_control_pub = MagicMock()
        stt_node.result_pub = MagicMock()
        stt_node.publish_result = stt_node_module.STTNode.publish_result.__get__(
            stt_node, stt_node_module.STTNode
        )

        stt_node.publish_result("не расслышал скажи")

        stt_node.tts_control_pub.publish.assert_not_called()
        assert stt_node.result_pub.publish.call_count == 1

    def test_default_policy_is_replace_before_any_topic_message(self, stt_node):
        """Fail-safe: до первого сообщения от dialogue_node __init__
        оставляет ``_barge_in_policy == "replace"`` — сохраняет поведение
        issue #993, а не молча его выключает."""
        assert stt_node._barge_in_policy == "replace"


class TestBargeInPolicyCallback:
    """Issue #1734 — приём политики от dialogue_node через latched-топик
    ``/voice/dialogue/barge_in_policy`` (``barge_in_policy_callback``)."""

    def test_classify_message_updates_policy(self, stt_node):
        msg = MagicMock()
        msg.data = "classify"
        stt_node.barge_in_policy_callback(msg)
        assert stt_node._barge_in_policy == "classify"

    def test_replace_message_updates_policy(self, stt_node):
        stt_node._barge_in_policy = "classify"
        msg = MagicMock()
        msg.data = "replace"
        stt_node.barge_in_policy_callback(msg)
        assert stt_node._barge_in_policy == "replace"

    def test_case_insensitive_and_whitespace_tolerant(self, stt_node):
        msg = MagicMock()
        msg.data = "  CLASSIFY  "
        stt_node.barge_in_policy_callback(msg)
        assert stt_node._barge_in_policy == "classify"

    def test_unknown_value_ignored_keeps_current_policy(self, stt_node):
        """Опечатка/невалидное значение — dialogue_node уже сам провалидировал
        и залогировал warning; здесь просто не трогаем текущее значение."""
        stt_node._barge_in_policy = "replace"
        msg = MagicMock()
        msg.data = "yolo"
        stt_node.barge_in_policy_callback(msg)
        assert stt_node._barge_in_policy == "replace"

    def test_empty_value_ignored(self, stt_node):
        stt_node._barge_in_policy = "classify"
        msg = MagicMock()
        msg.data = ""
        stt_node.barge_in_policy_callback(msg)
        assert stt_node._barge_in_policy == "classify"


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


class TestVoskLazyLoad:
    """Issue #2609 — Vosk грузится при первом fallback, а не на старте.

    Модель держит ~400 МБ RSS в stt_node, а на Vision Pi (8 ГБ) нужна только
    когда Yandex не ответил.
    """

    @staticmethod
    def _node_with_model_on_disk(monkeypatch):
        monkeypatch.setattr("os.path.isdir", lambda _p: True)
        node = _make_stt_node_stub()
        from rob_box_voice import stt_node as stt_node_module

        return node, stt_node_module

    def test_model_not_loaded_at_startup(self, monkeypatch):
        node, mod = self._node_with_model_on_disk(monkeypatch)
        assert node._vosk_available is True
        assert node.recognizer is None
        mod.Model.assert_not_called()

    def test_first_vosk_call_loads_model_once(self, monkeypatch):
        node, mod = self._node_with_model_on_disk(monkeypatch)
        mod.KaldiRecognizer.return_value.FinalResult.return_value = '{"text": "привет"}'

        assert node._recognize_vosk(b"\x00" * 8000) == "привет"
        assert node._recognize_vosk(b"\x00" * 8000) == "привет"
        assert mod.Model.call_count == 1

    def test_fallback_offers_vosk_before_it_is_loaded(self, monkeypatch):
        node, _mod = self._node_with_model_on_disk(monkeypatch)
        node.yandex_stub = None
        with pytest.MonkeyPatch.context() as mp:
            mp.setattr(node, "_recognize_vosk", MagicMock(return_value="привет робот"))
            text, attempts = node._recognize_with_fallback(b"\x00" * 8000)
        assert text == "привет робот"
        assert [a.provider for a in attempts] == ["vosk"]

    def test_preload_loads_model_at_init(self, monkeypatch):
        node, mod = self._node_with_model_on_disk(monkeypatch)
        node.vosk_preload = True
        node.initialize_vosk()
        assert node.recognizer is not None
        assert mod.Model.call_count == 1

    def test_load_failure_disables_vosk(self, monkeypatch):
        node, mod = self._node_with_model_on_disk(monkeypatch)
        mod.Model.side_effect = RuntimeError("broken model")
        assert node._recognize_vosk(b"\x00" * 8000) is None
        assert node._vosk_available is False
        assert node._recognize_vosk(b"\x00" * 8000) is None
        assert mod.Model.call_count == 1

    def test_missing_model_dir_leaves_vosk_unavailable(self, monkeypatch):
        monkeypatch.setattr("os.path.isdir", lambda _p: False)
        node = _make_stt_node_stub()
        assert node._vosk_available is False
        assert node._recognize_vosk(b"\x00" * 8000) is None


def test_vosk_adapter_prepare_loads_model_outside_timeout(monkeypatch):
    """Issue #2609 — загрузка Vosk не должна съедать таймаут первой фразы."""
    monkeypatch.setattr("os.path.isdir", lambda _p: True)
    node = _make_stt_node_stub()
    from rob_box_voice import stt_node as mod

    node.yandex_stub = None
    node.yandex_timeout_s = 0.2
    mod.KaldiRecognizer.return_value.FinalResult.return_value = '{"text": "привет робот"}'

    real_model = mod.Model

    def _slow_model(*a, **kw):
        time.sleep(0.4)
        return real_model(*a, **kw)

    monkeypatch.setattr(mod, "Model", _slow_model)
    text, attempts = node._recognize_with_fallback(b"\x00" * 8000)
    assert text == "привет робот"
    assert [(a.provider, a.reason) for a in attempts] == [("vosk", "ok")]


# ---------------------------------------------------------------------------
# Issue #2365 Phase 2 (ADR-0124): цепочка minimax → yandex → vosk
# ---------------------------------------------------------------------------


class TestProviderChainNormalization:
    """Инварианты ``_normalize_provider_chain`` (аналог tts_node, #1083)."""

    @staticmethod
    def _normalize(chain, logger=None):
        from rob_box_voice.stt_node import STTNode

        return STTNode._normalize_provider_chain(chain, logger=logger)

    def test_default_order_is_minimax_yandex_vosk(self):
        from rob_box_voice.stt_node import DEFAULT_STT_PROVIDER_CHAIN

        assert DEFAULT_STT_PROVIDER_CHAIN == ["minimax", "yandex", "vosk"]

    def test_empty_chain_falls_back_to_default(self):
        assert self._normalize([]) == ["minimax", "yandex", "vosk"]

    def test_vosk_is_forced_last(self):
        """Vosk в середине — переносится в конец: он последний рубеж."""
        assert self._normalize(["vosk", "minimax", "yandex"]) == [
            "minimax",
            "yandex",
            "vosk",
        ]

    def test_vosk_appended_when_missing(self):
        """Без Vosk цепочка оставила бы робота глухим при мёртвых облаках."""
        assert self._normalize(["minimax", "yandex"]) == [
            "minimax",
            "yandex",
            "vosk",
        ]

    def test_duplicates_removed_keeping_first_position(self):
        assert self._normalize(["yandex", "minimax", "yandex"]) == [
            "yandex",
            "minimax",
            "vosk",
        ]

    def test_vosk_only_is_a_legitimate_offline_mode(self):
        assert self._normalize(["vosk"]) == ["vosk"]

    def test_unknown_provider_is_dropped_with_warning(self):
        logger = MagicMock()
        assert self._normalize(["whisper", "yandex"], logger=logger) == [
            "yandex",
            "vosk",
        ]
        assert logger.warning.called

    def test_garbage_chain_falls_back_to_default(self):
        assert self._normalize(["whisper", "azure"]) == [
            "minimax",
            "yandex",
            "vosk",
        ]

    def test_case_and_whitespace_tolerated(self):
        assert self._normalize([" MiniMax ", "YANDEX"]) == [
            "minimax",
            "yandex",
            "vosk",
        ]


class TestProviderChainWiring:
    """``_build_provider_chain`` — кого и в каком порядке реально зовём."""

    def test_chain_order_follows_parameter(self, stt_node):
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        stt_node.minimax_stt_enabled = True
        stt_node.minimax_stt_api_key = "FAKE"
        stt_node.yandex_stub = MagicMock()
        stt_node.recognizer = MagicMock()

        names = [p.name for p in stt_node._build_provider_chain()]

        assert names == ["minimax", "yandex", "vosk"]

    def test_minimax_skipped_without_key(self, stt_node):
        """Нет ключа — MiniMax тихо выпадает (ADR-0091 §5.2), без ошибок."""
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        stt_node.minimax_stt_enabled = True
        stt_node.minimax_stt_api_key = ""
        stt_node.yandex_stub = MagicMock()
        stt_node.recognizer = MagicMock()

        names = [p.name for p in stt_node._build_provider_chain()]

        assert names == ["yandex", "vosk"]

    def test_minimax_skipped_when_disabled(self, stt_node):
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        stt_node.minimax_stt_enabled = False
        stt_node.minimax_stt_api_key = "FAKE"
        stt_node.yandex_stub = MagicMock()
        stt_node.recognizer = MagicMock()

        names = [p.name for p in stt_node._build_provider_chain()]

        assert names == ["yandex", "vosk"]

    def test_yandex_skipped_without_stub(self, stt_node):
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        stt_node.minimax_stt_enabled = True
        stt_node.minimax_stt_api_key = "FAKE"
        stt_node.yandex_stub = None
        stt_node.recognizer = MagicMock()

        names = [p.name for p in stt_node._build_provider_chain()]

        assert names == ["minimax", "vosk"]

    def test_vosk_prepare_is_wired(self, stt_node):
        """Vosk грузит модель через prepare() — вне таймаута (#2609)."""
        stt_node.provider_chain = ["vosk"]
        stt_node.recognizer = MagicMock()
        stt_node._ensure_vosk_loaded = MagicMock(return_value=True)

        chain = stt_node._build_provider_chain()
        chain[0].prepare()

        assert stt_node._ensure_vosk_loaded.called

    def test_per_provider_policies(self, stt_node):
        stt_node.minimax_stt_timeout_s = 5.0
        stt_node.minimax_stt_max_retries = 1
        stt_node.yandex_timeout_s = 12.0
        stt_node.yandex_max_retries = 1

        policies = stt_node._provider_policies()

        assert policies["minimax"].timeout_s == 5.0
        assert policies["minimax"].max_retries == 1
        assert policies["yandex"].timeout_s == 12.0
        # Vosk офлайновый: повтор мусора даст тот же мусор.
        assert policies["vosk"].max_retries == 0


class TestRecognizeChainEndToEnd:
    """Полный прогон ``_recognize_with_fallback`` по новой цепочке."""

    @staticmethod
    def _prepare(node):
        node.provider_chain = ["minimax", "yandex", "vosk"]
        node.minimax_stt_enabled = True
        node.minimax_stt_api_key = "FAKE"
        node.yandex_stub = MagicMock()
        node.recognizer = MagicMock()
        node.retry_backoff_s = 0.0
        node.minimax_stt_max_retries = 0
        node.yandex_max_retries = 0
        node._ensure_vosk_loaded = MagicMock(return_value=True)
        return node

    def test_minimax_wins_when_alive(self, stt_node):
        node = self._prepare(stt_node)
        node._recognize_minimax = MagicMock(return_value="робот расскажи анекдот")
        node._recognize_yandex = MagicMock(return_value="яндекс не нужен")
        node._recognize_vosk = MagicMock(return_value="воск не нужен")

        text, attempts = node._recognize_with_fallback(b"\x00" * 1000)

        assert text == "робот расскажи анекдот"
        assert attempts[0].provider == "minimax"
        assert node._recognize_yandex.called is False
        assert node._recognize_vosk.called is False

    def test_falls_through_to_yandex_then_vosk(self, stt_node):
        node = self._prepare(stt_node)
        node._recognize_minimax = MagicMock(side_effect=STTQuotaError("2056"))
        node._recognize_yandex = MagicMock(side_effect=STTQuotaError("RESOURCE_EXHAUSTED"))
        node._recognize_vosk = MagicMock(return_value="робот расскажи анекдот")

        text, attempts = node._recognize_with_fallback(b"\x00" * 1000)

        assert text == "робот расскажи анекдот"
        assert [a.provider for a in attempts] == ["minimax", "yandex", "vosk"]
        assert [a.reason for a in attempts] == ["error", "error", "ok"]

    def test_dead_clouds_are_skipped_on_next_phrase(self, stt_node):
        """Сценарий 21.09: оба облака без денег — вторая фраза идёт в Vosk сразу."""
        node = self._prepare(stt_node)
        node._recognize_minimax = MagicMock(side_effect=STTQuotaError("2056"))
        node._recognize_yandex = MagicMock(side_effect=STTQuotaError("RESOURCE_EXHAUSTED"))
        node._recognize_vosk = MagicMock(return_value="робот расскажи анекдот")

        node._recognize_with_fallback(b"\x00" * 1000)
        text, attempts = node._recognize_with_fallback(b"\x00" * 1000)

        assert node._recognize_minimax.call_count == 1
        assert node._recognize_yandex.call_count == 1
        assert node._recognize_vosk.call_count == 2
        assert text == "робот расскажи анекдот"
        assert [a.reason for a in attempts] == ["dead", "dead", "ok"]

    def test_no_providers_returns_none(self, stt_node):
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        stt_node.minimax_stt_enabled = False
        stt_node.yandex_stub = None
        stt_node.recognizer = None
        stt_node._vosk_available = False

        text, attempts = stt_node._recognize_with_fallback(b"\x00" * 1000)

        assert text is None
        assert attempts == []


class TestEffectiveProvider:
    """Фактический провайдер после фолбека (лог + файл состояния)."""

    def test_effective_is_head_of_chain_when_all_alive(self, stt_node):
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        assert stt_node._effective_provider() == "minimax"

    def test_effective_skips_dead_providers(self, stt_node):
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        stt_node._provider_dead_cache.mark_dead("minimax", "quota", transient=False)
        assert stt_node._effective_provider() == "yandex"

    def test_effective_is_last_when_everyone_dead(self, stt_node):
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        for name in stt_node.provider_chain:
            stt_node._provider_dead_cache.mark_dead(name, "dead", transient=False)
        assert stt_node._effective_provider() == "vosk"

    def test_state_persisted_only_on_change(self, stt_node, tmp_path):
        """Строка в логе и запись в файл — только при СМЕНЕ провайдера."""
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        stt_node.provider_state_file = str(tmp_path / "state.json")
        stt_node._last_effective_provider = None
        stt_node._persist_provider_state = MagicMock()

        stt_node._log_provider_state("startup")
        assert stt_node._persist_provider_state.call_count == 1

        stt_node._log_provider_state("recognize")  # ничего не изменилось
        assert stt_node._persist_provider_state.call_count == 1

        stt_node._provider_dead_cache.mark_dead("minimax", "quota", transient=False)
        stt_node._log_provider_state("recognize")
        assert stt_node._persist_provider_state.call_count == 2

    def test_persisted_payload_names_provider_and_dead(self, stt_node):
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        stt_node._last_effective_provider = None
        stt_node._persist_provider_state = MagicMock()
        stt_node._provider_dead_cache.mark_dead("minimax", "quota", transient=False)

        stt_node._log_provider_state("startup")

        payload = stt_node._persist_provider_state.call_args[0][0]
        assert payload["provider"] == "yandex"
        assert "minimax" in payload["dead_providers"]


class TestPersistedProviderState:
    """Рестарт ноды не должен снова платить таймаут мёртвому облаку (#2676)."""

    def test_round_trip_through_file(self, stt_node, tmp_path):
        state_file = tmp_path / "stt_provider_state.json"
        stt_node.provider_state_file = str(state_file)
        stt_node.provider_chain = ["minimax", "yandex", "vosk"]
        stt_node._last_effective_provider = None
        stt_node._provider_dead_cache.mark_dead("minimax", "quota", transient=False)

        stt_node._log_provider_state("startup")
        assert state_file.exists()

        fresh = _make_stt_node_stub(provider_state_file=str(state_file))
        fresh.provider_state_file = str(state_file)
        fresh._load_persisted_provider_state()

        assert fresh._provider_dead_cache.is_dead("minimax") is True

    def test_missing_file_does_not_break_startup(self, stt_node, tmp_path):
        stt_node.provider_state_file = str(tmp_path / "nope.json")
        stt_node._load_persisted_provider_state()  # не должно бросить

    def test_broken_json_does_not_break_startup(self, stt_node, tmp_path):
        broken = tmp_path / "broken.json"
        broken.write_text("{не json", encoding="utf-8")
        stt_node.provider_state_file = str(broken)
        stt_node._load_persisted_provider_state()  # не должно бросить


class TestMiniMaxErrorMapping:
    """MiniMax-исключения → типизированные ошибки цепочки (для кэша)."""

    @staticmethod
    def _node_with_provider(stt_node, exc):
        provider = MagicMock()
        provider.transcribe.side_effect = exc
        stt_node._ensure_minimax_provider = lambda: provider
        return stt_node

    def test_auth_error_becomes_stt_auth_error(self, stt_node):
        from rob_box_voice.stt_providers.minimax_provider import MiniMaxSTTAuthError

        node = self._node_with_provider(stt_node, MiniMaxSTTAuthError("HTTP 401"))
        with pytest.raises(STTAuthError):
            node._recognize_minimax(b"\x00" * 100)

    def test_rate_limit_becomes_quota_error(self, stt_node):
        from rob_box_voice.stt_providers.minimax_provider import (
            MiniMaxSTTRateLimitError,
        )

        node = self._node_with_provider(
            stt_node, MiniMaxSTTRateLimitError("HTTP 429")
        )
        with pytest.raises(STTQuotaError):
            node._recognize_minimax(b"\x00" * 100)

    def test_timeout_becomes_stt_timeout_error(self, stt_node):
        from rob_box_voice.stt_providers.minimax_provider import (
            MiniMaxSTTUnavailableError,
        )

        node = self._node_with_provider(
            stt_node, MiniMaxSTTUnavailableError("timeout: read")
        )
        with pytest.raises(STTTimeoutError):
            node._recognize_minimax(b"\x00" * 100)

    def test_unconfigured_provider_returns_none(self, stt_node):
        stt_node._ensure_minimax_provider = lambda: None
        assert stt_node._recognize_minimax(b"\x00" * 100) is None

    def test_text_is_returned_on_success(self, stt_node):
        provider = MagicMock()
        provider.transcribe.return_value = SimpleNamespace(text="робот привет")
        stt_node._ensure_minimax_provider = lambda: provider

        assert stt_node._recognize_minimax(b"\x00" * 100) == "робот привет"
