#!/usr/bin/env python3
"""
STTNode - Speech-to-Text с Yandex STT gRPC v3 (primary) + Vosk (fallback)

Единственная точка маршрутизации речи (целевая §7, issue #1990):

| вход | выход |
|---|---|
| ``/audio/speech_audio`` (ReSpeaker, люди рядом) | ``/voice/stt/result`` → личность |
| ``/audio/quest_in`` (левый грип, PTT robot-voice) | ``/avatar/ptt/result`` → пайплайн грипа |
| ``/audio/quest_wake`` (wake-поток шлема, вейк «ТАРС») | ``/avatar/stt/result`` → агент оператора |

Namespace вейк-слов привязан к источнику аудио, а не только к тексту
(``config/wake_words.yaml`` — SSoT, см. ``core.dialogue_text``): «ТАРС» из
ReSpeaker игнорируется, вейк личности из микрофона шлема игнорируется.
"""

import json
import os
import threading
import time
from typing import Optional

import grpc
import rclpy
from audio_common_msgs.msg import AudioData
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vosk import KaldiRecognizer, Model

# Pure-Python fallback/retry/short-phrase логика (issue #979).
# Держим rclpy-зависимости подальше от чистой логики, чтобы её можно
# было гонять в юнит-тестах без ROS-окружения.
try:
    from rob_box_voice.stt_fallback import (
        DEFAULT_DEAD_TTL_S,
        DEFAULT_DEAD_TTL_TRANSIENT_S,
        DEFAULT_MAX_TOTAL_BUDGET_S,
        DEFAULT_MIN_TEXT_CHARS,
        DEFAULT_YANDEX_MAX_RETRIES,
        DEFAULT_YANDEX_TIMEOUT_S,
        ProviderDeadCache,
        ProviderPolicy,
        STTAuthError,
        STTQuotaError,
        STTTimeoutError,
        is_short_phrase,
        log_attempts,
        select_recognition,
    )

    _STT_FALLBACK_AVAILABLE = True
except ImportError:  # pragma: no cover — модуль всегда есть в нашем пакете
    _STT_FALLBACK_AVAILABLE = False
    DEFAULT_MIN_TEXT_CHARS = 3
    DEFAULT_YANDEX_MAX_RETRIES = 1
    DEFAULT_YANDEX_TIMEOUT_S = 5.0
    DEFAULT_DEAD_TTL_S = 300.0
    DEFAULT_DEAD_TTL_TRANSIENT_S = 30.0
    DEFAULT_MAX_TOTAL_BUDGET_S = 20.0
    ProviderDeadCache = None  # type: ignore[assignment]
    ProviderPolicy = None  # type: ignore[assignment]

    class STTTimeoutError(TimeoutError):  # type: ignore[no-redef]
        pass

    class STTAuthError(Exception):  # type: ignore[no-redef]
        pass

    class STTQuotaError(Exception):  # type: ignore[no-redef]
        pass


# Issue #2365 Phase 2 — цепочка STT-провайдеров по приоритету.
#
# Порядок yandex → minimax → vosk — issue #2866 (23.09.2026): товарищ
# Шифу пополнил счёт Yandex SpeechKit и поставил его primary. До этого
# (21.09, ADR-0124 §2.1) первым был MiniMax; ADR-0124 в свою очередь
# заменил vosk → minimax → yandex из ADR-0091 §2.2. Принцип прежний:
# облака вперёд за качеством, локальная модель — последний рубеж,
# который работает всегда. Значение обязано совпадать с
# ``stt_provider_chain`` в обоих stt_node.yaml (гард:
# test/test_yaml_param_consistency.py) — YAML перекрывает дефолт
# ``declare_parameter``, и рассинхрон молча врал бы в dev-окружении.
DEFAULT_STT_PROVIDER_CHAIN = ["yandex", "minimax", "vosk"]

# Провайдеры, которые нода умеет собирать. Всё остальное в
# ``stt_provider_chain`` — опечатка оператора, молча игнорируем с warning.
KNOWN_STT_PROVIDERS = frozenset({"minimax", "yandex", "vosk"})

# Vosk — офлайновый последний рубеж: он единственный работает без сети и
# без денег на счету. Инвариант «vosk всегда последний» — прямой аналог
# «silero всегда последний» в ``tts_node._normalize_provider_chain``.
_LAST_RESORT_PROVIDER = "vosk"

# Issue #2767 — порог для диагностического warning'а «сигнал очень тихий».
# Живой инцидент 23.09: 7/8 фраз отклонены при audio_rms_dbfs=-56.8 (пиковый
# -33.7 dBFS) — это НЕ доказывает, что дело в микрофоне (может быть и
# каскад/эхо), но это отдельная, непроверенная гипотеза, которую стоит
# явно видеть в логе рядом с cascade-логами, а не откапывать заново на
# роботе. НЕ используется для автоусиления/AGC — только для лога.
_QUIET_SIGNAL_HINT_DBFS = -50.0


class _NodeSTTAdapter:
    """``STTProvider``-адаптер поверх метода ноды (issue #2365 Phase 2).

    Раньше адаптеры были локальными классами внутри
    ``_recognize_with_fallback`` — с тремя провайдерами и кэшем «мёртвых»
    это перестало читаться. Здесь тот же контракт: ``name`` для метрик,
    ``recognize`` — работа, ``prepare`` — ленивая загрузка вне таймаута
    (issue #2609, Vosk).
    """

    __slots__ = ("name", "_recognize", "_prepare")

    def __init__(self, name, recognize, prepare=None):
        self.name = name
        self._recognize = recognize
        self._prepare = prepare

    def prepare(self) -> None:
        if self._prepare is not None:
            self._prepare()

    def recognize(self, data: bytes) -> Optional[str]:
        return self._recognize(data)


def _map_grpc_error(exc: "grpc.RpcError", timeout_s: float) -> BaseException:
    """gRPC-код Yandex → типизированная ошибка ``stt_fallback``.

    Нужно кэшу «мёртвых»: по строке ошибки нельзя отличить «кончились
    деньги» (RESOURCE_EXHAUSTED — лежит надолго) от «моргнула сеть»
    (UNAVAILABLE — лежит секунды).
    """
    code = exc.code()
    if code == grpc.StatusCode.DEADLINE_EXCEEDED:
        return STTTimeoutError(f"Yandex STT deadline exceeded ({timeout_s}s)")
    if code in (grpc.StatusCode.UNAUTHENTICATED, grpc.StatusCode.PERMISSION_DENIED):
        return STTAuthError(f"Yandex STT auth failure: {code} {exc.details()}")
    if code == grpc.StatusCode.RESOURCE_EXHAUSTED:
        return STTQuotaError(f"Yandex STT quota exhausted: {code} {exc.details()}")
    return exc

# Issue #2158, ADR-0076 — сбор STT-семплов с wake-сегментов шлема для
# эмпирического пополнения wake-листа ТАРС. Kill-switch через переменную
# ``ROBBOX_STT_COLLECT=1``; по умолчанию модуль no-op (см. ADR §2.2).
# Pure-Python, тестируется отдельно в test_tars_sample_logger.py.
# Делаем ленивый lookup функции на каждый вызов: в юнит-тестах среда
# может подменить переменную окружения, и тогда первый же вызов увидит
# обновлённое значение. Никаких «импорт как имя» — это лечит проблему
# Pyright с re-export type-mismatch в fallback-пути ``ImportError``.
def _maybe_emit_tars_sample(
    *,
    raw_text,
    has_operator_wake,
    duration_s,
    attempts,
    operator_wake_words,
):
    """Site-channel: пишет wake-сегмент шлема в JSONL при ROBBOX_STT_COLLECT=1.

    Ничего не делает, если модуль сборщика недоступен или env не выставлен.
    Не бросает — файл/диск не должны ронять STT-ноду.
    """
    import os as _os

    if _os.environ.get("ROBBOX_STT_COLLECT") != "1":
        return False
    try:
        from rob_box_voice.core import tars_sample_logger as _tsl
    except ImportError:
        return False
    return _tsl.append_sample(
        raw_text=raw_text,
        has_operator_wake=has_operator_wake,
        duration_s=duration_s,
        attempts=_tsl.build_attempts_snapshot(attempts),
        operator_wake_words=operator_wake_words,
    )


# Issue #1160 — Prometheus metrics (этап 1 observability).
# ``prometheus_client`` — optional dep; если её нет, всё превращается в
# no-op и старт сервера тихо возвращает ``False``.
# Issue #1234 — OpenTelemetry traces (этап 2): init_tracing в __init__,
# ``start_span`` — span ``stt.recognize`` вокруг распознавания.
from rob_box_voice.observability import (
    init_tracing,
    is_metrics_enabled,
    record_stt_recognize,
    start_metrics_server,
    start_span,
)

# ADR-0101 §3.3.5 / Issue #2536 / PR-E: «unclear cooldown» живёт в
# едином OccasionGate (см. core/occasion.py). Импортируем типы на
# уровне модуля: ``OccasionGate`` создаётся в ``__init__`` (default, если
# kwarg не передан), а ``Occasion`` / ``VerdictKind`` нужны в
# ``_maybe_speak_unclear`` напрямую.
from rob_box_voice.core.occasion import (
    Occasion,
    OccasionGate,
    VerdictKind,
)
from rob_box_voice.core.utterance_id import compute_utterance_id
from rob_box_voice.core.yandex_stt_segments import YandexSegmentCollector

# #1990 (оператор-agent 05) — источники аудио для wake-роутера (_process_audio).
# Namespace вейк-слов привязан к источнику, а не только к тексту (целевая §7.1).
_SRC_RESPEAKER = "respeaker"  # /audio/speech_audio → /voice/stt/result (личность)
_SRC_PTT = "ptt"  # /audio/quest_in (левый грип) → /avatar/ptt/result (пайплайн грипа)
_SRC_WAKE = "wake"  # /audio/quest_wake → /avatar/stt/result (агент оператора)

try:
    from rob_box_voice.core.dialogue_text import (
        DEFAULT_OPERATOR_WAKE_WORDS,
        DEFAULT_WAKE_WORDS,
        has_wake_word,
        resolve_wake_word_namespaces,
        strip_wake_word,
    )

    _HAS_WAKE_WORD_AVAILABLE = True
except ImportError:  # pragma: no cover — защита для standalone-запуска
    _HAS_WAKE_WORD_AVAILABLE = False
    DEFAULT_WAKE_WORDS = ()
    DEFAULT_OPERATOR_WAKE_WORDS = ()

    def has_wake_word(text_lower: str, wake_words: list) -> bool:  # type: ignore[no-redef]
        if not wake_words:
            return True
        return any(w in text_lower for w in wake_words)

    def strip_wake_word(text: str, wake_words: list | None = None) -> str:  # type: ignore[no-redef]
        if not wake_words:
            return text.strip()
        text_lower = text.lower()
        for word in sorted(wake_words, key=len, reverse=True):
            if word in text_lower:
                idx = text_lower.find(word)
                return (text[:idx] + text[idx + len(word) :]).strip()
        return text.strip()

    def resolve_wake_word_namespaces(  # type: ignore[no-redef]
        path=None, personality_fallback=DEFAULT_WAKE_WORDS, operator_fallback=DEFAULT_OPERATOR_WAKE_WORDS
    ):
        return list(personality_fallback), list(operator_fallback)

try:
    from rob_box_voice.core.speak_helpers import build_ssml_payload

    _BUILD_SSML_AVAILABLE = True
except ImportError:  # pragma: no cover — защита для standalone-запуска
    _BUILD_SSML_AVAILABLE = False

    def build_ssml_payload(text: str, animation: str = "neutral") -> str:  # type: ignore[no-redef]
        # voice-vr 12 (issue #2197): единый сборщик SSML — ``Utterance``.
        # Раньше было ``f"<speak>{text}</speak>"`` без экранирования.
        from rob_box_core.utterance import Sink, Utterance

        return json.dumps(
            Utterance(
                text=text,
                sink=Sink.SPEAKERS,
                emotion=animation,
                extra={"speech_id": "stt-unclear"},
            ).to_request(),
            ensure_ascii=False,
        )

# Yandex Cloud STT API v3 (gRPC)
try:
    from yandex.cloud.ai.stt.v3 import stt_pb2, stt_service_pb2_grpc

    YANDEX_GRPC_AVAILABLE = True
except ImportError:
    YANDEX_GRPC_AVAILABLE = False
    print("⚠️  yandex-cloud-ml-sdk не установлен! Используем только Vosk.")


class STTNode(Node):
    """Нода для распознавания речи: Yandex STT gRPC v3 (primary) + Vosk (fallback)."""

    def __init__(self, *, occasion_gate: OccasionGate | None = None) -> None:
        # ADR-0101 §3.3.5 / Issue #2536 / PR-E: «unclear cooldown» идёт
        # через единый ``OccasionGate``. ``occasion_gate`` — optional kwarg:
        # если передан (dialogue_node создаёт и прокидывает в ноду),
        # ``_maybe_speak_unclear`` спрашивает у gate (OccasionGate.may_speak)
        # и фиксирует ``mark_consumed``. Если НЕ передан (legacy-тесты /
        # standalone запуск) — создаём default-инстанс ниже с cooldown'ом
        # из ``unclear_cooldown_s``.
        super().__init__("stt_node")

        # Issue #1234 — OpenTelemetry traces (этап 2). STT-нода не создаёт
        # httpx-клиентов (Yandex gRPC + Vosk offline), но нам нужен
        # корневой span ``stt.recognize``. Если opentelemetry-пакетов нет —
        # no-op (см. observability.tracing).
        init_tracing("stt_node")

        # Параметры Vosk (fallback)
        self.declare_parameter("model_path", "/models/vosk-model-small-ru-0.22")
        self.declare_parameter("sample_rate", 16000)
        # Issue #2609 — Vosk держит ~400 МБ RSS, а нужен только когда Yandex
        # не ответил. По умолчанию модель грузится при первом fallback
        # (платим ~1-2 с один раз); ``true`` возвращает загрузку на старте.
        self.declare_parameter("vosk_preload", False)

        self.model_path = self.get_parameter("model_path").value
        self.sample_rate = self.get_parameter("sample_rate").value
        self.vosk_preload = bool(self.get_parameter("vosk_preload").value)

        # Параметры Yandex STT (primary)
        self.declare_parameter("yandex_api_key", "")
        self.declare_parameter("yandex_language", "ru-RU")
        self.declare_parameter("yandex_model", "general")

        # EOU (End of Utterance) profile: fast | balanced | patient
        self.declare_parameter("eou_profile", "balanced")

        # AEC mode: 'software' (drop while TTS plays) | 'hardware' (trust XVF-3000 AEC chip)
        # 'hardware' requires audio playback through ReSpeaker (hw:1,0) for AEC reference signal.
        # With 'hardware' mode the robot can be interrupted mid-speech.
        self.declare_parameter("aec_mode", "hardware")

        # Wake words для немедленного STOP TTS (barge-in). Должны совпадать с dialogue_node!
        # Один список на весь проект — rob_box_voice.core.dialogue_text.
        # Он же фолбек strip_wake_word, и его порядок неслучаен (длинные
        # варианты первыми, иначе «роб» съедает «роб бокс»). Копий было
        # семь и они разошлись на три разных списка: здесь и в
        # dialogue_node.py лежало 13 вариантов, в четырёх YAML — 21, в
        # e2e-конфиге — те же 13. Тот самый класс ошибки, из-за которого
        # завели #1252 и заплатили #1734.
        self.declare_parameter("wake_words", list(DEFAULT_WAKE_WORDS))
        # #1990 (оператор-agent 05) — SSoT wake-слов: config/wake_words.yaml
        # (docker, монтируется в /config). Ключ wake_words остаётся фолбеком
        # для dev-env/юнит-тестов без файла; wake_words_file при наличии
        # переопределяет personality из файла (см. self.wake_words ниже).
        self.declare_parameter("wake_words_file", "")

        # Параметры fallback/retry (issue #979): единое место для таймаутов,
        # retry и правила коротких фраз. См. rob_box_voice/stt_fallback.py.
        # 5s — окно для Yandex gRPC streaming на фразах 3-4 слов. Раньше
        # gRPC deadline был ~1.3s, из-за чего валидные фразы после TTS
        # попадали на мусорный Vosk fallback.
        self.declare_parameter("yandex_timeout_s", DEFAULT_YANDEX_TIMEOUT_S)
        self.declare_parameter("yandex_max_retries", DEFAULT_YANDEX_MAX_RETRIES)
        self.declare_parameter("retry_backoff_s", 1.0)
        self.declare_parameter("min_text_chars", DEFAULT_MIN_TEXT_CHARS)

        # ===== Цепочка STT-провайдеров (issue #2365 Phase 2, ADR-0124) =====
        # Порядок = приоритет. [0] — primary, дальше фолбеки. Vosk
        # принудительно переносится в конец (см. _normalize_provider_chain):
        # он офлайновый и работает, когда не работает ничего.
        #
        # Почему ROS-параметр, а не config/stt_chain.yaml: issue #1004 и
        # #1252/#1734 — второй YAML-источник для того же значения уже
        # дважды стоил нам инцидента. Единственный источник истины —
        # declare_parameter + stt_node.yaml.
        self.declare_parameter("stt_provider_chain", list(DEFAULT_STT_PROVIDER_CHAIN))
        # MiniMax STT (issue #2365). Ключ берётся из ENV (приоритет) или
        # из параметра; без ключа провайдер просто не попадает в цепочку —
        # без шумных ошибок (ADR-0091 §5.2).
        self.declare_parameter("minimax_stt_enabled", True)
        self.declare_parameter("minimax_stt_api_key", "")
        self.declare_parameter("minimax_stt_api_key_env", "MINIMAX_API_KEY")
        self.declare_parameter("minimax_stt_base_url", "https://api.minimax.io")
        self.declare_parameter("minimax_stt_model", "asr-1.0")
        self.declare_parameter("minimax_stt_language", "ru")
        # 5с — бюджет MiniMax по ADR-0091 §2.2 (мягче Yandex, он второй
        # в очереди и не должен съедать весь бюджет фразы).
        self.declare_parameter("minimax_stt_timeout_s", 5.0)
        self.declare_parameter("minimax_stt_max_retries", 1)

        # ===== Кэш «мёртвых» провайдеров (issue #2365 Phase 2) =====
        # Те же имена и дефолты, что в tts_node (issue #1083/#1229), чтобы
        # оператор не держал в голове две разные модели одного поведения.
        self.declare_parameter("provider_dead_ttl_s", DEFAULT_DEAD_TTL_S)
        self.declare_parameter(
            "provider_dead_ttl_transient_s", DEFAULT_DEAD_TTL_TRANSIENT_S
        )
        # Issue #2767 — общий бюджет времени на цепочку одной фразы. Живой
        # инцидент 23.09: без него фраза тонула в каскаде на 9-10с (empty
        # ретраился, Yandex auth гонялся заново) — этот параметр ставит
        # верхний предел на АНОМАЛЬНЫЕ повторы, не трогая нормальный
        # однопроходный сценарий (~18с холодным стартом) и не отбирая
        # попытку у последнего провайдера в цепочке (обычно Vosk).
        # 0 или отрицательное значение отключает бюджет (legacy).
        self.declare_parameter(
            "stt_phrase_budget_s", DEFAULT_MAX_TOTAL_BUDGET_S
        )
        # Персистентность кэша: рестарт ноды (а их много — см. #2676 OOM)
        # не должен снова слать фразу в облако, про которое мы уже знаем,
        # что оно лежит. Пустая строка отключает файл.
        self.declare_parameter("provider_state_file", "/data/stt_provider_state.json")

        # Issue #1160 — Prometheus metrics endpoint. 9111 — STT-нода в voice
        # (recognize counter). 0 = отключить старт сервера.
        self.declare_parameter("metrics_port", 9111)

        # Фраза при неясном результате (issue #979 acceptance: робот должен
        # попросить повторить, а не молчать). Пустая строка отключает ответ.
        self.declare_parameter("unclear_phrase", "Не расслышал, скажи ещё раз")
        # Анти-петля: не повторять фразу чаще раза в N секунд (иначе эхо
        # собственного TTS снова триггерит VAD → бесконечный цикл).
        self.declare_parameter("unclear_cooldown_s", 5.0)

        # Grace period после окончания TTS (issue 989 Fix B): 2-3 секунды
        # игнорируем ВСЕ фразы от audio_node — робот должен «дослушать» эхо
        # собственного голоса, а не триггериться на него. Раньше было 0.3s
        # и только для коротких фраз (<0.8s), из-за чего эхо TTS длиной
        # >0.8s снова попадало в STT и замыкало петлю «не расслышал».
        self.declare_parameter("tts_grace_s", 2.5)

        # Issue #1251 — ранний «бульк» (сигнал «услышал, wake word есть»).
        # Как только Yandex partial/final содержит wake word — публикуем
        # звуковой триггер на /voice/sound/trigger, чтобы sound_node сыграл
        # короткий «бульк» ЧЕРЕЗ ~2-3с после конца фразы (а не через ~8с,
        # когда LLM+TTS закончат). Полный акцепт/ответ — как раньше.
        # wake word gate: тот же список wake_words, что у dialogue_node.
        self.declare_parameter("early_boop_enabled", True)
        self.declare_parameter("early_boop_trigger", "boop")

        self.yandex_api_key = self.get_parameter("yandex_api_key").value or os.environ.get("YANDEX_API_KEY", "")
        self.yandex_language = self.get_parameter("yandex_language").value
        self.yandex_model = self.get_parameter("yandex_model").value
        self.eou_profile = self.get_parameter("eou_profile").value
        self.aec_mode = self.get_parameter("aec_mode").value
        if self.aec_mode not in ("software", "hardware"):
            self.get_logger().warning(f"⚠️ Неизвестный aec_mode '{self.aec_mode}', используется 'software'")
            self.aec_mode = "software"
        self.wake_words: list = list(self.get_parameter("wake_words").value)
        # #1990 — разделяем namespace: personality (ReSpeaker, /voice/stt/result)
        # и operator (wake-поток шлема, /avatar/stt/result). Источник — файл
        # wake_words_file (SSoT); без файла — кодовые фолбеки (тот же список).
        _wake_file = str(self.get_parameter("wake_words_file").value or "")
        self.wake_words, self.operator_wake_words = resolve_wake_word_namespaces(
            _wake_file,
            personality_fallback=self.wake_words or list(DEFAULT_WAKE_WORDS),
            operator_fallback=DEFAULT_OPERATOR_WAKE_WORDS,
        )
        self.wake_words = list(self.wake_words)
        self.operator_wake_words = list(self.operator_wake_words)
        # #1990 — источник текущей обрабатываемой фразы (для boop/barge-гейтов).
        self._active_source: str = _SRC_RESPEAKER
        # ADR-0076: последние STT-попытки (используется side-channel-сборщиком
        # wake-семплов; см. _route_wake_result / _log_wake_rejection).
        self._last_attempts: list = []
        # Issue #1734 — barge_in_policy НЕ читаем как свой параметр (это
        # был бы второй YAML-источник для того же значения — ровно класс
        # ошибки, который уже случился с wake_words выше, issue #1252, и
        # который и породил #1734: stt_node не знал про
        # dialogue_node.barge_in_policy=classify). Единственный источник
        # истины — dialogue_node, публикующий latched-топик
        # /voice/dialogue/barge_in_policy (см. create_subscription ниже и
        # barge_in_policy_callback). "replace" — fail-safe дефолт ДО
        # первого сообщения (или если dialogue_node ещё не стартовал/не
        # публиковал): сохраняет поведение issue #993 (немедленный STOP
        # TTS на wake-word), а не молча его выключает.
        self._barge_in_policy: str = "replace"
        self.yandex_timeout_s: float = float(self.get_parameter("yandex_timeout_s").value)
        self.yandex_max_retries: int = int(self.get_parameter("yandex_max_retries").value)
        self.retry_backoff_s: float = float(self.get_parameter("retry_backoff_s").value)
        self.min_text_chars: int = int(self.get_parameter("min_text_chars").value)
        # ── Issue #2365 Phase 2: цепочка провайдеров + кэш «мёртвых» ──
        # Вынесено в хелпер: ADR-0021 держит __init__ в CC-бюджете 20,
        # а одних только `or`-дефолтов у цепочки набирается на полтора
        # десятка ветвей.
        self._init_provider_chain_params()
        self.unclear_phrase: str = str(self.get_parameter("unclear_phrase").value)
        self.unclear_cooldown_s: float = float(self.get_parameter("unclear_cooldown_s").value)
        self.tts_grace_s: float = float(self.get_parameter("tts_grace_s").value)
        self._last_unclear_at: float = 0.0  # монотонное время последней фразы «не расслышал»
        # ADR-0101 §3.3.5 (PR-E): единый gate для всех поводов. Если
        # ``occasion_gate`` не передан (типичный случай — ROS launch
        # передаёт только YAML-параметры, kwargs не доходят), создаём
        # gate локально с cooldown'ом из ``unclear_cooldown_s``. Внешний
        # wiring (kwarg) остаётся для юнит-тестов и случаев, когда
        # gate живёт в другом Python-процессе/классе.
        self._occasion_gate: OccasionGate | None = occasion_gate
        if self._occasion_gate is None:
            self._occasion_gate = OccasionGate(
                global_debounce_s=2.0,
                source_cooldowns={
                    "unclear_acknowledgement": self.unclear_cooldown_s,
                },
            )
        # Issue #1251 — ранний «бульк».
        self.early_boop_enabled: bool = bool(self.get_parameter("early_boop_enabled").value)
        self.early_boop_trigger: str = str(self.get_parameter("early_boop_trigger").value)
        # «Бульк» играем ОДИН раз за фразу: Yandex шлёт несколько partials,
        # каждый с тем же wake word — без флага продублировали бы звук N раз.
        self._boop_fired: bool = False
        # Момент начала обработки фразы (для телеметрии boop_latency_ms).
        self._phrase_started_at: float = 0.0

        # EOU profiles configuration
        self.eou_profiles = {
            "fast": {
                "type": stt_pb2.DefaultEouClassifier.HIGH,  # Быстрое определение конца
                "max_pause_ms": 700,  # Default Yandex value
                "description": "Быстрое определение конца фразы (для коротких команд)",
            },
            "balanced": {
                "type": stt_pb2.DefaultEouClassifier.DEFAULT,  # Консервативное определение
                "max_pause_ms": 1200,  # Текущее значение
                "description": "Сбалансированное определение (по умолчанию)",
            },
            "patient": {
                "type": stt_pb2.DefaultEouClassifier.DEFAULT,
                "max_pause_ms": 2000,  # Для длинных фраз с паузами
                "description": "Терпеливое ожидание (для медленной речи)",
            },
        }

        # Валидация профиля
        if self.eou_profile not in self.eou_profiles:
            self.get_logger().warning(f"⚠️ Неизвестный EOU profile '{self.eou_profile}', используется 'balanced'")
            self.eou_profile = "balanced"

        profile = self.eou_profiles[self.eou_profile]
        self.get_logger().info(
            f"📊 EOU Profile: {self.eou_profile} - {profile['description']} " f"(pause: {profile['max_pause_ms']}ms)"
        )

        # Yandex gRPC клиент
        self.yandex_channel = None
        self.yandex_stub = None

        # QoS для аудио потока
        audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10
        )

        # Subscriber - слушаем только speech_audio (уже готовые фразы)
        self.audio_sub = self.create_subscription(
            AudioData, "/audio/speech_audio", self.speech_audio_callback, audio_qos
        )
        # #1990 (оператор-agent 05): /audio/quest_in — ЛЕВЫЙ ГРИП (PTT
        # robot-voice). quest-сервер публикует сюда готовую фразу (буфер за
        # время PTT); результат — /avatar/ptt/result → пайплайн грипа (§7.5).
        # Отдельного топика /voice/stt/quest больше НЕТ (заменён на /avatar/*).
        self.quest_audio_sub = self.create_subscription(
            AudioData, "/audio/quest_in", self.quest_audio_callback, audio_qos
        )
        # #1990: /audio/quest_wake — ВСЕГДА-ВКЛЮЧЁННЫЙ wake-поток микрофона
        # шлема (шаг 5а). Дремлющая подписка: до появления публикатора
        # (quest_node, шаг 5а) безвредна, в ROS пустой топик не создаёт.
        self.quest_wake_audio_sub = self.create_subscription(
            AudioData, "/audio/quest_wake", self.quest_wake_audio_callback, audio_qos
        )

        # Подписка на состояние TTS (чтобы не слышать себя)
        self.tts_state_sub = self.create_subscription(String, "/voice/tts/state", self.tts_state_callback, 10)

        # Issue #1734 — подписка на действующую barge_in_policy от
        # dialogue_node (единый источник истины, см. self._barge_in_policy
        # выше). TRANSIENT_LOCAL: поздний subscriber (STT стартовал позже
        # ИЛИ раньше dialogue_node) всё равно получает последний
        # опубликованный семпл — durability закрывает и «порядок старта
        # нод», и «потерю сообщения» (пока жив publisher на стороне
        # dialogue_node).
        barge_in_policy_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.barge_in_policy_sub = self.create_subscription(
            String,
            "/voice/dialogue/barge_in_policy",
            self.barge_in_policy_callback,
            barge_in_policy_qos,
        )

        # Publishers
        self.result_pub = self.create_publisher(String, "/voice/stt/result", 10)
        # #1990 (оператор-agent 05) — wake-роутер. /voice/stt/quest удалён:
        # маршрут оператора уезжает в /avatar/* (см. docstring модуля).
        # /avatar/ptt/result — пайплайн грипа (супервизор, #1989), plain text;
        # /avatar/stt/result — агент оператора (супервизор, #1988), JSON v1
        # (source/client_id/text/ts_ms, как /avatar/command).
        self.avatar_ptt_result_pub = self.create_publisher(String, "/avatar/ptt/result", 10)
        self.avatar_stt_result_pub = self.create_publisher(String, "/avatar/stt/result", 10)
        self.state_pub = self.create_publisher(String, "/voice/stt/state", 10)
        self.tts_control_pub = self.create_publisher(String, "/voice/tts/control", 10)  # Для прерывания TTS
        # Issue #1077 — speaker_analysis (speaker_tag) от Yandex SpeechKit v3.
        # Публикуем отдельным топиком (String, JSON: {"speaker_tag", "text"}),
        # чтобы НЕ ломать контракт /voice/stt/result (plain text — его читают
        # telegram/perception/transport). dialogue_node подписывается и создаёт
        # профиль спикера (scope=speaker:<tag>).
        self.speaker_pub = self.create_publisher(String, "/voice/stt/speaker", 10)
        # Issue #2829 (ADR-0131) — utterance_id для ЭТОЙ фразы, publish'ится
        # ПЕРЕД /voice/stt/result (тот же порядок гарантий, что и у
        # speaker_pub выше: dialogue_node._on_stt читает pending id,
        # выставленный этим сообщением, до того как читает сам текст).
        # Отдельный топик, а не поле в /voice/stt/result — контракт
        # (plain text) там не трогаем, его читают
        # telegram/perception/GUI/harness-бенчи (см. stt_node.py:585).
        self.utterance_pub = self.create_publisher(String, "/voice/stt/utterance", 10)
        # Прямой запрос TTS для фразы «не расслышал» (issue #979). tts_node
        # слушает /voice/tts/request тем же JSON-SSML контрактом, что и
        # /voice/dialogue/response — build_ssml_payload даёт ровно это.
        self.tts_request_pub = self.create_publisher(String, "/voice/tts/request", 10)
        # Issue #1251 — ранний «бульк»: публикуем триггер на /voice/sound/trigger
        # (sound_node воспроизведёт короткий звук «услышал, wake word есть»).
        self.boop_pub = self.create_publisher(String, "/voice/sound/trigger", 10)

        # Vosk модель и распознаватель
        self.model: Optional[Model] = None
        self.recognizer: Optional[KaldiRecognizer] = None
        # Модель на диске прошла проверку, но в память может быть ещё не
        # загружена (issue #2609) — см. _ensure_vosk_loaded.
        self._vosk_available = False
        self._vosk_load_lock = threading.Lock()

        # Состояние
        self.is_robot_speaking = False  # Флаг: робот говорит (только для aec_mode=software)
        self._tts_ended_at: float = 0.0  # Время окончания TTS (для grace period)
        # Issue #1077 — speaker_tag последней распознанной фразы (Yandex
        # speaker_analysis). None для Vosk fallback. Сбрасывается на старте
        # каждой фразы в speech_audio_callback.
        self._last_speaker_tag: Optional[str] = None

        # Issue #1160 — Prometheus metrics server (этап 1).
        # Порт 9111 — стандартный для stt_node (см. observability/__init__.py).
        self._metrics_port: int = int(
            self.get_parameter("metrics_port").value or 0
        )
        if self._metrics_port > 0 and is_metrics_enabled():
            started = start_metrics_server(self._metrics_port)
            if started:
                self.get_logger().info(
                    f"📊 STT metrics server listening on :{self._metrics_port}/metrics"
                )
            else:
                self.get_logger().warning(
                    f"📊 STT metrics port {self._metrics_port} not bound "
                    "(busy or prometheus_client missing)"
                )

        # Инициализация
        self.get_logger().info(
            f"STTNode инициализирован | aec_mode={self.aec_mode} "
            f'({"software echo suppression" if self.aec_mode == "software" else "hardware AEC (XVF-3000), simultaneous RX/TX enabled"})'
            f" | wake_words={self.wake_words}"
        )
        self.initialize_yandex()
        self.initialize_vosk()
        self._load_persisted_provider_state()
        self._log_provider_state("startup")

    def initialize_yandex(self):
        """Инициализация Yandex STT gRPC v3."""
        if not YANDEX_GRPC_AVAILABLE:
            self.get_logger().warn("⚠️  Yandex Cloud ML SDK недоступен, используем только Vosk")
            return

        if not self.yandex_api_key:
            self.get_logger().warn("⚠️  YANDEX_API_KEY не задан, используем только Vosk")
            return

        try:
            self.get_logger().info("🔌 Подключение к Yandex STT gRPC v3...")
            self.yandex_channel = grpc.secure_channel("stt.api.cloud.yandex.net:443", grpc.ssl_channel_credentials())
            self.yandex_stub = stt_service_pb2_grpc.RecognizerStub(self.yandex_channel)
            self.get_logger().info(f"✅ Yandex STT gRPC v3 инициализирован (язык: {self.yandex_language})")
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка инициализации Yandex STT: {e}")
            self.yandex_stub = None

    def _on_yandex_stream_error(self, exc, context: str) -> BaseException:
        """gRPC-ошибка стрима Yandex STT → лог + типизированное исключение.

        Issue #2924: 23.09 с 23:22 по 23:39 каждый вызов стоял до дедлайна
        (5 с), не прислав НИ ОДНОГО partial (на фразах с «Робот» не сработал
        ранний «бульк», который в норме срабатывает через 0.5 с), потом сам
        ожил без рестарта ноды. Канал — один долгоживущий, без keepalive.
        Гипотеза (не доказана): зависшее TCP-соединение канала, которое ядро
        рвёт только через ~15 мин ретрансмиссий. Поэтому после
        DEADLINE_EXCEEDED канал пересоздаём: следующая фраза пойдёт по
        свежему соединению, а не по тому, что висит. Цена — один TLS-хендшейк.
        """
        self.get_logger().warning(
            f"⚠️ [issue 1477] stream error: {exc.code()} {exc.details()} [#2924 {context}]"
        )
        if exc.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
            self._reset_yandex_channel()
        return _map_grpc_error(exc, self.yandex_timeout_s)

    def _reset_yandex_channel(self) -> None:
        """Закрыть канал Yandex STT и открыть новый (issue #2924)."""
        old = getattr(self, "yandex_channel", None)
        if old is not None:
            try:
                old.close()
            except Exception as e:  # noqa: BLE001 — закрытие не должно ронять STT
                self.get_logger().warning(f"⚠️ [#2924] close старого канала Yandex STT: {e}")
        self.get_logger().warning("🔌 [#2924] DEADLINE_EXCEEDED → пересоздаю канал Yandex STT")
        self.initialize_yandex()

    def initialize_vosk(self):
        """Load Vosk model (fallback provider).

        Architect decision G-VOSK: the model is bundled in the voice_base
        image at /models/vosk-model-small-ru-0.22 (see
        docker/vision/voice_base/Dockerfile). We fail-fast with a helpful
        error message (instead of the opaque 'Folder does not contain model
        files' from the Vosk C++ binding) when it isn't there, pointing the
        operator at the docs.

        Set the env var ROS_VOSK_DISABLE=1 to skip Vosk entirely —
        stt_node will then use only the Yandex gRPC provider.

        Issue #2609: here we only check that the model is on disk. The model
        itself is loaded on the first fallback (``_ensure_vosk_loaded``)
        unless ``vosk_preload`` is true.
        """
        # Operator opt-out — useful when the model isn't available and we
        # explicitly want a Yandex-only deploy.
        if os.environ.get("ROS_VOSK_DISABLE", "").lower() in ("1", "true", "yes"):
            self.get_logger().warn(
                "⚠️  ROS_VOSK_DISABLE is set — skipping Vosk init. " "STT will rely on Yandex gRPC only."
            )
            self.publish_state("error")
            return

        # Pre-flight check: surface a clear error if the model dir is missing
        # rather than letting the Vosk C++ binding emit the cryptic "Folder
        # does not contain model files" stderr (task card G-VOSK).
        if not os.path.isdir(self.model_path):
            self.get_logger().error(
                f'❌ Vosk model not found at "{self.model_path}".\n'
                f"   The model should be bundled in the voice_base image "
                f"(see docker/vision/voice_base/Dockerfile).\n"
                f"   To install on a bare-metal host, see "
                f"docs/development/VOICE_ASSISTANT_DOCKER.md or run "
                f"src/rob_box_voice/scripts/quick_start_stt.sh.\n"
                f"   To skip Vosk entirely and use Yandex gRPC only, "
                f"set ROS_VOSK_DISABLE=1 in the container env."
            )
            self.publish_state("error")
            return

        self._vosk_available = True
        if not self.vosk_preload:
            self.get_logger().info(
                "🪶 Vosk загрузится при первом fallback " "(issue #2609: vosk_preload=false)"
            )
            self.publish_state("ready")
            return
        if self._ensure_vosk_loaded():
            self.publish_state("ready")

    def _ensure_vosk_loaded(self) -> bool:
        """Загрузить Vosk в память, если ещё не загружен (issue #2609)."""
        if self.recognizer is not None:
            return True
        if not self._vosk_available:
            return False
        with self._vosk_load_lock:
            if self.recognizer is not None:
                return True
            return self._load_vosk_model()

    def _load_vosk_model(self) -> bool:
        try:
            t0 = time.monotonic()
            self.get_logger().info(f"Загрузка Vosk модели из {self.model_path}...")
            self.model = Model(self.model_path)
            recognizer = KaldiRecognizer(self.model, self.sample_rate)
            recognizer.SetWords(True)  # Получать разметку по словам
            self.recognizer = recognizer
            self.get_logger().info(
                f"✅ Vosk модель загружена (fallback, {time.monotonic() - t0:.1f} с)"
            )
            return True
        except Exception as e:
            # Defensive: even after the isdir() check above, the model files
            # inside the directory could still be missing/corrupt (e.g. a
            # half-extracted zip). Surface a clear message + the exception.
            self.get_logger().error(
                f"❌ Ошибка загрузки Vosk из {self.model_path}: {e}\n"
                f"   Path exists but the model files are missing or invalid.\n"
                f"   See docker/vision/voice_base/Dockerfile — the model "
                f"should be bundled at build time."
            )
            self._vosk_available = False
            self.publish_state("error")
            return False

    def tts_state_callback(self, msg: String):
        """Отслеживание состояния TTS.

        software mode: выключаем STT пока робот говорит.
        hardware mode: только логируем; AEC на чипе XVF-3000 фильтрует эхо.
        """
        import time

        if msg.data in ["synthesizing", "playing"]:
            if not self.is_robot_speaking:
                if self.aec_mode == "software":
                    self.get_logger().info("🔇 [software AEC] Робот говорит - распознавание отключено")
                else:
                    self.get_logger().debug("🎤 [hardware AEC] Робот говорит - XVF-3000 фильтрует эхо")
                self.is_robot_speaking = True
        # ``stopped`` — STOP/barge-in (``tts_node._handle_stop_command``),
        # ``tts_silero_warming`` — чанк пропущен и озвучен не будет.
        # Оба означают «робот молчит». Без них ``is_robot_speaking``
        # залипал в ``True``, и ниже (hardware-AEC, конфиг робота) фразы
        # короче 0.8 с молча отбрасывались — «робот», «стоп», «да».
        elif msg.data in ["ready", "idle", "stopped", "tts_silero_warming"]:
            if self.is_robot_speaking:
                self._tts_ended_at = time.monotonic()
                if self.aec_mode == "software":
                    self.get_logger().info("🎙️ [software AEC] Робот замолчал - распознавание включено")
                else:
                    self.get_logger().debug("🎙️ [hardware AEC] Робот замолчал")
                self.is_robot_speaking = False

    def speech_audio_callback(self, msg: AudioData) -> None:
        """Фраза от audio_node (ReSpeaker) → /voice/stt/result (личность)."""
        self._process_audio(msg, source=_SRC_RESPEAKER)

    def quest_audio_callback(self, msg: AudioData) -> None:
        """Фраза с микрофона Quest (левый грип, PTT robot-voice).

        Маршрут пайплайна грипа (§7.5, #1990): распознаём и публикуем как есть
        (вейк не нужен — грип сам является вейком); результат — в
        ``/avatar/ptt/result`` (его читает avatar_supervisor, шаг 4б/#1989).
        """
        self._process_audio(msg, source=_SRC_PTT)

    def quest_wake_audio_callback(self, msg: AudioData) -> None:
        """Всегда-включённый wake-поток микрофона шлема (шаг 5а, #1990).

        Вейк-поиск: распознаём VAD-сегмент (клиент гейтит локальным VAD) и
        публикуем в ``/avatar/stt/result`` ТОЛЬКО если в тексте есть operator-
        вейк («ТАРС», namespace operator из wake_words.yaml) — сам вейк при
        этом вырезается. Фоновая речь без вейка молча отбрасывается.
        """
        self._process_audio(msg, source=_SRC_WAKE)

    def _process_audio(self, msg: AudioData, source: str) -> None:
        """
        Общий пайплайн распознавания (Yandex STT → Vosk fallback) + маршрутизация.

        ``source`` — откуда пришёл аудио (см. ``_SRC_*``). Единственная точка
        маршрутизации речи (целевая §7, #1990). От источника зависит:

          * ``respeaker`` — AEC/grace-фильтры, speaker-профиль, ранний «бульк»,
            wake-word barge-in; результат → ``/voice/stt/result`` (личность).
          * ``ptt`` — фильтры/бульк/barge не нужны (оператор зажал грип, barge
            уже сделан quest-сервером); результат → ``/avatar/ptt/result``.
          * ``wake`` — фильтры не нужны; публикуем в ``/avatar/stt/result``
            только при operator-вейке («ТАРС»), иначе молча отбрасываем.
        """
        import time

        # Конвертируем список в bytes
        audio_bytes = bytes(msg.data)
        duration = len(audio_bytes) / (self.sample_rate * 2)  # 16-bit = 2 bytes

        if source != _SRC_RESPEAKER:
            # Источник шлема: AEC/grace не нужны (см. docstring выше).
            pass
        elif self.aec_mode == "software":
            # Программная подавление эха: дропаем всё пока робот говорит
            if self.is_robot_speaking:
                self.get_logger().info(f"🔇 [software AEC] Игнор фразы {duration:.2f}с: робот говорит")
                return
        else:
            # Аппаратное AEC: XVF-3000 фильтрует эхо в чипе, но не справляется
            # с громкой музыкой/собственным TTS (issue 989). Grace period после
            # окончания TTS: игнорируем ВСЕ фразы (не только короткие <0.8s),
            # чтобы эхо собственного голоса не замкнуло петлю «не расслышал».
            time_since_tts = time.monotonic() - self._tts_ended_at
            if time_since_tts < self.tts_grace_s:
                self.get_logger().info(
                    f"🔇 [issue 989] Игнор фразы {duration:.2f}с — grace "
                    f"{time_since_tts:.2f}с/{self.tts_grace_s}с после TTS "
                    "(эхо собственного голоса)"
                )
                return
            if self.is_robot_speaking:
                if duration < 0.8:
                    self.get_logger().info(
                        f"🔇 [hardware AEC] Игнор короткой фразы {duration:.2f}с — TTS активен"
                    )
                    return
                self.get_logger().info(
                    f"🎤 [hardware AEC] Фраза {duration:.2f}с во время TTS — "
                    "обрабатываем (возможно прерывание)"
                )

        self.get_logger().info(f"🎤 Получена фраза: {duration:.2f}с ({len(audio_bytes)} bytes)")
        # Issue 1076 (телеметрия): фиксируем момент получения фразы, чтобы
        # замерить честный «фраза → ПРИНЯТО». Полный «замолчал → акцепт» =
        # silence_to_phrase_s (audio_node, включает speech_continuation)
        # + phrase_to_accept_ms (здесь).
        _phrase_received_at = time.monotonic()
        # #1990 — источник активной фразы (boop/barge гейтятся по respeaker).
        self._active_source = source
        # wake-поток — фоновый слушатель: стейт /voice/stt/state не дёргаем
        # (иначе спамили бы recognizing/ready на каждый VAD-сегмент).
        if source != _SRC_WAKE:
            self.publish_state("recognizing")

        # Issue #1077 — сбрасываем speaker_tag на старте каждой фразы.
        # _recognize_yandex заполнит его из speaker_analysis; Vosk fallback
        # (без speaker-анализа) оставит None — профиль не создаётся.
        self._last_speaker_tag = None
        # Issue #1251 — сбрасываем флаг «булька» и фиксируем старт фразы
        # для телеметрии задержки (boop_latency_ms).
        self._boop_fired = False
        self._phrase_started_at = time.monotonic()

        # Идём через единый select_recognition: primary=Yandex, fallback=Vosk,
        # 1 retry на primary, soft-timeout yandex_timeout_s. Возвращает
        # (text, attempts) — text может быть None при итоговом отклонении.
        # Issue #1234 — OpenTelemetry span ``stt.recognize`` (этап 2):
        # обёртка всего распознавания (включая retry/fallback). Атрибуты
        # provider/success/duration проставляем после. no-op без OTel.
        _stt_trace_start = time.monotonic()
        if _STT_FALLBACK_AVAILABLE:
            with start_span("stt.recognize") as _stt_span:
                text, attempts = self._recognize_with_fallback(audio_bytes)
                # ADR-0076: сохраняем попытки для side-channel-сборщика
                # wake-сегментов (вызывается ниже в _route_wake_result /
                # _log_wake_rejection). Никуда больше не уходит.
                self._last_attempts = list(attempts)
                # Issue #979 — final_text передаём только если фраза реально
                # ПРИНЯТА: иначе rejected(short) («не» от Vosk) залогируется как
                # «accepted» — ложь, вводит в заблуждение при отладке.
                _accepted = bool(text) and not is_short_phrase(text, min_chars=self.min_text_chars)
                log_attempts(self.get_logger(), attempts, final_text=text if _accepted else None)
                # Финальный провайдер — последняя попытка (или "yandex" по
                # умолчанию; при пустом списке попыток — "unknown").
                _stt_provider = (
                    attempts[-1].provider if attempts else "unknown"
                )
                _stt_span.set_attribute("provider", _stt_provider)
                _stt_span.set_attribute("success", _accepted)
                _stt_span.set_attribute(
                    "duration_s", time.monotonic() - _stt_trace_start
                )
            # Issue #1160 — Prometheus metrics: учитываем каждую попытку
            # (включая retry и fallback на Vosk). ``result``:
            # success = финальный непустой текст; empty = итоговый отказ.
            # Для каждой попытки отдельно используем свой reason
            # (timeout/empty/error/low_confidence → "empty" в нашем
            # counter, потому что метрика бинарная: «распознал / не
            # распознал»). Суммарный итог (text непустой → success) — для
            # последней попытки.
            if is_metrics_enabled():
                for attempt in attempts:
                    if attempt.reason == "ok":
                        record_stt_recognize(
                            attempt.provider, success=True,
                            duration_s=attempt.latency_ms / 1000.0,
                        )
                    else:
                        # timeout / empty / error / low_confidence →
                        # попытка провалилась (на этой попытке текста нет).
                        record_stt_recognize(
                            attempt.provider, success=False,
                            duration_s=attempt.latency_ms / 1000.0,
                        )
        else:
            # Защитный путь — модуль stt_fallback не импортировался (не должно
            # случиться в нашем пакете, но пусть будет legacy-fallback).
            text = self._recognize_legacy(audio_bytes)
            attempts = []
            # ADR-0076: фиксируем пустые попытки для wake-сборщика.
            self._last_attempts = []

        # Публикация результата по источнику (wake-роутер, #1990)
        if text and not is_short_phrase(text, min_chars=self.min_text_chars):
            # Issue 1076 (телеметрия): честный «фраза → ПРИНЯТО» (STT-часть).
            _accept_ms = int((time.monotonic() - _phrase_received_at) * 1000)
            self.get_logger().info(
                f"📊 [telemetry] phrase_to_accept_ms={_accept_ms} "
                f"(text={text!r})"
            )
            self.get_logger().info(f"✅ ПРИНЯТО ({source}): {text}")
            if source == _SRC_RESPEAKER:
                # Issue #2829 (ADR-0131) — utterance_id фразы. speaker_id_node
                # считает его тем же способом от тех же PCM-байт
                # /audio/speech_audio — id совпадёт без координации нод.
                # Issue #2862 — id едет ВМЕСТЕ с текстом: dialogue_node
                # связывает его с /voice/stt/result по тексту, порядок
                # доставки двух топиков не важен.
                self._publish_utterance_id(audio_bytes, text)
                # Issue #1077 — speaker публикуем ПЕРЕД результатом: dialogue_node
                # хранит tag по тексту и забирает его в _on_stt. Если бы speaker
                # шёл после result, гонка топиков могла бы потерять корреляцию.
                # Для источника шлема speaker-профиль не нужен (это оператор).
                self._publish_speaker(text, duration)
                self.publish_result(text)
                self.publish_state("ready")
            elif source == _SRC_PTT:
                # PTT-поток: вейк не нужен (грип сам вейк), текст «как есть»
                # → пайплайн грипа в супервизоре (/avatar/ptt/result, #1989).
                self._publish_ptt_result(text)
                self.publish_state("ready")
            else:  # wake
                self._route_wake_result(text)
            self._active_source = _SRC_RESPEAKER
        else:
            self._handle_rejected_text(text, source, duration)

    def _handle_rejected_text(self, text: str, source: str, duration: float) -> None:
        """Ветка «текста нет / текст слишком короткий»: лог + решение молчать.

        Issue 989 Fix A: различаем rejected(empty) и rejected(short).
        rejected(empty) — это почти наверняка эхо собственной музыки/голоса
        робота или шум: НЕ говорим «не расслышал», молчим. Иначе робот
        говорит фразу → её эхо снова ловится → снова empty → бесконечный цикл.
        rejected(short) — Vosk/Yandex вернули что-то (например «не»/«пути»),
        т.е. был реальный речевой ввод, но слишком короткий — можно переспросить.
        wake-поток: фон/ложное VAD-срабатывание — молчим, робот не должен
        «переспрашивать» на каждый сегмент без operator-вейка.

        Вынесено из ``_process_audio`` (issue #2135): разведение причин для
        wake добавляло ветку в метод, который и так на потолке CC-бюджета
        (ADR-0021 R1, ``scripts/lint/cc_budget_baseline.json``).
        """
        if text:
            self.get_logger().warning(f'❌ ОТКЛОНЕНО (короткое, <{self.min_text_chars} chars): "{text}"')
        else:
            self.get_logger().warning("❌ ОТКЛОНЕНО (пустое)")
        if source == _SRC_WAKE:
            self._log_wake_rejection(text, duration)
        elif text:
            self._maybe_speak_unclear()
        else:
            self.get_logger().info("🔇 [issue 989] Пустой STT (эхо/музыка) — молчу, без «не расслышал»")
        if source != _SRC_WAKE:
            self.publish_state("ready")
        self._active_source = _SRC_RESPEAKER

    def _log_wake_rejection(self, text: str, duration: float) -> None:
        """Wake-сегмент отклонён: сказать в лог ПОЧЕМУ (issue #2135).

        Раньше обе причины печатались одной строкой «🔇 [wake] Сегмент без
        operator-вейка/отклонён — молчу», хотя вейк в этой ветке вообще не
        искали: сюда попадает только «STT ничего не вернул» или «вернул
        слишком короткое». Именно это маскировало дефект #2135 — в логах
        робота поток 20мс-кадров выглядел как «оператор говорит, но не
        вейк-слово», а на самом деле распознавание запускалось на 0.02 с.
        Длительность сегмента в строке — чтобы такой перекос было видно
        сразу, без сопоставления с соседним «🎤 Получена фраза».
        """
        # ADR-0076: side-channel. Пусто/короткое на wake — это ровно тот
        # случай, когда STT «не расслышал» или «съел первую букву». Пишем
        # его как семпл (с raw_text=None или коротким text), чтобы в сводке
        # увидеть «а сколько вообще сегментов пропадает молча».
        _maybe_emit_tars_sample(
            raw_text=text,
            has_operator_wake=False,
            duration_s=duration,
            attempts=getattr(self, "_last_attempts", None),
            operator_wake_words=self.operator_wake_words,
        )
        if text:
            self.get_logger().info(
                f'🔇 [wake] Сегмент {duration:.2f}с: STT вернул короткое '
                f'"{text}" (<{self.min_text_chars} chars) — вейк не искали, молчу'
            )
        else:
            self.get_logger().info(
                f"🔇 [wake] Сегмент {duration:.2f}с: STT вернул пусто — "
                "вейк не искали, молчу"
            )

    def _publish_ptt_result(self, text: str) -> None:
        """Опубликовать распознанную PTT-фразу в ``/avatar/ptt/result``.

        Plain text (канонический payload, супервизор._extract_grip_ptt_text
        принимает и голый String, и ``{"text": ...}``).
        """
        msg = String()
        msg.data = text
        self.avatar_ptt_result_pub.publish(msg)
        self.get_logger().info(f"📤 [ptt] /avatar/ptt/result: {text}")

    def _route_wake_result(self, text: str) -> None:
        """Wake-ветка: оператор-вейк («ТАРС») → /avatar/stt/result, иначе drop.

        Проверяем operator-namespace (не personality — «роббокс» из микрофона
        шлема не адресует агента). Вейк вырезаем из текста ПЕРЕД публикацией
        (как ``remove_wake_word`` у личности). Payload — JSON v1 как
        /avatar/command (source/client_id/text/ts_ms): обработчик супервизора
        на /avatar/stt/result — тот же, что на /avatar/command (04a §3.5).
        client_id сессии шлема stt_node не знает — привяжет шаг 5а/quest-слой.
        """
        # ADR-0076: site-channel для эмпирического сбора STT-семплов
        # шлема. Никак не меняет маршрутизацию ниже — это side-channel.
        text_lower = text.lower()
        if not has_wake_word(text_lower, self.operator_wake_words):
            # Длительность сегмента: пишем «от последней фразы до текущего
            # момента». self._phrase_started_at обнуляется между сегментами
            # в ``_process_audio`` (инициализируется при старте каждой
            # фразы). Если ноль (тест) — 0.0, что ОК для сортировки.
            _started = getattr(self, "_phrase_started_at", 0)
            _duration = (time.monotonic() - _started) if _started > 0 else 0.0
            _maybe_emit_tars_sample(
                raw_text=text,
                has_operator_wake=False,
                duration_s=_duration,
                attempts=getattr(self, "_last_attempts", None),
                operator_wake_words=self.operator_wake_words,
            )
            self.get_logger().info(
                f"🔇 [wake] Нет operator-вейка в {text[:40]!r} — не маршрутизирую"
            )
            return
        # Вейк найден — это НЕ семпл «не сработало», а нормальная публикация.
        # В ADR §2.2 нет требования писать и успешные wake: для текущей задачи
        # интересны «арс расскажи анекдот», а не «тарс расскажи анекдот».
        stripped = strip_wake_word(text, self.operator_wake_words)
        payload = json.dumps(
            {
                "source": "quest",
                "client_id": "",
                "text": stripped,
                "ts_ms": int(time.time() * 1000),
            },
            ensure_ascii=False,
        )
        msg = String()
        msg.data = payload
        self.avatar_stt_result_pub.publish(msg)
        self.get_logger().info(
            f"🎯 [wake] Operator wake → /avatar/stt/result: {stripped!r}"
            f" (was {text[:40]!r})"
        )

    def _maybe_speak_unclear(self) -> None:
        """Проговорить «Не расслышал, скажи ещё раз» при неясном результате.

        Ограничение по времени (``unclear_cooldown_s``) защищает от петли:
        робот говорит фразу → микрофон слышит эхо → VAD триггерит новую
        фразу → снова неясный результат → снова «не расслышал»...

        ADR-0101 §3.3.5 / Issue #2536 (PR-E): решение принимает
        ``OccasionGate`` — ``Occasion(kind="unclear_acknowledgement")``
        → ``may_speak`` → ``ALLOW``/``DEFER``. На ``ALLOW`` публикуем
        SSML-payload, вызываем ``mark_consumed`` и обновляем
        ``self._last_unclear_at`` (метрика, уйдёт в PR-F когда выпилим
        прямое чтение ``unclear_cooldown_s``).
        """
        if not self.unclear_phrase:
            return

        assert self._occasion_gate is not None  # создаётся в __init__

        occasion = Occasion(
            kind="unclear_acknowledgement",
            payload={"text": self.unclear_phrase},
        )
        verdict = self._occasion_gate.may_speak(occasion)
        if verdict.kind is not VerdictKind.ALLOW:
            self.get_logger().info(
                f"unclear ack deferred: {verdict.reason}"
            )
            return

        # ALLOW → публикуем и фиксируем факт отправки.
        now = time.monotonic()
        self._last_unclear_at = now  # метрика (PR-F выпилит)
        self._occasion_gate.mark_consumed(occasion)
        payload = build_ssml_payload(self.unclear_phrase, animation="confused")
        msg = String()
        msg.data = payload
        self.tts_request_pub.publish(msg)
        self.get_logger().info(
            f"🗣️ Неясный результат → говорю: {self.unclear_phrase!r}"
        )

    # ── Issue #2365 Phase 2: цепочка minimax → yandex → vosk (ADR-0124) ────

    def _resolve_phrase_budget_s(self) -> Optional[float]:
        """Общий бюджет фразы (issue #2767 п.3) или ``None`` (легаси).

        Вынесено из :meth:`_init_provider_chain_params` отдельным методом
        (cc_budget, ADR-0021 R1) — инлайн-условие толкало метод за лимит
        CC=15. 0/отрицательное значение параметра отключает бюджет
        (``None`` передаётся в ``select_recognition`` как легаси-режим).
        """
        raw = self.get_parameter("stt_phrase_budget_s").value
        budget = float(raw) if raw is not None else DEFAULT_MAX_TOTAL_BUDGET_S
        return budget if budget > 0 else None

    def _init_provider_chain_params(self) -> None:
        """Прочитать параметры цепочки и собрать кэш «мёртвых».

        Отдельный метод, а не кусок ``__init__``: ADR-0021 держит
        ``__init__`` в CC-бюджете 20, а дефолты цепочки и MiniMax сами по
        себе дают полтора десятка ветвей.
        """
        self.provider_chain: list = self._normalize_provider_chain(
            [str(p) for p in (self.get_parameter("stt_provider_chain").value or [])],
            logger=self.get_logger(),
        )
        self.minimax_stt_enabled: bool = bool(
            self.get_parameter("minimax_stt_enabled").value
        )
        self.minimax_stt_api_key_env: str = str(
            self.get_parameter("minimax_stt_api_key_env").value or "MINIMAX_API_KEY"
        )
        # ENV имеет приоритет над YAML: ключи приезжают из docker/vision/.env,
        # а не лежат в репозитории.
        self.minimax_stt_api_key: str = str(
            os.environ.get(self.minimax_stt_api_key_env, "")
            or self.get_parameter("minimax_stt_api_key").value
            or ""
        ).strip()
        self.minimax_stt_base_url: str = str(
            self.get_parameter("minimax_stt_base_url").value or "https://api.minimax.io"
        )
        self.minimax_stt_model: str = str(
            self.get_parameter("minimax_stt_model").value or "asr-1.0"
        )
        self.minimax_stt_language: str = str(
            self.get_parameter("minimax_stt_language").value or "ru"
        )
        self.minimax_stt_timeout_s: float = float(
            self.get_parameter("minimax_stt_timeout_s").value or 5.0
        )
        self.minimax_stt_max_retries: int = int(
            self.get_parameter("minimax_stt_max_retries").value or 0
        )
        self.provider_dead_ttl_s: float = float(
            self.get_parameter("provider_dead_ttl_s").value or DEFAULT_DEAD_TTL_S
        )
        self.provider_dead_ttl_transient_s: float = float(
            self.get_parameter("provider_dead_ttl_transient_s").value
            or DEFAULT_DEAD_TTL_TRANSIENT_S
        )
        self.stt_phrase_budget_s: Optional[float] = (
            self._resolve_phrase_budget_s()
        )
        self.provider_state_file: str = str(
            self.get_parameter("provider_state_file").value or ""
        )
        self._provider_dead_cache = (
            ProviderDeadCache(
                ttl_s=self.provider_dead_ttl_s,
                transient_ttl_s=self.provider_dead_ttl_transient_s,
            )
            if ProviderDeadCache is not None
            else None
        )
        # MiniMax-клиент создаётся лениво — при первом обращении к
        # провайдеру, а не на старте (httpx.Client держит сокеты, а
        # нода может вообще не дойти до MiniMax: Vosk-only режим).
        self._minimax_stt_provider = None
        self._minimax_stt_lock = threading.Lock()
        self._minimax_stt_initialized = False
        self._last_effective_provider: Optional[str] = None

    @staticmethod
    def _normalize_provider_chain(chain, logger=None) -> list:
        """Привести цепочку к инвариантам (аналог tts_node, issue #1083).

        Инварианты:

        * только известные провайдеры (:data:`KNOWN_STT_PROVIDERS`);
        * без дубликатов, порядок первого вхождения сохраняется;
        * ``vosk`` — всегда последний (офлайновый последний рубеж);
        * пустая/битая цепочка → :data:`DEFAULT_STT_PROVIDER_CHAIN`.

        Осознанное исключение: цепочка ровно из одного ``vosk`` —
        легитимный офлайн-режим (робот в поле без сети), её не трогаем.
        """
        deduped: list = []
        for name in chain or []:
            name = str(name).strip().lower()
            if name not in KNOWN_STT_PROVIDERS:
                if name and logger is not None:
                    logger.warning(
                        f"⚠️ stt_provider_chain: неизвестный провайдер "
                        f"{name!r} — игнорирую "
                        f"(известные: {sorted(KNOWN_STT_PROVIDERS)})"
                    )
                continue
            if name not in deduped:
                deduped.append(name)
        if not deduped:
            return list(DEFAULT_STT_PROVIDER_CHAIN)
        if deduped == [_LAST_RESORT_PROVIDER]:
            return deduped
        if _LAST_RESORT_PROVIDER in deduped:
            deduped.remove(_LAST_RESORT_PROVIDER)
        deduped.append(_LAST_RESORT_PROVIDER)
        return deduped

    def _ensure_minimax_provider(self):
        """Лениво собрать MiniMax STT-клиент. ``None`` — провайдер не настроен.

        Отсутствие ключа — не ошибка: MiniMax просто выпадает из цепочки
        (ADR-0091 §5.2 «enabled: false + нет ключа → тихо пропускаем»).
        Импорт тоже ленивый: без ключа мы не тянем httpx-клиент в память
        на Pi, где за каждый мегабайт RSS идёт бой (issue #2676).
        """
        if self._minimax_stt_initialized:
            return self._minimax_stt_provider
        with self._minimax_stt_lock:
            if self._minimax_stt_initialized:
                return self._minimax_stt_provider
            self._minimax_stt_initialized = True
            if not self.minimax_stt_enabled or not self.minimax_stt_api_key:
                return None
            try:
                from rob_box_voice.stt_providers.minimax_provider import (
                    MiniMaxSTTProvider,
                )

                self._minimax_stt_provider = MiniMaxSTTProvider(
                    base_url=self.minimax_stt_base_url,
                    api_key=self.minimax_stt_api_key,
                    model=self.minimax_stt_model,
                    language=self.minimax_stt_language or None,
                    sample_rate=self.sample_rate,
                )
                self.get_logger().info(
                    f"✅ MiniMax STT инициализирован "
                    f"(model={self.minimax_stt_model}, "
                    f"language={self.minimax_stt_language})"
                )
            except Exception as exc:  # noqa: BLE001 — провайдер опционален
                self.get_logger().warning(
                    f"⚠️ MiniMax STT недоступен ({exc!r}) — цепочка пойдёт дальше"
                )
                self._minimax_stt_provider = None
        return self._minimax_stt_provider

    def _recognize_minimax(self, audio_bytes: bytes) -> Optional[str]:
        """Распознавание через MiniMax STT (cloud, issue #2365).

        В отличие от ``MiniMaxSTTProvider.recognize()``, который глушит все
        ошибки в ``None``, здесь они пробрасываются ТИПИЗИРОВАННЫМИ: кэш
        «мёртвых» обязан отличить «кончились деньги» (на 5 минут) от
        «моргнула сеть» (на 30 секунд), а по ``None`` это неразличимо —
        он выглядит как «тишина» (``reason="empty"``).
        """
        provider = self._ensure_minimax_provider()
        if provider is None:
            return None
        from rob_box_voice.stt_providers.minimax_provider import (
            MiniMaxSTTAuthError,
            MiniMaxSTTRateLimitError,
            MiniMaxSTTUnavailableError,
        )

        try:
            return provider.transcribe(audio_bytes).text
        except MiniMaxSTTAuthError as exc:
            raise STTAuthError(str(exc)) from exc
        except MiniMaxSTTRateLimitError as exc:
            raise STTQuotaError(str(exc)) from exc
        except MiniMaxSTTUnavailableError as exc:
            if str(exc).startswith("timeout"):
                raise STTTimeoutError(str(exc)) from exc
            raise

    def _build_provider_chain(self) -> list:
        """Собрать адаптеры в порядке приоритета, пропустив ненастроенных.

        Кэш «мёртвых» здесь НЕ применяется — это забота
        ``select_recognition`` (он же залогирует пропуск как
        ``reason="dead"``, чтобы оператор видел причину в одной строке).
        """
        builders = {
            "minimax": self._minimax_adapter,
            "yandex": self._yandex_adapter,
            "vosk": self._vosk_adapter,
        }
        providers: list = []
        for name in self.provider_chain:
            builder = builders.get(name)
            if builder is None:
                continue
            adapter = builder()
            if adapter is not None:
                providers.append(adapter)
        return providers

    def _minimax_adapter(self):
        """Адаптер MiniMax или ``None``, если провайдер не настроен."""
        if not self.minimax_stt_enabled or not self.minimax_stt_api_key:
            return None
        return _NodeSTTAdapter("minimax", self._recognize_minimax)

    def _yandex_adapter(self):
        """Адаптер Yandex или ``None``, если gRPC-стаб не поднялся."""
        if self.yandex_stub is None:
            return None
        return _NodeSTTAdapter("yandex", self._recognize_yandex)

    def _vosk_adapter(self):
        """Адаптер Vosk или ``None``, если модели нет на диске.

        ``prepare`` грузит модель ВНЕ soft-timeout распознавания —
        иначе первая фраза после отказа облака отбрасывалась по таймауту
        (issue #2609).
        """
        if self.recognizer is None and not self._vosk_available:
            return None
        return _NodeSTTAdapter(
            "vosk", self._recognize_vosk, prepare=self._ensure_vosk_loaded
        )

    def _provider_policies(self) -> dict:
        """Per-provider бюджет таймаута/повторов (issue #2365 Phase 2).

        До Phase 2 бюджет был один на всех, и retry доставались только
        первому в цепочке. С тремя провайдерами это неверно: у MiniMax
        свой таймаут (5с), у Yandex свой (12с, issue #1477), а Vosk
        офлайновый — ему ни таймаут, ни повтор не нужны.
        """
        if ProviderPolicy is None:  # pragma: no cover — без stt_fallback
            return {}
        return {
            "minimax": ProviderPolicy(
                timeout_s=self.minimax_stt_timeout_s,
                max_retries=self.minimax_stt_max_retries,
                retry_backoff_s=self.retry_backoff_s,
            ),
            "yandex": ProviderPolicy(
                timeout_s=self.yandex_timeout_s,
                max_retries=self.yandex_max_retries,
                retry_backoff_s=self.retry_backoff_s,
            ),
            # Vosk: повтор мусора даст тот же мусор. Таймаут берём
            # с запасом — на Pi холодный прогон бывает ~2-4с.
            "vosk": ProviderPolicy(
                timeout_s=max(self.yandex_timeout_s, 10.0),
                max_retries=0,
                retry_backoff_s=self.retry_backoff_s,
            ),
        }

    def _recognize_with_fallback(self, audio_bytes: bytes) -> "tuple[Optional[str], list]":
        """Прогнать фразу по цепочке провайдеров (ADR-0124).

        Порядок — из ``stt_provider_chain`` (дефолт yandex → minimax →
        vosk, issue #2866), бюджеты — из ``_provider_policies``, пропуск лежащих
        облаков — через кэш «мёртвых».

        Возвращает ``(text, attempts)``.
        """
        providers = self._build_provider_chain()
        if not providers:
            self.get_logger().warning(
                "⚠️  Нет ни одного STT-провайдера "
                f"(цепочка {self.provider_chain}: MiniMax без ключа, "
                "Yandex без стаба, Vosk без модели)"
            )
            return None, []

        text, attempts = select_recognition(
            providers,
            audio_bytes,
            timeout_s=self.yandex_timeout_s,
            max_retries=self.yandex_max_retries,
            retry_backoff_s=self.retry_backoff_s,
            min_text_chars=self.min_text_chars,
            policies=self._provider_policies(),
            dead_cache=self._provider_dead_cache,
            max_total_s=self.stt_phrase_budget_s,
        )
        self._log_provider_state("recognize", attempts=attempts)
        return text, attempts

    # ── Issue #2365 Phase 2: фактический провайдер + персистентный кэш ────

    def _effective_provider(self) -> Optional[str]:
        """Первый «живой» провайдер цепочки — то, что реально слушает робота.

        Аналог ``tts_node._effective_provider`` (issue #1229): номинальный
        порядок из параметра и фактический после фолбека — разные вещи, и
        оператору нужен второй.
        """
        chain = getattr(self, "provider_chain", None) or list(
            DEFAULT_STT_PROVIDER_CHAIN
        )
        cache = getattr(self, "_provider_dead_cache", None)
        if cache is None:
            return chain[0] if chain else None
        for name in chain:
            if not cache.is_dead(name):
                return name
        return chain[-1] if chain else None

    def _load_persisted_provider_state(self) -> None:
        """Восстановить кэш «мёртвых» из файла (аналог tts_node, issue #1229).

        Зачем: voice-assistant перезапускается чаще, чем хотелось бы
        (#2676 — OOM и каскадные рестарты). Без персистентности каждый
        рестарт снова платит полный таймаут мёртвому облаку.
        """
        cache = getattr(self, "_provider_dead_cache", None)
        path = getattr(self, "provider_state_file", "") or ""
        if cache is None or not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError, TypeError):
            return  # нет файла / битый JSON — не мешаем старту
        if not isinstance(data, dict):
            return
        dead = data.get("dead_providers") or {}
        if not isinstance(dead, dict):
            return
        restored = cache.restore_wall(dead)
        if restored:
            self.get_logger().info(
                f"💾 Восстановлен кэш мёртвых STT-провайдеров из {path}: "
                f"{restored}"
            )

    def _persist_provider_state(self, payload: dict) -> None:
        """Записать состояние провайдеров в файл (best-effort)."""
        path = getattr(self, "provider_state_file", "") or ""
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False)
        except OSError as exc:  # noqa: BLE001 — файл не критичен для STT
            self.get_logger().debug(
                f"⚠️ Не удалось записать stt provider_state {path}: {exc}"
            )

    def _log_provider_state(self, reason: str, attempts=None) -> None:
        """Зафиксировать смену фактического провайдера: лог + файл состояния.

        У TTS аналогичное состояние уезжает в топик
        ``/voice/tts/provider_state`` (issue #1229) — там у него четыре
        потребителя (mcp_server, dialogue_node, quest_node, supervisor).
        У STT потребителя пока нет, а топик без потребителя — ровно то,
        что ловит сторож issue #2118, поэтому здесь только лог и
        персистентный файл. Появится потребитель — топик добавляется
        одной строкой рядом.

        Пишем только при СМЕНЕ эффективного провайдера (или на
        ``startup``): иначе строка сыпалась бы на каждую фразу.
        """
        effective = self._effective_provider()
        if effective is None:
            return
        if reason != "startup" and effective == self._last_effective_provider:
            return
        self._last_effective_provider = effective
        cache = getattr(self, "_provider_dead_cache", None)
        dead = {
            name: round(cache.remaining_s(name), 1)
            for name in self.provider_chain
            if cache is not None and cache.is_dead(name)
        }
        self.get_logger().info(
            f"🎧 STT provider → '{effective}' "
            f"(chain={self.provider_chain}, dead={dead}, reason={reason}, "
            f"last_attempt={attempts[-1].provider if attempts else None})"
        )
        self._persist_provider_state(
            {
                "provider": effective,
                "dead_providers": (
                    cache.snapshot_wall() if cache is not None else {}
                ),
            }
        )

    def _recognize_legacy(self, audio_bytes: bytes) -> Optional[str]:
        """Legacy-путь, если модуль stt_fallback.py не импортировался."""
        text: Optional[str] = None
        if self.yandex_stub:
            try:
                text = self._recognize_yandex(audio_bytes)
                if text:
                    self.get_logger().info(f'✅ Yandex STT: "{text}"')
            except Exception as e:
                self.get_logger().error(f"⚠️  Yandex STT ошибка: {e}, fallback на Vosk")
        if not text and (self.recognizer is not None or self._vosk_available):
            text = self._recognize_vosk(audio_bytes)
            if text:
                self.get_logger().info(f'✅ Vosk (fallback): "{text}"')
        return text

    def _maybe_fire_early_boop(self, text: str) -> None:
        """Issue #1251 — ранний «бульк»: сигнал «услышал, wake word есть».

        Вызывается из _recognize_yandex (на partial И final) и из
        _recognize_vosk (на final). Как только в распознанном тексте появился
        wake word — публикуем триггер на /voice/sound/trigger, sound_node
        сыграет короткий «бульк» ЧЕРЕЗ ~2-3с после конца фразы (а не через
        ~8с, когда закончатся LLM+TTS). Один раз за фразу: Yandex шлёт
        несколько partials с тем же wake word — _boop_fired глушит повторы.

        Булек НЕ блокирует barge-in: sound_node сам решает, когда играть
        (у него свой is_playing guard и AudioPlaybackManager); если юзер
        начал говорить — звук просто пропустится или прервётся.
        """
        # #1990: бульк — сигнал личности (ReSpeaker). На потоках шлема
        # (ptt/wake) не играем: оператор и так слышит себя, а wake-поток —
        # фоновый слушатель.
        if getattr(self, "_active_source", _SRC_RESPEAKER) != _SRC_RESPEAKER:
            return
        if not self.early_boop_enabled or self._boop_fired:
            return
        if not text or not text.strip():
            return
        text_lower = text.lower()
        if not has_wake_word(text_lower, self.wake_words):
            return
        self._boop_fired = True
        try:
            msg = String()
            msg.data = self.early_boop_trigger
            self.boop_pub.publish(msg)
        except Exception as e:  # noqa: BLE001 — звук не критичен для STT
            self.get_logger().warning(f"⚠️ [boop] Ошибка публикации триггера: {e}")
            return
        _boop_ms = int((time.monotonic() - self._phrase_started_at) * 1000)
        self.get_logger().info(
            f"🔔 [boop] Ранний сигнал: wake word в тексте {text[:40]!r} "
            f"(boop_latency_ms={_boop_ms})"
        )

    def _recognize_yandex(self, audio_bytes: bytes) -> Optional[str]:
        """
        Распознавание через Yandex Cloud STT gRPC v3 (Streaming API).
        Двухфазный прогон (issue #1477):
          1. PRIMARY: REAL_TIME + speech_analysis (speaker_tag для issue #1077).
          2. FALLBACK (если фаза 1 дала empty): FULL_DATA без speech_analysis.
             FULL_DATA обрабатывает весь чанк за один проход — устойчив к
             EOU race и проблемам «REAL_TIME + speech_analysis» на коротких
             фразах с pre-roll/эхо TTS.
        Возвращает распознанный текст или None.
        """

        # Issue #1477 — телеметрия фразы ДО отправки в Yandex: RMS/duration,
        # чтобы в следующий раз не гадать, почему Yandex вернул empty. Дешёвая
        # операция (<1ms) — гоняется всегда.
        import struct as _struct
        import math as _math

        n_samples = len(audio_bytes) // 2
        if n_samples == 0:
            return None
        # Берём каждый 4-й сэмпл (быстрее; для 16kHz 1с → 4000 семплов).
        stride = 4
        sum_sq = 0
        peak = 0
        count = 0
        for i in range(0, n_samples, stride):
            v = _struct.unpack_from("<h", audio_bytes, i * 2)[0]
            sum_sq += v * v
            if abs(v) > peak:
                peak = abs(v)
            count += 1
        rms = (sum_sq / count) ** 0.5 if count else 0
        rms_dbfs = 20.0 * _math.log10(rms / 32768.0 + 1e-9)
        peak_dbfs = 20.0 * _math.log10(peak / 32768.0 + 1e-9)
        duration_s = n_samples / max(self.sample_rate, 1)
        self.get_logger().info(
            f"📊 [issue 1477] audio_rms_dbfs={rms_dbfs:.1f} peak_dbfs={peak_dbfs:.1f} "
            f"duration={duration_s:.2f}s samples={n_samples}"
        )
        if rms_dbfs < _QUIET_SIGNAL_HINT_DBFS:
            # Issue #2767 — гипотеза (НЕ доказанная): частые rejected(empty)
            # могут объясняться слабым сигналом канала микрофона, а не
            # только каскадом STT. Только диагностика, никакого автогейна.
            self.get_logger().warning(
                f"📉 [issue 2767] Очень тихий сигнал: "
                f"audio_rms_dbfs={rms_dbfs:.1f} "
                f"< {_QUIET_SIGNAL_HINT_DBFS:.0f} — возможная причина частых "
                "empty от облачных STT (канал/AGC микрофона), отдельная "
                "непроверенная гипотеза, усиление НЕ применяется"
            )

        # Фаза 1: REAL_TIME + speech_analysis (production-настройки, нужны
        # для speaker_tag и «булька» — issue #1077/#1251).
        text_phase1 = self._recognize_yandex_phase(
            audio_bytes,
            phase="REAL_TIME",
            enable_speech_analysis=True,
        )
        if text_phase1:
            return text_phase1

        # Issue #1477 — фаза 2 fallback: FULL_DATA без speech_analysis.
        # FULL_DATA обрабатывает всю фразу целиком, без streaming EOU —
        # устойчив к гонкам EOU на коротких фразах с pre-roll. Без
        # speech_analysis (он несовместим с FULL_DATA — speaker_labeling
        # требует, а speaker_analysis для FULL_DATA не даёт полезных
        # дополнительных событий). speaker_tag сбрасывается (Vosk-fallback
        # ниже тоже без tag).
        self.get_logger().info(
            "📊 [issue 1477] phase1=empty → фаза 2 (FULL_DATA без speech_analysis)"
        )
        text_phase2 = self._recognize_yandex_phase(
            audio_bytes,
            phase="FULL_DATA",
            enable_speech_analysis=False,
        )
        if text_phase2:
            # Получили текст через FULL_DATA — speaker_tag сбрасываем
            # (фаза без speech_analysis его не вернёт).
            self._last_speaker_tag = None
            return text_phase2

        # Не распознано — сбрасываем tag, fallback на Vosk без tag.
        self._last_speaker_tag = None
        return None

    def _recognize_yandex_phase(
        self,
        audio_bytes: bytes,
        *,
        phase: str,
        enable_speech_analysis: bool,
    ) -> Optional[str]:
        """Один проход Yandex STT gRPC v3 с заданным audio_processing_type.

        Фазы:
        - REAL_TIME + speech_analysis=True: production. speaker_tag, partials.
        - FULL_DATA + speech_analysis=False: fallback (issue #1477). Устойчив
          к EOU race, но без speaker_tag и partials.
        """
        # Генератор для streaming запроса
        def gen():
            # audio_processing_type зависит от фазы.
            if phase == "FULL_DATA":
                audio_proc = stt_pb2.RecognitionModelOptions.FULL_DATA
            else:
                audio_proc = stt_pb2.RecognitionModelOptions.REAL_TIME

            recognition_model = stt_pb2.RecognitionModelOptions(
                model=self.yandex_model,
                audio_format=stt_pb2.AudioFormatOptions(
                    raw_audio=stt_pb2.RawAudio(
                        audio_encoding=stt_pb2.RawAudio.LINEAR16_PCM,
                        sample_rate_hertz=self.sample_rate,
                        audio_channel_count=1,
                    )
                ),
                text_normalization=stt_pb2.TextNormalizationOptions(
                    text_normalization=stt_pb2.TextNormalizationOptions.TEXT_NORMALIZATION_DISABLED,
                    profanity_filter=False,
                    literature_text=False,
                ),
                language_restriction=stt_pb2.LanguageRestrictionOptions(
                    restriction_type=stt_pb2.LanguageRestrictionOptions.WHITELIST,
                    language_code=[self.yandex_language],
                ),
                audio_processing_type=audio_proc,
            )

            # EOU — как раньше. У Yandex REAL_TIME EOU настраивается через
            # default_classifier (type + max_pause). FULL_DATA EOU игнорируется,
            # но параметр обязателен в StreamingOptions.
            eou_classifier = stt_pb2.EouClassifierOptions(
                default_classifier=stt_pb2.DefaultEouClassifier(
                    type=self.eou_profiles[self.eou_profile]["type"],
                    max_pause_between_words_hint_ms=self.eou_profiles[
                        self.eou_profile
                    ]["max_pause_ms"],
                )
            )

            opts = stt_pb2.StreamingOptions(
                recognition_model=recognition_model,
                eou_classifier=eou_classifier,
            )
            if enable_speech_analysis:
                # Issue #1077 — speech_analysis нужен для speaker_tag и
                # conversation_analysis. Несовместим с FULL_DATA (для
                # FULL_DATA есть speaker_labeling — отдельная опция, мы её
                # не используем, потому что у нас задача real-time barge-in).
                opts.speech_analysis.CopyFrom(
                    stt_pb2.SpeechAnalysisOptions(
                        enable_speaker_analysis=True,
                        enable_conversation_analysis=True,
                    )
                )
            yield stt_pb2.StreamingRequest(session_options=opts)

            # 2. Отправляем аудио данные чанками по 4096 байт
            chunk_size = 4096
            for i in range(0, len(audio_bytes), chunk_size):
                chunk = audio_bytes[i : i + chunk_size]
                yield stt_pb2.StreamingRequest(chunk=stt_pb2.AudioChunk(data=chunk))

        # Выполняем streaming запрос
        try:
            responses = self.yandex_stub.RecognizeStreaming(
                gen(),
                metadata=(("authorization", f"Api-Key {self.yandex_api_key}"),),
                timeout=self.yandex_timeout_s,
            )
        except grpc.RpcError as e:
            self.get_logger().warning(
                f"⚠️ [issue 1477] phase={phase} grpc error: {e.code()} {e.details()}"
            )
            # Issue #2365 Phase 2 — типизируем код gRPC, чтобы кэш «мёртвых»
            # отличил кончившуюся квоту от моргнувшей сети (_map_grpc_error).
            raise _map_grpc_error(e, self.yandex_timeout_s)

        # Обрабатываем ответы. Issue #2891: Yandex шлёт final/final_refinement
        # на КАЖДЫЙ сегмент фразы (сегменты режет его EOU) — собираем все,
        # из стрима не выходим до конца (раньше break на первом refinement
        # оставлял только первый сегмент: «робот здравствуй» из 7.6 с).
        segments = YandexSegmentCollector()
        last_partial = None
        speaker_tag: Optional[str] = None
        eou_events = 0
        partial_count = 0
        # Issue #2924 — сколько ответов сервер успел прислать до ошибки.
        # Отличает «сервер молчал весь дедлайн» от «прислал сегменты, но не
        # закрыл стрим» — 23.09 этого не было в логе, и причину пришлось
        # выводить по косвенному признаку (не сработал ранний «бульк»).
        response_count = 0
        # Issue #2365 Phase 2: gRPC-ошибка стрима прилетает ЗДЕСЬ, при
        # итерации, а не на вызове RecognizeStreaming — тот лишь открывает
        # стрим. До 21.09.2026 цикл не был обёрнут, поэтому реальный код
        # (на роботе — UNAVAILABLE «Network is unreachable», Yandex STT
        # резолвится в IPv6, которого у робота нет) не доходил ни до лога,
        # ни до кэша «мёртвых» — в метрике стояло голое reason=error.
        try:
            for response in responses:
                response_count += 1
                event_type = response.WhichOneof("Event")
                # #2931: сборщику нужны и partial (текст, который сервер не
                # зафиксировал в final), и eou_update (для trace).
                segments.feed(response, event_type)

                if event_type == "partial":
                    partial_count += 1
                    if response.partial.alternatives:
                        _pt = response.partial.alternatives[0].text
                        if _pt and _pt.strip():
                            last_partial = _pt
                            self._maybe_fire_early_boop(_pt)
                    continue

                elif event_type == "speaker_analysis":
                    sa = response.speaker_analysis
                    tag = getattr(sa, "speaker_tag", None)
                    if tag is not None and str(tag) != "":
                        speaker_tag = str(tag)
                    continue

                elif event_type == "conversation_analysis":
                    continue

                elif event_type == "eou_update":
                    # #2924: в v3 oneof-поле называется eou_update (было
                    # "end_of_utterance" — такого нет, счётчик всегда был 0).
                    eou_events += 1
                    continue
        except grpc.RpcError as e:
            raise self._on_yandex_stream_error(
                e,
                f"phase={phase} responses={response_count} "
                f"partials={partial_count} eou={eou_events} "
                f"segments={segments.segment_count} "
                f"stream=[{segments.trace()}]",
            )

        final_text = segments.text()
        # Issue #1477 — телеметрия по фазе: partials/finals/eou;
        # #2891 — число склеенных сегментов; #2931 — события стрима на INFO:
        # без них «робот» из начала фразы пропал, а чем именно (partial без
        # final / пустой final / уточнение не туда / сервер не услышал) —
        # по логу робота было не понять.
        self.get_logger().info(
            f"📊 [#2931] yandex phase={phase} partials={partial_count} "
            f"eou={eou_events} segments={segments.segment_count} "
            f"stream=[{segments.trace()}] final={final_text!r}"
        )

        result_text = None
        if final_text and final_text.strip():
            result_text = final_text.strip()
        elif last_partial:
            result_text = last_partial.strip()

        if result_text and enable_speech_analysis:
            # speaker_tag фиксируем только для фазы REAL_TIME+speech_analysis
            self._last_speaker_tag = speaker_tag

        if result_text:
            self._maybe_fire_early_boop(result_text)
            return result_text
        return None

    def _recognize_vosk(self, audio_bytes: bytes) -> Optional[str]:
        """Распознавание через Vosk (fallback)."""
        # Issue #1077 — Vosk не даёт speaker_analysis: tag=None, профиль
        # спикера не создаётся (edge case #4).
        self._last_speaker_tag = None
        if not self._ensure_vosk_loaded():
            return None
        # Кормим Vosk по кусочкам, как Yandex (4KB chunks)
        # Это важно! Vosk работает в streaming режиме и не может обработать всю фразу сразу
        chunk_size = 4096

        for i in range(0, len(audio_bytes), chunk_size):
            chunk = audio_bytes[i : i + chunk_size]
            self.recognizer.AcceptWaveform(chunk)

        # После всех чанков получаем финальный результат
        result = json.loads(self.recognizer.FinalResult())
        text = result.get("text", "").strip()

        # Issue #1251 — Vosk fallback тоже даёт ранний «бульк» (Vosk локальный,
        # распознаёт за ~0.5-1с, поэтому сигнал всё равно успевает в 2-3с).
        self._maybe_fire_early_boop(text)

        # Сбросить распознаватель для следующей фразы
        self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
        self.recognizer.SetWords(True)

        return text

    def _publish_utterance_id(self, audio_bytes: bytes, text: str) -> None:
        """Issue #2829 (ADR-0131) — publish this phrase's ``utterance_id``.

        Deterministic hash of the raw PCM bytes this node just recognised
        (see ``core/utterance_id.py``). speaker_id_node computes the same
        hash from the same ``/audio/speech_audio`` bytes independently —
        no coordination needed, both land on the same id. Published
        unconditionally for every accepted ReSpeaker phrase (unlike
        ``_publish_speaker``, which skips when there is no Yandex speaker
        tag) so dialogue_node ALWAYS has an id to correlate against, even
        on the Vosk-fallback path.

        Issue #2862 — ``text`` is the exact string that goes to
        ``/voice/stt/result`` next: dialogue_node joins the two topics by
        text, because DDS does not order delivery across topics.
        """
        utterance_id = compute_utterance_id(audio_bytes)
        msg = String()
        msg.data = json.dumps(
            {"utterance_id": utterance_id, "text": text}, ensure_ascii=False
        )
        self.utterance_pub.publish(msg)

    def _publish_speaker(self, text: str, duration_s: float = 0.0) -> None:
        """Публикация speaker_tag (issue #1077) на /voice/stt/speaker.

        Отдельный топик: контракт /voice/stt/result (plain text) НЕ меняем.
        JSON: ``{"speaker_tag": "0", "text": "...", "duration_s": 1.2}``.
        Если tag=None (Vosk fallback / Yandex без speaker_analysis) — не
        публикуем, dialogue_node не создаст профиль. ``duration_s`` — длина
        фразы в секундах; dialogue_node использует её для правила «короткие
        (<0.8с) не создают профиль».
        """
        tag = self._last_speaker_tag
        if not tag:
            return
        payload = json.dumps(
            {
                "speaker_tag": tag,
                "text": text,
                "duration_s": round(float(duration_s), 3),
            },
            ensure_ascii=False,
        )
        msg = String()
        msg.data = payload
        self.speaker_pub.publish(msg)
        self.get_logger().info(
            f"👤 [issue 1077] Speaker: tag={tag!r} duration={duration_s:.2f}s "
            f"text={text[:40]!r}"
        )

    def barge_in_policy_callback(self, msg: String):
        """Приём действующей ``barge_in_policy`` от dialogue_node (issue #1734).

        dialogue_node — единственный владелец этого параметра (см.
        комментарий у ``self._barge_in_policy`` в ``__init__`` — почему
        stt_node НЕ дублирует его в своём YAML). Публикует latched
        (TRANSIENT_LOCAL) на ``/voice/dialogue/barge_in_policy`` при
        старте и на каждое runtime-изменение (``ros2 param set
        /dialogue_node barge_in_policy ...`` → ``parameters_callback`` →
        ``_publish_barge_in_policy``).

        Неизвестное/пустое значение игнорируем и остаёмся на текущем —
        dialogue_node уже сам провалидировал ввод и залогировал warning
        при опечатке; здесь незачем повторять эту проверку строже.
        """
        value = str(getattr(msg, "data", "") or "").strip().lower()
        if value not in ("replace", "classify"):
            return
        if value != self._barge_in_policy:
            self.get_logger().info(
                f"🔄 [issue 1734] barge_in_policy → {value!r} "
                f"(было {self._barge_in_policy!r})"
            )
        self._barge_in_policy = value

    def publish_result(self, text: str) -> None:
        """Публикация финального результата распознавания (ReSpeaker-путь).

        Публикуем в ``/voice/stt/result`` (личность). Wake-word barge-in
        (немедленный STOP TTS) делаем ТОЛЬКО здесь: на потоках шлема
        (ptt/wake) barge уже выполнен quest-сервером при PTT start / его
        решает агент оператора.

        Issue #1734 — немедленный STOP на wake-word публикуется ТОЛЬКО
        при ``barge_in_policy="replace"`` (дефолт, issue #993 — робот
        должен реагировать на wake-word, даже пока сам говорит). При
        ``"classify"`` STOP здесь НЕ публикуется: dialogue_node._on_stt
        сам прогонит фразу через ``quick_decide`` и решит
        ``_cancel_run(stop_tts=...)`` — REPLACE (явный императив) шлёт
        STOP, MERGE/PENDING_LLM/IGNORE дают текущему сегменту доиграть
        (§2.5 SCHEDULER_DESIGN.md, «правка на лету без замолкания»).
        Раньше этот код публиковал STOP безусловно и обгонял решение
        dialogue_node — см. raw evidence issue #1734 (куплет про комара
        обрывался на «и ещё про енота», хотя quick_decide должен был
        смёржить сегменты).
        """
        pub = self.result_pub
        text_lower = text.lower()

        # Если фраза начинается с wake word — сработал wake-word barge-in.
        if any(text_lower.startswith(word) for word in self.wake_words):
            if self._barge_in_policy == "classify":
                self.get_logger().info(
                    f'🎯 [issue 1734] Wake word detected: "{text[:30]}" → '
                    f"STOP TTS отложен (barge_in_policy=classify, решает "
                    f"dialogue_node/quick_decide)"
                )
            else:
                self.get_logger().info(f'🎯 Wake word detected: "{text[:30]}" → STOP TTS')
                stop_msg = String()
                stop_msg.data = "STOP"
                self.tts_control_pub.publish(stop_msg)

        msg = String()
        msg.data = text
        pub.publish(msg)
        self.get_logger().info(f"📤 Опубликовал результат: {text}")

    def publish_state(self, state: str):
        """Публикация состояния ноды."""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
