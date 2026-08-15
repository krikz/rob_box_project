#!/usr/bin/env python3
"""
STTNode - Speech-to-Text с Yandex STT gRPC v3 (primary) + Vosk (fallback)
Подписывается: /audio/speech_audio (AudioData)
Публикует: /voice/stt/result (String)
"""

import json
import os
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
        DEFAULT_MIN_TEXT_CHARS,
        DEFAULT_YANDEX_MAX_RETRIES,
        DEFAULT_YANDEX_TIMEOUT_S,
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

    class STTTimeoutError(TimeoutError):  # type: ignore[no-redef]
        pass


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

try:
    from rob_box_voice.core.dialogue_text import has_wake_word

    _HAS_WAKE_WORD_AVAILABLE = True
except ImportError:  # pragma: no cover — защита для standalone-запуска
    _HAS_WAKE_WORD_AVAILABLE = False

    def has_wake_word(text_lower: str, wake_words: list) -> bool:  # type: ignore[no-redef]
        if not wake_words:
            return True
        return any(w in text_lower for w in wake_words)

try:
    from rob_box_voice.core.speak_helpers import build_ssml_payload

    _BUILD_SSML_AVAILABLE = True
except ImportError:  # pragma: no cover — защита для standalone-запуска
    _BUILD_SSML_AVAILABLE = False

    def build_ssml_payload(text: str, animation: str = "neutral") -> str:  # type: ignore[no-redef]
        return json.dumps(
            {"ssml": f"<speak>{text}</speak>", "speech_id": "stt-unclear", "emotion": animation},
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

    def __init__(self):
        super().__init__("stt_node")

        # Issue #1234 — OpenTelemetry traces (этап 2). STT-нода не создаёт
        # httpx-клиентов (Yandex gRPC + Vosk offline), но нам нужен
        # корневой span ``stt.recognize``. Если opentelemetry-пакетов нет —
        # no-op (см. observability.tracing).
        init_tracing("stt_node")

        # Параметры Vosk (fallback)
        self.declare_parameter("model_path", "/models/vosk-model-small-ru-0.22")
        self.declare_parameter("sample_rate", 16000)

        self.model_path = self.get_parameter("model_path").value
        self.sample_rate = self.get_parameter("sample_rate").value

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
        # 🔴 fix(voice #1252): синхронизировано с dialogue_node.yaml (12 вариантов) +
        # исторический «робик» (потерян при 9ca7fb29, 21.02). STT реально выдаёт
        # кривые варианты («робок», «роберт», «рыбок», «роботс») — все покрываем.
        self.declare_parameter(
            "wake_words",
            [
                "робок",
                "робот",
                "роббокс",
                "робокос",
                "роббос",
                "робокс",
                "роберт",
                "рыбок",
                "рома",
                "бот",
                "робо",
                "роб",
                "робик",
            ],
        )

        # Параметры fallback/retry (issue #979): единое место для таймаутов,
        # retry и правила коротких фраз. См. rob_box_voice/stt_fallback.py.
        # 5s — окно для Yandex gRPC streaming на фразах 3-4 слов. Раньше
        # gRPC deadline был ~1.3s, из-за чего валидные фразы после TTS
        # попадали на мусорный Vosk fallback.
        self.declare_parameter("yandex_timeout_s", DEFAULT_YANDEX_TIMEOUT_S)
        self.declare_parameter("yandex_max_retries", DEFAULT_YANDEX_MAX_RETRIES)
        self.declare_parameter("retry_backoff_s", 1.0)
        self.declare_parameter("min_text_chars", DEFAULT_MIN_TEXT_CHARS)

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
        self.yandex_timeout_s: float = float(self.get_parameter("yandex_timeout_s").value)
        self.yandex_max_retries: int = int(self.get_parameter("yandex_max_retries").value)
        self.retry_backoff_s: float = float(self.get_parameter("retry_backoff_s").value)
        self.min_text_chars: int = int(self.get_parameter("min_text_chars").value)
        self.unclear_phrase: str = str(self.get_parameter("unclear_phrase").value)
        self.unclear_cooldown_s: float = float(self.get_parameter("unclear_cooldown_s").value)
        self.tts_grace_s: float = float(self.get_parameter("tts_grace_s").value)
        self._last_unclear_at: float = 0.0  # монотонное время последней фразы «не расслышал»
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

        # Подписка на состояние TTS (чтобы не слышать себя)
        self.tts_state_sub = self.create_subscription(String, "/voice/tts/state", self.tts_state_callback, 10)

        # Publishers
        self.result_pub = self.create_publisher(String, "/voice/stt/result", 10)
        self.state_pub = self.create_publisher(String, "/voice/stt/state", 10)
        self.tts_control_pub = self.create_publisher(String, "/voice/tts/control", 10)  # Для прерывания TTS
        # Issue #1077 — speaker_analysis (speaker_tag) от Yandex SpeechKit v3.
        # Публикуем отдельным топиком (String, JSON: {"speaker_tag", "text"}),
        # чтобы НЕ ломать контракт /voice/stt/result (plain text — его читают
        # telegram/perception/transport). dialogue_node подписывается и создаёт
        # профиль спикера (scope=speaker:<tag>).
        self.speaker_pub = self.create_publisher(String, "/voice/stt/speaker", 10)
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

    def initialize_vosk(self):
        """Load Vosk model (fallback provider).

        Architect decision G-VOSK: the model is bundled in the voice_base
        image at /models/vosk-model-small-ru-0.22 (see
        docker/vision/voice_base/Dockerfile). We fail-fast with a helpful
        error message (instead of the opaque 'Folder does not contain model
        files' from the Vosk C++ binding) when it isn't there, pointing the
        operator at the docs.

        Set the env var ROS_VOSK_DISABLE=1 (or declare the
        ``enable_vosk`` ROS param as False) to skip Vosk entirely —
        stt_node will then use only the Yandex gRPC provider.
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

        try:
            self.get_logger().info(f"Загрузка Vosk модели из {self.model_path}...")
            self.model = Model(self.model_path)
            self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
            self.recognizer.SetWords(True)  # Получать разметку по словам
            self.get_logger().info("✅ Vosk модель загружена (fallback)")
            self.publish_state("ready")
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
            self.publish_state("error")

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
        elif msg.data in ["ready", "idle"]:
            if self.is_robot_speaking:
                self._tts_ended_at = time.monotonic()
                if self.aec_mode == "software":
                    self.get_logger().info("🎙️ [software AEC] Робот замолчал - распознавание включено")
                else:
                    self.get_logger().debug("🎙️ [hardware AEC] Робот замолчал")
                self.is_robot_speaking = False

    def speech_audio_callback(self, msg: AudioData):
        """
        Обработка готовых фраз от audio_node
        Пробуем Yandex STT → если не работает, используем Vosk
        """
        import time

        # Конвертируем список в bytes
        audio_bytes = bytes(msg.data)
        duration = len(audio_bytes) / (self.sample_rate * 2)  # 16-bit = 2 bytes

        if self.aec_mode == "software":
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

        # Публикация результата
        if text and not is_short_phrase(text, min_chars=self.min_text_chars):
            # Issue 1076 (телеметрия): честный «фраза → ПРИНЯТО» (STT-часть).
            _accept_ms = int((time.monotonic() - _phrase_received_at) * 1000)
            self.get_logger().info(
                f"📊 [telemetry] phrase_to_accept_ms={_accept_ms} "
                f"(text={text!r})"
            )
            self.get_logger().info(f"✅ ПРИНЯТО: {text}")
            # Issue #1077 — speaker публикуем ПЕРЕД результатом: dialogue_node
            # хранит tag по тексту и забирает его в _on_stt. Если бы speaker
            # шёл после result, гонка топиков могла бы потерять корреляцию.
            self._publish_speaker(text, duration)
            self.publish_result(text)
            self.publish_state("ready")
        else:
            if text:
                self.get_logger().warning(f'❌ ОТКЛОНЕНО (короткое, <{self.min_text_chars} chars): "{text}"')
            else:
                self.get_logger().warning("❌ ОТКЛОНЕНО (пустое)")
            # Issue 989 Fix A: различаем rejected(empty) и rejected(short).
            # rejected(empty) — это почти наверняка эхо собственной музыки/голоса
            # робота или шум: НЕ говорим «не расслышал», молчим. Иначе робот
            # говорит фразу → её эхо снова ловится → снова empty → бесконечный цикл.
            # rejected(short) — Vosk/Yandex вернули что-то (например «не»/«пути»),
            # т.е. был реальный речевой ввод, но слишком короткий — можно переспросить.
            if text:
                self._maybe_speak_unclear()
            else:
                self.get_logger().info("🔇 [issue 989] Пустой STT (эхо/музыка) — молчу, без «не расслышал»")
            self.publish_state("ready")

    def _maybe_speak_unclear(self) -> None:
        """Проговорить «Не расслышал, скажи ещё раз» при неясном результате.

        Ограничение по времени (``unclear_cooldown_s``) защищает от петли:
        робот говорит фразу → микрофон слышит эхо → VAD триггерит новую
        фразу → снова неясный результат → снова «не расслышал»...
        """
        if not self.unclear_phrase:
            return
        now = time.monotonic()
        if now - self._last_unclear_at < self.unclear_cooldown_s:
            self.get_logger().info(
                f"🔕 Пропуск «не расслышал» (cooldown {self.unclear_cooldown_s}s активен)"
            )
            return
        self._last_unclear_at = now
        payload = build_ssml_payload(self.unclear_phrase, animation="confused")
        msg = String()
        msg.data = payload
        self.tts_request_pub.publish(msg)
        self.get_logger().info(f"🗣️ Неясный результат → говорю: {self.unclear_phrase!r}")

    def _recognize_with_fallback(self, audio_bytes: bytes) -> "tuple[Optional[str], list]":
        """Прогоняем фразу через Yandex (primary, retry) + Vosk (fallback).

        Возвращаем ``(text, attempts)``. Используем локальные адаптеры,
        чтобы не тянуть протокол/STTAttempt наружу из speech_audio_callback.
        """

        # Lazy-адаптеры: name нужен для метрик, recognize вызывает наш метод.
        class _YandexAdapter:
            name = "yandex"

            def __init__(self, outer: "STTNode"):
                self._outer = outer

            def recognize(self, data: bytes) -> Optional[str]:
                return self._outer._recognize_yandex(data)

        class _VoskAdapter:
            name = "vosk"

            def __init__(self, outer: "STTNode"):
                self._outer = outer

            def recognize(self, data: bytes) -> Optional[str]:
                return self._outer._recognize_vosk(data)

        providers = []
        if self.yandex_stub is not None:
            providers.append(_YandexAdapter(self))
        if self.recognizer is not None:
            providers.append(_VoskAdapter(self))

        if not providers:
            self.get_logger().warning("⚠️  Нет ни одного STT-провайдера (Yandex и Vosk отключены)")
            return None, []

        return select_recognition(
            providers,
            audio_bytes,
            timeout_s=self.yandex_timeout_s,
            max_retries=self.yandex_max_retries,
            retry_backoff_s=self.retry_backoff_s,
            min_text_chars=self.min_text_chars,
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
        if not text and self.recognizer:
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
        Распознавание через Yandex Cloud STT gRPC v3 (Streaming API)
        Используем streaming для готовой фразы
        """

        # Генератор для streaming запроса
        def gen():
            # 1. Первым yield отправляем session options
            recognize_options = stt_pb2.StreamingOptions(
                recognition_model=stt_pb2.RecognitionModelOptions(
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
                    audio_processing_type=stt_pb2.RecognitionModelOptions.REAL_TIME,
                ),
                # Issue #1077 — ВАЖНО: включаем speech_analysis, иначе Yandex
                # НЕ присылает speaker_analysis (по умолчанию выключен!).
                # Проверено probe на 10.1.1.21: конфиг без этой опции даёт 0
                # speaker_events при yandex:ok; с enable_speaker_analysis=True
                # приходят события speaker_analysis (speaker_tag + границы).
                # speaker_labeling (SpeakerLabelingOptions) НЕ используем: он
                # требует FULL_DATA и несовместим с REAL_TIME (INVALID_ARGUMENT).
                speech_analysis=stt_pb2.SpeechAnalysisOptions(
                    enable_speaker_analysis=True,
                    enable_conversation_analysis=True,
                ),
                # ВАЖНО! Настройка EOU (End of Utterance) - определение конца фразы
                # Используем выбранный profile (fast/balanced/patient)
                eou_classifier=stt_pb2.EouClassifierOptions(
                    default_classifier=stt_pb2.DefaultEouClassifier(
                        type=self.eou_profiles[self.eou_profile]["type"],
                        max_pause_between_words_hint_ms=self.eou_profiles[self.eou_profile]["max_pause_ms"],
                    )
                ),
            )
            yield stt_pb2.StreamingRequest(session_options=recognize_options)

            # 2. Отправляем аудио данные чанками по 4096 байт
            chunk_size = 4096
            for i in range(0, len(audio_bytes), chunk_size):
                chunk = audio_bytes[i : i + chunk_size]
                yield stt_pb2.StreamingRequest(chunk=stt_pb2.AudioChunk(data=chunk))

        # Выполняем streaming запрос
        # timeout= — жёсткий gRPC deadline (issue #979): раньше deadline
        # отсутствовал и зависший стрим мог блокировать callback вечно.
        # Теперь gRPC сам поднимет DEADLINE_EXCEEDED через yandex_timeout_s,
        # а _measure/select_recognition классифицируют это как reason=timeout.
        try:
            responses = self.yandex_stub.RecognizeStreaming(
                gen(),
                metadata=(("authorization", f"Api-Key {self.yandex_api_key}"),),
                timeout=self.yandex_timeout_s,
            )
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
                raise STTTimeoutError(f"Yandex STT deadline exceeded ({self.yandex_timeout_s}s)")
            raise

        # Обрабатываем ответы
        final_text = None
        last_partial = None
        # Issue #1077 — собираем speaker_analysis события (speaker_tag +
        # границы) наравне с final. Yandex может прислать несколько событий
        # (один голос, разбитый на tag='0' и tag='1') — берём последний
        # достоверный tag; если фраза не распозналась (final пуст), tag
        # наружу НЕ уходит (fallback на Vosk без tag).
        speaker_tag: Optional[str] = None
        for response in responses:
            event_type = response.WhichOneof("Event")

            # partial - промежуточные результаты: запоминаем последний
            # (Yandex может прислать только partial + пустой final — см. 09.08)
            if event_type == "partial":
                if response.partial.alternatives:
                    _pt = response.partial.alternatives[0].text
                    if _pt and _pt.strip():
                        last_partial = _pt
                        # Issue #1251 — ранний «бульк»: Yandex шлёт partial
                        # как только распознал начало фразы (включая wake
                        # word «робот»), это и есть сигнал «услышал» через
                        # ~2-3с после конца речи.
                        self._maybe_fire_early_boop(_pt)
                continue

            # speaker_analysis - статистика говорящего (issue #1077).
            # Поля: speaker_tag, utterance_start_ms, utterance_end_ms,
            # words_count, utterance_index. Доступны только при
            # recognition_model audio_processing_type=REAL_TIME и
            # speaker-анализе, включённом Yandex по умолчанию.
            elif event_type == "speaker_analysis":
                sa = response.speaker_analysis
                tag = getattr(sa, "speaker_tag", None)
                if tag is not None and str(tag) != "":
                    speaker_tag = str(tag)
                    self.get_logger().debug(
                        "👤 [issue 1077] SPEAKER_ANALYSIS: "
                        f"tag={tag!r} "
                        f"boundaries=({getattr(sa, 'utterance_start_ms', 0)}-"
                        f"{getattr(sa, 'utterance_end_ms', 0)}ms) "
                        f"words={getattr(sa, 'words_count', 0)} "
                        f"utt={getattr(sa, 'utterance_index', 0)}"
                    )
                continue

            # conversation_analysis - общая статистика диалога
            # (speech_ms, silence_ms, interrupts_count). Логируем — метрика
            # «кто перебил» пригодится для edge case #2 (два человека).
            elif event_type == "conversation_analysis":
                ca = response.conversation_analysis
                self.get_logger().debug(
                    "💬 [issue 1077] CONVERSATION: "
                    f"speech={getattr(ca, 'speech_ms', 0)}ms "
                    f"silence={getattr(ca, 'silence_ms', 0)}ms "
                    f"interrupts={getattr(ca, 'interrupts_count', 0)}"
                )
                continue

            # final - финальный результат распознавания
            elif event_type == "final":
                if response.final.alternatives:
                    final_text = response.final.alternatives[0].text
                    # Продолжаем читать для возможного final_refinement

            # final_refinement - улучшенный результат с нормализацией
            elif event_type == "final_refinement":
                if response.final_refinement.normalized_text:
                    final_text = response.final_refinement.normalized_text.alternatives[0].text
                    break  # Это последний результат

        # Issue #1077 — фиксируем tag для текущей фразы. Только если фраза
        # реально распознана (final_text или last_partial есть): иначе Vosk
        # fallback без tag.
        result_text = None
        if final_text and final_text.strip():
            result_text = final_text.strip()
        elif last_partial:
            # 🔴 FIX (09.08): Yandex шлёт partial с текстом, но final может
            # быть пустым → был ложный «yandex:empty». Берём последний partial.
            result_text = last_partial.strip()
        if result_text:
            self._last_speaker_tag = speaker_tag
            # Issue #1251 — страховка: если Yandex прислал только final
            # (без partial), бульк всё равно должен прозвучать.
            self._maybe_fire_early_boop(result_text)
            return result_text
        # Не распознано — сбрасываем tag, fallback на Vosk без tag.
        self._last_speaker_tag = None
        return None

    def _recognize_vosk(self, audio_bytes: bytes) -> Optional[str]:
        """Распознавание через Vosk (fallback)."""
        # Issue #1077 — Vosk не даёт speaker_analysis: tag=None, профиль
        # спикера не создаётся (edge case #4).
        self._last_speaker_tag = None
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

    def publish_result(self, text: str):
        """Публикация финального результата распознавания."""
        text_lower = text.lower()

        # Если фраза начинается с wake word — немедленно прерываем TTS (barge-in)
        if any(text_lower.startswith(word) for word in self.wake_words):
            self.get_logger().info(f'🎯 Wake word detected: "{text[:30]}" → STOP TTS')
            stop_msg = String()
            stop_msg.data = "STOP"
            self.tts_control_pub.publish(stop_msg)

        msg = String()
        msg.data = text
        self.result_pub.publish(msg)
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
