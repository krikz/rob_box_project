#!/usr/bin/env python3
"""
TTSNode - Text-to-Speech с Yandex Cloud TTS API v3 (gRPC) + Silero fallback

Подписывается на: /voice/dialogue/response (JSON chunks)
Публикует: /voice/audio/speech (AudioData)
Использует:
  - Yandex Cloud TTS API v3 (gRPC, primary, anton voice)
  - Silero TTS v4 (fallback, офлайн, всегда работает)
"""

import io
import json
import os
import sys
import time
import wave
from contextlib import contextmanager
from pathlib import Path

import grpc
import numpy as np
import rclpy
import sounddevice as sd
import torch
from audio_common_msgs.msg import AudioData
from rclpy.node import Node
from std_msgs.msg import String
from .utils.audio_utils import find_respeaker_device_sounddevice
from .audio_playback_manager import AudioPlaybackManager


@contextmanager
def ignore_stderr(enable=True):
    """Подавить ALSA ошибки от sounddevice"""
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield


def resample_audio(audio: np.ndarray, orig_sr: float, target_sr: float) -> np.ndarray:
    """
    Resample audio from original sample rate to target sample rate using linear interpolation.
    
    This is a lightweight resampling implementation suitable for TTS audio where:
    - Low latency is important (no heavy dependencies like scipy/librosa)
    - Audio quality is acceptable for voice synthesis
    - Minimal artifacts for pitch shifting within reasonable range (1.0-3.0x)
    
    For higher quality resampling, consider using scipy.signal.resample or librosa.resample.
    
    Args:
        audio: Audio data as numpy array (mono, float32, range -1.0 to 1.0)
        orig_sr: Original sample rate (e.g., 22050 or 10022.7 for fractional rates)
        target_sr: Target sample rate (e.g., 16000)
    
    Returns:
        Resampled audio at target sample rate
    """
    if abs(orig_sr - target_sr) < 0.01:  # Use epsilon comparison for floats
        return audio
    
    # Calculate the resampling ratio
    duration = len(audio) / orig_sr
    target_length = int(duration * target_sr)
    
    # Create new time indices for interpolation
    orig_indices = np.linspace(0, len(audio) - 1, len(audio))
    target_indices = np.linspace(0, len(audio) - 1, target_length)
    
    # Linear interpolation
    resampled = np.interp(target_indices, orig_indices, audio)
    
    return resampled


# Импортируем text_normalizer и Yandex gRPC
scripts_path = Path(__file__).parent.parent / "scripts"
sys.path.insert(0, str(scripts_path))

try:
    from text_normalizer import normalize_for_tts
except ImportError:

    def normalize_for_tts(text):
        """Fallback если нет normalizer"""
        return text


# Yandex Cloud TTS API v3 (gRPC)
try:
    from yandex.cloud.ai.tts.v3 import tts_pb2, tts_service_pb2_grpc

    YANDEX_GRPC_AVAILABLE = True
except ImportError:
    YANDEX_GRPC_AVAILABLE = False
    print("⚠️  yandex-cloud-ml-sdk не установлен! Используем только Silero fallback.")


class TTSNode(Node):
    """ROS2 нода для синтеза речи с YandexSpeechKit + Silero fallback"""

    def __init__(self):
        super().__init__("tts_node")

        # Параметры
        self.declare_parameter("provider", "yandex")  # yandex (primary) | silero (fallback)

        # Yandex Cloud TTS gRPC v3 (оригинальный ROBBOX голос!)
        self.declare_parameter("yandex_api_key", "")
        self.declare_parameter("yandex_voice", "anton")  # anton (ОРИГИНАЛЬНЫЙ ГОЛОС РОББОКСА!)
        self.declare_parameter("yandex_speed", 1.0)  # 0.1-3.0 (1.0 = нормальная скорость речи)

        # Silero TTS (fallback)
        self.declare_parameter("silero_speaker", "baya")  # aidar (male) | baya (female) | kseniya | xenia | eugene (NEW in v5!)
        self.declare_parameter("silero_sample_rate", 48000)  # v5: можно повысить до 48000 для лучшего качества
        
        # Silero v5: новые флаги для расстановки ударений
        self.declare_parameter("silero_put_accent", True)  # Ударения в обычных словах
        self.declare_parameter("silero_put_yo", True)  # Автоматическая буква ё
        self.declare_parameter("silero_put_stress_homo", True)  # Ударения в омографах (замОк/зАмок)
        self.declare_parameter("silero_put_yo_homo", True)  # Ударения в омографах с ё

        # Общие параметры
        self.declare_parameter("chipmunk_mode", True)  # ВКЛЮЧЕНО: True для весёлого голоса бурундука! 🐿️
        self.declare_parameter("pitch_shift", 1.0)  # Множитель для playback rate (1.0 = нормальная скорость)
        self.declare_parameter("normalize_text", True)
        self.declare_parameter("volume_db", -3.0)  # Громкость в dB (-3dB = 70%)

        # Читаем параметры
        self.provider = self.get_parameter("provider").value

        # Yandex Cloud TTS gRPC v3
        self.yandex_api_key = self.get_parameter("yandex_api_key").value or os.getenv("YANDEX_API_KEY", "")
        self.yandex_voice = self.get_parameter("yandex_voice").value
        self.yandex_speed = self.get_parameter("yandex_speed").value

        # Silero
        self.silero_speaker = self.get_parameter("silero_speaker").value
        self.silero_sample_rate = self.get_parameter("silero_sample_rate").value
        
        # Silero v5: новые флаги
        self.silero_put_accent = self.get_parameter("silero_put_accent").value
        self.silero_put_yo = self.get_parameter("silero_put_yo").value
        self.silero_put_stress_homo = self.get_parameter("silero_put_stress_homo").value
        self.silero_put_yo_homo = self.get_parameter("silero_put_yo_homo").value

        # Общие
        self.chipmunk_mode = self.get_parameter("chipmunk_mode").value
        self.pitch_shift = self.get_parameter("pitch_shift").value
        self.normalize_text = self.get_parameter("normalize_text").value
        self.volume_db = self.get_parameter("volume_db").value

        # Конвертируем dB в линейный множитель
        self.volume_gain = 10.0 ** (self.volume_db / 20.0)

        # Callback для изменения параметров во время работы
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Silero TTS модель (lazy loading - загружается только при первом использовании)
        self.silero_model = None
        self.silero_loading = False
        self.device = torch.device("cpu")

        # Если provider='silero' - загружаем сразу
        if self.provider == "silero":
            self.get_logger().info("🔄 Provider=silero → загрузка Silero TTS...")
            self._load_silero_model()

        # Yandex Cloud TTS gRPC v3 (оригинальный ROBBOX голос anton!)
        self.yandex_channel = None
        self.yandex_stub = None
        if YANDEX_GRPC_AVAILABLE and self.yandex_api_key:
            try:
                self.yandex_channel = grpc.secure_channel(
                    "tts.api.cloud.yandex.net:443", grpc.ssl_channel_credentials()
                )
                self.yandex_stub = tts_service_pb2_grpc.SynthesizerStub(self.yandex_channel)
                self.get_logger().info("✅ Yandex Cloud TTS gRPC v3 подключен")
            except Exception as e:
                self.get_logger().warn(f"⚠️  Не удалось подключиться к Yandex gRPC: {e}")

        # Инициализация аудио устройства для воспроизведения
        self.device_index = None
        self.initialize_audio_device()
        
        # Менеджер воспроизведения (предотвращает ALSA конфликты)
        self.playback_manager = AudioPlaybackManager.get_instance()

        # Подписка на dialogue response (от dialogue_node)
        self.dialogue_sub = self.create_subscription(String, "/voice/dialogue/response", self.dialogue_callback, 10)

        # Подписка на TTS requests (от reflection_node и других)
        self.tts_request_sub = self.create_subscription(
            String, "/voice/tts/request", self.dialogue_callback, 10  # Используем тот же callback
        )

        # Подписка на control commands (STOP)
        self.control_sub = self.create_subscription(String, "/voice/tts/control", self.control_callback, 10)

        # Подписка на новый dialogue_id от dialogue_node.
        # Позволяет отбрасывать устаревшие TTS-запросы от старого диалога после barge-in.
        self._new_dialogue_id_sub = self.create_subscription(
            String, "/voice/current_dialogue_id", self._on_new_dialogue_id, 1
        )

        # Публикация аудио и состояния
        self.audio_pub = self.create_publisher(AudioData, "/voice/audio/speech", 10)
        self.state_pub = self.create_publisher(String, "/voice/tts/state", 10)
        self.finished_pub = self.create_publisher(String, "/voice/tts/finished", 10)  # Публикация завершения произношения

        # Флаг для остановки воспроизведения
        self.stop_requested = False
        self.current_stream = None  # Текущий sounddevice stream

        # Dialogue session tracking (для синхронизации с dialogue_node)
        self.current_dialogue_id = None
        self.processing_dialogue_id = None  # ID диалога в процессе синтеза/воспроизведения
        self.current_speech_id = None  # ID текущего произношения (для MCP tools)

        # Публикуем начальное состояние
        self.publish_state("ready")

        self.get_logger().info("✅ TTSNode инициализирован")
        self.get_logger().info("  Provider: Yandex Cloud TTS gRPC v3 (primary) + Silero v5 (fallback)")
        self.get_logger().info(
            f"  Yandex gRPC v3: voice={self.yandex_voice} (ROBBOX original!), speed={self.yandex_speed} (медленный синтез)"
        )
        self.get_logger().info(
            f"  Silero v5: speaker={self.silero_speaker}, rate={self.silero_sample_rate} Hz, "
            f"homograph_stress={self.silero_put_stress_homo}"
        )
        self.get_logger().info(f"  Volume: {self.volume_db:.1f} dB (gain: {self.volume_gain:.2f}x)")
        self.get_logger().info(f"  Chipmunk mode: {self.chipmunk_mode}")
        if self.chipmunk_mode:
            self.get_logger().info(
                f"  Pitch shift: {self.pitch_shift}x "
                f"(эмуляция оригинального ROBBOX: медленный синтез + быстрое воспроизведение)"
            )

        if not self.yandex_stub:
            self.get_logger().warn("⚠️  Yandex gRPC не подключен - будет использован только Silero fallback")

    def initialize_audio_device(self):
        """Инициализация аудио устройства для воспроизведения.

        ВАЖНО: всегда используем device=None (ALSA default).
        asound.conf маршрутизирует default → dmix_respeaker → hw:1,0.
        dmix позволяет TTS и sound_node воспроизводить одновременно.
        Если использовать прямой hardware-индекс (hw:1,0), dmix обходится
        и второй sd.play() получает PaErrorCode -9985 (Device unavailable).
        """
        self.device_index = None  # ALSA default → dmix_respeaker (через asound.conf)
        try:
            # Логируем что именно sounddevice считает default-устройством
            default_out = sd.query_devices(kind='output')
            device_name = default_out.get('name', '?') if isinstance(default_out, dict) else str(default_out)
            self.get_logger().info(f"✅ TTS playback: ALSA default device → dmix_respeaker ({device_name[:60]})")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Не удалось получить info об ALSA default device: {e}")

    def _load_silero_model(self):
        """Загрузить Silero TTS модель (lazy loading)"""
        if self.silero_model is not None:
            return  # Уже загружена

        if self.silero_loading:
            self.get_logger().warn("⏳ Silero модель уже загружается...")
            return

        self.silero_loading = True
        self.get_logger().info("🔄 Загрузка Silero TTS v5...")

        # ⚡ КРИТИЧНЫЕ НАСТРОЙКИ ДЛЯ ARM64! ⚡
        torch.set_num_threads(4)
        torch._C._jit_set_profiling_mode(False)
        torch.set_grad_enabled(False)

        try:
            # Приоритет путей для модели Silero v5:
            # 1. /models/silero/v5_ru.pt - встроено в Docker образ (основной путь)
            # 2. /cache/tts/silero_v5_ru.pt - персистентный volume (fallback/legacy)
            model_paths = [
                "/models/silero/v5_ru.pt",  # Основной путь в Docker образе
                "/cache/tts/silero_v5_ru.pt",  # Legacy путь (volume)
            ]
            
            model_loaded = False
            for model_path in model_paths:
                if os.path.exists(model_path):
                    self.get_logger().info(f"📦 Загрузка Silero v5: {model_path}")
                    # Silero v5 использует torch.package (не torch.jit!)
                    # https://github.com/snakers4/silero-models#standalone-use
                    self.silero_model = torch.package.PackageImporter(model_path).load_pickle("tts_models", "model")
                    self.silero_model.to(self.device)
                    self.get_logger().info("✅ Silero TTS v5 загружен (ARM64 оптимизация)")
                    model_loaded = True
                    break
            
            if not model_loaded:
                # Fallback на онлайн загрузку через torch.hub
                self.get_logger().warn(f"⚠️ Модель не найдена в {model_paths}, загружаем через torch.hub")
                self.silero_model, _ = torch.hub.load(
                    repo_or_dir="snakers4/silero-models", model="silero_tts", language="ru", speaker="v5_ru"
                )
                self.silero_model.to(self.device)
                self.get_logger().info("✅ Silero TTS v5 загружен из GitHub (ARM64 оптимизация)")
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка загрузки Silero: {e}")
            self.silero_model = None
        finally:
            self.silero_loading = False

    def control_callback(self, msg: String):
        """Обработка control commands (STOP)"""
        command = msg.data.strip().upper()

        if command == "STOP":
            self.get_logger().warn("🔇 STOP command received - немедленная остановка TTS")
            self._interrupt_playback()
            self.publish_state("stopped")

    def _interrupt_playback(self):
        """Прервать текущее воспроизведение (helper метод)"""
        self.stop_requested = True
        # Сбрасываем current_dialogue_id: последующие TTS-запросы без dialogue_id
        # или с устаревшим dialogue_id будут отброшены.
        self.current_dialogue_id = None
        self.processing_dialogue_id = None

        # Остановить текущий sounddevice stream если есть
        if self.current_stream:
            try:
                sd.stop()
                self.current_stream = None
            except Exception as e:
                self.get_logger().error(f"❌ Ошибка остановки stream: {e}")

    def _on_new_dialogue_id(self, msg: String):
        """Обновляем current_dialogue_id как только dialogue_node начинает новый диалог.
        Если в очереди tts_node ещё остались запросы от старого диалога — они будут отброшены.
        """
        new_id = msg.data
        if new_id and new_id != self.current_dialogue_id:
            self.get_logger().info(
                f"🔄 Новый диалог: {new_id[:8]} "
                f"(старый: {self.current_dialogue_id[:8] if self.current_dialogue_id else 'None'}) "
                f"— устаревшие TTS-запросы будут отброшены"
            )
            self.current_dialogue_id = new_id

    def dialogue_callback(self, msg: String):
        """Обработка JSON chunks от dialogue_node"""
        try:
            chunk_data = json.loads(msg.data)

            if "ssml" not in chunk_data:
                self.get_logger().warn("⚠ Chunk без SSML")
                return

            # Генерируем speech_id для отслеживания
            import uuid
            speech_id = chunk_data.get("speech_id", str(uuid.uuid4()))
            self.current_speech_id = speech_id

            # Проверяем dialogue_id (если присутствует)
            dialogue_id = chunk_data.get("dialogue_id", None)

            # Старый запрос от устаревшего диалога — отбрасываем ДО синтеза
            if dialogue_id and self.current_dialogue_id and dialogue_id != self.current_dialogue_id:
                self.get_logger().warning(
                    f"❌ Отбрасываем устаревший TTS диалога {dialogue_id[:8]} "
                    f"(текущий: {self.current_dialogue_id[:8]})"
                )
                # Опубликуем finished с error=True чтобы MCP speak_text не вис в ожидании
                speech_id_to_drop = chunk_data.get("speech_id")
                if speech_id_to_drop:
                    import json as _json
                    _drop_msg = String()
                    _drop_msg.data = _json.dumps(
                        {"speech_id": speech_id_to_drop, "success": False, "error": "stale_dialogue"},
                        ensure_ascii=False
                    )
                    self.finished_pub.publish(_drop_msg)
                return

            if dialogue_id:
                # Если это новый диалог - прерываем предыдущий
                if self.current_dialogue_id and dialogue_id != self.current_dialogue_id:
                    self.get_logger().warning(
                        f"🔄 Новый диалог обнаружен! "
                        f"Прерываем предыдущий ({self.current_dialogue_id[:8]}...) → "
                        f"новый ({dialogue_id[:8]}...)"
                    )
                    # Прерываем воспроизведение
                    self._interrupt_playback()

                # Обновляем текущий dialogue_id
                self.current_dialogue_id = dialogue_id

                # Проверяем: если мы сейчас обрабатываем другой диалог - отбрасываем chunk
                if self.processing_dialogue_id and self.processing_dialogue_id != dialogue_id:
                    self.get_logger().warning(
                        f"❌ Отбрасываем устаревший chunk (dialogue_id: {dialogue_id[:8]}..., "
                        f"ожидается: {self.processing_dialogue_id[:8]}...)"
                    )
                    return

            ssml = chunk_data["ssml"]

            # Извлекаем текст из SSML
            text = self._extract_text_from_ssml(ssml)

            if not text.strip():
                return

            # Извлекаем атрибуты SSML (pitch, rate)
            ssml_attributes = self._parse_ssml_attributes(ssml)

            self.get_logger().info(
                f'🔊 TTS: {text[:50]}... (speech_id: {speech_id[:8]}, dialogue_id: {dialogue_id[:8] if dialogue_id else "None"}...)'
            )
            if ssml_attributes:
                self.get_logger().info(f"🎵 SSML атрибуты: {ssml_attributes}")

            # Синтез и воспроизведение
            self._synthesize_and_play(ssml, text, dialogue_id, ssml_attributes, speech_id)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ JSON parse error: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ TTS error: {e}")

    def _extract_text_from_ssml(self, ssml: str) -> str:
        """Извлекает текст из SSML тегов"""
        import re

        # Убираем все XML теги
        text = re.sub(r"<[^>]+>", "", ssml)
        return text.strip()

    def _parse_ssml_attributes(self, ssml: str) -> dict:
        """
        Извлекает атрибуты из SSML тегов (pitch, rate/speed)
        
        Returns:
            dict: {'pitch': float, 'rate': float} или пустой dict
        """
        import re
        
        attributes = {}
        
        # Ищем <prosody> теги с атрибутами
        # Примеры: <prosody pitch="+10%" rate="1.2">, <prosody pitch="high" rate="slow">
        prosody_pattern = r'<prosody\s+([^>]+)>'
        matches = re.finditer(prosody_pattern, ssml, re.IGNORECASE)
        
        for match in matches:
            attrs_str = match.group(1)
            
            # Парсим pitch
            pitch_match = re.search(r'pitch\s*=\s*["\']?([^"\'>\s]+)["\']?', attrs_str, re.IGNORECASE)
            if pitch_match:
                pitch_value = pitch_match.group(1)
                # Конвертируем в множитель для Yandex
                # "+10%" -> 1.1, "-10%" -> 0.9, "high" -> 1.2, "low" -> 0.8
                if '%' in pitch_value:
                    try:
                        percent = float(pitch_value.replace('%', ''))
                        attributes['pitch'] = 1.0 + (percent / 100.0)
                    except ValueError:
                        pass
                elif pitch_value == 'high':
                    attributes['pitch'] = 1.2
                elif pitch_value == 'low':
                    attributes['pitch'] = 0.8
                elif pitch_value == 'medium':
                    attributes['pitch'] = 1.0
                else:
                    try:
                        attributes['pitch'] = float(pitch_value)
                    except ValueError:
                        pass
            
            # Парсим rate (скорость речи)
            rate_match = re.search(r'rate\s*=\s*["\']?([^"\'>\s]+)["\']?', attrs_str, re.IGNORECASE)
            if rate_match:
                rate_value = rate_match.group(1)
                # "1.5" -> 1.5, "fast" -> 1.5, "slow" -> 0.7
                if '%' in rate_value:
                    try:
                        percent = float(rate_value.replace('%', ''))
                        attributes['rate'] = percent / 100.0
                    except ValueError:
                        pass
                elif rate_value == 'fast':
                    attributes['rate'] = 1.5
                elif rate_value == 'slow':
                    attributes['rate'] = 0.7
                elif rate_value == 'medium':
                    attributes['rate'] = 1.0
                else:
                    try:
                        attributes['rate'] = float(rate_value)
                    except ValueError:
                        pass
        
        return attributes

    def _synthesize_and_play(self, ssml: str, text: str, dialogue_id: str = None, ssml_attributes: dict = None, speech_id: str = None):
        """Синтез речи и воспроизведение"""
        # Сбрасываем флаг stop при новом запросе
        self.stop_requested = False
        
        # Устанавливаем processing_dialogue_id для этого синтеза
        if dialogue_id:
            self.processing_dialogue_id = dialogue_id
            self.get_logger().debug(f"🎯 Начинаем обработку dialogue_id: {dialogue_id[:8]}...")

        # Извлекаем SSML атрибуты (если не переданы)
        if ssml_attributes is None:
            ssml_attributes = {}

        # Нормализация (если включена)
        if self.normalize_text:
            text = normalize_for_tts(text)

        try:
            # Сначала пробуем Yandex
            audio_np = None
            sample_rate = 16000  # Yandex возвращает 16kHz

            if self.yandex_stub:  # Проверяем что gRPC канал инициализирован
                try:
                    self.publish_state("synthesizing")
                    self.get_logger().info("🔊 Синтез через Yandex Cloud TTS gRPC v3 (anton)...")
                    audio_np = self._synthesize_yandex(text, ssml_attributes)
                    sample_rate = 22050  # Yandex обычно возвращает 22050 Hz или 48000 Hz
                    # sample_rate уже получен в _synthesize_yandex, но пока захардкодим
                except Exception as e:
                    self.get_logger().warn(f"⚠️  Yandex gRPC отвалился: {e}, переключаюсь на Silero fallback")
                    audio_np = None

            # Fallback на Silero если Yandex не сработал
            if audio_np is None:
                # Загружаем Silero при первом использовании (lazy loading)
                if self.silero_model is None:
                    self.get_logger().warn("⚠️  Silero модель не загружена, загружаю сейчас...")
                    self._load_silero_model()

                if self.silero_model is None:
                    raise Exception("Silero fallback недоступен - не удалось загрузить модель!")

                self.publish_state("synthesizing")
                self.get_logger().info("🔊 Синтез через Silero v5 (fallback)...")

                # Логируем SSML атрибуты если есть (для консистентности с Yandex)
                if ssml_attributes:
                    self.get_logger().info(f"🎵 SSML атрибуты для Silero: {ssml_attributes}")

                # Оборачиваем в SSML для Silero
                # Silero поддерживает SSML напрямую через apply_tts
                if not ssml.startswith("<speak>"):
                    ssml_text = f'<speak><prosody pitch="medium">{text}</prosody></speak>'
                else:
                    ssml_text = ssml

                # Используем новые флаги v5 для расстановки ударений
                audio = self.silero_model.apply_tts(
                    ssml_text=ssml_text, 
                    speaker=self.silero_speaker, 
                    sample_rate=self.silero_sample_rate,
                    put_accent=self.silero_put_accent,
                    put_yo=self.silero_put_yo,
                    put_stress_homo=self.silero_put_stress_homo,
                    put_yo_homo=self.silero_put_yo_homo
                )
                audio_np = audio.numpy()
                sample_rate = self.silero_sample_rate  # 48000 Hz (v5)
                self.get_logger().info(
                    f"✅ Silero v5 fallback успешен: {len(audio_np)} samples @ {sample_rate} Hz "
                    f"(homograph_stress={self.silero_put_stress_homo})"
                )

            # Публикуем в ROS topic
            self._publish_audio(audio_np)

            # КРИТИЧЕСКАЯ ПРОВЕРКА: dialogue_id не изменился во время синтеза?
            if dialogue_id and self.current_dialogue_id != dialogue_id:
                self.get_logger().warning(
                    f"⚠️  Dialogue изменился во время синтеза! "
                    f"Отменяем воспроизведение старого chunk "
                    f"(было: {dialogue_id[:8]}..., сейчас: {self.current_dialogue_id[:8]}...)"
                )
                self.processing_dialogue_id = None
                return

            # Воспроизводим локально
            self.publish_state("playing")

            # ВАЖНО: ReSpeaker поддерживает ТОЛЬКО 16kHz стерео!
            target_rate = 16000

            # Эффект "бурундука" ROBBOX:
            # В оригинале: Yandex возвращает ~22050 Hz (speed=0.4), читаем сырые PCM, воспроизводим на 44100 Hz
            # Результат: 44100/22050 = 2x pitch shift (голос выше и быстрее)
            # 
            # Новая реализация (правильная):
            # - chipmunk_mode=False: правильный resample для корректного воспроизведения
            # - chipmunk_mode=True: эмуляция оригинала через изменение эффективной частоты
            # - pitch_shift параметр: дополнительный множитель (1.0 = стандарт, 1.5 = ещё выше, 0.8 = ниже)
            #
            # Оригинальный ROBBOX эффект:
            # Yandex с speed=0.4 → ~22050 Hz → воспроизведение как 44100 Hz = 2x эффект
            # Но ReSpeaker работает на 16000 Hz, поэтому эмулируем через:
            # 22050 Hz → 11025 Hz (эффективно, делим на 2) → 16000 Hz
            
            if self.chipmunk_mode:
                # Эффект бурундука через изменение эффективной частоты
                # Оригинальный ROBBOX: соотношение 44100/22050 = 2.0
                # С учётом ReSpeaker 16kHz: применяем базовый множитель 2.0 * pitch_shift
                base_multiplier = 2.0  # Оригинальное соотношение частот в ROBBOX
                effective_multiplier = base_multiplier * self.pitch_shift
                
                # Вычисляем эффективную частоту после "ускорения"
                # Например: 22050 / (2.0 * 1.0) = 11025 Hz
                effective_rate = sample_rate / effective_multiplier
                
                # Сначала ресэмплим до эффективной частоты (ускорение)
                audio_processed = resample_audio(audio_np, sample_rate, effective_rate)
                
                # Затем ресэмплим до target_rate для ReSpeaker
                if abs(effective_rate - target_rate) > 0.01:
                    audio_processed = resample_audio(audio_processed, effective_rate, target_rate)
                
                self.get_logger().info(
                    f"🐿️  Эффект бурундука ROBBOX: {len(audio_np)} → {len(audio_processed)} samples "
                    f"({effective_multiplier:.1f}x ускорение, {sample_rate}Hz → {effective_rate:.1f}Hz → {target_rate}Hz)"
                )
            else:
                # Нормальное воспроизведение БЕЗ pitch shift
                # Resample audio to target rate для правильной скорости
                if sample_rate != target_rate:
                    self.get_logger().info(
                        f"🔄 Resampling: {sample_rate} Hz → {target_rate} Hz "
                        f"({len(audio_np)} samples)"
                    )
                    audio_processed = resample_audio(audio_np, sample_rate, target_rate)
                    self.get_logger().info(f"✅ Resampled to {len(audio_processed)} samples @ {target_rate} Hz")
                else:
                    audio_processed = audio_np
                self.get_logger().info(f"🎵 Нормальная скорость: {len(audio_processed)} samples")

            # Применяем громкость
            audio_np_adjusted = audio_processed * self.volume_gain

            # Конвертируем моно → стерео (ReSpeaker требует 2 канала!)
            audio_stereo = np.column_stack((audio_np_adjusted, audio_np_adjusted))
            self.get_logger().info(f"🔊 Воспроизведение: {len(audio_stereo)} frames, {target_rate} Hz, стерео")

            # Проверка STOP ДО воспроизведения
            if self.stop_requested:
                self.get_logger().warn("🔇 STOP: отменено ДО воспроизведения")
                self.publish_state("stopped")
                return

            # Блокирующее воспроизведение через менеджер (защита от ALSA конфликтов)
            with ignore_stderr(enable=True):
                self.current_stream = True  # Маркер что воспроизведение идёт
                
                # Используем AudioPlaybackManager для синхронизированного доступа
                success = self.playback_manager.play_audio(
                    audio_data=audio_stereo,
                    sample_rate=target_rate,
                    device_index=self.device_index,
                    blocking=True,  # Блокирующее воспроизведение для TTS
                    timeout=5.0,
                    node_name="tts_node"
                )
                
                if not success:
                    self.get_logger().warn("⚠️  Аудио устройство занято, пропуск воспроизведения")
                    self.current_stream = None
                    # КРИТИЧНО: публикуем события завершения даже при ошибке!
                    self.publish_state("ready")
                    
                    # Публикуем ошибку для MCP tools и animation_player
                    if speech_id:
                        finished_msg = String()
                        finished_msg.data = json.dumps(
                            {"speech_id": speech_id, "success": False, "error": "Device unavailable"}, 
                            ensure_ascii=False
                        )
                        self.finished_pub.publish(finished_msg)
                        self.get_logger().info(f"📢 TTS finished event (ошибка): speech_id={speech_id[:8]}...")
                    
                    # Очищаем processing_dialogue_id
                    if dialogue_id and self.processing_dialogue_id == dialogue_id:
                        self.processing_dialogue_id = None
                    
                    return
                
                self.current_stream = None

            # Cleanup для устранения белого шума после воспроизведения
            self.cleanup_playback_noise()

            # Закончили воспроизведение
            if self.stop_requested:
                self.publish_state("stopped")
                self.get_logger().warn("🔇 Воспроизведение прервано")
                # Публикуем ошибку для MCP tools
                if speech_id:
                    finished_msg = String()
                    finished_msg.data = json.dumps({"speech_id": speech_id, "success": False, "error": "stopped"}, ensure_ascii=False)
                    self.finished_pub.publish(finished_msg)
            else:
                self.publish_state("ready")
                self.get_logger().info("✅ Воспроизведение завершено")
                # Публикуем успех для MCP tools
                if speech_id:
                    finished_msg = String()
                    finished_msg.data = json.dumps({"speech_id": speech_id, "success": True}, ensure_ascii=False)
                    self.get_logger().info(f"📢 Публикую TTS finished event: speech_id={speech_id[:8]}..., success=True")
                    self.finished_pub.publish(finished_msg)
                    self.get_logger().info(f"✅ TTS finished event опубликован на /voice/tts/finished")

            # Очищаем processing_dialogue_id после завершения
            if dialogue_id and self.processing_dialogue_id == dialogue_id:
                self.processing_dialogue_id = None

        except Exception as e:
            self.get_logger().error(f"❌ Synthesis error: {e}")
            self.publish_state("ready")
            # Публикуем ошибку для MCP tools
            if speech_id:
                finished_msg = String()
                finished_msg.data = json.dumps({"speech_id": speech_id, "success": False, "error": str(e)}, ensure_ascii=False)
                self.finished_pub.publish(finished_msg)
            # Очищаем processing_dialogue_id при ошибке
            if dialogue_id and self.processing_dialogue_id == dialogue_id:
                self.processing_dialogue_id = None

    def _synthesize_yandex(self, text: str, ssml_attributes: dict = None) -> np.ndarray:
        """Синтез через Yandex Cloud TTS gRPC API v3 (anton voice!)
        
        Args:
            text: Текст для синтеза
            ssml_attributes: Словарь с атрибутами SSML (pitch, rate)
        """
        if not self.yandex_stub:
            raise Exception("Yandex gRPC stub не инициализирован")

        # Применяем SSML атрибуты если есть
        if ssml_attributes is None:
            ssml_attributes = {}
        
        # Скорость речи: берем из SSML или используем параметр ноды
        speech_rate = ssml_attributes.get('rate', self.yandex_speed)
        
        # Pitch для Yandex не поддерживается напрямую через hints,
        # но мы можем логировать для будущей реализации
        if 'pitch' in ssml_attributes:
            self.get_logger().info(f"🎵 SSML pitch={ssml_attributes['pitch']} (не применяется в Yandex TTS)")

        # Создаём запрос как в оригинальном ROBBOX коде
        request = tts_pb2.UtteranceSynthesisRequest(
            text=text,
            output_audio_spec=tts_pb2.AudioFormatOptions(
                container_audio=tts_pb2.ContainerAudio(container_audio_type=tts_pb2.ContainerAudio.WAV)
            ),
            hints=[
                tts_pb2.Hints(voice=self.yandex_voice),  # anton!
                tts_pb2.Hints(speed=speech_rate),  # Используем rate из SSML или параметр
            ],
            loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS,
        )

        try:
            # Отправляем запрос с авторизацией
            responses = self.yandex_stub.UtteranceSynthesis(
                request, metadata=(("authorization", f"Api-Key {self.yandex_api_key}"),)
            )

            # Собираем аудио данные из стрима
            audio_data = b""
            for response in responses:
                audio_data += response.audio_chunk.data

            if not audio_data:
                raise Exception("Пустой ответ от Yandex TTS")

            # ВАЖНО! Для оригинального звука ROBBOX:
            # Вариант 1 (оригинал): читаем сырые PCM с заголовком WAV (np.frombuffer)
            # Вариант 2 (новый): читаем правильно с декодированием WAV (soundfile)
            # Сейчас используем Вариант 1 для совместимости с оригиналом

            # Декодируем СЫРЫЕ байты (включая WAV заголовок!) как PCM
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

            # Для логов определим реальную частоту из WAV заголовка
            try:
                with io.BytesIO(audio_data) as wav_file:
                    with wave.open(wav_file, "rb") as wav:
                        actual_sample_rate = wav.getframerate()
            except Exception:  # noqa: E722
                actual_sample_rate = 22050  # fallback

            self.get_logger().info(
                f"✅ Yandex gRPC v3 (ROBBOX original!): {len(audio_np)} samples, "
                f"source {actual_sample_rate} Hz, speed={speech_rate}"
            )

            return audio_np

        except grpc.RpcError as e:
            raise Exception(f"Yandex gRPC error: {e.code()} - {e.details()}")
        except Exception as e:
            raise Exception(f"Yandex synthesis error: {e}")

    def _publish_audio(self, audio_np: np.ndarray):
        """Публикует аудио в ROS topic"""
        # Конвертируем в int16 для AudioData
        audio_int16 = (audio_np * 32767).astype(np.int16)

        msg = AudioData()
        msg.data = audio_int16.tobytes()

        self.audio_pub.publish(msg)

    def cleanup_playback_noise(self):
        """
        Устранение белого шума после воспроизведения TTS.

        Проблема: После воспроизведения через ReSpeaker (USB Audio Class 1.0)
        возникает постоянный белый шум из-за активного playback channel.

        Решение:
        1. Properly close sounddevice stream
        2. Flush audio buffers
        3. Small delay для стабилизации USB audio interface
        """
        try:
            # 1. Ensure sounddevice is fully stopped
            sd.stop()

            # 2. Small delay to let USB audio interface stabilize
            # ReSpeaker USB Audio Class 1.0 requires time to properly close playback path
            time.sleep(0.1)

            # 3. Log cleanup completion
            self.get_logger().debug("🧹 TTS playback noise cleanup completed")

        except Exception as e:
            self.get_logger().warn(f"⚠️ TTS noise cleanup failed: {e}")

    def publish_state(self, state: str):
        """Публикация состояния TTS"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def parameters_callback(self, params):
        """Callback для изменения параметров во время работы"""
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == "volume_db":
                self.volume_db = param.value
                self.volume_gain = 10.0 ** (self.volume_db / 20.0)
                self.get_logger().info(
                    f"🔊 Громкость изменена: {self.volume_db:.1f} dB (gain: {self.volume_gain:.2f}x)"
                )
            elif param.name == "pitch_shift":
                self.pitch_shift = param.value
                self.get_logger().info(f"🐿️ Pitch shift изменён: {self.pitch_shift}x")
            elif param.name == "chipmunk_mode":
                self.chipmunk_mode = param.value
                self.get_logger().info(f"🐿️ Chipmunk mode: {self.chipmunk_mode}")
            elif param.name == "yandex_speed":
                self.yandex_speed = param.value
                self.get_logger().info(f"🎵 Yandex speed (pitch) изменён: {self.yandex_speed}")

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
