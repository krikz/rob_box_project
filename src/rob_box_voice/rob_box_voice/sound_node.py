#!/usr/bin/env python3
"""
SoundNode - воспроизведение звуковых эффектов
Подписывается: /voice/sound/trigger (String, тул play_sound), /voice/expression/request (earcon рефлексов)
Публикует: /voice/sound/state (String), /voice/sound/event (SoundEvent), /voice/expression/event (ExpressionEvent)

На каждый запрос звука - событие: started, затем finished, либо skipped{busy|voice_stream|not_found|error}.
Молчаливого пропуска нет (ADR-0150 §5.5.2, V33, PR-3b #3690).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from audio_common_msgs.msg import AudioData
from rob_box_dialog_msgs.msg import ExpressionEvent, ExpressionRequest, SoundEvent

import os
import json
import random
import threading
import time
import uuid
import numpy as np
import sounddevice as sd
from typing import Dict, List, Optional
from pydub import AudioSegment
from .utils.audio_utils import find_respeaker_device_sounddevice
from .audio_playback_manager import AudioPlaybackManager
from .core.sound_requests import (  # noqa: F401 - EARCON_SOUNDS читают тесты и скрипты
    EARCON_SOUNDS, SKIP_BUSY, SKIP_ERROR, SoundRequestsMixin,
)


# Рация (voice passthrough): тишина дольше этого порога (сек) закрывает
# голосовой stream (план P1, Task 1.2, решение D7).
VOICE_SILENCE_TIMEOUT = 0.3


class SoundNode(SoundRequestsMixin, Node):
    """Нода для воспроизведения звуковых эффектов."""

    def __init__(self):
        super().__init__("sound_node")

        # Параметры
        self.declare_parameter("sound_pack_dir", "/ws/sound_pack")  # Docker path по умолчанию
        self.declare_parameter("volume_db", -12.0)  # Регулировка громкости (dB), -12dB ≈ 24% громкости
        self.declare_parameter("trigger_animations", True)
        self.declare_parameter("animation_topic", "/animations/trigger")

        sound_pack_dir = self.get_parameter("sound_pack_dir").value
        # Expand ~ только если это локальный путь (не /ws/...)
        if sound_pack_dir.startswith("~"):
            self.sound_pack_dir = os.path.expanduser(sound_pack_dir)
        else:
            self.sound_pack_dir = sound_pack_dir
        self.volume_db = self.get_parameter("volume_db").value
        self.trigger_animations = self.get_parameter("trigger_animations").value
        self.animation_topic = self.get_parameter("animation_topic").value

        # Callback для изменения параметров во время работы
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Subscribers
        self.trigger_sub = self.create_subscription(String, "/voice/sound/trigger", self.trigger_callback, 10)
        # Earcon рефлексов хода (ADR-0150 §5.5.3): «услышал», «готово», «не вышло».
        self.expression_sub = self.create_subscription(
            ExpressionRequest, "/voice/expression/request", self.expression_callback, 10
        )
        # STOP закрывает голосовой стрим рации (PTT Quest/Telegram). mp3 из библиотеки генераций здесь больше не
        # играет — он на деке движка (ADR-0158 PR-2), стоп — stop_music → PlayerOwner.
        self.sound_stop_sub = self.create_subscription(String, "/voice/sound/stop", self.sound_stop_callback, 10)
        # Рация (voice passthrough): голос оператора из Quest (/avatar/voice_in).
        # best-effort + volatile QoS, чтобы не доигрывать stale-чанки после
        # потери соединения (план P1, Task 1.1).
        voice_in_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.voice_in_sub = self.create_subscription(
            AudioData, "/avatar/voice_in", self.voice_in_callback, voice_in_qos
        )

        # Состояние голосового стрима (Task 1.2)
        self._voice_stream = None  # sd.OutputStream во время голоса оператора
        self._voice_last_activity = 0.0  # monotonic timestamp последнего чанка
        # Таймер: закрыть стрим при тишине (проверка каждый VOICE_SILENCE_TIMEOUT).
        self.create_timer(VOICE_SILENCE_TIMEOUT, self._voice_stream_watchdog)

        # Publishers
        self.state_pub = self.create_publisher(String, "/voice/sound/state", 10)
        self.sound_event_pub = self.create_publisher(SoundEvent, "/voice/sound/event", 10)
        self.expression_event_pub = self.create_publisher(ExpressionEvent, "/voice/expression/event", 10)

        # Опционально: триггер анимаций
        if self.trigger_animations:
            self.animation_pub = self.create_publisher(String, self.animation_topic, 10)

        # Инициализация аудио устройства
        self.device_index = None
        self.initialize_audio_device()

        # Менеджер воспроизведения (предотвращает ALSA конфликты)
        self.playback_manager = AudioPlaybackManager.get_instance()

        # Хранилище звуков
        self.sounds: Dict[str, AudioSegment] = {}  # filename (без .mp3) → AudioSegment
        self.trigger_map: Dict[str, str] = {}  # trigger → filename mapping из catalog.json
        self.sound_groups: Dict[str, List[str]] = {
            # Random groups for variety
            "talk": ["talk_1", "talk_2", "talk_3", "talk_4"],
            "cute": ["cute", "very_cute"],
            "confused": ["confused", "confused_alt"],
            "drip": ["robot_drip_a1", "robot_drip_d4", "robot_drip_d5", "robot_drip_e4"],
            "work": ["dot_matrix_1", "dot_matrix_2", "dot_matrix_3"],
            "talk_beep": ["videogame_talk_beep", "videogame_talk_beep_high"],
        }

        # Состояние
        self.is_playing = False
        self.current_sound: Optional[str] = None
        self.play_thread: Optional[threading.Thread] = None
        self._play_lock = threading.Lock()

        # Инициализация
        self.get_logger().info("SoundNode инициализирован")
        self.load_sounds()

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
            default_out = sd.query_devices(kind='output')
            device_name = default_out.get('name', '?') if isinstance(default_out, dict) else str(default_out)
            self.get_logger().info(f"✅ Sound playback: ALSA default device → dmix_respeaker ({device_name[:60]})")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Не удалось получить info об ALSA default device: {e}")

    def load_sounds(self):
        """Загрузка всех звуковых файлов из sound_pack/."""
        if not os.path.exists(self.sound_pack_dir):
            self.get_logger().error(f"❌ Директория sound_pack не найдена: {self.sound_pack_dir}")
            self.publish_state("error_no_dir")
            return

        self.get_logger().info(f"📂 Загрузка звуков из {self.sound_pack_dir}...")

        # Загрузить catalog.json для mapping trigger → filename
        catalog_path = os.path.join(self.sound_pack_dir, "sound_catalog.json")
        if os.path.exists(catalog_path):
            try:
                with open(catalog_path, 'r', encoding='utf-8') as f:
                    catalog = json.load(f)

                # Построить trigger_map из catalog['sounds']
                sounds = catalog.get('sounds', {})
                for filename, info in sounds.items():
                    if not filename.endswith('.mp3'):
                        continue
                    if 'trigger' in info:
                        sound_name = filename.replace(".mp3", "")
                        self.trigger_map[info['trigger']] = sound_name

                self.get_logger().info(f"✅ Загружено {len(self.trigger_map)} trigger mappings из catalog.json")

            except Exception as e:
                self.get_logger().error(f"❌ Ошибка чтения catalog.json: {e}")
        else:
            self.get_logger().warn(f"⚠️ Файл catalog.json не найден: {catalog_path}")

        # Загрузить все mp3 файлы
        loaded_count = 0
        for filename in os.listdir(self.sound_pack_dir):
            if filename.endswith(".mp3"):
                sound_name = filename.replace(".mp3", "")
                filepath = os.path.join(self.sound_pack_dir, filename)

                try:
                    # Загрузить MP3
                    audio = AudioSegment.from_mp3(filepath)

                    # Применить регулировку громкости
                    if self.volume_db != 0:
                        audio = audio + self.volume_db

                    self.sounds[sound_name] = audio
                    loaded_count += 1
                    self.get_logger().info(f"  ✓ {sound_name}: {len(audio)}ms, {audio.frame_rate}Hz")

                except Exception as e:
                    self.get_logger().error(f"  ❌ Ошибка загрузки {filename}: {e}")

        self.get_logger().info(f"✅ Загружено звуков: {loaded_count}/{len(os.listdir(self.sound_pack_dir))}")

        # Информация об аудио устройстве
        if self.device_index is not None:
            devices = sd.query_devices()
            device_name = devices[self.device_index]['name']
            self.get_logger().info(f"🔊 Аудио устройство: {device_name} (index {self.device_index})")
        else:
            self.get_logger().info("🔊 Аудио устройство: default system device")

        if loaded_count > 0:
            self.publish_state("ready")
        else:
            self.publish_state("error_no_sounds")

    def trigger_callback(self, msg: String):
        """Обработка триггера звукового эффекта (``play_sound``): исход - ``SoundEvent``."""
        trigger = msg.data.strip()
        self.get_logger().info(f"🔔 Триггер: {trigger}")
        self.start_sound(trigger, uuid.uuid4().hex[:8])

    def sound_stop_callback(self, msg: String):
        """STOP на ``/voice/sound/stop`` закрывает голосовой стрим рации (barge-in)."""
        if msg.data.strip().upper() == "STOP":
            self._close_voice_stream()

    def voice_in_callback(self, msg):
        """Голос оператора (/avatar/voice_in, AudioData int16 PCM 16 kHz).

        Стриминговый вывод: первый чанк открывает sd.OutputStream
        (16 kHz, stereo int16 — ReSpeaker требует 2 канала), последующие
        пишутся в него. Один stream на весь PTT-сеанс, чтобы не было
        кликов/лага от повторного открытия (план P1, Task 1.2, D7).
        """
        try:
            # AudioData.data приходит как list[int] (tts_node/audio_node публикуют
            # `list(bytes)`) — нормализуем через bytes() перед frombuffer.
            chunk = np.frombuffer(bytes(msg.data), dtype=np.int16)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"❌ voice_in: невалидный AudioData: {e}")
            return
        if chunk.size == 0:
            return
        # Mono → stereo (ReSpeaker playback требует 2 канала, как в tts_node).
        stereo = np.column_stack((chunk, chunk))

        if self._voice_stream is None:
            self._open_voice_stream()
        stream = self._voice_stream
        if stream is None:
            return

        try:
            stream.write(stereo)
            self._voice_last_activity = time.monotonic()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"❌ voice_in: ошибка записи в stream: {e}")
            self._close_voice_stream()

    def _open_voice_stream(self):
        """Открыть OutputStream для голоса (16 kHz, stereo int16)."""
        try:
            self._voice_stream = sd.OutputStream(
                samplerate=16000,
                channels=2,
                dtype="int16",
                device=self.device_index,
            )
            self._voice_stream.start()
            self._voice_last_activity = time.monotonic()
            self.get_logger().info("🎙️ Voice passthrough stream открыт (16k stereo int16)")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"❌ Не удалось открыть voice stream: {e}")
            self._voice_stream = None

    def _close_voice_stream(self):
        """Закрыть голосовой stream (если открыт)."""
        stream = self._voice_stream
        if stream is None:
            return
        for op in ("stop", "close"):
            try:
                getattr(stream, op)()
            except Exception:  # noqa: BLE001
                pass
        self._voice_stream = None

    def _close_voice_stream_if_idle(self, now: Optional[float] = None) -> bool:
        """Закрыть голосовой stream, если тишина дольше VOICE_SILENCE_TIMEOUT.

        Args:
            now: текущее время (time.monotonic). Для тестов можно передать
                явно, чтобы детерминированно проверить порог.

        Returns:
            True, если stream закрыт; False иначе.
        """
        if self._voice_stream is None:
            return False
        if now is None:
            now = time.monotonic()
        if now - self._voice_last_activity >= VOICE_SILENCE_TIMEOUT:
            self.get_logger().info("🎙️ Тишина — закрываю voice passthrough stream")
            self._close_voice_stream()
            return True
        return False

    def _voice_stream_watchdog(self):
        """Таймер: закрыть стрим при тишине."""
        self._close_voice_stream_if_idle()

    def select_sound(self, trigger: str) -> Optional[str]:
        """Выбрать звук по триггеру."""
        # 1. Проверить trigger_map (из catalog.json)
        if trigger in self.trigger_map:
            sound_name = self.trigger_map[trigger]
            if sound_name in self.sounds:
                return sound_name

        # 2. Прямое совпадение с filename (для обратной совместимости)
        if trigger in self.sounds:
            return trigger

        # 3. Проверить группы (например, trigger="talk" → random из talk_1..4)
        if trigger in self.sound_groups:
            available = [name for name in self.sound_groups[trigger] if name in self.sounds]
            if available:
                return random.choice(available)

        # 4. Попробовать найти похожий
        for sound_name in self.sounds.keys():
            if trigger.lower() in sound_name.lower():
                return sound_name

        return None

    def play_sound_thread(self, sound_name: str, trigger: str, request_id: str = ""):
        """Воспроизведение звука в отдельном потоке; ``started`` -> ``finished`` | ``skipped{busy|error}``."""
        self.is_playing = True
        self.current_sound = sound_name
        self.publish_state(f"playing_{sound_name}")
        self.publish_sound_event(request_id, SoundEvent.KIND_STARTED, sound_name)
        outcome, reason = SoundEvent.KIND_FINISHED, ""

        try:
            self.get_logger().info(f"▶️ Играю: {sound_name}")

            # Триггер анимации (если включено)
            if self.trigger_animations:
                self.trigger_animation(trigger)

            # Получить аудио
            audio = self.sounds[sound_name]

            # ReSpeaker playback требует: 16kHz, stereo (2 channels)
            # (проверено через /proc/asound/card1/stream0)
            if audio.frame_rate != 16000:
                self.get_logger().debug(f"Ресемплинг {audio.frame_rate} Hz → 16000 Hz")
                audio = audio.set_frame_rate(16000)

            # Конвертация в numpy array
            samples = np.array(audio.get_array_of_samples())

            # Mono → Stereo (ReSpeaker требует 2 канала)
            if audio.channels == 1:
                samples = np.column_stack((samples, samples))
            elif audio.channels == 2:
                samples = samples.reshape((-1, 2))

            # Нормализация float32
            samples = samples.astype(np.float32) / 32768.0

            # Воспроизведение через менеджер (защита от ALSA конфликтов)
            success = self.playback_manager.play_audio(
                audio_data=samples,
                sample_rate=16000,
                device_index=self.device_index,
                blocking=True,  # Блокирующее воспроизведение для звуков
                timeout=3.0,  # Меньший timeout для звуковых эффектов
                node_name="sound_node"
            )

            if not success:
                self.get_logger().warn(f"⚠️  Аудио устройство занято, пропуск {sound_name}")
                outcome, reason = SoundEvent.KIND_SKIPPED, SKIP_BUSY
            else:
                self.get_logger().info(f"✅ Завершено: {sound_name}")

            # Cleanup для устранения белого шума после воспроизведения
            self.cleanup_playback_noise()

        except Exception as e:
            self.get_logger().error(f"❌ Ошибка воспроизведения {sound_name}: {e}")
            outcome, reason = SoundEvent.KIND_SKIPPED, SKIP_ERROR

        finally:
            self.publish_sound_event(request_id, outcome, sound_name, reason)
            self.is_playing = False
            self.current_sound = None
            self.publish_state("ready")

    def cleanup_playback_noise(self):
        """
        Устранение белого шума после воспроизведения звука.

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
            self.get_logger().debug("🧹 Playback noise cleanup completed")

        except Exception as e:
            self.get_logger().warn(f"⚠️ Noise cleanup failed: {e}")

    def trigger_animation(self, trigger: str):
        """Триггер соответствующей анимации."""
        # Маппинг звуков на анимации (расширенный для новых звуков)
        animation_map = {
            # Legacy names
            "thinking": "thinking",
            "surprise": "surprise",
            "confused": "confused",
            "angry": "angry",
            "angry_1": "angry",
            "error": "error",
            "cute": "happy",
            "very_cute": "very_happy",
            "talk": "talking",
            "talk_1": "talking",
            "talk_2": "talking",
            "talk_3": "talking",
            "talk_4": "talking",

            # BASE robot emotional sounds
            "robot_thinking": "thinking",
            "robot_surprise": "surprise",
            "robot_confused": "confused",
            "robot_confused_alt": "confused",
            "robot_angry": "angry",
            "robot_error": "error",
            "robot_cute": "happy",
            "robot_very_cute": "very_happy",
            "robot_happy": "happy",
            "robot_sigh": "sad",
            "robot_concerned": "confused",
            "robot_affirm": None,  # No animation for affirmation
            "robot_confirm": None,  # No animation for confirmation
            "robot_talk_1": "talking",
            "robot_talk_2": "talking",
            "robot_talk_3": "talking",
            "robot_talk_4": "talking",

            # Drip sounds - no specific animation
            "robot_drip_a1": None,
            "robot_drip_d4": None,
            "robot_drip_d5": None,
            "robot_drip_e4": None,

            # UI sounds - no animations
            "ui_activate": None,
            "ui_bell": None,
            "ui_button": None,
            "ui_chime": None,
            "ui_confirm": None,
            "ui_dot": None,
            "ui_menu_click": None,
            "ui_note_e": None,
            "ui_notification": None,
            "ui_radio_start": None,
            "ui_random": None,
            "ui_roger": None,

            # Robot special effects
            "robot_glitch": "error",
            "robot_alert": "error",
            "robot_power_up": "thinking",
            "robot_bubbles": None,
            "robot_fantasy": None,
            "robot_flyby": None,
            "robot_impact": None,
            "robot_liquid": None,
            "robot_loop": "thinking",
            "robot_stinger": None,
            "robot_stun": None,
            "robot_talk_beep_1": "talking",
            "robot_talk_beep_2": "talking",
            "robot_terminal": "thinking",
            "robot_whoosh": None,
            "robot_work_1": "thinking",
            "robot_work_2": "thinking",
            "robot_work_3": "thinking",
        }

        animation = animation_map.get(trigger, trigger)

        # Skip animation trigger if explicitly set to None
        if animation is None:
            return

        try:
            msg = String()
            msg.data = animation
            self.animation_pub.publish(msg)
            self.get_logger().debug(f"🎬 Триггер анимации: {animation}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Ошибка триггера анимации: {e}")

    def publish_state(self, state: str):
        """Публикация состояния ноды."""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def parameters_callback(self, params):
        """Callback для изменения параметров во время работы."""
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == "volume_db":
                old_volume = self.volume_db
                self.volume_db = param.value
                self.get_logger().info(f"🔊 Громкость звуков изменена: {old_volume:.1f} → {self.volume_db:.1f} dB")
                # Перезагружаем звуки с новой громкостью
                self.load_sounds()

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = SoundNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
