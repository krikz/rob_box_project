#!/usr/bin/env python3
"""
SoundNode - воспроизведение звуковых эффектов
Подписывается: /voice/sound/trigger (String)
Публикует: /voice/sound/state (String)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
import sys
import json
import random
import threading
import time
import numpy as np
import sounddevice as sd
from typing import Dict, List, Optional
from contextlib import contextmanager
from pydub import AudioSegment
from .utils.audio_utils import find_respeaker_device_sounddevice
from .audio_playback_manager import AudioPlaybackManager


@contextmanager
def ignore_stderr(enable=True):
    """Подавить ALSA ошибки от PyAudio (как в audio_node)."""
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


class SoundNode(Node):
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

        # Publishers
        self.state_pub = self.create_publisher(String, "/voice/sound/state", 10)

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
        """Обработка триггера звукового эффекта."""
        trigger = msg.data.strip()

        self.get_logger().info(f"🔔 Триггер: {trigger}")

        # Проверить, не играет ли уже звук
        if self.is_playing:
            self.get_logger().warn(f"⚠️ Звук уже играет ({self.current_sound}), пропускаю {trigger}")
            return

        # Выбрать звук
        sound_name = self.select_sound(trigger)

        if sound_name is None:
            self.get_logger().warn(f'⚠️ Звук для триггера "{trigger}" не найден')
            return

        # Запустить воспроизведение в отдельном потоке
        self.play_thread = threading.Thread(target=self.play_sound_thread, args=(sound_name, trigger), daemon=True)
        self.play_thread.start()

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

    def play_sound_thread(self, sound_name: str, trigger: str):
        """Воспроизведение звука в отдельном потоке."""
        self.is_playing = True
        self.current_sound = sound_name
        self.publish_state(f"playing_{sound_name}")

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
            else:
                self.get_logger().info(f"✅ Завершено: {sound_name}")

            # Cleanup для устранения белого шума после воспроизведения
            self.cleanup_playback_noise()

        except Exception as e:
            self.get_logger().error(f"❌ Ошибка воспроизведения {sound_name}: {e}")

        finally:
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
