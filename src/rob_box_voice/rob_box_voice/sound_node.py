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
    """Подавить ALSA ошибки от PyAudio (как в audio_node)"""
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
    """Нода для воспроизведения звуковых эффектов"""

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
        self.sounds: Dict[str, AudioSegment] = {}
        self.sound_groups: Dict[str, List[str]] = {
            "talk": ["talk_1", "talk_2", "talk_3", "talk_4"],
            "cute": ["cute", "very_cute"],
        }

        # Состояние
        self.is_playing = False
        self.current_sound: Optional[str] = None
        self.play_thread: Optional[threading.Thread] = None

        # Инициализация
        self.get_logger().info("SoundNode инициализирован")
        self.load_sounds()

    def initialize_audio_device(self):
        """Инициализация аудио устройства для воспроизведения"""
        try:
            # Поиск ReSpeaker устройства
            self.device_index = find_respeaker_device_sounddevice()
            
            if self.device_index is not None:
                devices = sd.query_devices()
                device_name = devices[self.device_index]['name']
                self.get_logger().info(f"✅ ReSpeaker найден для playback: device {self.device_index} ({device_name})")
            else:
                # Fallback на default device
                self.get_logger().warn("⚠️ ReSpeaker не найден, используем default device")
                self.device_index = None  # None означает default device в sounddevice
                
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка инициализации аудио устройства: {e}")
            self.device_index = None

    def load_sounds(self):
        """Загрузка всех звуковых файлов из sound_pack/"""
        if not os.path.exists(self.sound_pack_dir):
            self.get_logger().error(f"❌ Директория sound_pack не найдена: {self.sound_pack_dir}")
            self.publish_state("error_no_dir")
            return

        self.get_logger().info(f"📂 Загрузка звуков из {self.sound_pack_dir}...")

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
        """Обработка триггера звукового эффекта"""
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
        """Выбрать звук по триггеру"""
        # Прямое совпадение
        if trigger in self.sounds:
            return trigger

        # Проверить группы (например, trigger="talk" → random из talk_1..4)
        if trigger in self.sound_groups:
            available = [name for name in self.sound_groups[trigger] if name in self.sounds]
            if available:
                return random.choice(available)

        # Попробовать найти похожий
        for sound_name in self.sounds.keys():
            if trigger.lower() in sound_name.lower():
                return sound_name

        return None

    def play_sound_thread(self, sound_name: str, trigger: str):
        """Воспроизведение звука в отдельном потоке"""
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
        """Триггер соответствующей анимации"""
        # Маппинг звуков на анимации
        animation_map = {
            "thinking": "thinking",
            "surprise": "surprise",
            "confused": "confused",
            "angry": "angry",
            "angry_1": "angry",
            "error": "error",  # error → error animation
            "cute": "happy",
            "very_cute": "very_happy",
            "talk": "talking",
        }

        animation = animation_map.get(trigger, trigger)

        try:
            msg = String()
            msg.data = animation
            self.animation_pub.publish(msg)
            self.get_logger().debug(f"🎬 Триггер анимации: {animation}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Ошибка триггера анимации: {e}")

    def publish_state(self, state: str):
        """Публикация состояния ноды"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def parameters_callback(self, params):
        """Callback для изменения параметров во время работы"""
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
