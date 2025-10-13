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
import random
import threading
from typing import Dict, List, Optional
from pydub import AudioSegment
from pydub.playback import play
import io


class SoundNode(Node):
    """Нода для воспроизведения звуковых эффектов"""
    
    def __init__(self):
        super().__init__('sound_node')
        
        # Параметры
        self.declare_parameter('sound_pack_dir', '~/rob_box_project/sound_pack')
        self.declare_parameter('volume_db', 0)  # Регулировка громкости (dB)
        self.declare_parameter('trigger_animations', True)
        self.declare_parameter('animation_topic', '/animations/trigger')
        
        sound_pack_dir = self.get_parameter('sound_pack_dir').value
        self.sound_pack_dir = os.path.expanduser(sound_pack_dir)
        self.volume_db = self.get_parameter('volume_db').value
        self.trigger_animations = self.get_parameter('trigger_animations').value
        self.animation_topic = self.get_parameter('animation_topic').value
        
        # Subscribers
        self.trigger_sub = self.create_subscription(
            String,
            '/voice/sound/trigger',
            self.trigger_callback,
            10
        )
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/voice/sound/state', 10)
        
        # Опционально: триггер анимаций
        if self.trigger_animations:
            self.animation_pub = self.create_publisher(
                String,
                self.animation_topic,
                10
            )
        
        # Хранилище звуков
        self.sounds: Dict[str, AudioSegment] = {}
        self.sound_groups: Dict[str, List[str]] = {
            'talk': ['talk_1', 'talk_2', 'talk_3', 'talk_4'],
            'angry': ['angry_1', 'angry_2'],
            'cute': ['cute', 'very_cute']
        }
        
        # Состояние
        self.is_playing = False
        self.current_sound: Optional[str] = None
        self.play_thread: Optional[threading.Thread] = None
        
        # Инициализация
        self.get_logger().info('SoundNode инициализирован')
        self.load_sounds()
    
    def load_sounds(self):
        """Загрузка всех звуковых файлов из sound_pack/"""
        if not os.path.exists(self.sound_pack_dir):
            self.get_logger().error(f'❌ Директория sound_pack не найдена: {self.sound_pack_dir}')
            self.publish_state('error_no_dir')
            return
        
        self.get_logger().info(f'📂 Загрузка звуков из {self.sound_pack_dir}...')
        
        loaded_count = 0
        for filename in os.listdir(self.sound_pack_dir):
            if filename.endswith('.mp3'):
                sound_name = filename.replace('.mp3', '')
                filepath = os.path.join(self.sound_pack_dir, filename)
                
                try:
                    # Загрузить MP3
                    audio = AudioSegment.from_mp3(filepath)
                    
                    # Применить регулировку громкости
                    if self.volume_db != 0:
                        audio = audio + self.volume_db
                    
                    self.sounds[sound_name] = audio
                    loaded_count += 1
                    self.get_logger().info(f'  ✓ {sound_name}: {len(audio)}ms, {audio.frame_rate}Hz')
                    
                except Exception as e:
                    self.get_logger().error(f'  ❌ Ошибка загрузки {filename}: {e}')
        
        self.get_logger().info(f'✅ Загружено звуков: {loaded_count}/{len(os.listdir(self.sound_pack_dir))}')
        
        if loaded_count > 0:
            self.publish_state('ready')
        else:
            self.publish_state('error_no_sounds')
    
    def trigger_callback(self, msg: String):
        """Обработка триггера звукового эффекта"""
        trigger = msg.data.strip()
        
        self.get_logger().info(f'🔔 Триггер: {trigger}')
        
        # Проверить, не играет ли уже звук
        if self.is_playing:
            self.get_logger().warn(f'⚠️ Звук уже играет ({self.current_sound}), пропускаю {trigger}')
            return
        
        # Выбрать звук
        sound_name = self.select_sound(trigger)
        
        if sound_name is None:
            self.get_logger().warn(f'⚠️ Звук для триггера "{trigger}" не найден')
            return
        
        # Запустить воспроизведение в отдельном потоке
        self.play_thread = threading.Thread(
            target=self.play_sound_thread,
            args=(sound_name, trigger),
            daemon=True
        )
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
        self.publish_state(f'playing_{sound_name}')
        
        try:
            self.get_logger().info(f'▶️ Играю: {sound_name}')
            
            # Триггер анимации (если включено)
            if self.trigger_animations:
                self.trigger_animation(trigger)
            
            # Воспроизведение
            audio = self.sounds[sound_name]
            play(audio)
            
            self.get_logger().info(f'✅ Завершено: {sound_name}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка воспроизведения {sound_name}: {e}')
        
        finally:
            self.is_playing = False
            self.current_sound = None
            self.publish_state('ready')
    
    def trigger_animation(self, trigger: str):
        """Триггер соответствующей анимации"""
        # Маппинг звуков на анимации
        animation_map = {
            'thinking': 'thinking',
            'surprise': 'surprise',
            'confused': 'confused',
            'angry': 'angry',
            'angry_1': 'angry',
            'angry_2': 'angry',
            'cute': 'happy',
            'very_cute': 'very_happy',
            'talk': 'talking'
        }
        
        animation = animation_map.get(trigger, trigger)
        
        try:
            msg = String()
            msg.data = animation
            self.animation_pub.publish(msg)
            self.get_logger().debug(f'🎬 Триггер анимации: {animation}')
        except Exception as e:
            self.get_logger().warn(f'⚠️ Ошибка триггера анимации: {e}')
    
    def publish_state(self, state: str):
        """Публикация состояния ноды"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)


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


if __name__ == '__main__':
    main()
