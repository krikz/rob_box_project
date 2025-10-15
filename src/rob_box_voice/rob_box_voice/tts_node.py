#!/usr/bin/env python3
"""
TTSNode - Text-to-Speech с Silero TTS v4

Подписывается на: /voice/dialogue/response (JSON chunks)
Публикует: /voice/audio/speech (AudioData)
Использует: Silero TTS v4 (aidar, baya, kseniya, xenia)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import json
import torch
import sounddevice as sd
import numpy as np
from typing import Optional
import sys
from pathlib import Path

# Импортируем text_normalizer
scripts_path = Path(__file__).parent.parent / 'scripts'
sys.path.insert(0, str(scripts_path))

try:
    from text_normalizer import normalize_for_tts
except ImportError:
    def normalize_for_tts(text):
        """Fallback если нет normalizer"""
        return text


class TTSNode(Node):
    """ROS2 нода для синтеза речи с Silero TTS"""
    
    def __init__(self):
        super().__init__('tts_node')
        
        # Параметры
        self.declare_parameter('speaker', 'aidar')  # aidar, baya, kseniya, xenia
        self.declare_parameter('sample_rate', 24000)
        self.declare_parameter('chipmunk_mode', True)
        self.declare_parameter('pitch_shift', 2.0)  # Множитель для playback rate
        self.declare_parameter('prosody_rate', 'x-slow')  # x-slow, slow, medium, fast
        self.declare_parameter('prosody_pitch', 'medium')  # x-low, low, medium, high, x-high
        self.declare_parameter('normalize_text', True)
        
        self.speaker = self.get_parameter('speaker').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chipmunk_mode = self.get_parameter('chipmunk_mode').value
        self.pitch_shift = self.get_parameter('pitch_shift').value
        self.prosody_rate = self.get_parameter('prosody_rate').value
        self.prosody_pitch = self.get_parameter('prosody_pitch').value
        self.normalize_text = self.get_parameter('normalize_text').value
        
        # Загрузка Silero TTS
        self.get_logger().info('🔄 Загрузка Silero TTS v4...')
        self.device = torch.device('cpu')
        
        try:
            self.model, _ = torch.hub.load(
                repo_or_dir='snakers4/silero-models',
                model='silero_tts',
                language='ru',
                speaker='v4_ru'
            )
            self.model.to(self.device)
            self.get_logger().info('✅ Silero TTS загружен')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка загрузки Silero: {e}')
            raise
        
        # Подписка на dialogue response
        self.dialogue_sub = self.create_subscription(
            String,
            '/voice/dialogue/response',
            self.dialogue_callback,
            10
        )
        
        # Публикация аудио и состояния
        self.audio_pub = self.create_publisher(AudioData, '/voice/audio/speech', 10)
        self.state_pub = self.create_publisher(String, '/voice/tts/state', 10)
        
        # Публикуем начальное состояние
        self.publish_state('ready')
        
        self.get_logger().info('✅ TTSNode инициализирован')
        self.get_logger().info(f'  Speaker: {self.speaker}')
        self.get_logger().info(f'  Sample rate: {self.sample_rate} Hz')
        self.get_logger().info(f'  Chipmunk mode: {self.chipmunk_mode}')
        if self.chipmunk_mode:
            self.get_logger().info(f'  Pitch shift: {self.pitch_shift}x')
            self.get_logger().info(f'  Prosody: rate={self.prosody_rate}, pitch={self.prosody_pitch}')
    
    def dialogue_callback(self, msg: String):
        """Обработка JSON chunks от dialogue_node"""
        try:
            chunk_data = json.loads(msg.data)
            
            if 'ssml' not in chunk_data:
                self.get_logger().warn('⚠ Chunk без SSML')
                return
            
            ssml = chunk_data['ssml']
            
            # Извлекаем текст из SSML
            text = self._extract_text_from_ssml(ssml)
            
            if not text.strip():
                return
            
            self.get_logger().info(f'🔊 TTS: {text[:50]}...')
            
            # Синтез и воспроизведение
            self._synthesize_and_play(ssml, text)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ JSON parse error: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ TTS error: {e}')
    
    def _extract_text_from_ssml(self, ssml: str) -> str:
        """Извлекает текст из SSML тегов"""
        import re
        # Убираем все XML теги
        text = re.sub(r'<[^>]+>', '', ssml)
        return text.strip()
    
    def _synthesize_and_play(self, ssml: str, text: str):
        """Синтез речи и воспроизведение"""
        # Нормализация (если включена)
        if self.normalize_text:
            text = normalize_for_tts(text)
        
        # Оборачиваем в SSML с prosody параметрами
        if not ssml.startswith('<speak>'):
            ssml_text = f'<speak><prosody rate="{self.prosody_rate}" pitch="{self.prosody_pitch}">{text}</prosody></speak>'
        else:
            # Уже есть SSML от DeepSeek, используем его
            ssml_text = ssml
        
        try:
            # Синтез
            self.publish_state('synthesizing')
            self.get_logger().info('🔊 Синтезирую речь...')
            
            audio = self.model.apply_tts(
                ssml_text=ssml_text,
                speaker=self.speaker,
                sample_rate=self.sample_rate
            )
            
            # Конвертируем в numpy
            audio_np = audio.numpy()
            
            # Публикуем в ROS topic
            self._publish_audio(audio_np)
            
            # Воспроизводим локально (для тестирования)
            self.publish_state('playing')
            
            if self.chipmunk_mode:
                playback_rate = int(self.sample_rate * self.pitch_shift)
                self.get_logger().info(f'🐿️  Бурундук режим: {self.pitch_shift}x')
            else:
                playback_rate = self.sample_rate
            
            # Снижаем громкость до 24%
            audio_np_quiet = audio_np * 0.24
            
            # Добавляем 200ms тишины в начало и конец чтобы убрать белый шум
            silence_samples = int(playback_rate * 0.2)  # 200ms
            silence = np.zeros(silence_samples, dtype=audio_np_quiet.dtype)
            audio_with_silence = np.concatenate([silence, audio_np_quiet, silence])
            
            # Блокирующее воспроизведение
            sd.play(audio_with_silence, playback_rate, blocking=True)
            
            # Принудительно останавливаем устройство и очищаем буферы
            sd.stop()
            sd.wait()
            
            # Закончили воспроизведение
            self.publish_state('ready')
            self.get_logger().info('✅ Воспроизведение завершено')
            
        except Exception as e:
            self.get_logger().error(f'❌ Synthesis error: {e}')
    
    def _publish_audio(self, audio_np: np.ndarray):
        """Публикует аудио в ROS topic"""
        # Конвертируем в int16 для AudioData
        audio_int16 = (audio_np * 32767).astype(np.int16)
        
        msg = AudioData()
        msg.data = audio_int16.tobytes()
        
        self.audio_pub.publish(msg)
    
    def publish_state(self, state: str):
        """Публикация состояния TTS"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)


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


if __name__ == '__main__':
    main()
