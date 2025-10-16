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
import os
from pathlib import Path
from contextlib import contextmanager


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
        self.declare_parameter('pitch_shift', 1.5)  # Множитель для playback rate (1.5x = умеренно быстро)
        # prosody_rate НЕ используем - генерируем нормальную скорость, ускоряем при воспроизведении
        self.declare_parameter('prosody_pitch', 'medium')  # x-low, low, medium, high, x-high
        self.declare_parameter('normalize_text', True)
        self.declare_parameter('volume_db', -3.0)  # Громкость в dB (-3dB = 70%, компромисс с шумом)
        
        self.speaker = self.get_parameter('speaker').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chipmunk_mode = self.get_parameter('chipmunk_mode').value
        self.pitch_shift = self.get_parameter('pitch_shift').value
        # prosody_rate не используем - ускоряем при воспроизведении, а не в TTS
        self.prosody_pitch = self.get_parameter('prosody_pitch').value
        self.normalize_text = self.get_parameter('normalize_text').value
        self.volume_db = self.get_parameter('volume_db').value
        
        # Конвертируем dB в линейный множитель
        # dB = 20 * log10(gain)  =>  gain = 10^(dB/20)
        self.volume_gain = 10.0 ** (self.volume_db / 20.0)
        
        # Загрузка Silero TTS
        self.get_logger().info('🔄 Загрузка Silero TTS v4...')
        self.device = torch.device('cpu')
        
        # ⚡ КРИТИЧНЫЕ НАСТРОЙКИ ДЛЯ ARM64! ⚡
        # Из статьи: https://habr.com/ru/companies/timeweb/articles/817929/
        torch.set_num_threads(4)  # Ограничение потоков для ARM64
        torch._C._jit_set_profiling_mode(False)  # Отключить JIT профилирование
        torch.set_grad_enabled(False)  # Отключить градиенты (только inference)
        
        try:
            self.model, _ = torch.hub.load(
                repo_or_dir='snakers4/silero-models',
                model='silero_tts',
                language='ru',
                speaker='v4_ru'
            )
            self.model.to(self.device)
            self.get_logger().info('✅ Silero TTS загружен (ARM64 оптимизация)')
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
        self.get_logger().info(f'  Volume: {self.volume_db:.1f} dB (gain: {self.volume_gain:.2f}x)')
        self.get_logger().info(f'  Chipmunk mode: {self.chipmunk_mode}')
        if self.chipmunk_mode:
            self.get_logger().info(f'  Pitch shift: {self.pitch_shift}x (ускоряем воспроизведение)')
            self.get_logger().info(f'  Prosody: pitch={self.prosody_pitch} (rate НЕ используется!)')
    
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
        
        # Оборачиваем в SSML (БЕЗ rate - ускоряем при воспроизведении!)
        if not ssml.startswith('<speak>'):
            ssml_text = f'<speak><prosody pitch="{self.prosody_pitch}">{text}</prosody></speak>'
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
            self.get_logger().info(f'✅ Синтез успешен: {len(audio_np)} samples @ {self.sample_rate} Hz')
            
            # Публикуем в ROS topic
            self._publish_audio(audio_np)
            
            # Воспроизводим локально (для тестирования)
            self.publish_state('playing')
            
            # ВАЖНО: ReSpeaker поддерживает ТОЛЬКО 16kHz стерео!
            # Ресемплинг для chipmunk mode или для ReSpeaker
            target_rate = 16000  # ReSpeaker требует 16kHz
            
            if self.chipmunk_mode:
                # Chipmunk: генерируем 24kHz, ускоряем playback, затем ресемплим до 16kHz
                playback_rate = int(self.sample_rate * self.pitch_shift)
                self.get_logger().info(f'🐿️  Бурундук режим: {self.pitch_shift}x → {playback_rate} Hz (промежуточный)')
                # Сначала интерполяция до промежуточного playback_rate
                num_samples_intermediate = int(len(audio_np) * playback_rate / self.sample_rate)
                audio_intermediate = np.interp(
                    np.linspace(0, len(audio_np) - 1, num_samples_intermediate),
                    np.arange(len(audio_np)),
                    audio_np
                )
                # Затем интерполяция до 16kHz для ReSpeaker
                num_samples_final = int(len(audio_intermediate) * target_rate / playback_rate)
                audio_resampled = np.interp(
                    np.linspace(0, len(audio_intermediate) - 1, num_samples_final),
                    np.arange(len(audio_intermediate)),
                    audio_intermediate
                )
                self.get_logger().info(f'🔄 Ресемплинг: {playback_rate} Hz → {target_rate} Hz (ReSpeaker)')
            else:
                # Обычный режим: простой ресемплинг с 24kHz на 16kHz
                num_samples = int(len(audio_np) * target_rate / self.sample_rate)
                audio_resampled = np.interp(
                    np.linspace(0, len(audio_np) - 1, num_samples),
                    np.arange(len(audio_np)),
                    audio_np
                )
                self.get_logger().info(f'🔄 Ресемплинг: {self.sample_rate} Hz → {target_rate} Hz (ReSpeaker)')
            
            # Применяем громкость из параметра (dB → линейный множитель)
            audio_np_adjusted = audio_resampled * self.volume_gain
            
            # Конвертируем моно → стерео (ReSpeaker требует 2 канала!)
            audio_stereo = np.column_stack((audio_np_adjusted, audio_np_adjusted))
            self.get_logger().info(f'🔊 Воспроизведение: {len(audio_stereo)} frames, {playback_rate} Hz, стерео')
            
            # Блокирующее воспроизведение (с подавлением ALSA ошибок)
            with ignore_stderr(enable=True):
                sd.play(audio_stereo, playback_rate, device=1, blocking=True)
                
                # Принудительно останавливаем устройство и очищаем буферы
                sd.stop()
                sd.wait()
            
            # Закончили воспроизведение
            self.publish_state('ready')
            self.get_logger().info('✅ Воспроизведение завершено')
            
        except Exception as e:
            self.get_logger().error(f'❌ Synthesis error: {e}')
            # ВАЖНО: сбрасываем флаг is_speaking при ошибке!
            self.publish_state('ready')
    
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
