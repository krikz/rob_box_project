#!/usr/bin/env python3
"""
TTSNode - Text-to-Speech с YandexSpeechKit + Silero fallback

Подписывается на: /voice/dialogue/response (JSON chunks)
Публикует: /voice/audio/speech (AudioData)
Использует: 
  - YandexSpeechKit (primary, онлайн, лучшее качество)
  - Silero TTS v4 (fallback, офлайн, всегда работает)
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
import requests
from io import BytesIO


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
    """ROS2 нода для синтеза речи с YandexSpeechKit + Silero fallback"""
    
    def __init__(self):
        super().__init__('tts_node')
        
        # Параметры
        self.declare_parameter('provider', 'yandex')  # yandex (primary) | silero (fallback)
        
        # Yandex SpeechKit
        self.declare_parameter('yandex_folder_id', '')
        self.declare_parameter('yandex_api_key', '')
        self.declare_parameter('yandex_voice', 'alena')  # anton, alena, oksana, jane
        self.declare_parameter('yandex_emotion', 'good')  # neutral, good, evil
        self.declare_parameter('yandex_speed', 0.9)  # 0.1-3.0
        
        # Silero TTS (fallback)
        self.declare_parameter('silero_speaker', 'baya')  # aidar (male) | baya (female) | kseniya | xenia
        self.declare_parameter('silero_sample_rate', 24000)
        
        # Общие параметры
        self.declare_parameter('chipmunk_mode', True)
        self.declare_parameter('pitch_shift', 2.0)  # Множитель для playback rate (2.0x = быстро)
        self.declare_parameter('normalize_text', True)
        self.declare_parameter('volume_db', -3.0)  # Громкость в dB (-3dB = 70%)
        
        # Читаем параметры
        self.provider = self.get_parameter('provider').value
        
        # Yandex
        self.yandex_folder_id = self.get_parameter('yandex_folder_id').value or os.getenv('YANDEX_FOLDER_ID', '')
        self.yandex_api_key = self.get_parameter('yandex_api_key').value or os.getenv('YANDEX_API_KEY', '')
        self.yandex_voice = self.get_parameter('yandex_voice').value
        self.yandex_emotion = self.get_parameter('yandex_emotion').value
        self.yandex_speed = self.get_parameter('yandex_speed').value
        
        # Silero
        self.silero_speaker = self.get_parameter('silero_speaker').value
        self.silero_sample_rate = self.get_parameter('silero_sample_rate').value
        
        # Общие
        self.chipmunk_mode = self.get_parameter('chipmunk_mode').value
        self.pitch_shift = self.get_parameter('pitch_shift').value
        self.normalize_text = self.get_parameter('normalize_text').value
        self.volume_db = self.get_parameter('volume_db').value
        
        # Конвертируем dB в линейный множитель
        self.volume_gain = 10.0 ** (self.volume_db / 20.0)
        
        # Callback для изменения параметров во время работы
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Загрузка Silero TTS (fallback, всегда готов)
        self.get_logger().info('🔄 Загрузка Silero TTS v4 (fallback)...')
        self.device = torch.device('cpu')
        
        # ⚡ КРИТИЧНЫЕ НАСТРОЙКИ ДЛЯ ARM64! ⚡
        torch.set_num_threads(4)
        torch._C._jit_set_profiling_mode(False)
        torch.set_grad_enabled(False)
        
        try:
            self.silero_model, _ = torch.hub.load(
                repo_or_dir='snakers4/silero-models',
                model='silero_tts',
                language='ru',
                speaker='v4_ru'
            )
            self.silero_model.to(self.device)
            self.get_logger().info('✅ Silero TTS загружен (fallback, ARM64 оптимизация)')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка загрузки Silero: {e}')
            self.silero_model = None
        
        # Yandex SpeechKit URL
        self.yandex_url = 'https://tts.api.cloud.yandex.net/speech/v1/tts:synthesize'
        
        # Yandex SpeechKit URL
        self.yandex_url = 'https://tts.api.cloud.yandex.net/speech/v1/tts:synthesize'
        
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
        self.get_logger().info(f'  Provider: Yandex (primary) + Silero (fallback)')
        self.get_logger().info(f'  Yandex: voice={self.yandex_voice}, emotion={self.yandex_emotion}, speed={self.yandex_speed}')
        self.get_logger().info(f'  Silero: speaker={self.silero_speaker}, rate={self.silero_sample_rate} Hz')
        self.get_logger().info(f'  Volume: {self.volume_db:.1f} dB (gain: {self.volume_gain:.2f}x)')
        self.get_logger().info(f'  Chipmunk mode: {self.chipmunk_mode}')
        if self.chipmunk_mode:
            self.get_logger().info(f'  Pitch shift: {self.pitch_shift}x (ускоряем воспроизведение)')
        
        if not self.yandex_api_key or not self.yandex_folder_id:
            self.get_logger().warn('⚠️  Yandex credentials не заданы - будет использован только Silero fallback')
    
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
        
        try:
            # Сначала пробуем Yandex
            audio_np = None
            sample_rate = 16000  # Yandex возвращает 16kHz
            
            if self.yandex_api_key and self.yandex_folder_id:
                try:
                    self.publish_state('synthesizing')
                    self.get_logger().info('🔊 Синтез через Yandex SpeechKit...')
                    audio_np = self._synthesize_yandex(text)
                    sample_rate = 16000  # Yandex возвращает 16kHz
                    self.get_logger().info(f'✅ Yandex синтез успешен: {len(audio_np)} samples @ {sample_rate} Hz')
                except Exception as e:
                    self.get_logger().warn(f'⚠️  Yandex отвалился: {e}, переключаюсь на Silero fallback')
                    audio_np = None
            
            # Fallback на Silero если Yandex не сработал
            if audio_np is None:
                if self.silero_model is None:
                    raise Exception('Silero fallback недоступен!')
                
                self.publish_state('synthesizing')
                self.get_logger().info('🔊 Синтез через Silero (fallback)...')
                
                # Оборачиваем в SSML для Silero
                if not ssml.startswith('<speak>'):
                    ssml_text = f'<speak><prosody pitch="medium">{text}</prosody></speak>'
                else:
                    ssml_text = ssml
                
                audio = self.silero_model.apply_tts(
                    ssml_text=ssml_text,
                    speaker=self.silero_speaker,
                    sample_rate=self.silero_sample_rate
                )
                audio_np = audio.numpy()
                sample_rate = self.silero_sample_rate  # 24000 Hz
                self.get_logger().info(f'✅ Silero fallback успешен: {len(audio_np)} samples @ {sample_rate} Hz')
            
            # Публикуем в ROS topic
            self._publish_audio(audio_np)
            
            # Воспроизводим локально
            self.publish_state('playing')
            
            # ВАЖНО: ReSpeaker поддерживает ТОЛЬКО 16kHz стерео!
            target_rate = 16000
            
            if self.chipmunk_mode:
                # Chipmunk: ускоряем playback
                playback_rate = int(sample_rate * self.pitch_shift)
                self.get_logger().info(f'🐿️  Бурундук режим: {self.pitch_shift}x → {playback_rate} Hz (промежуточный)')
                
                # Интерполяция до промежуточного playback_rate
                num_samples_intermediate = int(len(audio_np) * playback_rate / sample_rate)
                audio_intermediate = np.interp(
                    np.linspace(0, len(audio_np) - 1, num_samples_intermediate),
                    np.arange(len(audio_np)),
                    audio_np
                )
                
                # Интерполяция до 16kHz для ReSpeaker
                num_samples_final = int(len(audio_intermediate) * target_rate / playback_rate)
                audio_resampled = np.interp(
                    np.linspace(0, len(audio_intermediate) - 1, num_samples_final),
                    np.arange(len(audio_intermediate)),
                    audio_intermediate
                )
                self.get_logger().info(f'🔄 Ресемплинг: {playback_rate} Hz → {target_rate} Hz (ReSpeaker)')
            else:
                # Обычный режим: простой ресемплинг до 16kHz
                num_samples = int(len(audio_np) * target_rate / sample_rate)
                audio_resampled = np.interp(
                    np.linspace(0, len(audio_np) - 1, num_samples),
                    np.arange(len(audio_np)),
                    audio_np
                )
                self.get_logger().info(f'🔄 Ресемплинг: {sample_rate} Hz → {target_rate} Hz (ReSpeaker)')
            
            # Применяем громкость
            audio_np_adjusted = audio_resampled * self.volume_gain
            
            # Конвертируем моно → стерео (ReSpeaker требует 2 канала!)
            audio_stereo = np.column_stack((audio_np_adjusted, audio_np_adjusted))
            self.get_logger().info(f'🔊 Воспроизведение: {len(audio_stereo)} frames, {target_rate} Hz, стерео')
            
            # Блокирующее воспроизведение
            with ignore_stderr(enable=True):
                sd.play(audio_stereo, target_rate, device=1, blocking=True)
                sd.stop()
                sd.wait()
            
            # Закончили воспроизведение
            self.publish_state('ready')
            self.get_logger().info('✅ Воспроизведение завершено')
            
        except Exception as e:
            self.get_logger().error(f'❌ Synthesis error: {e}')
            self.publish_state('ready')
    
    def _synthesize_yandex(self, text: str) -> np.ndarray:
        """Синтез через Yandex SpeechKit"""
        headers = {
            'Authorization': f'Api-Key {self.yandex_api_key}'
        }
        
        data = {
            'text': text,
            'lang': 'ru-RU',
            'voice': self.yandex_voice,
            'emotion': self.yandex_emotion,
            'speed': self.yandex_speed,
            'format': 'lpcm',
            'sampleRateHertz': '16000',
            'folderId': self.yandex_folder_id
        }
        
        response = requests.post(self.yandex_url, headers=headers, data=data, timeout=10)
        
        if response.status_code != 200:
            raise Exception(f'Yandex API error: {response.status_code} - {response.text}')
        
        # Декодируем LPCM (16-bit signed PCM)
        audio_bytes = response.content
        audio_np = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32) / 32768.0
        
        return audio_np
    
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
    
    def parameters_callback(self, params):
        """Callback для изменения параметров во время работы"""
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name == 'volume_db':
                self.volume_db = param.value
                self.volume_gain = 10.0 ** (self.volume_db / 20.0)
                self.get_logger().info(f'🔊 Громкость изменена: {self.volume_db:.1f} dB (gain: {self.volume_gain:.2f}x)')
            elif param.name == 'pitch_shift':
                self.pitch_shift = param.value
                self.get_logger().info(f'🐿️ Pitch shift изменён: {self.pitch_shift}x')
            elif param.name == 'chipmunk_mode':
                self.chipmunk_mode = param.value
                self.get_logger().info(f'🐿️ Chipmunk mode: {self.chipmunk_mode}')
        
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


if __name__ == '__main__':
    main()
