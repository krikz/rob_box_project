#!/usr/bin/env python3
"""
TTSNode - Text-to-Speech с Yandex Cloud TTS API v3 (gRPC) + Silero fallback

Подписывается на: /voice/dialogue/response (JSON chunks)
Публикует: /voice/audio/speech (AudioData)
Использует: 
  - Yandex Cloud TTS API v3 (gRPC, primary, anton voice)
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
import grpc
import io
import wave


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

# Импортируем text_normalizer и Yandex gRPC
scripts_path = Path(__file__).parent.parent / 'scripts'
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
        super().__init__('tts_node')
        
        # Параметры
        self.declare_parameter('provider', 'yandex')  # yandex (primary) | silero (fallback)
        
        # Yandex Cloud TTS gRPC v3 (оригинальный ROBBOX голос!)
        self.declare_parameter('yandex_api_key', '')
        self.declare_parameter('yandex_voice', 'anton')  # anton (ОРИГИНАЛЬНЫЙ ГОЛОС РОББОКСА!)
        self.declare_parameter('yandex_speed', 0.4)  # 0.1-3.0 (0.4 = ОРИГИНАЛЬНАЯ СКОРОСТЬ РОББОКСА!)
        
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
        
        # Yandex Cloud TTS gRPC v3
        self.yandex_api_key = self.get_parameter('yandex_api_key').value or os.getenv('YANDEX_API_KEY', '')
        self.yandex_voice = self.get_parameter('yandex_voice').value
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
        
        # Yandex Cloud TTS gRPC v3 (оригинальный ROBBOX голос anton!)
        self.yandex_channel = None
        self.yandex_stub = None
        if YANDEX_GRPC_AVAILABLE and self.yandex_api_key:
            try:
                self.yandex_channel = grpc.secure_channel(
                    'tts.api.cloud.yandex.net:443',
                    grpc.ssl_channel_credentials()
                )
                self.yandex_stub = tts_service_pb2_grpc.SynthesizerStub(self.yandex_channel)
                self.get_logger().info('✅ Yandex Cloud TTS gRPC v3 подключен')
            except Exception as e:
                self.get_logger().warn(f'⚠️  Не удалось подключиться к Yandex gRPC: {e}')
        
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
        self.get_logger().info(f'  Provider: Yandex Cloud TTS gRPC v3 (primary) + Silero (fallback)')
        self.get_logger().info(f'  Yandex gRPC v3: voice={self.yandex_voice} (ROBBOX original!), speed={self.yandex_speed}')
        self.get_logger().info(f'  Silero: speaker={self.silero_speaker}, rate={self.silero_sample_rate} Hz')
        self.get_logger().info(f'  Volume: {self.volume_db:.1f} dB (gain: {self.volume_gain:.2f}x)')
        self.get_logger().info(f'  Chipmunk mode: {self.chipmunk_mode}')
        if self.chipmunk_mode:
            self.get_logger().info(f'  Pitch shift: {self.pitch_shift}x (ускоряем воспроизведение)')
        
        if not self.yandex_stub:
            self.get_logger().warn('⚠️  Yandex gRPC не подключен - будет использован только Silero fallback')
    
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
            
            if self.yandex_stub:  # Проверяем что gRPC канал инициализирован
                try:
                    self.publish_state('synthesizing')
                    self.get_logger().info('🔊 Синтез через Yandex Cloud TTS gRPC v3 (anton)...')
                    audio_np = self._synthesize_yandex(text)
                    sample_rate = 22050  # Yandex обычно возвращает 22050 Hz или 48000 Hz
                    # sample_rate уже получен в _synthesize_yandex, но пока захардкодим
                except Exception as e:
                    self.get_logger().warn(f'⚠️  Yandex gRPC отвалился: {e}, переключаюсь на Silero fallback')
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
        """Синтез через Yandex Cloud TTS gRPC API v3 (anton voice!)"""
        if not self.yandex_stub:
            raise Exception("Yandex gRPC stub не инициализирован")
        
        # Создаём запрос как в оригинальном ROBBOX коде
        request = tts_pb2.UtteranceSynthesisRequest(
            text=text,
            output_audio_spec=tts_pb2.AudioFormatOptions(
                container_audio=tts_pb2.ContainerAudio(
                    container_audio_type=tts_pb2.ContainerAudio.WAV
                )
            ),
            hints=[
                tts_pb2.Hints(voice=self.yandex_voice),  # anton!
                tts_pb2.Hints(speed=self.yandex_speed),  # 0.4
            ],
            loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS
        )
        
        try:
            # Отправляем запрос с авторизацией
            responses = self.yandex_stub.UtteranceSynthesis(
                request,
                metadata=(('authorization', f'Api-Key {self.yandex_api_key}'),)
            )
            
            # Собираем аудио данные из стрима
            audio_data = b""
            for response in responses:
                audio_data += response.audio_chunk.data
            
            if not audio_data:
                raise Exception("Пустой ответ от Yandex TTS")
            
            # Yandex возвращает WAV файл - нужно извлечь PCM данные
            # Пропускаем WAV заголовок (44 байта)
            with io.BytesIO(audio_data) as wav_file:
                with wave.open(wav_file, 'rb') as wav:
                    sample_rate = wav.getframerate()  # обычно 22050 Hz или 48000 Hz
                    audio_bytes = wav.readframes(wav.getnframes())
            
            # Декодируем PCM в numpy
            audio_np = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32) / 32768.0
            
            self.get_logger().info(f'✅ Yandex gRPC v3 синтез успешен: {len(audio_np)} samples @ {sample_rate} Hz')
            
            return audio_np
            
        except grpc.RpcError as e:
            raise Exception(f'Yandex gRPC error: {e.code()} - {e.details()}')
        except Exception as e:
            raise Exception(f'Yandex synthesis error: {e}')
    
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
