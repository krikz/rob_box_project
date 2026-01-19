#!/usr/bin/env python3
"""
STTNode - Speech-to-Text с Yandex STT gRPC v3 (primary) + Vosk (fallback)
Подписывается: /audio/speech_audio (AudioData)
Публикует: /voice/stt/result (String)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

import json
import os
from typing import Optional
from vosk import Model, KaldiRecognizer
import grpc
import numpy as np

# Yandex Cloud STT API v3 (gRPC)
try:
    from yandex.cloud.ai.stt.v3 import stt_pb2, stt_service_pb2_grpc
    YANDEX_GRPC_AVAILABLE = True
except ImportError:
    YANDEX_GRPC_AVAILABLE = False
    print("⚠️  yandex-cloud-ml-sdk не установлен! Используем только Vosk.")


class STTNode(Node):
    """Нода для распознавания речи: Yandex STT gRPC v3 (primary) + Vosk (fallback)"""
    
    def __init__(self):
        super().__init__('stt_node')
        
        # Параметры Vosk (fallback)
        self.declare_parameter('model_path', '/models/vosk-model-small-ru-0.22')
        self.declare_parameter('sample_rate', 16000)
        
        self.model_path = self.get_parameter('model_path').value
        self.sample_rate = self.get_parameter('sample_rate').value
        
        # Параметры Yandex STT (primary)
        self.declare_parameter('yandex_api_key', '')
        self.declare_parameter('yandex_language', 'ru-RU')
        self.declare_parameter('yandex_model', 'general')
        
        self.yandex_api_key = self.get_parameter('yandex_api_key').value or os.environ.get('YANDEX_API_KEY', '')
        self.yandex_language = self.get_parameter('yandex_language').value
        self.yandex_model = self.get_parameter('yandex_model').value
        
        # Yandex gRPC клиент
        self.yandex_channel = None
        self.yandex_stub = None
        
        # QoS для аудио потока
        audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscriber - слушаем только speech_audio (уже готовые фразы)
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio/speech_audio',
            self.speech_audio_callback,
            audio_qos
        )
        
        # Подписка на состояние TTS (чтобы не слышать себя)
        self.tts_state_sub = self.create_subscription(
            String,
            '/voice/tts/state',
            self.tts_state_callback,
            10
        )
        
        # Publishers
        self.result_pub = self.create_publisher(String, '/voice/stt/result', 10)
        self.state_pub = self.create_publisher(String, '/voice/stt/state', 10)
        self.tts_control_pub = self.create_publisher(String, '/voice/tts/control', 10)  # Для прерывания TTS
        
        # Vosk модель и распознаватель
        self.model: Optional[Model] = None
        self.recognizer: Optional[KaldiRecognizer] = None
        
        # Состояние
        self.is_robot_speaking = False  # Флаг: робот говорит (не слушать!)
        
        # Инициализация
        self.get_logger().info('STTNode инициализирован')
        self.initialize_yandex()
        self.initialize_vosk()
    
    def initialize_yandex(self):
        """Инициализация Yandex STT gRPC v3"""
        if not YANDEX_GRPC_AVAILABLE:
            self.get_logger().warn('⚠️  Yandex Cloud ML SDK недоступен, используем только Vosk')
            return
        
        if not self.yandex_api_key:
            self.get_logger().warn('⚠️  YANDEX_API_KEY не задан, используем только Vosk')
            return
        
        try:
            self.get_logger().info('🔌 Подключение к Yandex STT gRPC v3...')
            self.yandex_channel = grpc.secure_channel(
                'stt.api.cloud.yandex.net:443',
                grpc.ssl_channel_credentials()
            )
            self.yandex_stub = stt_service_pb2_grpc.RecognizerStub(self.yandex_channel)
            self.get_logger().info(f'✅ Yandex STT gRPC v3 инициализирован (язык: {self.yandex_language})')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка инициализации Yandex STT: {e}')
            self.yandex_stub = None
    
    def initialize_vosk(self):
        """Загрузка Vosk модели (fallback)"""
        try:
            self.get_logger().info(f'Загрузка Vosk модели из {self.model_path}...')
            self.model = Model(self.model_path)
            self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
            self.recognizer.SetWords(True)  # Получать разметку по словам
            self.get_logger().info('✅ Vosk модель загружена (fallback)')
            self.publish_state('ready')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка загрузки Vosk: {e}')
            self.publish_state('error')
    
    def tts_state_callback(self, msg: String):
        """Отслеживание состояния TTS - не слушать когда робот говорит!"""
        if msg.data in ['synthesizing', 'playing']:
            if not self.is_robot_speaking:
                self.get_logger().info('🔇 Робот говорит - распознавание отключено')
                self.is_robot_speaking = True
        elif msg.data in ['ready', 'idle']:
            if self.is_robot_speaking:
                self.get_logger().info('🎙️ Робот замолчал - распознавание включено')
                self.is_robot_speaking = False
    
    def speech_audio_callback(self, msg: AudioData):
        """
        Обработка готовых фраз от audio_node
        Пробуем Yandex STT → если не работает, используем Vosk
        """
        # НЕ СЛУШАТЬ когда робот говорит (самовозбуждение!)
        if self.is_robot_speaking:
            self.get_logger().info('🔇 Игнор: робот говорит')
            return
        
        # Конвертируем список в bytes
        audio_bytes = bytes(msg.data)
        duration = len(audio_bytes) / (self.sample_rate * 2)  # 16-bit = 2 bytes
        
        self.get_logger().info(f'🎤 Получена фраза: {duration:.2f}с ({len(audio_bytes)} bytes)')
        self.publish_state('recognizing')
        
        text = None
        
        # 1. Попытка Yandex STT (primary)
        if self.yandex_stub:
            try:
                text = self._recognize_yandex(audio_bytes)
                if text:
                    self.get_logger().info(f'✅ Yandex STT: "{text}"')
            except Exception as e:
                self.get_logger().error(f'⚠️  Yandex STT ошибка: {e}, fallback на Vosk')
        
        # 2. Fallback на Vosk если Yandex не сработал
        if not text and self.recognizer:
            text = self._recognize_vosk(audio_bytes)
            if text:
                self.get_logger().info(f'✅ Vosk (fallback): "{text}"')
        
        # Публикация результата
        if text and len(text) >= 3:
            self.get_logger().info(f'✅ ПРИНЯТО: {text}')
            self.publish_result(text)
            self.publish_state('ready')
        elif text:
            self.get_logger().warn(f'❌ ОТКЛОНЕНО (короткое): "{text}"')
            self.publish_state('ready')
        else:
            self.get_logger().warn(f'❌ ОТКЛОНЕНО (пустое)')
            self.publish_state('ready')
    
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
                            audio_channel_count=1
                        )
                    ),
                    text_normalization=stt_pb2.TextNormalizationOptions(
                        text_normalization=stt_pb2.TextNormalizationOptions.TEXT_NORMALIZATION_ENABLED,
                        profanity_filter=False,
                        literature_text=False
                    ),
                    language_restriction=stt_pb2.LanguageRestrictionOptions(
                        restriction_type=stt_pb2.LanguageRestrictionOptions.WHITELIST,
                        language_code=[self.yandex_language]
                    ),
                    audio_processing_type=stt_pb2.RecognitionModelOptions.REAL_TIME
                ),
                # ВАЖНО! Настройка EOU (End of Utterance) - определение конца фразы
                # Увеличиваем max_pause_between_words_hint_ms чтобы робот не обрывал речь
                # когда пользователь медленно говорит или делает паузы между словами
                eou_classifier=stt_pb2.EouClassifierOptions(
                    default_classifier=stt_pb2.DefaultEouClassifier(
                        type=stt_pb2.DefaultEouClassifier.DEFAULT,  # Консервативный (DEFAULT) vs быстрый (HIGH)
                        max_pause_between_words_hint_ms=1200  # 1.2 сек паузы между словами (default ~700ms)
                    )
                )
            )
            yield stt_pb2.StreamingRequest(session_options=recognize_options)
            
            # 2. Отправляем аудио данные чанками по 4096 байт
            chunk_size = 4096
            for i in range(0, len(audio_bytes), chunk_size):
                chunk = audio_bytes[i:i + chunk_size]
                yield stt_pb2.StreamingRequest(chunk=stt_pb2.AudioChunk(data=chunk))
        
        # Выполняем streaming запрос
        responses = self.yandex_stub.RecognizeStreaming(
            gen(),
            metadata=(('authorization', f'Api-Key {self.yandex_api_key}'),)
        )
        
        # Обрабатываем ответы
        final_text = None
        for response in responses:
            event_type = response.WhichOneof('Event')
            
            # partial - промежуточные результаты (игнорируем)
            if event_type == 'partial':
                continue
            
            # final - финальный результат распознавания
            elif event_type == 'final':
                if response.final.alternatives:
                    final_text = response.final.alternatives[0].text
                    # Продолжаем читать для возможного final_refinement
            
            # final_refinement - улучшенный результат с нормализацией
            elif event_type == 'final_refinement':
                if response.final_refinement.normalized_text:
                    final_text = response.final_refinement.normalized_text.alternatives[0].text
                    break  # Это последний результат
        
        return final_text.strip() if final_text else None
    
    def _recognize_vosk(self, audio_bytes: bytes) -> Optional[str]:
        """Распознавание через Vosk (fallback)"""
        # Кормим Vosk по кусочкам, как Yandex (4KB chunks)
        # Это важно! Vosk работает в streaming режиме и не может обработать всю фразу сразу
        chunk_size = 4096
        
        for i in range(0, len(audio_bytes), chunk_size):
            chunk = audio_bytes[i:i + chunk_size]
            self.recognizer.AcceptWaveform(chunk)
        
        # После всех чанков получаем финальный результат
        result = json.loads(self.recognizer.FinalResult())
        text = result.get('text', '').strip()
        
        # Сбросить распознаватель для следующей фразы
        self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
        self.recognizer.SetWords(True)
        
        return text
    
    def publish_result(self, text: str):
        """Публикация финального результата распознавания"""
        # Проверяем, обращается ли пользователь к роботу
        text_lower = text.lower()
        wake_words = ['робот', 'робик', 'роб', 'робо']
        
        # Если фраза начинается с обращения - прерываем TTS
        if any(text_lower.startswith(word) for word in wake_words):
            self.get_logger().info(f'🎯 Обнаружено обращение к роботу: "{text[:20]}..."')
            # Отправляем STOP на TTS
            stop_msg = String()
            stop_msg.data = "STOP"
            self.tts_control_pub.publish(stop_msg)
            self.get_logger().info('🔇 Отправлен STOP на TTS (пользователь обратился к роботу)')
        
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)
        self.get_logger().info(f'📤 Опубликовал результат: {text}')
    
    def publish_state(self, state: str):
        """Публикация состояния ноды"""
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


if __name__ == '__main__':
    main()
