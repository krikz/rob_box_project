#!/usr/bin/env python3
"""
STTNode - Speech-to-Text с Vosk
Подписывается: /audio/audio (AudioData), /audio/vad (Bool)
Публикует: /voice/stt/result (String), /voice/stt/partial (String)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String
from audio_common_msgs.msg import AudioData

import wave
import io
import json
from typing import Optional
from vosk import Model, KaldiRecognizer


class STTNode(Node):
    """Нода для распознавания речи с помощью Vosk"""
    
    def __init__(self):
        super().__init__('stt_node')
        
        # Параметры
        self.declare_parameter('model_path', '/models/vosk-model-small-ru-0.22')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('vad_timeout', 1.5)  # Секунды тишины до отправки результата
        self.declare_parameter('min_speech_duration', 0.5)  # Минимальная длина речи
        
        self.model_path = self.get_parameter('model_path').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.vad_timeout = self.get_parameter('vad_timeout').value
        self.min_speech_duration = self.get_parameter('min_speech_duration').value
        
        # QoS для аудио потока
        audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio/audio',
            self.audio_callback,
            audio_qos
        )
        self.vad_sub = self.create_subscription(
            Bool,
            '/audio/vad',
            self.vad_callback,
            10
        )
        
        # Publishers
        self.result_pub = self.create_publisher(String, '/voice/stt/result', 10)
        self.partial_pub = self.create_publisher(String, '/voice/stt/partial', 10)
        self.state_pub = self.create_publisher(String, '/voice/stt/state', 10)
        
        # Vosk модель и распознаватель
        self.model: Optional[Model] = None
        self.recognizer: Optional[KaldiRecognizer] = None
        
        # Состояние
        self.is_speech_active = False
        self.speech_start_time: Optional[float] = None
        self.silence_start_time: Optional[float] = None
        self.audio_buffer = bytearray()
        
        # Инициализация
        self.get_logger().info('STTNode инициализирован')
        self.initialize_vosk()
    
    def initialize_vosk(self):
        """Загрузка Vosk модели"""
        try:
            self.get_logger().info(f'Загрузка Vosk модели из {self.model_path}...')
            self.model = Model(self.model_path)
            self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
            self.recognizer.SetWords(True)  # Получать разметку по словам
            self.get_logger().info('✓ Vosk модель загружена')
            self.publish_state('ready')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка загрузки Vosk: {e}')
            self.publish_state('error')
    
    def vad_callback(self, msg: Bool):
        """Обработка VAD (Voice Activity Detection)"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if msg.data:  # Речь обнаружена
            if not self.is_speech_active:
                # Начало речи
                self.is_speech_active = True
                self.speech_start_time = current_time
                self.silence_start_time = None
                self.get_logger().info('🎙️ Начало речи')
                self.publish_state('listening')
            else:
                # Продолжение речи - сброс таймера тишины
                self.silence_start_time = None
        else:  # Тишина
            if self.is_speech_active:
                if self.silence_start_time is None:
                    # Начало тишины
                    self.silence_start_time = current_time
                else:
                    # Проверка таймаута тишины
                    silence_duration = current_time - self.silence_start_time
                    if silence_duration >= self.vad_timeout:
                        # Конец речи - отправка результата
                        self.finalize_recognition()
    
    def audio_callback(self, msg: AudioData):
        """Обработка аудио потока"""
        if not self.is_speech_active or self.recognizer is None:
            return
        
        # Добавить аудио в буфер
        self.audio_buffer.extend(msg.data)
        
        # Отправить аудио в Vosk для распознавания
        if self.recognizer.AcceptWaveform(bytes(msg.data)):
            # Финальный результат (конец фразы)
            result = json.loads(self.recognizer.Result())
            text = result.get('text', '').strip()
            if text:
                self.get_logger().info(f'📝 Финальный результат: {text}')
                self.publish_result(text)
        else:
            # Частичный результат (во время речи)
            partial = json.loads(self.recognizer.PartialResult())
            text = partial.get('partial', '').strip()
            if text:
                self.get_logger().debug(f'📝 Частичный: {text}')
                self.publish_partial(text)
    
    def finalize_recognition(self):
        """Завершение распознавания и отправка финального результата"""
        if self.recognizer is None:
            return
        
        # Проверка минимальной длины речи
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.speech_start_time is not None:
            speech_duration = current_time - self.speech_start_time
            if speech_duration < self.min_speech_duration:
                self.get_logger().info(f'⚠️ Речь слишком короткая ({speech_duration:.2f}s), игнорирую')
                self.reset_recognition()
                return
        
        # Получить финальный результат
        final_result = json.loads(self.recognizer.FinalResult())
        text = final_result.get('text', '').strip()
        
        if text:
            self.get_logger().info(f'✅ Финальный результат: "{text}"')
            self.publish_result(text)
        else:
            self.get_logger().info('⚠️ Пустой результат распознавания')
        
        self.reset_recognition()
    
    def reset_recognition(self):
        """Сброс состояния распознавания"""
        self.is_speech_active = False
        self.speech_start_time = None
        self.silence_start_time = None
        self.audio_buffer.clear()
        
        # Сброс распознавателя для новой фразы
        if self.recognizer is not None:
            self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
            self.recognizer.SetWords(True)
        
        self.publish_state('ready')
    
    def publish_result(self, text: str):
        """Публикация финального результата распознавания"""
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)
        self.get_logger().info(f'📤 Опубликовал результат: {text}')
    
    def publish_partial(self, text: str):
        """Публикация частичного результата"""
        msg = String()
        msg.data = text
        self.partial_pub.publish(msg)
    
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
