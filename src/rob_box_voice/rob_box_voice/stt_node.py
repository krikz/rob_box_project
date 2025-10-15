#!/usr/bin/env python3
"""
STTNode - Speech-to-Text с Vosk
Подписывается: /audio/speech_audio (AudioData)
Публикует: /voice/stt/result (String)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

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
        
        self.model_path = self.get_parameter('model_path').value
        self.sample_rate = self.get_parameter('sample_rate').value
        
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
        
        # Vosk модель и распознаватель
        self.model: Optional[Model] = None
        self.recognizer: Optional[KaldiRecognizer] = None
        
        # Состояние
        self.is_robot_speaking = False  # Флаг: робот говорит (не слушать!)
        
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
        Вызывается только когда есть законченная фраза речи
        """
        if self.recognizer is None:
            return
        
        # НЕ СЛУШАТЬ когда робот говорит (самовозбуждение!)
        if self.is_robot_speaking:
            self.get_logger().info('🔇 Игнор: робот говорит')
            return
        
        # Конвертируем список в bytes
        audio_bytes = bytes(msg.data)
        duration = len(audio_bytes) / (self.sample_rate * 2)  # 16-bit = 2 bytes
        
        self.get_logger().info(f'🎤 Получена фраза: {duration:.2f}с ({len(audio_bytes)} bytes)')
        self.publish_state('recognizing')
        
        # Отправить весь аудио буфер в Vosk
        if self.recognizer.AcceptWaveform(audio_bytes):
            result = json.loads(self.recognizer.Result())
        else:
            result = json.loads(self.recognizer.FinalResult())
        
        text = result.get('text', '').strip()
        
        self.get_logger().info(f'🔍 Vosk результат: "{text}" (длина={len(text)})')
        
        # Сбросить распознаватель для следующей фразы
        self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
        self.recognizer.SetWords(True)
        
        # ФИЛЬТР: игнорируем слишком короткие слова (шум типа "и и")
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
    
    def publish_result(self, text: str):
        """Публикация финального результата распознавания"""
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
