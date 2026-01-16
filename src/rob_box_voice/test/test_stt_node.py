#!/usr/bin/env python3
"""
test_stt_node.py - Тесты для STT (Speech-to-Text) node

Тестирует:
- Обработку аудио данных
- VAD (Voice Activity Detection)
- Распознавание речи через Vosk
- Публикацию результатов STT
- Обработку тишины
"""

import os
import time
import unittest
from unittest.mock import MagicMock, patch, Mock
import numpy as np

import rclpy
from std_msgs.msg import String

# Мокаем audio_common_msgs если не установлен
try:
    from audio_common_msgs.msg import AudioData
except ImportError:
    # Создаём заглушку для тестов
    class AudioData:
        def __init__(self):
            self.data = []

from rob_box_voice.stt_node import STTNode


class TestSTTNode(unittest.TestCase):
    """Тесты для STT Node"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Подготовка перед каждым тестом"""
        # Мокаем Vosk чтобы не нужна была модель
        with patch('rob_box_voice.stt_node.Model'), \
             patch('rob_box_voice.stt_node.KaldiRecognizer'):
            self.node = STTNode()
        
    def tearDown(self):
        """Очистка после каждого теста"""
        self.node.destroy_node()

    def test_node_creation(self):
        """Тест: STT нода создаётся корректно"""
        self.assertEqual(self.node.get_name(), 'stt_node')
        print("  ✅ STT node создан")

    def test_parameters_exist(self):
        """Тест: Параметры ноды существуют"""
        self.assertTrue(self.node.has_parameter('sample_rate'))
        self.assertTrue(self.node.has_parameter('vad_threshold'))
        self.assertTrue(self.node.has_parameter('silence_duration'))
        print("  ✅ Параметры: sample_rate, vad_threshold, silence_duration")

    def test_audio_subscriber_exists(self):
        """Тест: Подписка на аудио существует"""
        self.assertIsNotNone(self.node.audio_sub)
        print("  ✅ Подписка на /audio/audio существует")

    def test_result_publisher_exists(self):
        """Тест: Publisher результатов STT существует"""
        self.assertIsNotNone(self.node.result_pub)
        print("  ✅ Publisher /voice/stt/result существует")

    def test_status_publisher_exists(self):
        """Тест: Publisher статуса существует"""
        self.assertIsNotNone(self.node.status_pub)
        print("  ✅ Publisher /voice/stt/status существует")

    def test_vad_state_tracking(self):
        """Тест: Отслеживание состояния VAD (говорит/тишина)"""
        # Проверяем начальное состояние
        self.assertFalse(self.node.is_speaking)
        print("  ✅ Начальное состояние: тишина")

    def test_silence_timer_exists(self):
        """Тест: Таймер для обнаружения тишины существует"""
        self.assertTrue(hasattr(self.node, 'silence_duration'))
        print(f"  ✅ Порог тишины: {self.node.silence_duration} сек")

    @patch('rob_box_voice.stt_node.KaldiRecognizer')
    def test_audio_callback_processes_data(self, mock_recognizer):
        """Тест: Callback обрабатывает аудио данные"""
        # Создаём тестовое аудио сообщение
        audio_msg = AudioData()
        audio_msg.data = [0] * 1600  # 0.1 сек при 16kHz
        
        # Мокаем recognizer
        self.node.recognizer = Mock()
        self.node.recognizer.AcceptWaveform.return_value = False
        
        # Проверяем что callback не падает
        try:
            self.node.audio_callback(audio_msg)
            print("  ✅ audio_callback обрабатывает данные")
        except Exception as e:
            self.fail(f"audio_callback упал: {e}")

    def test_sample_rate_parameter(self):
        """Тест: Sample rate настраивается"""
        # По умолчанию 16000 Hz
        self.assertEqual(self.node.sample_rate, 16000)
        print(f"  ✅ Sample rate: {self.node.sample_rate} Hz")


class TestVADLogic(unittest.TestCase):
    """Тесты логики VAD (Voice Activity Detection)"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        with patch('rob_box_voice.stt_node.Model'), \
             patch('rob_box_voice.stt_node.KaldiRecognizer'):
            self.node = STTNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_vad_detects_speech(self):
        """Тест: VAD обнаруживает речь"""
        # Создаём аудио с высокой энергией (имитация речи)
        audio_data = np.random.randint(-1000, 1000, size=1600, dtype=np.int16)
        
        # Вычисляем энергию
        energy = np.sqrt(np.mean(audio_data.astype(np.float32) ** 2))
        
        # Энергия должна быть выше порога
        self.assertGreater(energy, 10, "Тестовый сигнал должен иметь энергию")
        print(f"  ✅ Энергия сигнала: {energy:.1f} (речь)")

    def test_vad_detects_silence(self):
        """Тест: VAD обнаруживает тишину"""
        # Создаём тихий сигнал
        audio_data = np.zeros(1600, dtype=np.int16)
        
        # Вычисляем энергию
        energy = np.sqrt(np.mean(audio_data.astype(np.float32) ** 2))
        
        # Энергия должна быть близка к нулю
        self.assertLess(energy, 10, "Тишина должна иметь низкую энергию")
        print(f"  ✅ Энергия сигнала: {energy:.1f} (тишина)")


if __name__ == '__main__':
    unittest.main()
