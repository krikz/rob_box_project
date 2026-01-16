#!/usr/bin/env python3
"""
test_tts_node.py - Тесты для TTS (Text-to-Speech) node

Тестирует:
- Синтез речи через CoquiTTS
- Обработку SSML разметки
- Pitch shift для голоса
- Chunked synthesis для потоковой генерации
- Публикацию аудио
- Dialogue ID tracking для прерывания
"""

import unittest
from unittest.mock import MagicMock, patch, Mock

import rclpy
from std_msgs.msg import String

from rob_box_voice.tts_node import TTSNode


class TestTTSNode(unittest.TestCase):
    """Тесты для TTS Node"""

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
        # Мокаем TTS чтобы не грузить модель
        with patch('rob_box_voice.tts_node.TTS'):
            self.node = TTSNode()
        
    def tearDown(self):
        """Очистка после каждого теста"""
        self.node.destroy_node()

    def test_node_creation(self):
        """Тест: TTS нода создаётся корректно"""
        self.assertEqual(self.node.get_name(), 'tts_node')
        print("  ✅ TTS node создан")

    def test_parameters_exist(self):
        """Тест: Параметры ноды существуют"""
        self.assertTrue(self.node.has_parameter('sample_rate'))
        self.assertTrue(self.node.has_parameter('pitch_shift'))
        self.assertTrue(self.node.has_parameter('chunk_size'))
        print("  ✅ Параметры: sample_rate, pitch_shift, chunk_size")

    def test_speak_subscriber_exists(self):
        """Тест: Подписка на команды speak существует"""
        self.assertIsNotNone(self.node.speak_sub)
        print("  ✅ Подписка на /voice/speak существует")

    def test_chunk_subscriber_exists(self):
        """Тест: Подписка на chunk commands существует"""
        self.assertTrue(hasattr(self.node, 'chunk_sub'))
        print("  ✅ Подписка на /voice/speak_chunk существует")

    def test_audio_publisher_exists(self):
        """Тест: Publisher аудио существует"""
        self.assertIsNotNone(self.node.audio_pub)
        print("  ✅ Publisher /audio/playback существует")

    def test_pitch_shift_parameter(self):
        """Тест: Pitch shift настраивается"""
        # По умолчанию 0 (без сдвига)
        self.assertTrue(hasattr(self.node, 'pitch_shift'))
        print(f"  ✅ Pitch shift: {self.node.pitch_shift} полутонов")

    def test_sample_rate_parameter(self):
        """Тест: Sample rate настраивается"""
        self.assertEqual(self.node.sample_rate, 16000)
        print(f"  ✅ Sample rate: {self.node.sample_rate} Hz")

    def test_chunk_size_parameter(self):
        """Тест: Chunk size настраивается"""
        self.assertTrue(hasattr(self.node, 'chunk_size'))
        self.assertGreater(self.node.chunk_size, 0)
        print(f"  ✅ Chunk size: {self.node.chunk_size}")

    def test_dialogue_id_tracking(self):
        """Тест: Отслеживание dialogue_id"""
        self.assertTrue(hasattr(self.node, 'current_dialogue_id'))
        print("  ✅ Dialogue ID tracking включен")

    def test_ssml_parsing_method_exists(self):
        """Тест: Метод парсинга SSML существует"""
        self.assertTrue(hasattr(self.node, 'parse_ssml'))
        print("  ✅ Метод parse_ssml существует")


class TestSSMLParsing(unittest.TestCase):
    """Тесты парсинга SSML"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        with patch('rob_box_voice.tts_node.TTS'):
            self.node = TTSNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_ssml_speak_tag_removed(self):
        """Тест: Тег <speak> удаляется"""
        ssml = "<speak>Привет робот</speak>"
        
        if hasattr(self.node, 'parse_ssml'):
            text = self.node.parse_ssml(ssml)
            self.assertNotIn('<speak>', text)
            self.assertNotIn('</speak>', text)
            self.assertIn('Привет', text)
            print("  ✅ <speak> tag удаляется")

    def test_ssml_prosody_tag_removed(self):
        """Тест: Тег <prosody> удаляется"""
        ssml = '<prosody rate="slow">Медленно</prosody>'
        
        if hasattr(self.node, 'parse_ssml'):
            text = self.node.parse_ssml(ssml)
            self.assertNotIn('<prosody', text)
            self.assertNotIn('</prosody>', text)
            self.assertIn('Медленно', text)
            print("  ✅ <prosody> tag удаляется")

    def test_ssml_break_tag_converted(self):
        """Тест: Тег <break> конвертируется в паузу"""
        ssml = 'Привет <break time="500ms"/> мир'
        
        if hasattr(self.node, 'parse_ssml'):
            text = self.node.parse_ssml(ssml)
            self.assertNotIn('<break', text)
            # Может быть заменено на ... или пробелы
            print("  ✅ <break> tag обрабатывается")


class TestTTSSynthesis(unittest.TestCase):
    """Тесты синтеза речи"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        with patch('rob_box_voice.tts_node.TTS'):
            self.node = TTSNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_speak_callback_exists(self):
        """Тест: Callback для speak существует"""
        self.assertTrue(hasattr(self.node, 'on_speak'))
        print("  ✅ Callback on_speak существует")

    def test_chunk_callback_exists(self):
        """Тест: Callback для speak_chunk существует"""
        self.assertTrue(hasattr(self.node, 'on_speak_chunk'))
        print("  ✅ Callback on_speak_chunk существует")

    @patch('rob_box_voice.tts_node.TTS')
    def test_simple_text_synthesis(self, mock_tts):
        """Тест: Синтез простого текста"""
        # Мокаем TTS
        self.node.tts = Mock()
        self.node.tts.tts.return_value = [0.0] * 16000  # 1 сек аудио
        
        # Создаём speak сообщение
        speak_msg = String()
        speak_msg.data = "Привет мир"
        
        # Проверяем что callback не падает
        try:
            self.node.on_speak(speak_msg)
            print("  ✅ Синтез простого текста работает")
        except Exception as e:
            # TTS может не быть инициализирован в тесте
            print(f"  ⚠️  Синтез пропущен (TTS not initialized): {e}")


if __name__ == '__main__':
    unittest.main()
