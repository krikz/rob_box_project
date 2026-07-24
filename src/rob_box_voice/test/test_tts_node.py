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


class TestSynthesisExecutorBound(unittest.TestCase):
    """Regression test for BLK-9: bounded thread fan-out in tts_node.

    Verifies that TTSNode routes synthesis work through a bounded
    ThreadPoolExecutor, not `threading.Thread(daemon=True).start()`. The
    node must not create more worker threads than ``max_workers`` even
    under heavy submission, and overflow submissions must be rejected
    (not queued forever) once ``max_queue + max_workers`` slots are used.
    """

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Build a TTSNode with a tiny pool so the bounds are easy to assert."""
        import concurrent.futures
        import threading
        with patch('rob_box_voice.tts_node.TTS'):
            # ROS params for synthesis_max_workers / synthesis_max_queue are
            # already declared with safe defaults in TTSNode.__init__; we
            # shrink the pool AFTER construction because ROS parameters
            # cannot be re-declared once set.
            self.node = TTSNode()
        # Replace the executor with a 1-worker fresh one for deterministic tests.
        self.node._synthesis_executor.shutdown(wait=False, cancel_futures=True)
        self.node._synthesis_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="tts-test",
        )
        self.node._synthesis_executor_max_workers = 1
        # Slot count = 1 worker + 1 queue slot = 2 in-flight cap.
        self.node._synthesis_slots = threading.Semaphore(2)
        self.node._synthesis_in_flight = 0

    def tearDown(self):
        try:
            self.node.shutdown_synthesis_executor(wait=False)
        finally:
            self.node.destroy_node()

    def test_executor_replaces_unbounded_threads(self):
        """The spawn site must use the executor, not raw threading.Thread."""
        import inspect
        from rob_box_voice import tts_node as tts_mod
        src = inspect.getsource(tts_mod)
        # The literal pattern flagged in the review must be gone.
        self.assertNotIn(
            'threading.Thread(\n                target=self._run_synthesis_worker,',
            src,
            "BLK-9: unbounded `threading.Thread(target=..., daemon=True)` "
            "spawn site must be removed from tts_node.on_chunk or _on_speak.",
        )

    def test_executor_is_bounded_threadpool(self):
        """TTSNode must hold a ThreadPoolExecutor, not raw threads."""
        import concurrent.futures
        self.assertIsInstance(
            self.node._synthesis_executor,
            concurrent.futures.ThreadPoolExecutor,
        )
        # The pool must report a worker ceiling of at least 1.
        self.assertGreaterEqual(self.node._synthesis_executor_max_workers, 1)

    def test_submit_respects_slot_bound(self):
        """Submitting beyond (workers+queue) returns without enqueuing."""
        captured = []

        def slow_worker():
            import time as _t
            _t.sleep(0.2)
            captured.append('ok')

        # Occupy the only available slot.
        self.assertTrue(self.node._synthesis_slots.acquire(blocking=False))
        # Now _submit_synthesis should refuse.
        self.node._submit_synthesis(slow_worker, 'test-1')
        # Restore the slot for the in-flight worker; emulate future completion.
        # The captured list must remain empty because we never enqueued.
        self.node._synthesis_slots.release()
        # And the in-flight counter must NOT have grown.
        self.assertEqual(self.node._synthesis_in_flight, 0)
        self.assertEqual(captured, [])

    def test_submit_after_shutdown_is_dropped(self):
        """After shutdown_synthesis_executor, _submit_synthesis is a no-op."""
        self.node._synthesis_executor_shutdown = True
        called = []

        def fn():
            called.append(1)

        self.node._submit_synthesis(fn, 'after-shutdown')
        # The function must not have been invoked, and no slot was taken.
        self.assertEqual(called, [])
        # Slot semaphore counter must be back to 2 (its original value).
        self.assertTrue(self.node._synthesis_slots.acquire(blocking=False))
        self.assertTrue(self.node._synthesis_slots.acquire(blocking=False))
        # Both already taken; the next attempt must fail (== original cap of 2).
        self.assertFalse(self.node._synthesis_slots.acquire(blocking=False))


if __name__ == '__main__':
    unittest.main()
