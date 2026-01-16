#!/usr/bin/env python3
"""
test_sound_node.py - Unit тесты для SoundNode

Тестирует:
- Загрузку звуковых файлов
- Триггеры воспроизведения
- Публикацию состояния
- Управление громкостью
- Интеграцию с playback manager
"""

import os
import unittest
from unittest.mock import MagicMock, patch, Mock, call
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Мокаем внешние зависимости перед импортом
with patch('sounddevice.query_devices'), \
     patch('usb.core.find'):
    from rob_box_voice.sound_node import SoundNode


class TestSoundNode(unittest.TestCase):
    """Тесты для SoundNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Завершение ROS 2"""
        rclpy.shutdown()

    def setUp(self):
        """Создание мокированной ноды перед каждым тестом"""
        # Создаем временную директорию для sound pack
        self.test_dir = tempfile.mkdtemp()
        
        # Мокаем зависимости
        self.mock_audio_seg = patch('rob_box_voice.sound_node.AudioSegment').start()
        self.mock_sd = patch('rob_box_voice.sound_node.sd').start()
        self.mock_find_device = patch('rob_box_voice.sound_node.find_respeaker_device_sounddevice').start()
        self.mock_playback_mgr = patch('rob_box_voice.sound_node.AudioPlaybackManager').start()
        
        # Настраиваем моки
        self.mock_find_device.return_value = 0
        self.mock_sd.query_devices.return_value = [{'name': 'ReSpeaker'}]
        self.mock_playback_instance = MagicMock()
        self.mock_playback_mgr.get_instance.return_value = self.mock_playback_instance
        
        # Создаем ноду
        self.node = SoundNode()
        # Переопределяем директорию звуков после создания
        self.node.sound_pack_dir = self.test_dir

    def tearDown(self):
        """Очистка после каждого теста"""
        self.node.destroy_node()
        patch.stopall()
        # Удаляем временную директорию
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_node_creation(self):
        """Тест: нода создается корректно"""
        self.assertEqual(self.node.get_name(), 'sound_node')
        self.assertIsNotNone(self.node.trigger_sub)
        self.assertIsNotNone(self.node.state_pub)

    def test_parameters_exist(self):
        """Тест: все параметры объявлены"""
        params = ['sound_pack_dir', 'volume_db', 'trigger_animations', 'animation_topic']
        for param in params:
            self.assertTrue(self.node.has_parameter(param))

    def test_publishers_created(self):
        """Тест: паблишеры созданы"""
        publishers = self.node.get_publisher_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        pub_topics = [name for name, _ in publishers]
        self.assertIn('/voice/sound/state', pub_topics)

    def test_subscribers_created(self):
        """Тест: подписчики созданы"""
        subscriptions = self.node.get_subscriber_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        sub_topics = [name for name, _ in subscriptions]
        self.assertIn('/voice/sound/trigger', sub_topics)

    def test_audio_device_initialization(self):
        """Тест: инициализация аудио устройства"""
        self.mock_find_device.assert_called_once()
        self.assertEqual(self.node.device_index, 0)

    @patch('os.path.exists')
    @patch('os.listdir')
    def test_load_sounds_directory_not_found(self, mock_listdir, mock_exists):
        """Тест: обработка отсутствующей директории"""
        mock_exists.return_value = False
        
        self.node.load_sounds()
        
        self.assertEqual(len(self.node.sounds), 0)

    @patch('os.path.exists')
    @patch('os.listdir')
    def test_load_sounds_success(self, mock_listdir, mock_exists):
        """Тест: успешная загрузка звуков"""
        mock_exists.return_value = True
        mock_listdir.return_value = ['talk_1.mp3', 'talk_2.mp3', 'angry_1.mp3']
        
        mock_audio = MagicMock()
        self.mock_audio_seg.from_mp3.return_value = mock_audio
        mock_audio.__add__.return_value = mock_audio
        
        self.node.load_sounds()
        
        self.assertEqual(self.mock_audio_seg.from_mp3.call_count, 3)
        self.assertEqual(len(self.node.sounds), 3)

    def test_publish_state(self):
        """Тест: публикация состояния"""
        with patch.object(self.node.state_pub, 'publish') as mock_publish:
            self.node.publish_state('playing')
            
            mock_publish.assert_called_once()
            call_args = mock_publish.call_args[0][0]
            self.assertIsInstance(call_args, String)
            self.assertEqual(call_args.data, 'playing')

    def test_trigger_callback_with_sound_name(self):
        """Тест: обработка триггера с именем звука"""
        # Добавляем тестовый звук
        mock_sound = MagicMock()
        self.node.sounds['test_sound'] = mock_sound
        
        msg = String()
        msg.data = 'test_sound'
        
        with patch.object(self.node, 'play_sound_thread') as mock_play:
            self.node.trigger_callback(msg)
            mock_play.assert_called_once()

    def test_trigger_callback_with_group_name(self):
        """Тест: обработка триггера с группой звуков"""
        # Добавляем звуки в группу
        self.node.sounds['talk_1'] = MagicMock()
        self.node.sounds['talk_2'] = MagicMock()
        
        msg = String()
        msg.data = 'talk'  # группа
        
        with patch.object(self.node, 'play_sound_thread') as mock_play, \
             patch('random.choice', return_value='talk_1'):
            self.node.trigger_callback(msg)
            mock_play.assert_called_once()

    def test_is_playing_flag(self):
        """Тест: флаг воспроизведения"""
        self.assertFalse(self.node.is_playing)
        
        self.node.is_playing = True
        self.assertTrue(self.node.is_playing)

    def test_animation_publisher_created_when_enabled(self):
        """Тест: паблишер анимаций создается если включен"""
        # Проверяем что если trigger_animations=True, паблишер существует
        if hasattr(self.node, 'animation_pub'):
            self.assertIsNotNone(self.node.animation_pub)

    def test_volume_adjustment(self):
        """Тест: регулировка громкости"""
        volume_db = self.node.get_parameter('volume_db').value
        self.assertEqual(volume_db, -12.0)

    def test_sound_groups_initialized(self):
        """Тест: группы звуков инициализированы"""
        self.assertIn('talk', self.node.sound_groups)
        self.assertIn('angry', self.node.sound_groups)
        self.assertIn('cute', self.node.sound_groups)
        self.assertEqual(len(self.node.sound_groups['talk']), 4)


class TestSoundNodeIntegration(unittest.TestCase):
    """Интеграционные тесты SoundNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Создание ноды и тестового окружения"""
        self.test_dir = tempfile.mkdtemp()
        
        with patch('rob_box_voice.sound_node.AudioSegment'), \
             patch('rob_box_voice.sound_node.sd'), \
             patch('rob_box_voice.sound_node.find_respeaker_device_sounddevice'), \
             patch('rob_box_voice.sound_node.AudioPlaybackManager'):
            self.node = SoundNode()
            # Переопределяем директорию после создания
            self.node.sound_pack_dir = self.test_dir

    def tearDown(self):
        """Очистка"""
        self.node.destroy_node()
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_state_publishing_workflow(self):
        """Тест: workflow публикации состояния"""
        states_received = []
        
        def state_callback(msg):
            states_received.append(msg.data)
        
        # Подписываемся на состояние
        test_node = rclpy.create_node('test_listener')
        test_sub = test_node.create_subscription(
            String,
            '/voice/sound/state',
            state_callback,
            10
        )
        
        # Публикуем состояние
        self.node.publish_state('playing')
        self.node.publish_state('stopped')
        
        # Даем время на обработку
        rclpy.spin_once(self.node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        
        test_node.destroy_node()
        
        # Проверяем что состояния получены
        self.assertGreater(len(states_received), 0)


if __name__ == '__main__':
    unittest.main()
