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
import numpy as np

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

    def test_select_sound_direct_match(self):
        """Тест: select_sound() с прямым совпадением"""
        # Добавляем тестовый звук
        mock_audio = MagicMock()
        self.node.sounds['test_sound'] = mock_audio
        
        # Прямое совпадение
        result = self.node.select_sound('test_sound')
        self.assertEqual(result, 'test_sound')

    def test_select_sound_from_group(self):
        """Тест: select_sound() выбирает случайный звук из группы"""
        # Добавляем звуки из группы "talk"
        for name in ['talk_1', 'talk_2', 'talk_3']:
            self.node.sounds[name] = MagicMock()
        
        # Выбираем из группы
        with patch('rob_box_voice.sound_node.random.choice', return_value='talk_2'):
            result = self.node.select_sound('talk')
            self.assertEqual(result, 'talk_2')

    def test_select_sound_fuzzy_match(self):
        """Тест: select_sound() находит похожий звук"""
        # Добавляем звуки
        self.node.sounds['angry_scream'] = MagicMock()
        self.node.sounds['happy_laugh'] = MagicMock()
        
        # Ищем похожий
        result = self.node.select_sound('angry')
        self.assertEqual(result, 'angry_scream')

    def test_select_sound_not_found(self):
        """Тест: select_sound() возвращает None если не найдено"""
        result = self.node.select_sound('nonexistent_sound')
        self.assertIsNone(result)

    def test_play_sound_thread_sets_flags(self):
        """Тест: play_sound_thread() устанавливает флаги"""
        # Добавляем тестовый звук
        mock_audio = MagicMock()
        mock_audio.frame_rate = 16000
        mock_audio.channels = 2
        mock_audio.get_array_of_samples.return_value = [0] * 1000
        self.node.sounds['test'] = mock_audio
        
        # Мокаем playback manager
        with patch.object(self.node.playback_manager, 'play_audio', return_value=True), \
             patch('rob_box_voice.sound_node.sd.stop'), \
             patch('rob_box_voice.sound_node.time.sleep'), \
             patch('rob_box_voice.sound_node.np.array', return_value=np.zeros((1000, 2), dtype=np.float32)):
            
            # Запускаем в синхронном режиме (не в потоке)
            self.node.play_sound_thread('test', 'test')
        
        # Проверяем что флаги сброшены после завершения
        self.assertFalse(self.node.is_playing)
        self.assertIsNone(self.node.current_sound)

    def test_play_sound_thread_with_resampling(self):
        """Тест: play_sound_thread() делает ресемплинг если нужно"""
        # Звук с частотой отличной от 16kHz
        mock_audio = MagicMock()
        mock_audio.frame_rate = 44100  # Требует ресемплинга
        mock_audio.channels = 1
        mock_audio.set_frame_rate.return_value = mock_audio  # Возвращает себя
        mock_audio.get_array_of_samples.return_value = [0] * 1000
        self.node.sounds['test'] = mock_audio
        
        with patch.object(self.node.playback_manager, 'play_audio', return_value=True), \
             patch('rob_box_voice.sound_node.sd.stop'), \
             patch('rob_box_voice.sound_node.time.sleep'), \
             patch('rob_box_voice.sound_node.np.array', return_value=np.zeros(1000, dtype=np.int16)), \
             patch('rob_box_voice.sound_node.np.column_stack', return_value=np.zeros((1000, 2))):
            
            self.node.play_sound_thread('test', 'test')
        
        # Проверяем что был вызван ресемплинг
        mock_audio.set_frame_rate.assert_called_once_with(16000)

    def test_play_sound_thread_mono_to_stereo(self):
        """Тест: play_sound_thread() конвертирует mono в stereo"""
        mock_audio = MagicMock()
        mock_audio.frame_rate = 16000
        mock_audio.channels = 1  # Mono
        mock_audio.get_array_of_samples.return_value = [0] * 1000
        self.node.sounds['test'] = mock_audio
        
        samples_array = np.zeros(1000, dtype=np.int16)
        expected_stereo = np.column_stack((samples_array, samples_array))
        
        with patch.object(self.node.playback_manager, 'play_audio', return_value=True) as mock_play, \
             patch('rob_box_voice.sound_node.sd.stop'), \
             patch('rob_box_voice.sound_node.time.sleep'), \
             patch('rob_box_voice.sound_node.np.array', return_value=samples_array), \
             patch('rob_box_voice.sound_node.np.column_stack', return_value=expected_stereo) as mock_stack:
            
            self.node.play_sound_thread('test', 'test')
        
        # Проверяем что был вызван column_stack для создания стерео
        mock_stack.assert_called_once()

    def test_play_sound_thread_triggers_animation(self):
        """Тест: play_sound_thread() триггерит анимацию"""
        self.node.trigger_animations = True
        self.node.animation_pub = MagicMock()
        
        mock_audio = MagicMock()
        mock_audio.frame_rate = 16000
        mock_audio.channels = 2
        mock_audio.get_array_of_samples.return_value = [0] * 1000
        self.node.sounds['thinking'] = mock_audio
        
        with patch.object(self.node.playback_manager, 'play_audio', return_value=True), \
             patch('rob_box_voice.sound_node.sd.stop'), \
             patch('rob_box_voice.sound_node.time.sleep'), \
             patch('rob_box_voice.sound_node.np.array', return_value=np.zeros((1000, 2), dtype=np.float32)):
            
            self.node.play_sound_thread('thinking', 'thinking')
        
        # Проверяем что анимация была отправлена
        self.node.animation_pub.publish.assert_called_once()
        published_msg = self.node.animation_pub.publish.call_args[0][0]
        self.assertEqual(published_msg.data, 'thinking')

    def test_play_sound_thread_handles_device_busy(self):
        """Тест: play_sound_thread() обрабатывает занятое устройство"""
        mock_audio = MagicMock()
        mock_audio.frame_rate = 16000
        mock_audio.channels = 2
        mock_audio.get_array_of_samples.return_value = [0] * 1000
        self.node.sounds['test'] = mock_audio
        
        # Playback manager возвращает False (устройство занято)
        with patch.object(self.node.playback_manager, 'play_audio', return_value=False), \
             patch('rob_box_voice.sound_node.np.array', return_value=np.zeros((1000, 2), dtype=np.float32)):
            
            self.node.play_sound_thread('test', 'test')
        
        # Флаги должны быть сброшены даже при ошибке
        self.assertFalse(self.node.is_playing)

    def test_play_sound_thread_exception_handling(self):
        """Тест: play_sound_thread() обрабатывает исключения"""
        mock_audio = MagicMock()
        mock_audio.frame_rate = 16000
        mock_audio.channels = 2
        mock_audio.get_array_of_samples.side_effect = Exception('Test error')
        self.node.sounds['test'] = mock_audio
        
        # Не должно упасть
        try:
            self.node.play_sound_thread('test', 'test')
        except Exception:
            self.fail('play_sound_thread should not raise exception')
        
        # Флаги сброшены
        self.assertFalse(self.node.is_playing)

    def test_cleanup_playback_noise(self):
        """Тест: cleanup_playback_noise() очищает шум"""
        with patch('rob_box_voice.sound_node.sd.stop') as mock_stop, \
             patch('rob_box_voice.sound_node.time.sleep') as mock_sleep:
            
            self.node.cleanup_playback_noise()
            
            # Проверяем что вызваны sd.stop() и sleep()
            mock_stop.assert_called_once()
            mock_sleep.assert_called_once_with(0.1)

    def test_cleanup_playback_noise_exception(self):
        """Тест: cleanup_playback_noise() обрабатывает исключения"""
        with patch('rob_box_voice.sound_node.sd.stop', side_effect=Exception('Test error')):
            # Не должно упасть
            try:
                self.node.cleanup_playback_noise()
            except Exception:
                self.fail('cleanup_playback_noise should not raise exception')

    def test_trigger_animation_mapping(self):
        """Тест: trigger_animation() маппирует звуки на анимации"""
        self.node.trigger_animations = True
        self.node.animation_pub = MagicMock()
        
        # Тестируем различные маппинги
        test_cases = [
            ('thinking', 'thinking'),
            ('angry_1', 'angry'),
            ('cute', 'happy'),
            ('very_cute', 'very_happy'),
            ('talk', 'talking'),
            ('unknown', 'unknown'),  # Fallback
        ]
        
        for trigger, expected_animation in test_cases:
            self.node.animation_pub.reset_mock()
            self.node.trigger_animation(trigger)
            
            published_msg = self.node.animation_pub.publish.call_args[0][0]
            self.assertEqual(published_msg.data, expected_animation,
                           f"Expected {expected_animation} for trigger {trigger}")

    def test_trigger_animation_exception_handling(self):
        """Тест: trigger_animation() обрабатывает ошибки публикации"""
        self.node.trigger_animations = True
        self.node.animation_pub = MagicMock()
        self.node.animation_pub.publish.side_effect = Exception('Publish failed')
        
        # Не должно упасть
        try:
            self.node.trigger_animation('test')
        except Exception:
            self.fail('trigger_animation should not raise exception')

    def test_parameters_callback_volume_change(self):
        """Тест: parameters_callback() обрабатывает изменение громкости"""
        from rclpy.parameter import Parameter
        
        # Создаем параметр с новой громкостью (используем rclpy Parameter)
        param = Parameter('volume_db', Parameter.Type.DOUBLE, -6.0)
        
        # Мокаем load_sounds
        with patch.object(self.node, 'load_sounds') as mock_load:
            result = self.node.parameters_callback([param])
            
            # Проверяем что громкость изменилась
            self.assertEqual(self.node.volume_db, -6.0)
            
            # Проверяем что звуки перезагружены
            mock_load.assert_called_once()
            
            # Проверяем успешный результат
            self.assertTrue(result.successful)

    def test_trigger_callback_when_playing(self):
        """Тест: trigger_callback() пропускает триггер если уже играет"""
        # Устанавливаем флаг is_playing
        self.node.is_playing = True
        self.node.current_sound = 'current_sound'
        
        # Добавляем звук
        self.node.sounds['new_sound'] = MagicMock()
        
        # Триггерим новый звук
        msg = String()
        msg.data = 'new_sound'
        self.node.trigger_callback(msg)
        
        # Проверяем что play_thread не был создан
        self.assertIsNone(self.node.play_thread)

    def test_trigger_callback_unknown_sound(self):
        """Тест: trigger_callback() игнорирует неизвестный звук"""
        msg = String()
        msg.data = 'nonexistent_sound'
        
        # Не должно создать thread
        self.node.trigger_callback(msg)
        self.assertIsNone(self.node.play_thread)


if __name__ == '__main__':
    unittest.main()
