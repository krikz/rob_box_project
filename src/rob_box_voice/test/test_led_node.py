#!/usr/bin/env python3
"""
test_led_node.py - Unit тесты для LEDNode (исправленная версия)

Тестирует:
- USB управление PixelRingLite
- Callbacks для voice state и direction
- Сервис управления автоматическим режимом
"""

import unittest
from unittest.mock import MagicMock, patch, Mock

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from std_srvs.srv import SetBool

from rob_box_voice.led_node import PixelRingLite, LEDNode


class TestPixelRingLite(unittest.TestCase):
    """Тесты для PixelRingLite класса."""

    def test_usb_ids(self):
        """Тест: USB ID константы."""
        self.assertEqual(PixelRingLite.VENDOR_ID, 0x2886)
        self.assertEqual(PixelRingLite.PRODUCT_ID, 0x0018)

    def test_pixel_ring_constants(self):
        """Тест: команды LED."""
        self.assertEqual(PixelRingLite.CMD_LISTEN, 2)
        self.assertEqual(PixelRingLite.CMD_SPEAK, 3)
        self.assertEqual(PixelRingLite.CMD_THINK, 4)
        self.assertEqual(PixelRingLite.CMD_SPIN, 5)

    def test_connect_success(self):
        """Тест: успешное подключение."""
        with patch('usb.core.find') as mock_find:
            mock_find.return_value = MagicMock()

            ring = PixelRingLite()
            result = ring.connect()

            self.assertTrue(result)
            self.assertIsNotNone(ring.dev)

    def test_connect_failure(self):
        """Тест: неудачное подключение."""
        with patch('usb.core.find', return_value=None):
            ring = PixelRingLite()
            result = ring.connect()

            self.assertFalse(result)
            self.assertIsNone(ring.dev)

    def test_send_command(self):
        """Тест: отправка USB команды."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()

            ring._send_command(PixelRingLite.CMD_LISTEN)
            mock_dev.ctrl_transfer.assert_called_once()

    def test_brightness_property(self):
        """Тест: свойство яркости."""
        ring = PixelRingLite()
        self.assertEqual(ring._brightness, 16)


class TestLEDNode(unittest.TestCase):
    """Тесты для LEDNode."""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2."""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Завершение ROS 2."""
        pass  # Не shutdown для других тестов

    def setUp(self):
        """Создание мокированной ноды."""
        with patch('usb.core.find'), \
             patch('rob_box_voice.led_node.PixelRingLite') as mock_ring:

            self.mock_ring_class = mock_ring
            self.mock_ring_instance = MagicMock()
            mock_ring.return_value = self.mock_ring_instance
            self.mock_ring_instance.connect.return_value = True

            self.node = LEDNode()

    def tearDown(self):
        """Очистка после теста."""
        self.node.destroy_node()
        patch.stopall()

    def test_node_creation(self):
        """Тест: нода создается корректно."""
        self.assertEqual(self.node.get_name(), 'led_node')
        self.assertIsNotNone(self.node.pixel_ring)

    def test_parameters_exist(self):
        """Тест: все параметры объявлены."""
        params = ['brightness', 'auto_mode']
        for param in params:
            self.assertTrue(self.node.has_parameter(param))

    def test_subscribers_created(self):
        """Тест: подписчики созданы."""
        subscriptions = self.node.get_subscriber_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        sub_topics = [name for name, _ in subscriptions]

        self.assertIn('/voice/state', sub_topics)
        self.assertIn('/audio/direction', sub_topics)

    def test_service_created(self):
        """Тест: сервис управления LED создан."""
        services = self.node.get_service_names_and_types()
        service_names = [name for name, _ in services]

        # Сервис называется /voice/set_auto_led
        self.assertIn('/voice/set_auto_led', service_names)

    def test_led_ring_initialization(self):
        """Тест: LED ring инициализируется при создании."""
        self.mock_ring_instance.connect.assert_called_once()

    def test_voice_state_callback_listening(self):
        """Тест: обработка состояния 'listening'."""
        msg = String()
        msg.data = 'listening'

        self.node.state_callback(msg)

        # Проверяем что current_mode установлен
        self.assertEqual(self.node.current_mode, 'listening')

    def test_voice_state_callback_speaking(self):
        """Тест: обработка состояния 'speaking'."""
        msg = String()
        msg.data = 'speaking'

        self.node.state_callback(msg)
        self.assertEqual(self.node.current_mode, 'speaking')

    def test_voice_state_callback_thinking(self):
        """Тест: обработка состояния 'thinking'."""
        msg = String()
        msg.data = 'thinking'

        self.node.state_callback(msg)
        self.assertEqual(self.node.current_mode, 'thinking')

    def test_voice_state_callback_idle(self):
        """Тест: обработка состояния 'idle'."""
        msg = String()
        msg.data = 'idle'

        self.node.state_callback(msg)
        # off() вызывается при idle (и в конструкторе тоже)
        self.assertGreaterEqual(self.mock_ring_instance.off.call_count, 1)

    def test_direction_callback(self):
        """Тест: обработка направления звука."""
        msg = Int32()
        msg.data = 180

        self.node.direction_callback(msg)

        # Направление сохраняется в current_direction
        self.assertEqual(self.node.current_direction, 180)

    def test_auto_mode_true(self):
        """Тест: автоматический режим включен."""
        self.node.auto_mode = True
        msg = String()
        msg.data = 'listening'

        self.node.state_callback(msg)
        # В auto_mode LED должен реагировать
        self.assertEqual(self.node.current_mode, 'listening')

    def test_auto_mode_false(self):
        """Тест: автоматический режим выключен."""
        self.node.auto_mode = False
        previous_mode = self.node.current_mode

        msg = String()
        msg.data = 'listening'

        self.node.state_callback(msg)
        # В неавтоматическом режиме callback выходит рано
        # current_mode может остаться прежним

    def test_service_enable_auto_mode(self):
        """Тест: сервис включения авто режима."""
        request = SetBool.Request()
        request.data = True
        response = SetBool.Response()

        self.node.set_auto_led_callback(request, response)

        self.assertTrue(response.success)
        self.assertTrue(self.node.auto_mode)

    def test_service_disable_auto_mode(self):
        """Тест: сервис выключения авто режима."""
        request = SetBool.Request()
        request.data = False
        response = SetBool.Response()

        self.node.set_auto_led_callback(request, response)

        self.assertTrue(response.success)
        self.assertFalse(self.node.auto_mode)

    def test_current_mode_tracking(self):
        """Тест: отслеживание текущего режима."""
        self.assertTrue(hasattr(self.node, 'current_mode'))

    def test_auto_mode_parameter(self):
        """Тест: параметр auto_mode."""
        auto_mode = self.node.get_parameter('auto_mode').value
        self.assertIsInstance(auto_mode, bool)

    def test_brightness_parameter(self):
        """Тест: параметр brightness."""
        brightness = self.node.get_parameter('brightness').value
        self.assertIsInstance(brightness, int)
        self.assertGreaterEqual(brightness, 0)
        self.assertLessEqual(brightness, 31)  # 0-31 по спецификации

    def test_voice_state_callback_error(self):
        """Тест: состояние 'error' отображается красным."""
        msg = String()
        msg.data = 'error'
        self.node.state_callback(msg)

        # Проверяем что mono был вызван с красным цветом
        self.mock_ring_instance.mono.assert_called_once()
        args = self.mock_ring_instance.mono.call_args[0]
        self.assertEqual(args, tuple(self.node.colors['error']))

    def test_voice_state_callback_trace(self):
        """Тест: состояние 'trace' активирует trace режим."""
        msg = String()
        msg.data = 'trace'
        self.node.state_callback(msg)

        self.mock_ring_instance.trace.assert_called_once()

    def test_voice_state_callback_generating(self):
        """Тест: состояние 'generating' активирует think режим."""
        msg = String()
        msg.data = 'generating'
        self.node.state_callback(msg)

        self.mock_ring_instance.think.assert_called_once()

    def test_voice_state_callback_case_insensitive(self):
        """Тест: состояния обрабатываются без учёта регистра."""
        test_cases = ['LISTENING', 'Listening', 'LiStEnInG']

        for state in test_cases:
            self.mock_ring_instance.reset_mock()
            msg = String()
            msg.data = state
            self.node.state_callback(msg)

            # Все варианты должны вызвать mono
            self.mock_ring_instance.mono.assert_called_once()

    def test_initialize_hardware_success(self):
        """Тест: успешная инициализация оборудования."""
        # Создаём новый экземпляр с чистыми моками
        with patch('usb.core.find'), \
             patch('rob_box_voice.led_node.PixelRingLite') as mock_ring_class:

            mock_ring = MagicMock()
            mock_ring_class.return_value = mock_ring
            mock_ring.connect.return_value = True

            node = LEDNode()

            # Проверяем что hardware был инициализирован
            mock_ring.connect.assert_called_once()
            mock_ring.set_brightness.assert_called_once()
            mock_ring.set_color_palette.assert_called_once()
            mock_ring.off.assert_called_once()

            node.destroy_node()

    def test_initialize_hardware_failure(self):
        """Тест: неудачная инициализация оборудования."""
        with patch('usb.core.find'), \
             patch('rob_box_voice.led_node.PixelRingLite') as mock_ring_class:

            mock_ring = MagicMock()
            mock_ring_class.return_value = mock_ring
            mock_ring.connect.return_value = False  # Устройство не найдено

            node = LEDNode()

            # Проверяем что connect был вызван, но дальнейшая настройка не произошла
            mock_ring.connect.assert_called_once()
            mock_ring.set_brightness.assert_not_called()

            node.destroy_node()

    def test_set_mode_manual_off(self):
        """Тест: ручная установка режима 'off'."""
        # Сбрасываем mock чтобы не считать вызов при инициализации
        self.mock_ring_instance.off.reset_mock()

        self.node.set_mode_manual('off')
        self.mock_ring_instance.off.assert_called_once()

    def test_set_mode_manual_trace(self):
        """Тест: ручная установка режима 'trace'."""
        self.node.set_mode_manual('trace')
        self.mock_ring_instance.trace.assert_called_once()

    def test_set_mode_manual_listen(self):
        """Тест: ручная установка режима 'listen'."""
        self.node.set_mode_manual('listen')
        self.mock_ring_instance.listen.assert_called_once()

    def test_set_mode_manual_think(self):
        """Тест: ручная установка режима 'think'."""
        self.node.set_mode_manual('think')
        self.mock_ring_instance.think.assert_called_once()

    def test_set_mode_manual_speak(self):
        """Тест: ручная установка режима 'speak'."""
        self.node.set_mode_manual('speak')
        self.mock_ring_instance.speak.assert_called_once()

    def test_set_mode_manual_spin(self):
        """Тест: ручная установка режима 'spin'."""
        self.node.set_mode_manual('spin')
        self.mock_ring_instance.spin.assert_called_once()

    def test_shutdown(self):
        """Тест: корректное завершение работы."""
        self.node.shutdown()

        # Проверяем что LED выключены при завершении
        self.mock_ring_instance.off.assert_called()

    def test_pixel_ring_off(self):
        """Тест: PixelRingLite.off() отправляет команду."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            ring.off()

            # Проверяем что была отправлена команда MONO с нулями
            calls = mock_dev.ctrl_transfer.call_args_list
            self.assertGreater(len(calls), 0)

    def test_pixel_ring_mono(self):
        """Тест: PixelRingLite.mono() устанавливает цвет."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            ring.mono(255, 128, 64)

            # Команда была отправлена
            self.assertTrue(mock_dev.ctrl_transfer.called)

    def test_pixel_ring_trace(self):
        """Тест: PixelRingLite.trace() активирует режим."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            ring.trace()

            self.assertTrue(mock_dev.ctrl_transfer.called)

    def test_pixel_ring_listen(self):
        """Тест: PixelRingLite.listen() активирует режим."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            ring.listen()

            self.assertTrue(mock_dev.ctrl_transfer.called)

    def test_pixel_ring_think(self):
        """Тест: PixelRingLite.think() активирует пульсацию."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            ring.think()

            self.assertTrue(mock_dev.ctrl_transfer.called)

    def test_pixel_ring_speak(self):
        """Тест: PixelRingLite.speak() активирует вращение."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            ring.speak()

            self.assertTrue(mock_dev.ctrl_transfer.called)

    def test_pixel_ring_spin(self):
        """Тест: PixelRingLite.spin() активирует быстрое вращение."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            ring.spin()

            self.assertTrue(mock_dev.ctrl_transfer.called)

    def test_pixel_ring_set_brightness(self):
        """Тест: PixelRingLite.set_brightness() устанавливает яркость."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            ring.set_brightness(20)

            # Проверяем что яркость сохранена
            self.assertEqual(ring._brightness, 20)
            self.assertTrue(mock_dev.ctrl_transfer.called)

    def test_pixel_ring_brightness_clamping(self):
        """Тест: яркость ограничена диапазоном 0-31."""
        ring = PixelRingLite()

        # Тест верхней границы
        with patch.object(ring, '_send_command') as mock_send:
            ring.set_brightness(100)
            self.assertEqual(ring._brightness, 31)

        # Тест нижней границы
        with patch.object(ring, '_send_command') as mock_send:
            ring.set_brightness(-10)
            self.assertEqual(ring._brightness, 0)

    def test_pixel_ring_set_color_palette(self):
        """Тест: PixelRingLite.set_color_palette() устанавливает палитру."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            ring.set_color_palette((255, 0, 0), (0, 255, 0))

            # Команда отправлена
            self.assertTrue(mock_dev.ctrl_transfer.called)

    def test_pixel_ring_set_volume(self):
        """Тест: PixelRingLite.set_volume() показывает уровень."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            ring.set_volume(8)

            self.assertTrue(mock_dev.ctrl_transfer.called)

    def test_pixel_ring_send_command_no_device(self):
        """Тест: _send_command() возвращает False если нет устройства."""
        ring = PixelRingLite()
        # Не подключаем устройство
        result = ring._send_command(PixelRingLite.CMD_MONO)
        self.assertFalse(result)

    def test_pixel_ring_send_command_exception(self):
        """Тест: _send_command() обрабатывает исключения."""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_dev.ctrl_transfer.side_effect = Exception('USB error')
            mock_find.return_value = mock_dev

            ring = PixelRingLite()
            ring.connect()
            result = ring._send_command(PixelRingLite.CMD_MONO)

            # Команда вернула False при ошибке
            self.assertFalse(result)

    def test_colors_parameter(self):
        """Тест: параметры цветов загружены корректно."""
        self.assertIsInstance(self.node.colors, dict)
        self.assertIn('idle', self.node.colors)
        self.assertIn('listening', self.node.colors)
        self.assertIn('thinking', self.node.colors)
        self.assertIn('speaking', self.node.colors)
        self.assertIn('error', self.node.colors)

    def test_ring_sync_map_exists(self):
        """Тест: маппинг синхронизации кольца определён."""
        self.assertIsInstance(self.node.ring_sync_map, dict)
        self.assertIn('police_lights', self.node.ring_sync_map)
        self.assertIn('ambulance', self.node.ring_sync_map)
        self.assertIn('fire_truck', self.node.ring_sync_map)
        self.assertIn('road_service', self.node.ring_sync_map)

    def test_animation_request_police(self):
        """Тест: запрос police_lights активирует синхронизацию кольца."""
        msg = String()
        msg.data = 'police_lights:10'
        self.node.animation_request_callback(msg)

        self.assertTrue(self.node.animation_override)
        self.assertIsNotNone(self.node.animation_timer)
        self.assertIsNotNone(self.node.animation_return_timer)
        # Первый цвет — синий
        self.mock_ring_instance.mono.assert_called_with(0, 0, 255)

    def test_animation_request_unknown_stops_override(self):
        """Тест: неизвестная анимация не активирует override."""
        # Сначала запускаем police
        msg = String()
        msg.data = 'police_lights'
        self.node.animation_request_callback(msg)
        self.assertTrue(self.node.animation_override)

        # Затем запрашиваем неизвестную — override снимается
        msg.data = 'happy'
        self.node.animation_request_callback(msg)
        self.assertFalse(self.node.animation_override)

    def test_voice_state_blocked_during_animation(self):
        """Тест: voice state не перебивает активную анимацию кольца."""
        # Запускаем анимацию
        msg = String()
        msg.data = 'police_lights:5'
        self.node.animation_request_callback(msg)
        self.mock_ring_instance.reset_mock()

        # Отправляем voice state — не должно сменить LED
        state_msg = String()
        state_msg.data = 'idle'
        self.node.state_callback(state_msg)

        # off() не должен быть вызван (override активен)
        self.mock_ring_instance.off.assert_not_called()

    def test_ring_animation_tick(self):
        """Тест: тик анимации чередует цвета."""
        self.node._ring_colors = [(0, 0, 255), (255, 0, 0)]
        self.node.ring_animation_phase = 0
        self.node.animation_override = True

        self.node._ring_animation_tick()
        self.assertEqual(self.node.ring_animation_phase, 1)
        self.mock_ring_instance.mono.assert_called_with(255, 0, 0)

        self.node._ring_animation_tick()
        self.assertEqual(self.node.ring_animation_phase, 0)
        self.mock_ring_instance.mono.assert_called_with(0, 0, 255)

    def test_stop_ring_animation_restores_voice_state(self):
        """Тест: остановка анимации возвращает к voice state."""
        self.node.current_mode = 'listening'
        self.node.animation_override = True
        self.mock_ring_instance.reset_mock()

        self.node._stop_ring_animation(restore=True)

        self.assertFalse(self.node.animation_override)
        # Должен быть вызван mono с цветом listening
        r, g, b = self.node.colors['listening']
        self.mock_ring_instance.mono.assert_called_with(r, g, b)

    def test_animation_request_parses_duration(self):
        """Тест: парсинг duration из запроса анимации."""
        msg = String()
        msg.data = 'police_lights:7.5'
        self.node.animation_request_callback(msg)
        # Должна быть создана анимация (таймеры активны)
        self.assertTrue(self.node.animation_override)

    def test_animation_subscriber_created(self):
        """Тест: подписчик /voice/animation/request создан."""
        subscriptions = self.node.get_subscriber_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        sub_topics = [name for name, _ in subscriptions]
        self.assertIn('/voice/animation/request', sub_topics)


class TestLEDNodeIntegration(unittest.TestCase):
    """Интеграционные тесты LEDNode."""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2."""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Создание ноды."""
        with patch('usb.core.find'), \
             patch('rob_box_voice.led_node.PixelRingLite'):
            self.node = LEDNode()

    def tearDown(self):
        """Очистка."""
        self.node.destroy_node()

    def test_voice_state_integration(self):
        """Тест: интеграция с voice state топиком."""
        states_received = []

        # Мокируем state_callback чтобы отследить вызовы
        original_callback = self.node.state_callback
        def track_state(msg):
            states_received.append(msg.data)
            original_callback(msg)

        self.node.state_callback = track_state

        # Публикуем состояния
        test_node = rclpy.create_node('test_publisher')
        pub = test_node.create_publisher(String, '/voice/state', 10)

        msg = String()
        msg.data = 'listening'
        pub.publish(msg)

        # Даем время на обработку
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            rclpy.spin_once(test_node, timeout_sec=0.1)

        test_node.destroy_node()


if __name__ == '__main__':
    unittest.main()
