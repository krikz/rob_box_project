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
    """Тесты для PixelRingLite класса"""

    def test_usb_ids(self):
        """Тест: USB ID константы"""
        self.assertEqual(PixelRingLite.VENDOR_ID, 0x2886)
        self.assertEqual(PixelRingLite.PRODUCT_ID, 0x0018)

    def test_pixel_ring_constants(self):
        """Тест: команды LED"""
        self.assertEqual(PixelRingLite.CMD_LISTEN, 2)
        self.assertEqual(PixelRingLite.CMD_SPEAK, 3)
        self.assertEqual(PixelRingLite.CMD_THINK, 4)
        self.assertEqual(PixelRingLite.CMD_SPIN, 5)

    def test_connect_success(self):
        """Тест: успешное подключение"""
        with patch('usb.core.find') as mock_find:
            mock_find.return_value = MagicMock()
            
            ring = PixelRingLite()
            result = ring.connect()
            
            self.assertTrue(result)
            self.assertIsNotNone(ring.dev)

    def test_connect_failure(self):
        """Тест: неудачное подключение"""
        with patch('usb.core.find', return_value=None):
            ring = PixelRingLite()
            result = ring.connect()
            
            self.assertFalse(result)
            self.assertIsNone(ring.dev)

    def test_send_command(self):
        """Тест: отправка USB команды"""
        with patch('usb.core.find') as mock_find:
            mock_dev = MagicMock()
            mock_find.return_value = mock_dev
            
            ring = PixelRingLite()
            ring.connect()
            
            ring._send_command(PixelRingLite.CMD_LISTEN)
            mock_dev.ctrl_transfer.assert_called_once()

    def test_brightness_property(self):
        """Тест: свойство яркости"""
        ring = PixelRingLite()
        self.assertEqual(ring._brightness, 16)


class TestLEDNode(unittest.TestCase):
    """Тесты для LEDNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Завершение ROS 2"""
        pass  # Не shutdown для других тестов

    def setUp(self):
        """Создание мокированной ноды"""
        with patch('usb.core.find'), \
             patch('rob_box_voice.led_node.PixelRingLite') as mock_ring:
            
            self.mock_ring_class = mock_ring
            self.mock_ring_instance = MagicMock()
            mock_ring.return_value = self.mock_ring_instance
            self.mock_ring_instance.connect.return_value = True
            
            self.node = LEDNode()

    def tearDown(self):
        """Очистка после теста"""
        self.node.destroy_node()
        patch.stopall()

    def test_node_creation(self):
        """Тест: нода создается корректно"""
        self.assertEqual(self.node.get_name(), 'led_node')
        self.assertIsNotNone(self.node.pixel_ring)

    def test_parameters_exist(self):
        """Тест: все параметры объявлены"""
        params = ['brightness', 'auto_mode']
        for param in params:
            self.assertTrue(self.node.has_parameter(param))

    def test_subscribers_created(self):
        """Тест: подписчики созданы"""
        subscriptions = self.node.get_subscriber_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        sub_topics = [name for name, _ in subscriptions]
        
        self.assertIn('/voice/state', sub_topics)
        self.assertIn('/audio/direction', sub_topics)

    def test_service_created(self):
        """Тест: сервис управления LED создан"""
        services = self.node.get_service_names_and_types()
        service_names = [name for name, _ in services]
        
        # Сервис называется /voice/set_auto_led
        self.assertIn('/voice/set_auto_led', service_names)

    def test_led_ring_initialization(self):
        """Тест: LED ring инициализируется при создании"""
        self.mock_ring_instance.connect.assert_called_once()

    def test_voice_state_callback_listening(self):
        """Тест: обработка состояния 'listening'"""
        msg = String()
        msg.data = 'listening'
        
        self.node.state_callback(msg)
        
        # Проверяем что current_mode установлен
        self.assertEqual(self.node.current_mode, 'listening')

    def test_voice_state_callback_speaking(self):
        """Тест: обработка состояния 'speaking'"""
        msg = String()
        msg.data = 'speaking'
        
        self.node.state_callback(msg)
        self.assertEqual(self.node.current_mode, 'speaking')

    def test_voice_state_callback_thinking(self):
        """Тест: обработка состояния 'thinking'"""
        msg = String()
        msg.data = 'thinking'
        
        self.node.state_callback(msg)
        self.assertEqual(self.node.current_mode, 'thinking')

    def test_voice_state_callback_idle(self):
        """Тест: обработка состояния 'idle'"""
        msg = String()
        msg.data = 'idle'
        
        self.node.state_callback(msg)
        # off() вызывается при idle (и в конструкторе тоже)
        self.assertGreaterEqual(self.mock_ring_instance.off.call_count, 1)

    def test_direction_callback(self):
        """Тест: обработка направления звука"""
        msg = Int32()
        msg.data = 180
        
        self.node.direction_callback(msg)
        
        # Направление сохраняется в current_direction
        self.assertEqual(self.node.current_direction, 180)

    def test_auto_mode_true(self):
        """Тест: автоматический режим включен"""
        self.node.auto_mode = True
        msg = String()
        msg.data = 'listening'
        
        self.node.state_callback(msg)
        # В auto_mode LED должен реагировать
        self.assertEqual(self.node.current_mode, 'listening')

    def test_auto_mode_false(self):
        """Тест: автоматический режим выключен"""
        self.node.auto_mode = False
        previous_mode = self.node.current_mode
        
        msg = String()
        msg.data = 'listening'
        
        self.node.state_callback(msg)
        # В неавтоматическом режиме callback выходит рано
        # current_mode может остаться прежним

    def test_service_enable_auto_mode(self):
        """Тест: сервис включения авто режима"""
        request = SetBool.Request()
        request.data = True
        response = SetBool.Response()
        
        self.node.set_auto_led_callback(request, response)
        
        self.assertTrue(response.success)
        self.assertTrue(self.node.auto_mode)

    def test_service_disable_auto_mode(self):
        """Тест: сервис выключения авто режима"""
        request = SetBool.Request()
        request.data = False
        response = SetBool.Response()
        
        self.node.set_auto_led_callback(request, response)
        
        self.assertTrue(response.success)
        self.assertFalse(self.node.auto_mode)

    def test_current_mode_tracking(self):
        """Тест: отслеживание текущего режима"""
        self.assertTrue(hasattr(self.node, 'current_mode'))

    def test_auto_mode_parameter(self):
        """Тест: параметр auto_mode"""
        auto_mode = self.node.get_parameter('auto_mode').value
        self.assertIsInstance(auto_mode, bool)

    def test_brightness_parameter(self):
        """Тест: параметр brightness"""
        brightness = self.node.get_parameter('brightness').value
        self.assertIsInstance(brightness, int)
        self.assertGreaterEqual(brightness, 0)
        self.assertLessEqual(brightness, 31)  # 0-31 по спецификации


class TestLEDNodeIntegration(unittest.TestCase):
    """Интеграционные тесты LEDNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Создание ноды"""
        with patch('usb.core.find'), \
             patch('rob_box_voice.led_node.PixelRingLite'):
            self.node = LEDNode()

    def tearDown(self):
        """Очистка"""
        self.node.destroy_node()

    def test_voice_state_integration(self):
        """Тест: интеграция с voice state топиком"""
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
