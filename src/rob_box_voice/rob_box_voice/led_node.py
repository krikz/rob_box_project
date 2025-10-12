#!/usr/bin/env python3
"""
LEDNode - управление 12× RGB LED на ReSpeaker Mic Array v2.0
Подписывается на: /voice/state, /audio/direction
Предоставляет сервис: /voice/set_led_mode
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from std_srvs.srv import SetBool
import usb.core
import usb.util
from typing import Optional


class PixelRingLite:
    """Упрощённая версия pixel_ring для управления LED"""
    
    # USB control commands
    CMD_TRACE = 0
    CMD_MONO = 1
    CMD_LISTEN = 2
    CMD_SPEAK = 3
    CMD_THINK = 4
    CMD_SPIN = 5
    CMD_BRIGHTNESS = 0x20
    CMD_COLOR_PALETTE = 0x21
    CMD_VAD_LED = 0x22
    CMD_VOLUME = 0x23
    
    VENDOR_ID = 0x2886
    PRODUCT_ID = 0x0018
    TIMEOUT = 1000
    
    def __init__(self):
        self.dev: Optional[usb.core.Device] = None
        self._brightness = 16
    
    def connect(self) -> bool:
        """Подключиться к ReSpeaker"""
        try:
            self.dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)
            return self.dev is not None
        except Exception as e:
            print(f"Ошибка подключения: {e}")
            return False
    
    def _send_command(self, command: int, data: bytes = b'\x00\x00\x00\x00'):
        """Отправить USB control command"""
        if self.dev is None:
            return False
        
        try:
            self.dev.ctrl_transfer(
                usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
                0,  # request
                command,  # value
                0x1C,  # index
                data,
                self.TIMEOUT
            )
            return True
        except Exception as e:
            print(f"Ошибка отправки команды {command}: {e}")
            return False
    
    def set_brightness(self, brightness: int):
        """Установить яркость (0-31)"""
        brightness = max(0, min(31, brightness))
        self._brightness = brightness
        data = bytes([brightness, 0, 0, 0])
        self._send_command(self.CMD_BRIGHTNESS, data)
    
    def off(self):
        """Выключить LED"""
        data = bytes([0, 0, 0, 0])
        self._send_command(self.CMD_MONO, data)
    
    def mono(self, r: int, g: int, b: int):
        """Установить один цвет для всех LED"""
        data = bytes([r, g, b, 0])
        self._send_command(self.CMD_MONO, data)
    
    def trace(self):
        """Режим trace (следит за DoA и VAD)"""
        data = bytes([0, 0, 0, 0])
        self._send_command(self.CMD_TRACE, data)
    
    def listen(self):
        """Режим listen (постоянное свечение по направлению)"""
        data = bytes([0, 0, 0, 0])
        self._send_command(self.CMD_LISTEN, data)
    
    def think(self):
        """Режим think (пульсация)"""
        data = bytes([0, 0, 0, 0])
        self._send_command(self.CMD_THINK, data)
    
    def speak(self):
        """Режим speak (вращение)"""
        data = bytes([0, 0, 0, 0])
        self._send_command(self.CMD_SPEAK, data)
    
    def spin(self):
        """Режим spin (быстрое вращение)"""
        data = bytes([0, 0, 0, 0])
        self._send_command(self.CMD_SPIN, data)
    
    def set_color_palette(self, color1_rgb: tuple, color2_rgb: tuple):
        """Установить цветовую палитру для think/speak режимов"""
        r1, g1, b1 = color1_rgb
        r2, g2, b2 = color2_rgb
        data = bytes([r1, g1, b1, 0, r2, g2, b2, 0])
        self._send_command(self.CMD_COLOR_PALETTE, data)
    
    def set_volume(self, volume: int):
        """Показать уровень громкости (0-12)"""
        volume = max(0, min(12, volume))
        data = bytes([volume, 0, 0, 0])
        self._send_command(self.CMD_VOLUME, data)


class LEDNode(Node):
    """Нода для управления LED индикацией ReSpeaker"""
    
    def __init__(self):
        super().__init__('led_node')
        
        # Параметры
        self.declare_parameter('brightness', 16)  # 0-31
        self.declare_parameter('auto_mode', True)
        self.declare_parameter('colors.idle', [0, 0, 0])
        self.declare_parameter('colors.listening', [0, 255, 0])  # Зелёный
        self.declare_parameter('colors.thinking', [0, 0, 255])   # Синий
        self.declare_parameter('colors.speaking', [0, 255, 255]) # Голубой
        self.declare_parameter('colors.error', [255, 0, 0])      # Красный
        
        self.brightness = self.get_parameter('brightness').value
        self.auto_mode = self.get_parameter('auto_mode').value
        
        # Цвета
        self.colors = {
            'idle': self.get_parameter('colors.idle').value,
            'listening': self.get_parameter('colors.listening').value,
            'thinking': self.get_parameter('colors.thinking').value,
            'speaking': self.get_parameter('colors.speaking').value,
            'error': self.get_parameter('colors.error').value,
        }
        
        # Pixel Ring
        self.pixel_ring = PixelRingLite()
        
        # Subscribers
        self.state_sub = self.create_subscription(
            String,
            '/voice/state',
            self.state_callback,
            10
        )
        
        self.direction_sub = self.create_subscription(
            Int32,
            '/audio/direction',
            self.direction_callback,
            10
        )
        
        # Service
        self.led_service = self.create_service(
            SetBool,
            '/voice/set_auto_led',
            self.set_auto_led_callback
        )
        
        # Состояние
        self.current_mode = 'idle'
        self.current_direction = 0
        
        # Инициализация
        self.get_logger().info('LEDNode инициализирован')
        self.initialize_hardware()
    
    def initialize_hardware(self):
        """Инициализация ReSpeaker LED"""
        if self.pixel_ring.connect():
            self.get_logger().info('✓ ReSpeaker LED подключен')
            self.pixel_ring.set_brightness(self.brightness)
            self.get_logger().info(f'  Яркость: {self.brightness}/31')
            
            # Настроить цветовую палитру для режимов
            thinking_color = tuple(self.colors['thinking'])
            speaking_color = tuple(self.colors['speaking'])
            self.pixel_ring.set_color_palette(thinking_color, speaking_color)
            
            # Начальное состояние
            self.pixel_ring.off()
        else:
            self.get_logger().error('❌ ReSpeaker LED не найден')
    
    def state_callback(self, msg: String):
        """Обработка изменения состояния голосового ассистента"""
        if not self.auto_mode:
            return
        
        state = msg.data.lower()
        self.current_mode = state
        
        self.get_logger().debug(f'State changed: {state}')
        
        # Переключение режима LED
        if state == 'idle':
            self.pixel_ring.off()
        
        elif state == 'listening':
            # Зелёное свечение
            r, g, b = self.colors['listening']
            self.pixel_ring.mono(r, g, b)
        
        elif state in ['thinking', 'generating']:
            # Синяя пульсация
            self.pixel_ring.think()
        
        elif state == 'speaking':
            # Голубое вращение
            self.pixel_ring.speak()
        
        elif state == 'error':
            # Красное свечение
            r, g, b = self.colors['error']
            self.pixel_ring.mono(r, g, b)
        
        elif state == 'trace':
            # Следование за направлением звука
            self.pixel_ring.trace()
    
    def direction_callback(self, msg: Int32):
        """Обработка изменения направления на источник звука"""
        self.current_direction = msg.data
        # DoA автоматически обрабатывается в trace режиме
    
    def set_auto_led_callback(self, request, response):
        """Сервис для включения/выключения автоматического режима"""
        self.auto_mode = request.data
        response.success = True
        response.message = f"Auto LED mode: {'ON' if self.auto_mode else 'OFF'}"
        self.get_logger().info(response.message)
        
        if not self.auto_mode:
            # Выключить LED при отключении авто режима
            self.pixel_ring.off()
        
        return response
    
    def set_mode_manual(self, mode: str):
        """Ручная установка режима (для тестирования)"""
        if mode == 'off':
            self.pixel_ring.off()
        elif mode == 'trace':
            self.pixel_ring.trace()
        elif mode == 'listen':
            self.pixel_ring.listen()
        elif mode == 'think':
            self.pixel_ring.think()
        elif mode == 'speak':
            self.pixel_ring.speak()
        elif mode == 'spin':
            self.pixel_ring.spin()
    
    def shutdown(self):
        """Корректное завершение"""
        self.get_logger().info('Остановка LEDNode...')
        self.pixel_ring.off()
        self.get_logger().info('✓ LEDNode остановлен')


def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
