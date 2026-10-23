#!/usr/bin/env python3
"""
LEDNode - управление 12× RGB LED на ReSpeaker Mic Array v2.0
Подписывается на: /dialog/turn_event, /dialog/session (только тишина; v1: /voice/dialogue/state), /audio/direction
Публикует: /voice/expression/event (ExpressionEvent{executor=ring})
Предоставляет сервис: /voice/set_auto_led

Режим кольца на стадии хода (addressed -> thinking -> speaking -> done|failed) задаёт таблица
rob_box_dialog.knowledge.REFLEXES, исполняет core/ring_reflex.py (ADR-0150 §5.5, PR-3b #3690).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rob_box_core import voice_state
from rob_box_dialog_msgs.msg import ExpressionEvent, SessionState, TurnEvent
from std_msgs.msg import String, Int32
from std_srvs.srv import SetBool
import usb.core
import usb.util
from typing import Optional

from .core.ring_reflex import RingReflex

# TEMP(ADR-0148, PR-15 15-0a): состояние диалога v1 публикует dialogue_node (std_msgs/String, имя
# DialogueStateKind: IDLE / LISTENING / DIALOGUE / SILENCED); при dialog_engine=v2 тишину даёт сцена ``Silence``
# в SessionState (``/dialog/session``, issue #3911). Топика ``/voice/state`` на роботе никто не публикует — это
# старое имя (ADR-0027 #2, arbiter_node.VOICE_DIALOGUE_STATE_TOPIC), issue #3107.
VOICE_STATE_TOPIC = '/voice/dialogue/state'


class PixelRingLite:
    """Упрощённая версия pixel_ring для управления LED."""

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
        """Подключиться к ReSpeaker."""
        try:
            self.dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)
            return self.dev is not None
        except Exception as e:
            print(f"Ошибка подключения: {e}")
            return False

    def _send_command(self, command: int, data: bytes = b'\x00\x00\x00\x00'):
        """Отправить USB control command."""
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
        """Установить яркость (0-31)."""
        brightness = max(0, min(31, brightness))
        self._brightness = brightness
        data = bytes([brightness, 0, 0, 0])
        self._send_command(self.CMD_BRIGHTNESS, data)

    def off(self):
        """Выключить LED."""
        data = bytes([0, 0, 0, 0])
        return self._send_command(self.CMD_MONO, data)

    def mono(self, r: int, g: int, b: int):
        """Установить один цвет для всех LED."""
        data = bytes([r, g, b, 0])
        return self._send_command(self.CMD_MONO, data)

    def trace(self):
        """Режим trace (следит за DoA и VAD)."""
        data = bytes([0, 0, 0, 0])
        return self._send_command(self.CMD_TRACE, data)

    def listen(self):
        """Режим listen (постоянное свечение по направлению)."""
        data = bytes([0, 0, 0, 0])
        return self._send_command(self.CMD_LISTEN, data)

    def think(self):
        """Режим think (пульсация)."""
        data = bytes([0, 0, 0, 0])
        return self._send_command(self.CMD_THINK, data)

    def speak(self):
        """Режим speak (вращение)."""
        data = bytes([0, 0, 0, 0])
        return self._send_command(self.CMD_SPEAK, data)

    def spin(self):
        """Режим spin (быстрое вращение)."""
        data = bytes([0, 0, 0, 0])
        return self._send_command(self.CMD_SPIN, data)

    def set_color_palette(self, color1_rgb: tuple, color2_rgb: tuple):
        """Установить цветовую палитру для think/speak режимов."""
        r1, g1, b1 = color1_rgb
        r2, g2, b2 = color2_rgb
        data = bytes([r1, g1, b1, 0, r2, g2, b2, 0])
        self._send_command(self.CMD_COLOR_PALETTE, data)

    def set_volume(self, volume: int):
        """Показать уровень громкости (0-12)."""
        volume = max(0, min(12, volume))
        data = bytes([volume, 0, 0, 0])
        self._send_command(self.CMD_VOLUME, data)


class LEDNode(Node):
    """Нода кольца LED ReSpeaker: исполнитель рефлексов хода (ADR-0150 §5.5.2) и тишины."""

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

        # Цвета (палитра think/speak аппаратных режимов кольца)
        self.colors = {
            'idle': self.get_parameter('colors.idle').value,
            'listening': self.get_parameter('colors.listening').value,
            'thinking': self.get_parameter('colors.thinking').value,
            'speaking': self.get_parameter('colors.speaking').value,
            'error': self.get_parameter('colors.error').value,
        }

        # Pixel Ring
        self.pixel_ring = PixelRingLite()

        # Режим кольца на стадии хода решает таблица REFLEXES; ответ на каждое включение - ExpressionEvent.
        self._event_pub = self.create_publisher(ExpressionEvent, '/voice/expression/event', 10)
        self.reflex = RingReflex(self.pixel_ring, self._call_later, self._answer)

        # Subscribers
        # Стадии хода (ADR-0150 §2.3): кольцо живёт по ним; тишина - сцена Silence в /dialog/session (v2) или
        # строка /voice/dialogue/state (v1, TEMP до PR-15).
        self.turn_sub = self.create_subscription(TurnEvent, '/dialog/turn_event', self.turn_event_callback, 10)
        self._voice = voice_state.VoiceStateTracker()
        self.session_sub = self.create_subscription(
            SessionState,
            voice_state.SESSION_TOPIC,
            self.session_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.state_sub = self.create_subscription(
            String,
            VOICE_STATE_TOPIC,
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

    def _call_later(self, delay_s: float, callback):
        """Одноразовый таймер для ``RingReflex`` (вспышка, страховка от зависшего режима)."""
        timer = None

        def fire():
            timer.cancel()
            callback()

        timer = self.create_timer(delay_s, fire)
        return timer

    def _answer(self, turn_id: str, started: bool, reason: str) -> None:
        ev = ExpressionEvent()
        ev.executor, ev.turn_id, ev.reason = 'ring', turn_id, reason
        ev.kind = ExpressionEvent.KIND_STARTED if started else ExpressionEvent.KIND_SKIPPED
        self._event_pub.publish(ev)

    def initialize_hardware(self):
        """Инициализация ReSpeaker LED."""
        if self.pixel_ring.connect():
            self.get_logger().info('✓ ReSpeaker LED подключен')
            self.pixel_ring.set_brightness(self.brightness)
            self.get_logger().info(f'  Яркость: {self.brightness}/31')

            # Настроить цветовую палитру для режимов
            thinking_color = tuple(self.colors['thinking'])
            speaking_color = tuple(self.colors['speaking'])
            self.pixel_ring.set_color_palette(thinking_color, speaking_color)

            # Начальное состояние — trace (DoA tracking)
            self.pixel_ring.trace()
        else:
            self.get_logger().error('❌ ReSpeaker LED не найден')

    def turn_event_callback(self, msg: TurnEvent):
        """Стадия хода -> режим кольца из ``REFLEXES`` (``addressed`` - сторона говорящего, ``failed`` - красная)."""
        if not self.auto_mode:
            return
        mode = self.reflex.on_turn_event(msg.turn_id, msg.stage, msg.status)
        if mode is not None:
            self.get_logger().info(f'кольцо: {msg.stage} -> {mode} (turn {msg.turn_id[:8]})')

    def session_callback(self, msg: SessionState):
        """Снимок сессии v2: сцена ``Silence`` гасит кольцо, её снятие возвращает покой (остальное - стадии хода)."""
        if not self.auto_mode:
            return
        changed = self._voice.on_session(msg)
        if changed is not None:
            self.reflex.on_state('silenced' if changed.detail == voice_state.SILENCED_DETAIL else 'idle')

    def state_callback(self, msg: String):
        """TEMP(ADR-0148, PR-15): состояние диалога v1 - кольцо гаснет в тишине (SILENCED), остальное - стадии хода."""
        if not self.auto_mode:
            return
        state = msg.data.lower()
        self.current_mode = state
        self.reflex.on_state(state)

    def direction_callback(self, msg: Int32):
        """Обработка изменения направления на источник звука."""
        self.current_direction = msg.data
        # DoA автоматически обрабатывается в trace режиме

    def set_auto_led_callback(self, request, response):
        """Сервис для включения/выключения автоматического режима."""
        self.auto_mode = request.data
        response.success = True
        response.message = f"Auto LED mode: {'ON' if self.auto_mode else 'OFF'}"
        self.get_logger().info(response.message)

        if not self.auto_mode:
            # Переключить на trace при отключении авто режима
            self.pixel_ring.trace()

        return response

    def set_mode_manual(self, mode: str):
        """Ручная установка режима (для тестирования)."""
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
        """Корректное завершение."""
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
