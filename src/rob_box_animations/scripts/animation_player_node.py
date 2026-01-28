#!/usr/bin/env python3
"""
Animation Player Node

ROS2 node for playing LED matrix animations.
Provides services for loading and controlling animations.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from ament_index_python.packages import get_package_share_directory
import os
import random

from rob_box_animations.animation_player import AnimationPlayer


class AnimationPlayerNode(Node):
    """ROS2 node for animation playback"""

    def __init__(self):
        super().__init__('animation_player')

        # Parameters
        self.declare_parameter('animations_dir', '')
        self.declare_parameter('autostart_animation', '')
        self.declare_parameter('loop', True)

        animations_dir = self.get_parameter('animations_dir').value
        if not animations_dir:
            # Use package share directory
            try:
                pkg_share = get_package_share_directory('rob_box_animations')
                animations_dir = os.path.join(pkg_share, 'animations')
            except Exception:
                animations_dir = None

        # Create player
        self.player = AnimationPlayer(self, animations_dir)

        # Subscription for loading animations (instead of service)
        self.load_subscription = self.create_subscription(
            String,
            '~/load_animation',
            self.load_animation_callback,
            10
        )

        # Subscription for voice animation requests (from GUI or dialogue)
        self.voice_animation_subscription = self.create_subscription(
            String,
            '/voice/animation/request',
            self.voice_animation_callback,
            10
        )

        # Subscription for TTS state (auto-switch animations)
        self.tts_state_subscription = self.create_subscription(
            String,
            '/voice/tts/state',
            self.tts_state_callback,
            10
        )

        # Track TTS state for animation switching
        self.idle_animation = self.get_parameter('autostart_animation').value or 'idle'
        self.talking_animation = 'talking'
        self.is_robot_speaking = False
        self.manual_animation_active = False  # Флаг ручной анимации
        self.emotion_timer = None  # Таймер для возврата из эмоциональной анимации

        self.get_logger().info(f'✅ Subscribed to /voice/tts/state for automatic animation switching')
        self.get_logger().info(f'   Idle animation: {self.idle_animation}')
        self.get_logger().info(f'   Talking animation: {self.talking_animation}')

        # Services
        self.srv_play = self.create_service(
            Trigger,
            '~/play',
            self.play_callback
        )

        self.srv_stop = self.create_service(
            Trigger,
            '~/stop',
            self.stop_callback
        )

        self.srv_pause = self.create_service(
            SetBool,
            '~/pause',
            self.pause_callback
        )

        self.srv_list = self.create_service(
            Trigger,
            '~/list_animations',
            self.list_animations_callback
        )

        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            '~/status',
            10
        )

        # Status timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Animation Player Node started')

        # Autostart animation if specified
        autostart = self.get_parameter('autostart_animation').value
        if autostart:
            self.get_logger().info(f'Autostarting animation: {autostart}')
            if self.player.load_animation(f'{autostart}.yaml'):
                self.player.play()

    def load_animation_callback(self, msg):
        """Load animation callback (subscription)"""
        manifest_path = msg.data

        if not manifest_path.endswith('.yaml'):
            manifest_path += '.yaml'

        success = self.player.load_animation(manifest_path)

        if success:
            self.get_logger().info(f'Loaded animation: {manifest_path}')
        else:
            self.get_logger().error(f'Failed to load animation: {manifest_path}')

    def voice_animation_callback(self, msg):
        """Handle voice animation requests from GUI or dialogue node"""
        animation_request = msg.data.strip()
        
        self.get_logger().info(f'🎨 Получен запрос на анимацию: {animation_request}')
        
        # Парсим формат: "animation_name" или "animation_name:duration"
        animation_name = animation_request
        duration = None
        
        if ':' in animation_request:
            parts = animation_request.split(':', 1)
            animation_name = parts[0].strip()
            try:
                duration = float(parts[1].strip())
                self.get_logger().info(f'📏 Указана длительность: {duration}s')
            except ValueError:
                self.get_logger().warn(f'⚠️  Некорректный формат длительности: {parts[1]}, игнорирую')
        
        if not animation_name.endswith('.yaml'):
            animation_name += '.yaml'
        
        # Отменяем предыдущий таймер если есть
        if self.emotion_timer is not None:
            self.emotion_timer.cancel()
            self.emotion_timer = None
        
        # Устанавливаем флаг ручной анимации
        self.manual_animation_active = True
        
        # Load and play the animation
        if self.player.load_animation(animation_name):
            self.player.play()
            self.get_logger().info(f'✅ Анимация {animation_name} загружена и запущена (ручной режим)')
            
            # Используем указанную длительность или случайную (5-10 секунд)
            if duration is not None and duration > 0:
                timeout = duration
                self.get_logger().info(f'⏱️  Таймер возврата к idle (из запроса): {timeout:.1f}s')
            else:
                timeout = random.uniform(5.0, 10.0)
                self.get_logger().info(f'⏱️  Таймер возврата к idle (случайный): {timeout:.1f}s')
            
            self.emotion_timer = self.create_timer(timeout, self._return_to_idle)
        else:
            self.get_logger().error(f'❌ Не удалось загрузить анимацию: {animation_name}')
            self.manual_animation_active = False
    
    def _return_to_idle(self):
        """Callback для возврата к idle анимации после эмоциональной анимации"""
        if self.emotion_timer is not None:
            self.emotion_timer.cancel()
            self.emotion_timer = None
        
        self.manual_animation_active = False
        
        # Если робот не говорит - возвращаемся к idle
        if not self.is_robot_speaking:
            self.get_logger().info('⏰ Таймер истёк - возврат к idle анимации')
            if self.player.load_animation(f'{self.idle_animation}.yaml'):
                self.player.play()
            else:
                self.get_logger().warn(f'⚠️  Не найдена анимация {self.idle_animation}.yaml')
        else:
            # Если робот говорит - переключаемся на talking
            self.get_logger().info('⏰ Таймер истёк - переключение на talking (робот говорит)')
            if self.player.load_animation(f'{self.talking_animation}.yaml'):
                self.player.play()
            else:
                self.get_logger().warn(f'⚠️  Не найдена анимация {self.talking_animation}.yaml')

    def tts_state_callback(self, msg):
        """Handle TTS state changes - switch between idle and talking animations"""
        state = msg.data

        # Если активна ручная анимация - не переключаем автоматически
        if self.manual_animation_active:
            self.get_logger().debug(f'⏸️  Пропуск авто-переключения: активна ручная анимация (TTS state: {state})')
            # Сбрасываем флаг при завершении речи, чтобы вернуться к idle
            if state in ['ready', 'idle', 'stopped']:
                self.manual_animation_active = False
                self.get_logger().info('🔄 Ручной режим завершён, возврат к автопереключению')
            return

        if state in ['synthesizing', 'playing']:
            # Robot is speaking - switch to talking animation
            if not self.is_robot_speaking:
                self.get_logger().info('🗣️ Робот говорит - переключаюсь на talking анимацию')
                self.is_robot_speaking = True
                if self.player.load_animation(f'{self.talking_animation}.yaml'):
                    self.player.play()
                else:
                    self.get_logger().warn(f'⚠️  Не найдена анимация {self.talking_animation}.yaml')

        elif state in ['ready', 'idle', 'stopped']:
            # Robot stopped speaking - switch back to idle animation
            if self.is_robot_speaking:
                self.get_logger().info('🤐 Робот замолчал - возвращаюсь на idle анимацию')
                self.is_robot_speaking = False
                if self.player.load_animation(f'{self.idle_animation}.yaml'):
                    self.player.play()
                else:
                    self.get_logger().warn(f'⚠️  Не найдена анимация {self.idle_animation}.yaml')

    def play_callback(self, request, response):
        """Play animation service callback"""
        success = self.player.play()

        response.success = success
        if success:
            response.message = 'Playback started'
        else:
            response.message = 'Failed to start playback'

        return response

    def stop_callback(self, request, response):
        """Stop animation service callback"""
        self.player.stop()

        response.success = True
        response.message = 'Playback stopped'

        return response

    def pause_callback(self, request, response):
        """Pause/resume animation service callback"""
        if request.data:
            self.player.pause()
            response.success = True
            response.message = 'Playback paused'
        else:
            self.player.resume()
            response.success = True
            response.message = 'Playback resumed'

        return response

    def list_animations_callback(self, request, response):
        """List animations service callback"""
        animations = self.player.list_animations()

        response.success = True
        response.message = '\n'.join(animations) if animations else 'No animations found'

        return response

    def publish_status(self):
        """Publish current status"""
        status = self.player.get_status()

        msg = String()
        msg.data = (
            f"Animation: {status['animation']}, "
            f"Playing: {status['is_playing']}, "
            f"Paused: {status['is_paused']}, "
            f"Frames: {status['frames_played']}, "
            f"Loops: {status['loops_completed']}"
        )

        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = AnimationPlayerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.player.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
