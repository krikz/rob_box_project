#!/usr/bin/env python3
"""
vision_stub_node.py - Заглушка для Vision Context

Временная нода, которая публикует фейковый семантический контекст
вместо реальной обработки изображений.

В будущем здесь будет:
- Обработка на AI HAT 26 TOPS
- YOLO детекция объектов
- Анализ нескольких камер (front stereo, up camera)

Сейчас:
- Подписывается на /oak/rgb/image_raw/compressed
- Публикует /perception/vision_context с заглушкой
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class VisionStubNode(Node):
    """Заглушка для vision processing"""
    
    def __init__(self):
        super().__init__('vision_stub_node')
        
        # Параметры
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Статистика
        self.frame_count = 0
        self.last_frame_time = None
        
        # Подписка на камеру
        # ИСПРАВЛЕНО: OAK-D публикует в /camera/rgb/..., а не /oak/rgb/...
        self.camera_sub = self.create_subscription(
            CompressedImage,
            '/camera/rgb/image_raw/compressed',
            self.on_camera_frame,
            10
        )
        
        # Публикация контекста
        self.context_pub = self.create_publisher(
            String,
            '/perception/vision_context',
            10
        )
        
        # Таймер для публикации (независимо от камеры)
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_context)
        
        self.get_logger().info('👁️  Vision Stub Node запущен')
        self.get_logger().info(f'   Частота публикации: {self.publish_rate} Hz')
        self.get_logger().warn('⚠️  Это ЗАГЛУШКА! Используйте AI HAT + YOLO для реальной обработки')
    
    def on_camera_frame(self, msg: CompressedImage):
        """Получен кадр с камеры"""
        self.frame_count += 1
        self.last_frame_time = time.time()
        
        # В реальной версии здесь будет:
        # 1. Декодирование JPEG
        # 2. Inference на AI HAT (YOLO)
        # 3. Парсинг результатов
        # 4. Публикация семантического контекста
        
        if self.frame_count % 30 == 0:  # Каждые 30 кадров
            self.get_logger().debug(f'📸 Получено {self.frame_count} кадров')
    
    def publish_context(self):
        """Публикация семантического контекста (СТАБ)"""
        # Фейковый контекст для тестирования
        context = {
            'timestamp': time.time(),
            'camera': 'oak-d-front-stereo',
            'frame_count': self.frame_count,
            'camera_active': self.last_frame_time is not None,
            'objects': [
                # Пока пусто - в будущем YOLO детекции
                # {'type': 'person', 'confidence': 0.95, 'bbox': [x, y, w, h]},
                # {'type': 'chair', 'confidence': 0.87, 'bbox': [x, y, w, h]},
            ],
            'description': 'Заглушка: камера работает, ждём AI HAT + YOLO',
            'note': 'TODO: Реализовать обработку на AI HAT с YOLO v8/v11'
        }
        
        # Если камера недавно отправляла кадры
        if self.last_frame_time:
            elapsed = time.time() - self.last_frame_time
            if elapsed < 5.0:  # Кадры были в последние 5 секунд
                context['camera_active'] = True
                context['last_frame_ago'] = f'{elapsed:.1f}s'
            else:
                context['camera_active'] = False
                context['description'] = 'Камера не отправляет кадры'
        
        # Публикация
        msg = String()
        msg.data = json.dumps(context, ensure_ascii=False)
        self.context_pub.publish(msg)
        
        if self.frame_count % 10 == 0:
            self.get_logger().debug(
                f'📤 Vision context: frames={self.frame_count}, '
                f'active={context["camera_active"]}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VisionStubNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
