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
- Подписывается на камерные топики (параметр camera_topics)
- Публикует /perception/vision_context с заглушкой
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class VisionStubNode(Node):
    """Заглушка для vision processing с поддержкой нескольких камер"""
    
    def __init__(self):
        super().__init__('vision_stub_node')
        
        # Параметры
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('context_topic', '/perception/vision_context')
        self.declare_parameter('camera_topics', [
            '/camera/rgb/image_raw/compressed',
            # '/camera/stereo/image_raw/compressed',  # Можно добавить стерео
            # '/camera_up/rgb/image_raw/compressed',  # Можно добавить верхнюю камеру
        ])
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.context_topic = self.get_parameter('context_topic').value
        self.camera_topics = self.get_parameter('camera_topics').value
        
        # Статистика по каждой камере
        self.camera_stats = {}
        for topic in self.camera_topics:
            self.camera_stats[topic] = {
                'frame_count': 0,
                'last_frame_time': None,
            }
        
        # Подписка на все камерные топики
        self.camera_subs = []
        for topic in self.camera_topics:
            sub = self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, t=topic: self.on_camera_frame(msg, t),
                10
            )
            self.camera_subs.append(sub)
            self.get_logger().info(f'📷 Подписка на камеру: {topic}')
        
        # Публикация контекста
        self.context_pub = self.create_publisher(
            String,
            self.context_topic,
            10
        )
        
        # Таймер для публикации (независимо от камеры)
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_context)
        
        self.get_logger().info('👁️  Vision Stub Node запущен')
        self.get_logger().info(f'   Частота публикации: {self.publish_rate} Hz')
        self.get_logger().info(f'   Топик контекста: {self.context_topic}')
        self.get_logger().info(f'   Камер: {len(self.camera_topics)}')
        self.get_logger().warn('⚠️  Это ЗАГЛУШКА! Используйте AI HAT + YOLO для реальной обработки')
    
    def on_camera_frame(self, msg: CompressedImage, topic: str):
        """Обработка фрейма с камеры
        
        В реальной версии здесь будет:
        1. Декодирование JPEG
        2. Inference на AI HAT (YOLO)
        3. Парсинг результатов
        4. Публикация семантического контекста
        
        Args:
            msg: Сжатое изображение
            topic: Топик, откуда пришел фрейм
        """
        stats = self.camera_stats[topic]
        stats['frame_count'] += 1
        stats['last_frame_time'] = time.time()
        
        if stats['frame_count'] % 30 == 0:  # Каждые 30 фреймов
            self.get_logger().debug(
                f'📸 Камера {topic}: {stats["frame_count"]} фреймов'
            )
    
    def publish_context(self):
        """Публикация фейкового vision context"""
        
        # Собираем статистику по всем камерам
        active_cameras = []
        total_frames = 0
        
        for topic, stats in self.camera_stats.items():
            total_frames += stats['frame_count']
            if stats['last_frame_time'] and (time.time() - stats['last_frame_time'] < 2.0):
                active_cameras.append({
                    'topic': topic,
                    'frames': stats['frame_count']
                })
        
        # Формируем контекст
        context = {
            'timestamp': time.time(),
            'source': 'vision_stub',
            'cameras': {
                'active': len(active_cameras),
                'total': len(self.camera_topics),
                'details': active_cameras,
            },
            'summary': f"Активных камер: {len(active_cameras)}/{len(self.camera_topics)}, получено {total_frames} фреймов",
            
            # Фейковые данные (заглушка)
            'detected_objects': [
                {'class': 'person', 'confidence': 0.95, 'camera': 'front'},
                {'class': 'chair', 'confidence': 0.87, 'camera': 'front'},
            ] if active_cameras else [],
            
            'scene_description': "Помещение с мебелью" if active_cameras else "Нет активных камер",
            'is_stub': True,
        }
        
        msg = String()
        msg.data = json.dumps(context, ensure_ascii=False)
        self.context_pub.publish(msg)


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
