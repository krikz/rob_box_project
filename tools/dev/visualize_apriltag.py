#!/usr/bin/env python3
"""
Визуализация AprilTag детекций с наложением на изображение
Показывает рамки вокруг маркеров и их данные (ID, расстояние, углы)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from apriltag_msgs.msg import AprilTagDetectionArray
import cv2
import numpy as np
from cv_bridge import CvBridge
import math


class AprilTagVisualizer(Node):
    def __init__(self):
        super().__init__('apriltag_visualizer')
        
        self.bridge = CvBridge()
        self.latest_detections = None
        self.latest_image = None
        
        # Подписки
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/rgb/image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        self.get_logger().info('🎨 AprilTag Visualizer запущен')
        self.get_logger().info('📷 Подписка на: /camera/rgb/image_raw/compressed')
        self.get_logger().info('🏷️  Подписка на: /detections')
        
    def detection_callback(self, msg):
        """Сохранить последние детекции"""
        self.latest_detections = msg
        
    def image_callback(self, msg):
        """Обработать изображение и наложить детекции"""
        try:
            # Декодировать compressed изображение
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                return
            
            # Наложить детекции если есть
            if self.latest_detections is not None:
                image = self.draw_detections(image, self.latest_detections)
            
            # Показать изображение
            cv2.imshow('AprilTag Detection', image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Ошибка обработки изображения: {e}')
    
    def draw_detections(self, image, detections_msg):
        """Нарисовать рамки и данные для всех детекций"""
        
        for detection in detections_msg.detections:
            # Получить углы
            corners = detection.corners
            
            # Нарисовать рамку (4 линии между углами)
            pts = []
            for corner in corners:
                pts.append([int(corner.x), int(corner.y)])
            pts = np.array(pts, np.int32)
            pts = pts.reshape((-1, 1, 2))
            
            # Зелёная рамка
            cv2.polylines(image, [pts], True, (0, 255, 0), 3)
            
            # Нарисовать углы (красные точки)
            for corner in corners:
                cv2.circle(image, (int(corner.x), int(corner.y)), 5, (0, 0, 255), -1)
            
            # Центр маркера
            center_x = int(detection.centre.x)
            center_y = int(detection.centre.y)
            cv2.circle(image, (center_x, center_y), 8, (255, 0, 0), -1)
            
            # Вычислить расстояние (если есть pose)
            distance_text = ""
            try:
                # Попробовать получить позицию из pose
                # AprilTag detection может иметь pose_R и pose_t
                # Но обычно расстояние в z coordinate
                distance_text = f"{detection.id}"
            except:
                distance_text = f"ID: {detection.id}"
            
            # Текст с данными
            tag_id = detection.id
            margin = detection.decision_margin
            
            # Основной текст - ID
            text_id = f"TAG {tag_id}"
            cv2.putText(image, text_id, 
                       (center_x - 40, center_y - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Качество детекции
            text_quality = f"Q: {margin:.1f}"
            cv2.putText(image, text_quality,
                       (center_x - 40, center_y + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Hamming distance (ошибки)
            if detection.hamming == 0:
                text_hamming = "Perfect"
                color = (0, 255, 0)
            else:
                text_hamming = f"Err: {detection.hamming}"
                color = (0, 165, 255)
            
            cv2.putText(image, text_hamming,
                       (center_x - 40, center_y + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Информация о количестве детекций
        info_text = f"Detections: {len(detections_msg.detections)}"
        cv2.putText(image, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        return image


def main(args=None):
    rclpy.init(args=args)
    
    visualizer = AprilTagVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
