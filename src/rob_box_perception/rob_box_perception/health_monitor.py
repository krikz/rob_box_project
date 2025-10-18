#!/usr/bin/env python3
"""
health_monitor.py - Простой мониторинг здоровья ROS2 системы

Утилита для проверки активных нод, топиков и логов.
Может использоваться для диагностики проблем.

Usage:
    ros2 run rob_box_perception health_monitor
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String
import time


class HealthMonitor(Node):
    """Простой монитор здоровья системы"""
    
    def __init__(self):
        super().__init__('health_monitor')
        
        # Параметры
        self.declare_parameter('enable_sounds', True)
        self.enable_sounds = self.get_parameter('enable_sounds').value
        
        self.errors = []
        self.warnings = []
        self.last_status = None  # Для отслеживания изменений статуса
        
        # Подписка на логи
        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self.on_log,
            100
        )
        
        # Publisher для звуков
        self.sound_pub = self.create_publisher(String, '/voice/sound/trigger', 10)
        
        # Таймер для отчётов
        self.report_timer = self.create_timer(5.0, self.print_report)
        
        self.get_logger().info('🏥 Health Monitor запущен')
        self.get_logger().info('   Слушаем /rosout...')
    
    def on_log(self, msg: Log):
        """Получен лог"""
        # Log levels: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
        if msg.level >= 40:  # ERROR or FATAL
            self.errors.append({
                'node': msg.name,
                'level': 'ERROR' if msg.level == 40 else 'FATAL',
                'msg': msg.msg,
                'time': time.time()
            })
            # Оставляем только последние 20 ошибок
            if len(self.errors) > 20:
                self.errors = self.errors[-20:]
        
        elif msg.level == 30:  # WARN
            self.warnings.append({
                'node': msg.name,
                'msg': msg.msg,
                'time': time.time()
            })
            # Оставляем только последние 10 предупреждений
            if len(self.warnings) > 10:
                self.warnings = self.warnings[-10:]
    
    def print_report(self):
        """Печать отчёта о здоровье системы"""
        print("\n" + "="*70)
        print("🏥 HEALTH REPORT")
        print("="*70)
        
        # Статус
        critical = sum(1 for e in self.errors if e['level'] == 'FATAL')
        recent_errors = sum(1 for e in self.errors if time.time() - e['time'] < 60)
        
        if critical > 0:
            status = "🚨 CRITICAL"
        elif recent_errors >= 5:
            status = "⚠️  DEGRADED"
        else:
            status = "✅ HEALTHY"
        
        # Звуковой сигнал при изменении статуса
        if self.enable_sounds and status != self.last_status:
            if status == "🚨 CRITICAL":
                self._play_sound('angry_2')
            elif status == "⚠️  DEGRADED":
                self._play_sound('confused')
            elif status == "✅ HEALTHY" and self.last_status is not None:  # Восстановление
                self._play_sound('cute')
            
            self.last_status = status
        
        print(f"Status: {status}")
        print(f"Total Errors: {len(self.errors)} (последние {recent_errors} за минуту)")
        print(f"Total Warnings: {len(self.warnings)}")
        
        # Последние ошибки
        if self.errors:
            print("\n--- Recent Errors ---")
            for e in self.errors[-5:]:
                age = int(time.time() - e['time'])
                print(f"  [{e['level']}] {e['node']} ({age}s ago): {e['msg'][:60]}")
        
        # Последние предупреждения
        if self.warnings:
            print("\n--- Recent Warnings ---")
            for w in self.warnings[-3:]:
                age = int(time.time() - w['time'])
                print(f"  [WARN] {w['node']} ({age}s ago): {w['msg'][:60]}")
        
        print("="*70)
    
    def _play_sound(self, sound_name: str):
        """Проиграть звуковой эффект"""
        try:
            msg = String()
            msg.data = sound_name
            self.sound_pub.publish(msg)
            self.get_logger().info(f'🎵 Health звук: {sound_name}')
        except Exception as e:
            self.get_logger().warn(f'⚠️  Ошибка триггера звука: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
