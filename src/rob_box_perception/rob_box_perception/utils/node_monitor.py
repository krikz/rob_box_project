#!/usr/bin/env python3
"""
node_monitor.py - ROS2 Node Availability Monitor

Мониторинг доступности ROS2 нод в реальном времени.
Использует `ros2 node list` для проверки активных нод.
"""

import subprocess
import time
from typing import Dict, List

from rclpy.node import Node


class NodeAvailabilityMonitor:
    """
    Мониторинг доступности ROS2 нод.

    Периодически проверяет список активных нод и сравнивает
    с ожидаемым списком критичных нод системы.

    Attributes:
        node: ROS2 нода для логирования
        expected_nodes: Список ожидаемых критичных нод
        node_status: Словарь со статусами нод
    """

    def __init__(self, node: Node, expected_nodes: List[str] = None):
        """
        Инициализация монитора.

        Args:
            node: ROS2 нода для логирования
            expected_nodes: Список ожидаемых нод (опционально)
        """
        self.node = node
        self.expected_nodes = [
            "/audio_node",
            "/stt_node",
            "/tts_node",
            "/dialogue_node",
            "/context_aggregator",
            "/camera",               # OAK-D camera node (oak-d container)
            "/lslidar_driver_node",  # LS LiDAR driver node (lslidar container)
            "/rtabmap/rtabmap",      # RTAB-Map SLAM node (rtabmap container, namespaced)
        ]
        self.node_status: Dict[str, Dict] = {}

        # Периодическая проверка (каждые 5 секунд)
        self.check_timer = self.node.create_timer(5.0, self.check_nodes)

        self.node.get_logger().info(f"📡 Node Monitor: отслеживаем {len(self.expected_nodes)} нод")

    def check_nodes(self):
        """
        Проверить доступность всех ожидаемых нод.
        
        Note: Uses subprocess to call 'ros2 node list' which may be slower on
        resource-constrained devices like Raspberry Pi. Consider using rclpy's
        node.get_node_names() API for better performance, or increase check_interval
        to reduce CPU usage.
        """
        try:
            result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, timeout=2.0)

            if result.returncode != 0:
                self.node.get_logger().warning(f"⚠️ Ошибка вызова ros2 node list: {result.stderr}")
                return

            active_nodes = result.stdout.strip().split("\n") if result.stdout.strip() else []

            for expected_node in self.expected_nodes:
                if expected_node in active_nodes:
                    # Нода активна
                    prev_status = self.node_status.get(expected_node, {}).get("status")
                    if prev_status != "active":
                        self.node.get_logger().info(f"✅ Нода восстановлена: {expected_node}")

                    self.node_status[expected_node] = {"status": "active", "last_seen": time.time()}
                else:
                    # Нода отсутствует
                    if expected_node not in self.node_status:
                        # Первая проверка - нода missing
                        self.node_status[expected_node] = {"status": "missing", "last_seen": None}
                        self.node.get_logger().warn(f"⚠️ Нода не найдена: {expected_node}")
                    elif self.node_status[expected_node]["status"] == "active":
                        # Нода была активна, теперь пропала
                        self.node_status[expected_node]["status"] = "failed"
                        self.node.get_logger().error(f"❌ Нода упала: {expected_node}")

        except subprocess.TimeoutExpired:
            self.node.get_logger().warn("⚠️ Timeout при вызове ros2 node list")
        except Exception as e:
            self.node.get_logger().error(f"❌ Ошибка проверки нод: {e}")

    def get_active_nodes(self) -> List[str]:
        """
        Получить список активных нод.

        Returns:
            Список названий активных нод
        """
        return [node for node, status in self.node_status.items() if status["status"] == "active"]

    def get_failed_nodes(self) -> List[str]:
        """
        Получить список упавших нод.

        Returns:
            Список названий упавших нод
        """
        return [node for node, status in self.node_status.items() if status["status"] == "failed"]

    def get_missing_nodes(self) -> List[str]:
        """
        Получить список отсутствующих нод.

        Returns:
            Список названий отсутствующих нод
        """
        return [node for node, status in self.node_status.items() if status["status"] == "missing"]

    def get_status_summary(self) -> Dict:
        """
        Получить сводку статусов нод.

        Returns:
            Словарь со статистикой
        """
        active = self.get_active_nodes()
        failed = self.get_failed_nodes()
        missing = self.get_missing_nodes()

        return {
            "total": len(self.expected_nodes),
            "active": len(active),
            "failed": len(failed),
            "missing": len(missing),
            "active_list": active,
            "failed_list": failed,
            "missing_list": missing,
        }
