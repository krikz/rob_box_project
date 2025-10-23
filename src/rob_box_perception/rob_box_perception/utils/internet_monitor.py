#!/usr/bin/env python3
"""
internet_monitor.py - Internet Connectivity Monitor

Мониторинг доступности интернета через ping к надёжным серверам.
"""

import subprocess
import time
from typing import Dict

from rclpy.node import Node


class InternetConnectivityMonitor:
    """
    Мониторинг доступности интернета.

    Периодически проверяет доступность через ping к
    надёжным DNS серверам (Google, Cloudflare).

    Attributes:
        node: ROS2 нода для логирования
        is_online: Текущий статус подключения
        last_check_time: Время последней проверки
        test_hosts: Список хостов для ping-теста
    """

    def __init__(self, node: Node, check_interval: float = 30.0):
        """
        Инициализация монитора.

        Args:
            node: ROS2 нода для логирования
            check_interval: Интервал проверки в секундах
        """
        self.node = node
        self.is_online = None
        self.last_check_time = None
        self.check_interval = check_interval

        # Список серверов для проверки
        self.test_hosts = [
            "8.8.8.8",  # Google DNS
            "1.1.1.1",  # Cloudflare DNS
        ]

        # Таймер проверки
        self.check_timer = self.node.create_timer(self.check_interval, self.check_connectivity)

        # Первая проверка сразу
        self.check_connectivity()

        self.node.get_logger().info(f"🌐 Internet Monitor: проверка каждые {check_interval}s")

    def check_connectivity(self) -> bool:
        """
        Проверить доступность интернета.

        Returns:
            True если интернет доступен, False иначе
        """
        for host in self.test_hosts:
            try:
                result = subprocess.run(
                    ["ping", "-c", "1", "-W", "2", host], capture_output=True, timeout=2.5
                )

                if result.returncode == 0:
                    # Интернет доступен
                    if self.is_online is False:
                        self.node.get_logger().info("✅ Интернет восстановлен")

                    self.is_online = True
                    self.last_check_time = time.time()
                    return True

            except (subprocess.TimeoutExpired, Exception):
                continue

        # Интернет недоступен
        if self.is_online is True or self.is_online is None:
            self.node.get_logger().warn("⚠️ Интернет недоступен")

        self.is_online = False
        self.last_check_time = time.time()
        return False

    def get_status(self) -> Dict:
        """
        Получить статус подключения.

        Returns:
            Словарь со статусом подключения
        """
        return {
            "is_online": self.is_online if self.is_online is not None else False,
            "last_check": self.last_check_time,
            "check_interval": self.check_interval,
        }
