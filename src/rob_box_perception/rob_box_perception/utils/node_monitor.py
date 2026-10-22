#!/usr/bin/env python3
"""
node_monitor.py - ROS2 Node Availability Monitor.

Мониторинг доступности ROS2 нод в реальном времени.
Использует `ros2 node list` для проверки активных нод.
"""

import subprocess
import time
from typing import Dict, List, Optional, TYPE_CHECKING

if TYPE_CHECKING:  # rclpy нужен только для аннотаций: монитор тестируется без ROS
    from rclpy.node import Node

#: Ноды диалога по значению флага ``dialog_engine`` (``rob_box_core.dialog_engine``, ADR-0150 §11): на роботе
#: стартует ровно одна из двух - ``v2`` ``/dialog_node``, ``v1`` ``/dialogue_node``. Сам флаг монитору недоступен:
#: образ perception (Main Pi) не содержит ``rob_box_core``, а ``dialog_engine.yaml`` лежит на Sound Pi. Поэтому
#: монитор не гадает по флагу, а берёт ту из двух нод, что реально есть в графе (``DIALOG_NODES`` - её
#: альтернативы). После PR-15 (#3711, v1 удалён) остаётся одна ``/dialog_node``.
DIALOG_NODES = ('/dialog_node', '/dialogue_node')
#: Подпись ожидаемой диалоговой ноды, пока ни одна из двух не появлялась в графе.
DIALOG_NODES_LABEL = '|'.join(DIALOG_NODES)


class NodeAvailabilityMonitor:
    """
    Мониторинг доступности ROS2 нод.

    Периодически проверяет список активных нод и сравнивает
    с ожидаемым списком критичных нод системы.

    Attributes
    ----------
    node : Node
        ROS2 нода для логирования
    expected_nodes : List[str]
        Список ожидаемых критичных нод
    node_status : Dict[str, Dict]
        Словарь со статусами нод

    """

    def __init__(self, node: 'Node', expected_nodes: Optional[List[str]] = None):
        """
        Инициализация монитора.

        Parameters
        ----------
        node : Node
            ROS2 нода для логирования
        expected_nodes : List[str], optional
            Список ожидаемых нод (опционально); любая из :data:`DIALOG_NODES` в нём означает
            «нода диалога текущего движка»

        """
        self.node = node
        self._base_expected = list(expected_nodes) if expected_nodes is not None else [
            '/audio_node',
            '/stt_node',
            '/tts_node',
            DIALOG_NODES[0],
            '/context_aggregator',
            # OAK-D camera node (oak-d container, namespace /camera)
            '/camera/camera',
            '/lslidar_driver_node',  # LS LiDAR driver node (lslidar container)
            # RTAB-Map SLAM node (rtabmap container, namespaced)
            '/rtabmap/rtabmap',
        ]
        #: Последняя увиденная в графе нода диалога (чтобы её пропажа была ``failed``, а не молчание).
        self._dialog_node: Optional[str] = None
        self.node_status: Dict[str, Dict] = {}
        self.expected_nodes = self._resolve_expected([])

        # Периодическая проверка (каждые 5 секунд)
        self.check_timer = self.node.create_timer(5.0, self.check_nodes)

        self.node.get_logger().info(
            f'📡 Node Monitor: отслеживаем {len(self.expected_nodes)} нод'
        )

    def _resolve_expected(self, active_nodes: List[str]) -> List[str]:
        """Ожидаемые ноды с нодой диалога текущего движка вместо альтернатив :data:`DIALOG_NODES`.

        Присутствующая в графе нода диалога запоминается; пропала - ожидается та же (``failed``); не
        появлялась ни разу - :data:`DIALOG_NODES_LABEL` (``missing``). Запись другого движка из
        ``node_status`` убирается: при смене ``dialog_engine`` старая нода не должна висеть «упавшей».
        """
        present = [name for name in DIALOG_NODES if name in active_nodes]
        if present:
            self._dialog_node = present[0]
        dialog = self._dialog_node or DIALOG_NODES_LABEL
        for name in (*DIALOG_NODES, DIALOG_NODES_LABEL):
            if name != dialog:
                self.node_status.pop(name, None)
        expected: List[str] = []
        for name in self._base_expected:
            name = dialog if name in DIALOG_NODES else name
            if name not in expected:
                expected.append(name)
        return expected

    def check_nodes(self):
        """
        Проверить доступность всех ожидаемых нод.

        Note: Uses subprocess to call 'ros2 node list' which may be slower on
        resource-constrained devices like Raspberry Pi. Consider using
        rclpy's node.get_node_names() API for better performance, or increase
        check_interval to reduce CPU usage.
        """
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=2.0,
            )

            if result.returncode != 0:
                self.node.get_logger().warning(
                    f'⚠️ Ошибка вызова ros2 node list: {result.stderr}'
                )
                return

            stdout = result.stdout.strip()
            active_nodes = stdout.split('\n') if stdout else []

            self.expected_nodes = self._resolve_expected(active_nodes)
            for expected_node in self.expected_nodes:
                if expected_node in active_nodes:
                    # Нода активна
                    prev_status = (
                        self.node_status.get(expected_node, {}).get('status')
                    )
                    if prev_status != 'active':
                        self.node.get_logger().info(
                            f'✅ Нода восстановлена: {expected_node}'
                        )

                    self.node_status[expected_node] = {
                        'status': 'active', 'last_seen': time.time()}
                else:
                    # Нода отсутствует
                    if expected_node not in self.node_status:
                        # Первая проверка - нода missing
                        self.node_status[expected_node] = {
                            'status': 'missing', 'last_seen': None
                        }
                        self.node.get_logger().warn(
                            f'⚠️ Нода не найдена: {expected_node}'
                        )
                    elif self.node_status[expected_node]['status'] == 'active':
                        # Нода была активна, теперь пропала
                        self.node_status[expected_node]['status'] = 'failed'
                        self.node.get_logger().error(
                            f'❌ Нода упала: {expected_node}'
                        )

        except subprocess.TimeoutExpired:
            self.node.get_logger().warn('⚠️ Timeout при вызове ros2 node list')
        except Exception as e:
            self.node.get_logger().error(f'❌ Ошибка проверки нод: {e}')

    def get_active_nodes(self) -> List[str]:
        """
        Получить список активных нод.

        Returns
        -------
        List[str]
            Список названий активных нод

        """
        return [
            node for node, status in self.node_status.items()
            if status['status'] == 'active'
        ]

    def get_failed_nodes(self) -> List[str]:
        """
        Получить список упавших нод.

        Returns
        -------
        List[str]
            Список названий упавших нод

        """
        return [
            node for node, status in self.node_status.items()
            if status['status'] == 'failed'
        ]

    def get_missing_nodes(self) -> List[str]:
        """
        Получить список отсутствующих нод.

        Returns
        -------
        List[str]
            Список названий отсутствующих нод

        """
        return [
            node for node, status in self.node_status.items()
            if status['status'] == 'missing'
        ]

    def get_status_summary(self) -> Dict:
        """
        Получить сводку статусов нод.

        Returns
        -------
        Dict
            Словарь со статистикой

        """
        active = self.get_active_nodes()
        failed = self.get_failed_nodes()
        missing = self.get_missing_nodes()

        return {
            'total': len(self.expected_nodes),
            'active': len(active),
            'failed': len(failed),
            'missing': len(missing),
            'active_list': active,
            'failed_list': failed,
            'missing_list': missing,
        }
