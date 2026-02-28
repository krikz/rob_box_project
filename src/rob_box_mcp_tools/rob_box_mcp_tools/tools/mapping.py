#!/usr/bin/env python3
"""
mapping.py - Инструменты для управления картографированием (RTABMap)

Инструменты:
- StartMappingTool: Начать новое картографирование (с backup)
- ContinueMappingTool: Продолжить картографирование
- FinishMappingTool: Завершить картографирование и перейти в локализацию
"""

from typing import List, Optional, TYPE_CHECKING
import subprocess

# Ленивый импорт ROS 2 модулей для поддержки unit тестов
if TYPE_CHECKING:
    from std_srvs.srv import Empty
    from ..waypoint_store import WaypointStore

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType


class StartMappingTool(MCPTool):
    """Инструмент для начала нового картографирования"""

    def __init__(self, node, waypoint_store: Optional["WaypointStore"] = None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        # Динамический импорт во время выполнения
        from std_srvs.srv import Empty
        
        # Service clients для RTABMap
        self.reset_memory_client = node.create_client(Empty, "/rtabmap/reset_memory")
        self.set_mode_mapping_client = node.create_client(Empty, "/rtabmap/set_mode_mapping")

    @property
    def name(self) -> str:
        return "start_mapping"

    @property
    def description(self) -> str:
        return "Начать новое картографирование территории. Создаст backup текущей карты и сбросит память RTABMap."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    def execute(self) -> MCPToolResult:
        """Начать картографирование"""
        self.log_info("Запуск нового картографирования")

        # 1. Создать backup текущей карты
        self.log_info("Создание backup карты...")
        backup_success = self._create_backup()
        if not backup_success:
            return MCPToolResult(success=False, error="Не удалось создать backup карты", message="Операция отменена")

        # 2. Сбросить память RTABMap
        if not self.reset_memory_client.wait_for_service(timeout_sec=2.0):
            return MCPToolResult(success=False, error="RTABMap reset service недоступен")

        request = Empty.Request()
        future = self.reset_memory_client.call_async(request)
        # NOTE: В реальном использовании нужно дождаться результата

        self.log_info("Память RTABMap сброшена")

        # 3. Переключить в режим mapping
        if not self.set_mode_mapping_client.wait_for_service(timeout_sec=2.0):
            return MCPToolResult(success=False, error="RTABMap set_mode service недоступен")

        request = Empty.Request()
        future = self.set_mode_mapping_client.call_async(request)

        self.log_info("Режим mapping активирован")

        # 4. Создать новую карту в WaypointStore
        map_id = None
        if self.waypoint_store:
            try:
                map_id = self.waypoint_store.create_map()
                self.log_info(f"📍 Новая карта создана: {map_id[:8]}...")
            except Exception as e:
                self.log_warning(f"⚠️ Не удалось создать карту в WaypointStore: {e}")

        return MCPToolResult(
            success=True,
            data={"map_id": map_id} if map_id else None,
            message="Начинаю новое исследование. Старая карта сохранена в резервной копии."
        )

    def _create_backup(self) -> bool:
        """Создать backup RTABMap базы данных через Docker"""
        try:
            result = subprocess.run(
                [
                    "docker",
                    "exec",
                    "rtabmap",
                    "bash",
                    "-c",
                    "mkdir -p /maps/backups && "
                    "cp /maps/rtabmap.db /maps/backups/rtabmap_backup_$(date +%Y%m%d_%H%M%S).db && "
                    "echo OK",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )

            if result.returncode == 0 and "OK" in result.stdout:
                self.log_info("✅ Backup создан успешно")
                return True
            else:
                self.log_error(f"❌ Backup failed: {result.stderr}")
                return False
        except subprocess.TimeoutExpired:
            self.log_error("❌ Backup timeout (10s)")
            return False
        except Exception as e:
            self.log_error(f"❌ Backup error: {e}")
            return False


class ContinueMappingTool(MCPTool):
    """Инструмент для продолжения картографирования"""

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_srvs.srv import Empty
        
        self.set_mode_mapping_client = node.create_client(Empty, "/rtabmap/set_mode_mapping")

    @property
    def name(self) -> str:
        return "continue_mapping"

    @property
    def description(self) -> str:
        return "Продолжить картографирование территории (режим SLAM). Используй для добавления новых областей к карте."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM  # Переключение режима 2-10s

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM  # Переключение режима 2-10s

    def execute(self) -> MCPToolResult:
        """Продолжить картографирование"""
        self.log_info("Продолжение картографирования")

        if not self.set_mode_mapping_client.wait_for_service(timeout_sec=2.0):
            return MCPToolResult(success=False, error="RTABMap set_mode service недоступен")

        request = Empty.Request()
        future = self.set_mode_mapping_client.call_async(request)

        self.log_info("Режим mapping активирован")

        return MCPToolResult(success=True, message="Продолжаю исследование территории")


class FinishMappingTool(MCPTool):
    """Инструмент для завершения картографирования"""

    def __init__(self, node, waypoint_store: Optional["WaypointStore"] = None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        # Динамический импорт во время выполнения
        from std_srvs.srv import Empty
        
        self.set_mode_localization_client = node.create_client(Empty, "/rtabmap/set_mode_localization")

    @property
    def name(self) -> str:
        return "finish_mapping"

    @property
    def description(self) -> str:
        return (
            "Завершить картографирование и перейти в режим навигации по готовой карте (локализация). "
            "Можно указать имя карты (например 'квартира', 'офис')."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="map_name",
                type="string",
                description="Название карты (опционально, например 'квартира')",
                required=False,
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    def execute(self, map_name: str = "") -> MCPToolResult:
        """Завершить картографирование"""
        self.log_info("Завершение картографирования")

        if not self.set_mode_localization_client.wait_for_service(timeout_sec=2.0):
            return MCPToolResult(success=False, error="RTABMap set_mode service недоступен")

        request = Empty.Request()
        future = self.set_mode_localization_client.call_async(request)

        self.log_info("Режим localization активирован")

        # Имя карты (если указано)
        if map_name and self.waypoint_store:
            try:
                active_map_id = self.waypoint_store.get_active_map_id()
                if active_map_id:
                    self.waypoint_store.rename_map(active_map_id, map_name.strip())
                    self.log_info(f"📍 Карта названа: '{map_name}'")
            except Exception as e:
                self.log_warning(f"⚠️ Не удалось назвать карту: {e}")

        suffix = f" Карта: '{map_name}'." if map_name else ""
        return MCPToolResult(
            success=True,
            message=f"Заканчиваю исследование. Переключаюсь в режим навигации.{suffix}"
        )
