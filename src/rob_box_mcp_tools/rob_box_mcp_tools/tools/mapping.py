#!/usr/bin/env python3
"""
mapping.py - Инструменты для управления картографированием (RTABMap)

Инструменты:
- StartMappingTool: Начать новое картографирование (с backup)
- ContinueMappingTool: Продолжить картографирование
- FinishMappingTool: Завершить картографирование и перейти в локализацию
"""

from typing import List
import subprocess

from std_srvs.srv import Empty

from ..base import MCPTool, MCPToolParameter, MCPToolResult


class StartMappingTool(MCPTool):
    """Инструмент для начала нового картографирования"""

    def __init__(self, node):
        super().__init__(node)
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

        return MCPToolResult(
            success=True, message="Начинаю новое исследование. Старая карта сохранена в резервной копии."
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

    def __init__(self, node):
        super().__init__(node)
        self.set_mode_localization_client = node.create_client(Empty, "/rtabmap/set_mode_localization")

    @property
    def name(self) -> str:
        return "finish_mapping"

    @property
    def description(self) -> str:
        return "Завершить картографирование и перейти в режим навигации по готовой карте (локализация)."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    def execute(self) -> MCPToolResult:
        """Завершить картографирование"""
        self.log_info("Завершение картографирования")

        if not self.set_mode_localization_client.wait_for_service(timeout_sec=2.0):
            return MCPToolResult(success=False, error="RTABMap set_mode service недоступен")

        request = Empty.Request()
        future = self.set_mode_localization_client.call_async(request)

        self.log_info("Режим localization активирован")

        return MCPToolResult(success=True, message="Заканчиваю исследование. Переключаюсь в режим навигации.")
