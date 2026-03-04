#!/usr/bin/env python3
"""
mapping.py - Инструменты для управления картографированием (RTABMap)

Инструменты:
- StartMappingTool: Начать новое картографирование (с backup)
- ContinueMappingTool: Продолжить картографирование
- FinishMappingTool: Завершить картографирование и перейти в локализацию
- OptimizeMapTool: Постобработка карты (loop closures + bundle adjustment + cleanup)
- LoadMapTool: Загрузить существующую карту и перейти в режим локализации
"""

from typing import List, Optional, TYPE_CHECKING

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
        from rtabmap_msgs.srv import LoadDatabase  # type: ignore
        self.backup_client = node.create_client(Empty, "/rtabmap/rtabmap/backup")
        self.load_database_client = node.create_client(LoadDatabase, "/rtabmap/rtabmap/load_database")
        self.set_mode_mapping_client = node.create_client(Empty, "/rtabmap/rtabmap/set_mode_mapping")

    @property
    def name(self) -> str:
        return "start_mapping"

    @property
    def description(self) -> str:
        return (
            "Начать картографирование. new_location=true — создать чистую базу (новая локация). "
            "new_location=false (по умолчанию) — продолжить добавлять в существующую карту."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="map_name",
                type="string",
                description="Название новой карты (опционально, например 'квартира', 'офис')",
                required=False,
            ),
            MCPToolParameter(
                name="new_location",
                type="boolean",
                description="True — стереть текущую базу и начать с нуля (новая локация). False — продолжить картографирование текущей локации.",
                required=False,
            ),
        ]

    def execute(self, map_name: str = "", new_location: bool = False) -> MCPToolResult:
        """Начать картографирование"""
        self.log_info(f"Запуск картографирования (new_location={new_location})")

        # 1. Создать backup текущей карты
        self.log_info("Создание backup карты...")
        backup_success = self._create_backup()
        if not backup_success:
            self.log_warning("⚠️ Backup не удался, продолжаем без backup")

        # 2. Загрузить базу данных (clear=True если новая локация, иначе только reload)
        from rtabmap_msgs.srv import LoadDatabase  # type: ignore
        if self.load_database_client.service_is_ready():
            req = LoadDatabase.Request()
            req.path = "/maps/rtabmap.db"
            req.clear = bool(new_location)
            self.load_database_client.call_async(req)
            action = "очищена и перезагружена" if new_location else "перезагружена"
            self.log_info(f"База данных {action}")
        else:
            self.log_warning("⚠️ RTABMap load_database service не готов, пропускаем")

        # 3. Переключить в режим mapping
        if self.set_mode_mapping_client.service_is_ready():
            request = Empty.Request()
            self.set_mode_mapping_client.call_async(request)
            self.log_info("Режим mapping активирован")
        else:
            self.log_warning("⚠️ RTABMap set_mode service не готов, пропускаем")

        # 4. Создать новую карту в WaypointStore
        map_id = None
        if self.waypoint_store and new_location:
            try:
                map_id = self.waypoint_store.create_map(map_name.strip() if map_name else None)
                self.log_info(f"📍 Новая карта создана: {map_id[:8]}...")
            except Exception as e:
                self.log_warning(f"⚠️ Не удалось создать карту в WaypointStore: {e}")

        suffix = f" Карта: '{map_name}'." if map_name else ""
        mode_msg = "чистой базе" if new_location else "существующей карте"
        return MCPToolResult(
            success=True,
            data={"map_id": map_id} if map_id else None,
            message=f"Начинаю картографирование на {mode_msg}. Старая карта сохранена в резервной копии.{suffix}"
        )

    def _create_backup(self) -> bool:
        """Создать backup RTABMap базы данных через ROS 2 сервис /rtabmap/rtabmap/backup (fire-and-forget)"""
        try:
            if not self.backup_client.service_is_ready():
                self.log_warning("⚠️ RTABMap backup service ещё не готов, пропускаем")
                return False

            from std_srvs.srv import Empty
            request = Empty.Request()
            self.backup_client.call_async(request)
            self.log_info("✅ Backup запрос отправлен (fire-and-forget)")
            return True
        except Exception as e:
            self.log_error(f"❌ Backup error: {e}")
            return False


class ContinueMappingTool(MCPTool):
    """Инструмент для продолжения картографирования"""

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_srvs.srv import Empty
        
        self.set_mode_mapping_client = node.create_client(Empty, "/rtabmap/rtabmap/set_mode_mapping")

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

    def execute(self) -> MCPToolResult:
        """Продолжить картографирование"""
        self.log_info("Продолжение картографирования")

        if not self.set_mode_mapping_client.service_is_ready():
            return MCPToolResult(success=False, error="RTABMap set_mode service не готов")

        from std_srvs.srv import Empty
        request = Empty.Request()
        self.set_mode_mapping_client.call_async(request)

        self.log_info("Режим mapping активирован")

        return MCPToolResult(success=True, message="Продолжаю исследование территории")


class FinishMappingTool(MCPTool):
    """Инструмент для завершения картографирования"""

    def __init__(self, node, waypoint_store: Optional["WaypointStore"] = None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        # Динамический импорт во время выполнения
        from std_srvs.srv import Empty
        
        self.set_mode_localization_client = node.create_client(Empty, "/rtabmap/rtabmap/set_mode_localization")

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

        if self.set_mode_localization_client.service_is_ready():
            from std_srvs.srv import Empty
            request = Empty.Request()
            self.set_mode_localization_client.call_async(request)
            self.log_info("Режим localization активирован")
        else:
            self.log_warning("⚠️ RTABMap localization service не готов")

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


class OptimizeMapTool(MCPTool):
    """Постобработка карты: loop closures + bundle adjustment + cleanup + backup"""

    def __init__(self, node):
        super().__init__(node)
        from std_srvs.srv import Empty

        self.loop_closures_client = node.create_client(Empty, "/rtabmap/rtabmap/detect_more_loop_closures")
        self.bundle_adjustment_client = node.create_client(Empty, "/rtabmap/rtabmap/global_bundle_adjustment")
        self.cleanup_client = node.create_client(Empty, "/rtabmap/rtabmap/cleanup_local_grids")
        self.backup_client = node.create_client(Empty, "/rtabmap/rtabmap/backup")

    @property
    def name(self) -> str:
        return "optimize_map"

    @property
    def description(self) -> str:
        return (
            "Оптимизировать карту после завершения картографирования: "
            "поиск дополнительных loop closures, bundle adjustment, очистка occupancy grid, backup."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG  # Может занять 30-120+s

    def execute(self) -> MCPToolResult:
        """Запустить постобработку карты"""
        from std_srvs.srv import Empty

        steps = []

        if self.loop_closures_client.service_is_ready():
            self.loop_closures_client.call_async(Empty.Request())
            self.log_info("🔄 Поиск loop closures запущен")
            steps.append("loop closures")
        else:
            self.log_warning("⚠️ detect_more_loop_closures service не готов")

        if self.bundle_adjustment_client.service_is_ready():
            self.bundle_adjustment_client.call_async(Empty.Request())
            self.log_info("📐 Bundle adjustment запущен")
            steps.append("bundle adjustment")
        else:
            self.log_warning("⚠️ global_bundle_adjustment service не готов")

        if self.cleanup_client.service_is_ready():
            self.cleanup_client.call_async(Empty.Request())
            self.log_info("🧹 Очистка occupancy grid запущена")
            steps.append("cleanup grids")
        else:
            self.log_warning("⚠️ cleanup_local_grids service не готов")

        if self.backup_client.service_is_ready():
            self.backup_client.call_async(Empty.Request())
            self.log_info("💾 Backup запущен")
            steps.append("backup")
        else:
            self.log_warning("⚠️ backup service не готов")

        if not steps:
            return MCPToolResult(success=False, error="Ни один сервис RTABMap не был доступен")

        return MCPToolResult(
            success=True,
            message=f"Оптимизация карты запущена: {', '.join(steps)}. Это может занять несколько минут."
        )


class LoadMapTool(MCPTool):
    """Загрузить существующую карту и перейти в режим локализации"""

    def __init__(self, node, waypoint_store: Optional["WaypointStore"] = None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        from rtabmap_msgs.srv import LoadDatabase  # type: ignore
        from std_srvs.srv import Empty

        self.load_database_client = node.create_client(LoadDatabase, "/rtabmap/rtabmap/load_database")
        self.set_mode_localization_client = node.create_client(Empty, "/rtabmap/rtabmap/set_mode_localization")

    @property
    def name(self) -> str:
        return "load_map"

    @property
    def description(self) -> str:
        return (
            "Загрузить сохранённую карту и перейти в режим локализации (навигации). "
            "Используй когда нужно переключиться на уже исследованную локацию."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="map_name",
                type="string",
                description="Название карты для загрузки (например 'квартира', 'офис'). Если не указано — перезагружает текущую.",
                required=False,
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG

    def execute(self, map_name: str = "") -> MCPToolResult:
        """Загрузить карту и переключиться в режим локализации"""
        from rtabmap_msgs.srv import LoadDatabase  # type: ignore
        from std_srvs.srv import Empty

        # Найти карту в WaypointStore и сделать её активной
        if map_name.strip() and self.waypoint_store:
            try:
                found = self.waypoint_store.set_active_map_by_name(map_name.strip())
                if found:
                    self.log_info(f"📍 Активная карта: '{map_name}'")
                else:
                    self.log_warning(f"⚠️ Карта '{map_name}' не найдена в WaypointStore")
            except AttributeError:
                # Метод set_active_map_by_name может отсутствовать в старой версии
                self.log_warning("⚠️ WaypointStore не поддерживает set_active_map_by_name, пропускаем")
            except Exception as e:
                self.log_warning(f"⚠️ Ошибка WaypointStore: {e}")

        # Перезагрузить базу данных (clear=False — не стирать, просто reload)
        if self.load_database_client.service_is_ready():
            req = LoadDatabase.Request()
            req.path = "/maps/rtabmap.db"
            req.clear = False
            self.load_database_client.call_async(req)
            self.log_info("🗺️ База данных перезагружена")
        else:
            self.log_warning("⚠️ RTABMap load_database service не готов")

        # Переключить в режим локализации
        if self.set_mode_localization_client.service_is_ready():
            self.set_mode_localization_client.call_async(Empty.Request())
            self.log_info("🧭 Режим локализации активирован")
        else:
            self.log_warning("⚠️ RTABMap localization service не готов")

        suffix = f" Карта: '{map_name}'." if map_name.strip() else ""
        return MCPToolResult(
            success=True,
            message=f"Карта загружена, переключаюсь в режим навигации.{suffix}"
        )
