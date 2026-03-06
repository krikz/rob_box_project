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
    from ..mapping_state import MappingState

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType


class StartMappingTool(MCPTool):
    """Инструмент для начала нового картографирования"""

    def __init__(self, node, waypoint_store: Optional["WaypointStore"] = None, mapping_state=None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        self.mapping_state = mapping_state
        # Динамический импорт во время выполнения
        from std_srvs.srv import Empty
        
        # Service clients для RTABMap
        self.backup_client = node.create_client(Empty, "/rtabmap/rtabmap/backup")
        # LoadDatabase импортируется лениво в execute() — rtabmap_msgs может отсутствовать на Vision Pi
        try:
            from rtabmap_msgs.srv import LoadDatabase  # type: ignore
            self.load_database_client = node.create_client(LoadDatabase, "/rtabmap/rtabmap/load_database")
        except ImportError:
            self.load_database_client = None
        self.set_mode_mapping_client = node.create_client(Empty, "/rtabmap/rtabmap/set_mode_mapping")

    @property
    def name(self) -> str:
        return "start_mapping"

    @property
    def description(self) -> str:
        return (
            "Начать картографирование. "
            "Если пользователь называет место (квартира, офис, склад, дом, комната) или говорит 'новая карта' / 'с нуля' — "
            "передай map_name с названием; new_location автоматически станет True. "
            "Если говорит 'продолжить', 'продолжай', 'добавь' — передай new_location=false."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="map_name",
                type="string",
                description="Название новой карты (например 'квартира', 'офис'). Если передан — автоматически включает new_location=True.",
                required=False,
            ),
            MCPToolParameter(
                name="new_location",
                type="boolean",
                description="True — стереть базу и начать с нуля. False — продолжить текущую карту. Если не указан — True когда передан map_name, иначе False.",
                required=False,
            ),
        ]

    def execute(self, map_name: str = "", new_location: Optional[bool] = None) -> MCPToolResult:
        """Начать картографирование"""
        # Если new_location не указан явно — выводим из map_name
        if new_location is None:
            new_location = bool(map_name.strip())
        self.log_info(f"Запуск картографирования (new_location={new_location}, map_name='{map_name}')")

        # 1. Создать backup текущей карты
        self.log_info("Создание backup карты...")
        backup_success = self._create_backup()
        if not backup_success:
            self.log_warning("⚠️ Backup не удался, продолжаем без backup")

        # 2. Загрузить базу данных (clear=True если новая локация, иначе только reload)
        if self.load_database_client is not None and self.load_database_client.service_is_ready():
            from rtabmap_msgs.srv import LoadDatabase  # type: ignore
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
            from std_srvs.srv import Empty
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

        # 5. Обновить FSM состояние → mapping
        if self.mapping_state is not None:
            self.mapping_state.set_mapping(
                map_name=map_name.strip() if map_name else None,
                map_id=map_id,
            )
            self.log_info(f"🗺️  MappingState → mapping (map='{map_name}')")

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

    def __init__(self, node, waypoint_store: Optional["WaypointStore"] = None, mapping_state=None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        self.mapping_state = mapping_state
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

        # Имя карты (если указано) — до переключения, пока ещё знаем map_id
        active_map_id = None
        if self.waypoint_store:
            try:
                active_map_id = self.waypoint_store.get_active_map_id()
                if map_name and active_map_id:
                    self.waypoint_store.rename_map(active_map_id, map_name.strip())
                    self.log_info(f"📍 Карта названа: '{map_name}'")
            except Exception as e:
                self.log_warning(f"⚠️ Не удалось назвать карту: {e}")

        if self.set_mode_localization_client.service_is_ready():
            from std_srvs.srv import Empty
            request = Empty.Request()
            self.set_mode_localization_client.call_async(request)
            self.log_info("Режим localization активирован")
        else:
            self.log_warning("⚠️ RTABMap localization service не готов")

        # Обновить FSM состояние → localization
        if self.mapping_state is not None:
            self.mapping_state.set_localization(
                map_name=map_name.strip() if map_name else None,
                map_id=active_map_id,
            )
            self.log_info(f"🗺️  MappingState → localization (map='{map_name}')")

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

        request = Empty.Request()
        steps = []
        failed = []

        for client, svc_name, ok_msg, step_label in [
            (self.loop_closures_client,    "detect_more_loop_closures", "🔄 Поиск loop closures запущен",    "loop closures"),
            (self.bundle_adjustment_client, "global_bundle_adjustment",  "📐 Bundle adjustment запущен",       "bundle adjustment"),
            (self.cleanup_client,           "cleanup_local_grids",       "🧹 Очистка occupancy grid запущена", "cleanup grids"),
            (self.backup_client,            "backup",                    "💾 Backup запущен",                  "backup"),
        ]:
            if client.service_is_ready():
                client.call_async(request)
                self.log_info(ok_msg)
                steps.append(step_label)
            else:
                self.log_warning(f"⚠️ {svc_name} service не готов")
                failed.append(svc_name)

        if not steps:
            return MCPToolResult(
                success=False,
                error=f"RTABMap сервисы недоступны: {', '.join(failed)}. Убедись что RTABMap запущен."
            )

        result_msg = f"Оптимизация карты запущена: {', '.join(steps)}."
        if failed:
            result_msg += f" Недоступны (пропущены): {', '.join(failed)}."
        return MCPToolResult(success=True, message=result_msg)


class LoadMapTool(MCPTool):
    """Загрузить существующую карту и перейти в режим локализации"""

    def __init__(self, node, waypoint_store: Optional["WaypointStore"] = None, mapping_state=None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        self.mapping_state = mapping_state
        from std_srvs.srv import Empty

        # LoadDatabase импортируется лениво — rtabmap_msgs может отсутствовать на Vision Pi
        try:
            from rtabmap_msgs.srv import LoadDatabase  # type: ignore
            self.load_database_client = node.create_client(LoadDatabase, "/rtabmap/rtabmap/load_database")
        except ImportError:
            self.load_database_client = None
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

        # Обновить FSM состояние → localization
        if self.mapping_state is not None:
            resolved_id = None
            if self.waypoint_store:
                try:
                    resolved_id = self.waypoint_store.get_active_map_id()
                except Exception:
                    pass
            self.mapping_state.set_localization(
                map_name=map_name.strip() if map_name.strip() else None,
                map_id=resolved_id,
            )
            self.log_info(f"🗺️  MappingState → localization via load_map (map='{map_name}')")

        suffix = f" Карта: '{map_name}'." if map_name.strip() else ""
        return MCPToolResult(
            success=True,
            message=f"Карта загружена, переключаюсь в режим навигации.{suffix}"
        )
