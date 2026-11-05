#!/usr/bin/env python3
"""
mapping.py - Инструменты для управления картографированием (RTABMap)

Инструменты:
- StartMappingTool: Начать новое картографирование (с backup)
- ContinueMappingTool: Продолжить картографирование
- FinishMappingTool: Завершить картографирование и перейти в локализацию
- OptimizeMapTool: Постобработка карты (loop closures + bundle adjustment + cleanup)
- LoadMapTool: Перезагрузить карту из базы и перейти в режим локализации
- ListMapsTool: Какие карты робот хранит
- DeleteMapTool: Забыть карту по имени (кроме той, что в базе)

Хранение карт (#4392). База rtabmap у робота ОДНА - ``/maps/rtabmap.db`` (:data:`SLAM_DB_PATH`):
``start_mapping`` с новой картой стирает её и пишет заново, ``load_map`` перезагружает тот же файл.
Таблица ``maps`` в БД инструментов - описания карт (имя, точки), геометрия есть только у активной
(``is_active``): её строку создаёт ``start_mapping`` после успешной очистки базы, переименовывает
``finish_mapping``. Остальные строки - имена карт, чья геометрия уже стёрта. Поэтому активная строка
``maps`` - единственное место правды о том, какая карта в базе, и никто, кроме ``create_map``, её не
переключает: ``load_map`` с чужим именем честно отказывает (``map_not_stored``), а не «загружает».
"""

from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

try:
    from rclpy.callback_groups import ReentrantCallbackGroup
except ImportError:
    ReentrantCallbackGroup = None  # type: ignore[assignment]

# Ленивый импорт ROS 2 модулей для поддержки unit тестов
if TYPE_CHECKING:
    from std_srvs.srv import Empty
    from ..waypoint_adapter import WaypointAdapter
    from ..waypoint_store import WaypointStore
    from ..mapping_state import MappingState

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType, wait_future
from rob_box_dialog import knowledge as kn

from ..core.action_outcome import Outcome, classify_service_call
from ..waypoint_store import map_name_key

#: Единственная база rtabmap (том ``/maps`` контейнера rtabmap на Main Pi): все карты пишутся в неё (#4346, #4392).
SLAM_DB_PATH = "/maps/rtabmap.db"


def _create_service_client(node, srv_type, srv_name, callback_group):
    if callback_group is not None:
        return node.create_client(srv_type, srv_name, callback_group=callback_group)
    return node.create_client(srv_type, srv_name)


def _call_service(client, request, timeout_sec: float, label: str) -> Outcome:
    """Вызвать сервис и дождаться ответа; исход разбирает ``classify_service_call`` (#3789)."""
    if client is None or request is None or not client.service_is_ready():
        return classify_service_call(ready=False, completed=False, response=None, label=label)
    future = client.call_async(request)
    completed = wait_future(future, timeout_sec=timeout_sec)
    response, error = None, None
    if completed:
        try:
            response = future.result()  # rclpy.Future.result() бросает исключение сервиса
        except Exception as exc:  # noqa: BLE001 - исключение будущего = вызов не удался
            error = str(exc) or type(exc).__name__
    return classify_service_call(ready=True, completed=completed, response=response, label=label, error=error)


def _failed_call(tool, outcome: Outcome, message: str) -> MCPToolResult:
    """Неуспех вызова сервиса: причину и фразу строит код, LLM их не переписывает."""
    tool.log_warning(f"⚠️ {outcome.error}")
    return MCPToolResult(success=False, error=outcome.error, message=message, data={"outcome": outcome.kind, "reason": outcome.kind})


def _start_data(map_id, new_location, map_name: str) -> dict:
    """Данные исхода старта для фразы кода (``kn.MAP_START_DONE``): чистая ли база и под каким именем."""
    return {
        **({"map_id": map_id} if map_id else {}),
        kn.MAP_NEW_FIELD: bool(new_location),
        kn.MAP_NAME_FIELD: map_name.strip() or None,
    }


def _active_map(store) -> Optional[Dict]:
    """Строка карты, что сейчас в базе rtabmap (``maps.is_active``), или ``None``. Ошибку БД отдаёт вызывающему."""
    return store.get_active_map() if store is not None else None


def _active_name(store) -> Optional[str]:
    """Имя карты в базе для фразы кода; БД недоступна или карта безымянная - ``None`` (имени не выдумываем)."""
    try:
        return (_active_map(store) or {}).get("name") or None
    except Exception:  # noqa: BLE001 - имя только для фразы: нет имени - безымянная фраза, а не падение тула
        return None


#: Вердикты запроса карты по имени (:func:`resolve_map_request`).
MAP_IN_DB = "in_db"
MAP_NOT_STORED = "not_stored"
MAP_UNKNOWN = "unknown"


def resolve_map_request(wanted: str, active: Optional[Dict], maps: List[Dict]) -> str:
    """Что значит имя ``wanted`` при одной базе: карта в базе (:data:`MAP_IN_DB`), карта была, но её геометрия
    стёрта (:data:`MAP_NOT_STORED`), такой карты не было (:data:`MAP_UNKNOWN`). Пустое имя - карта в базе."""
    key = map_name_key(wanted)
    if not key or (active is not None and map_name_key(active.get("name")) == key):
        return MAP_IN_DB
    if any(map_name_key(m.get("name")) == key for m in maps):
        return MAP_NOT_STORED
    return MAP_UNKNOWN


class StartMappingTool(MCPTool):
    """Инструмент для начала нового картографирования."""

    event_source = "srv:/rtabmap/rtabmap/set_mode_mapping"

    def __init__(self, node, waypoint_store: Optional["WaypointAdapter"] = None, mapping_state=None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        self.mapping_state = mapping_state
        self._service_cb_group = ReentrantCallbackGroup() if ReentrantCallbackGroup is not None else None
        # Динамический импорт во время выполнения
        from std_srvs.srv import Empty

        # Service clients для RTABMap
        self.backup_client = _create_service_client(
            node, Empty, "/rtabmap/rtabmap/backup", self._service_cb_group
        )
        # LoadDatabase импортируется лениво в execute() — rtabmap_msgs может отсутствовать на Vision Pi
        try:
            from rtabmap_msgs.srv import LoadDatabase  # type: ignore
            self.load_database_client = _create_service_client(
                node, LoadDatabase, "/rtabmap/rtabmap/load_database", self._service_cb_group
            )
        except ImportError:
            self.load_database_client = None
        self.set_mode_mapping_client = _create_service_client(
            node, Empty, "/rtabmap/rtabmap/set_mode_mapping", self._service_cb_group
        )

    @property
    def name(self) -> str:
        return "start_mapping"

    @property
    def description(self) -> str:
        return (
            "Начать физическое сканирование и построение карты помещения (SLAM). "
            "Вызывай ТОЛЬКО если пользователь явно просит СКАНИРОВАТЬ, КАРТИРОВАТЬ или СТРОИТЬ КАРТУ — "
            "например: 'начни картографирование', 'построй карту', 'давай сканируем квартиру', 'старт маппинга'. "
            "НЕ вызывай если пользователь: играет в ролевую игру, просит провести экскурсию, "
            "упоминает место в разговоре, просит рассказать о чём-то, даёт тебе роль или персонажа. "
            "Если передан map_name — new_location автоматически True (новая карта с нуля). "
            "Если говорит 'продолжить картирование' — передай new_location=false."
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

    @property
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: SLAM, personality.
        return "personality"

    def execute(self, map_name: str = "", new_location: Optional[bool] = None) -> MCPToolResult:
        """Начать картографирование."""
        # Если new_location не указан явно — выводим из map_name
        if new_location is None:
            new_location = bool(map_name.strip())
        self.log_info(f"Запуск картографирования (new_location={new_location}, map_name='{map_name}')")

        # 1. Создать backup текущей карты
        self.log_info("Создание backup карты...")
        backup_success = self._create_backup()
        if not backup_success:
            self.log_warning("⚠️ Backup не удался, продолжаем без backup")

        # 2. Переключить в режим mapping СНАЧАЛА (до LoadDatabase!)
        # LoadDatabase(clear=True) в localization режиме вызывает краш rtabmap
        # (pure virtual method called / SIGABRT)
        from std_srvs.srv import Empty

        # Успех шага — только по ответу сервиса (#3789): без ответа карту не создаём, FSM не трогаем.
        switched = _call_service(self.set_mode_mapping_client, Empty.Request(), 10.0, "RTABMap set_mode_mapping")
        if not switched.success:
            return _failed_call(self, switched, "Не удалось начать картографирование: режим не включился")
        self.log_info("Режим mapping активирован")

        # 3. Загрузить базу данных (clear=True если новая локация, иначе только reload)
        # Вызывается ПОСЛЕ set_mode_mapping чтобы избежать краша в localization mode
        if self.load_database_client is None:
            return MCPToolResult(
                success=False,
                error="RTABMap load_database service недоступен",
                data={"outcome": "unavailable", "reason": "unavailable"},
            )

        from rtabmap_msgs.srv import LoadDatabase  # type: ignore
        req = LoadDatabase.Request()
        req.database_path = SLAM_DB_PATH
        req.clear = bool(new_location)
        loaded = _call_service(self.load_database_client, req, 20.0, "RTABMap load_database")
        if not loaded.success:
            return _failed_call(self, loaded, "Не удалось начать картографирование: база не загрузилась")
        action = "очищена и перезагружена" if new_location else "перезагружена"
        self.log_info(f"База данных {action}")

        # 4. Строка карты в ``maps`` - описание того, что теперь в базе (#4392)
        map_id, spoken_name = self._record_map(map_name, bool(new_location))

        # 5. Обновить FSM состояние → mapping
        if self.mapping_state is not None:
            self.mapping_state.set_mapping()
            self.log_info(f"🗺️  MappingState → mapping (map='{spoken_name}')")

        suffix = f" Карта: '{spoken_name}'." if spoken_name else ""
        mode_msg = "чистой базе" if new_location else "существующей карте"
        return MCPToolResult(
            success=True,
            data=_start_data(map_id, new_location, spoken_name),
            message=f"Начинаю картографирование на {mode_msg}. Старая карта сохранена в резервной копии.{suffix}"
        )

    def _record_map(self, map_name: str, new_location: bool) -> Tuple[Optional[str], str]:
        """Новая карта - строка в ``maps`` (становится активной: её геометрия теперь в базе). Возвращает
        ``(map_id, имя для фразы)``: у новой - названное (если строка создалась), у продолжения - имя карты, что уже
        в базе, а не аргумент вызова."""
        if not new_location:
            return None, _active_name(self.waypoint_store) or ""
        if self.waypoint_store is None:
            return None, map_name.strip()
        try:
            map_id = self.waypoint_store.create_map(map_name.strip() or None)
        except Exception as e:  # noqa: BLE001 - строка не создалась: имени у карты в базе нет, так и говорим
            self.log_warning(f"⚠️ Не удалось создать карту в WaypointStore: {e}")
            return None, ""
        self.log_info(f"📍 Новая карта создана: {map_id[:8]}...")
        return map_id, map_name.strip()

    def _create_backup(self) -> bool:
        """Создать backup RTABMap базы данных через ROS 2 сервис /rtabmap/rtabmap/backup (fire-and-forget)."""
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
    """Инструмент для продолжения картографирования."""

    event_source = "srv:/rtabmap/rtabmap/set_mode_mapping"

    def __init__(self, node):
        super().__init__(node)
        self._service_cb_group = ReentrantCallbackGroup() if ReentrantCallbackGroup is not None else None
        # Динамический импорт во время выполнения
        from std_srvs.srv import Empty

        self.set_mode_mapping_client = _create_service_client(
            node, Empty, "/rtabmap/rtabmap/set_mode_mapping", self._service_cb_group
        )

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
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: SLAM, personality.
        return "personality"

    def execute(self) -> MCPToolResult:
        """Продолжить картографирование."""
        self.log_info("Продолжение картографирования")

        from std_srvs.srv import Empty

        # Успех — только по ответу сервиса переключения режима (#3789).
        switched = _call_service(self.set_mode_mapping_client, Empty.Request(), 10.0, "RTABMap set_mode_mapping")
        if not switched.success:
            return _failed_call(self, switched, "Не удалось продолжить картографирование: режим не включился")

        self.log_info("Режим mapping активирован")

        return MCPToolResult(success=True, message="Продолжаю исследование территории", data={"outcome": "continued"})


class FinishMappingTool(MCPTool):
    """Инструмент для завершения картографирования."""

    event_source = "srv:/rtabmap/rtabmap/set_mode_localization"

    def __init__(self, node, waypoint_store: Optional["WaypointAdapter"] = None, mapping_state=None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        self.mapping_state = mapping_state
        self._service_cb_group = ReentrantCallbackGroup() if ReentrantCallbackGroup is not None else None
        # Динамический импорт во время выполнения
        from std_srvs.srv import Empty
        try:
            from rtabmap_msgs.srv import PublishMap  # type: ignore
        except ImportError:
            PublishMap = None

        self.set_mode_localization_client = _create_service_client(
            node, Empty, "/rtabmap/rtabmap/set_mode_localization", self._service_cb_group
        )
        self.publish_map_client = (
            _create_service_client(node, PublishMap, "/rtabmap/rtabmap/publish_map", self._service_cb_group)
            if PublishMap is not None
            else None
        )
        self._publish_map_request = PublishMap.Request() if PublishMap is not None else None

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

    @property
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: SLAM, personality.
        return "personality"

    def execute(self, map_name: str = "") -> MCPToolResult:
        """Завершить картографирование."""
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

        from std_srvs.srv import Empty

        # Успех — только по ответу сервиса переключения режима (#3789).
        switch = _call_service(self.set_mode_localization_client, Empty.Request(), 10.0, "RTABMap set_mode_localization")
        if not switch.success:
            return _failed_call(
                self, switch, "Не удалось завершить картографирование: режим локализации не включился"
            )
        self.log_info("Режим localization активирован")

        # Публикация карты вторична: режим уже переключён, но о провале говорим честно.
        publish = _call_service(self.publish_map_client, self._publish_map_request, 15.0, "RTABMap publish_map")
        if publish.success:
            self.log_info("Опубликована occupancy grid карта")
        else:
            self.log_warning(f"⚠️ {publish.error}")

        # Обновить FSM состояние → localization
        if self.mapping_state is not None:
            self.mapping_state.set_localization()
            self.log_info(f"🗺️  MappingState → localization (map='{map_name}')")

        # Имя сохранённой карты - из владельца (после переименования), без него - названное в вызове.
        saved_name = _active_name(self.waypoint_store) if self.waypoint_store else (map_name or "").strip()
        suffix = f" Карта: '{saved_name}'." if saved_name else ""
        note = "" if publish.success else " Но карту на экран опубликовать не удалось."
        return MCPToolResult(
            success=True,
            message=f"Заканчиваю исследование. Переключаюсь в режим навигации.{suffix}{note}",
            data={kn.MAP_NAME_FIELD: saved_name or None},
        )


class OptimizeMapTool(MCPTool):
    """Постобработка карты: loop closures + bundle adjustment + cleanup + backup."""

    event_source = "srv:/rtabmap/rtabmap/detect_more_loop_closures"

    def __init__(self, node):
        super().__init__(node)
        self._service_cb_group = ReentrantCallbackGroup() if ReentrantCallbackGroup is not None else None
        from std_srvs.srv import Empty
        try:
            from rtabmap_msgs.srv import CleanupLocalGrids, DetectMoreLoopClosures, GlobalBundleAdjustment, PublishMap  # type: ignore
        except ImportError:
            CleanupLocalGrids = None
            DetectMoreLoopClosures = None
            GlobalBundleAdjustment = None
            PublishMap = None

        self.loop_closures_client = (
            _create_service_client(
                node, DetectMoreLoopClosures, "/rtabmap/rtabmap/detect_more_loop_closures", self._service_cb_group
            )
            if DetectMoreLoopClosures is not None
            else None
        )
        self.bundle_adjustment_client = (
            _create_service_client(
                node, GlobalBundleAdjustment, "/rtabmap/rtabmap/global_bundle_adjustment", self._service_cb_group
            )
            if GlobalBundleAdjustment is not None
            else None
        )
        self.cleanup_client = (
            _create_service_client(
                node, CleanupLocalGrids, "/rtabmap/rtabmap/cleanup_local_grids", self._service_cb_group
            )
            if CleanupLocalGrids is not None
            else None
        )
        self.backup_client = _create_service_client(
            node, Empty, "/rtabmap/rtabmap/backup", self._service_cb_group
        )
        self.publish_map_client = (
            _create_service_client(node, PublishMap, "/rtabmap/rtabmap/publish_map", self._service_cb_group)
            if PublishMap is not None
            else None
        )

        self._detect_more_loop_closures_request = DetectMoreLoopClosures.Request() if DetectMoreLoopClosures is not None else None
        self._global_bundle_adjustment_request = GlobalBundleAdjustment.Request() if GlobalBundleAdjustment is not None else None
        self._cleanup_local_grids_request = CleanupLocalGrids.Request() if CleanupLocalGrids is not None else None
        self._backup_request = Empty.Request()
        self._publish_map_request = PublishMap.Request() if PublishMap is not None else None

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
        """Запустить постобработку карты и дождаться завершения сервисов."""
        completed = []
        failed = []

        for client, request, svc_name, ok_msg, step_label, timeout_sec in [
            (
                self.loop_closures_client,
                self._detect_more_loop_closures_request,
                "detect_more_loop_closures",
                "🔄 Поиск loop closures завершён",
                "loop closures",
                30.0,
            ),
            (
                self.bundle_adjustment_client,
                self._global_bundle_adjustment_request,
                "global_bundle_adjustment",
                "📐 Bundle adjustment завершён",
                "bundle adjustment",
                60.0,
            ),
            (
                self.cleanup_client,
                self._cleanup_local_grids_request,
                "cleanup_local_grids",
                "🧹 Очистка occupancy grid завершена",
                "cleanup grids",
                30.0,
            ),
            (
                self.publish_map_client,
                self._publish_map_request,
                "publish_map",
                "🗺️ Публикация карты завершена",
                "publish map",
                15.0,
            ),
            (
                self.backup_client,
                self._backup_request,
                "backup",
                "💾 Backup завершён",
                "backup",
                15.0,
            ),
        ]:
            if client is None or request is None:
                self.log_warning(f"⚠️ {svc_name} service недоступен в текущей сборке")
                failed.append(svc_name)
                continue

            if not client.service_is_ready():
                self.log_warning(f"⚠️ {svc_name} service не готов")
                failed.append(svc_name)
                continue

            future = client.call_async(request)
            if not wait_future(future, timeout_sec=timeout_sec) or future.result() is None:
                self.log_warning(f"⚠️ {svc_name} не завершился успешно")
                failed.append(svc_name)
                continue

            self.log_info(ok_msg)
            completed.append(step_label)

        if not completed:
            return MCPToolResult(
                success=False,
                error=f"RTABMap оптимизация не выполнена. Проблемные сервисы: {', '.join(failed)}.",
                data={"reason": "unavailable", "failed": failed},
            )

        result_msg = f"Оптимизация карты завершена: {', '.join(completed)}."
        if failed:
            result_msg += f" Не удалось: {', '.join(failed)}."
        return MCPToolResult(
            success=True, message=result_msg, data={kn.MAP_PARTIAL_FIELD: bool(failed), "failed": failed}
        )


class LoadMapTool(MCPTool):
    """Загрузить существующую карту и перейти в режим локализации."""

    event_source = "srv:/rtabmap/rtabmap/load_database"

    def __init__(self, node, waypoint_store: Optional["WaypointAdapter"] = None, mapping_state=None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        self.mapping_state = mapping_state
        self._service_cb_group = ReentrantCallbackGroup() if ReentrantCallbackGroup is not None else None
        from std_srvs.srv import Empty

        # LoadDatabase импортируется лениво — rtabmap_msgs может отсутствовать на Vision Pi
        try:
            from rtabmap_msgs.srv import LoadDatabase  # type: ignore
            self.load_database_client = _create_service_client(
                node, LoadDatabase, "/rtabmap/rtabmap/load_database", self._service_cb_group
            )
        except ImportError:
            self.load_database_client = None
        self.set_mode_localization_client = _create_service_client(
            node, Empty, "/rtabmap/rtabmap/set_mode_localization", self._service_cb_group
        )

    @property
    def name(self) -> str:
        return "load_map"

    @property
    def description(self) -> str:
        return (
            "Перезагрузить карту из базы и перейти в режим локализации (навигации). "
            "База карт у робота одна: загрузить можно только карту, что в ней сейчас; "
            "про карту с другим именем тул честно ответит, что её нет."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="map_name",
                type="string",
                description="Название карты (например 'квартира'). Если не указано — перезагружает текущую.",
                required=False,
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG

    def _refuse(self, wanted: str, active_name: Optional[str], reason: str, message: str,
                error: Optional[str] = None) -> MCPToolResult:
        """Честный отказ без перезагрузки: причину и фразу строит код (#4392)."""
        self.log_warning(f"⚠️ load_map('{wanted}'): {reason}, в базе карта '{active_name or ''}'")
        return MCPToolResult(
            success=False,
            error=error or message,
            message=message,
            data={"outcome": reason, "reason": reason, kn.MAP_NAME_FIELD: wanted or None,
                  kn.MAP_ACTIVE_FIELD: active_name},
        )

    def _check_request(self, wanted: str) -> Tuple[Optional[MCPToolResult], Optional[str]]:
        """Можно ли загрузить карту ``wanted``: ``(отказ или None, имя карты в базе)``. Ничего не переключает."""
        if not wanted:
            return None, _active_name(self.waypoint_store)
        lookup_failed = f"Не удалось проверить карту «{wanted}», ничего не загружал"
        if self.waypoint_store is None:
            return self._refuse(wanted, None, "map_lookup_failed", lookup_failed), None
        try:
            active = _active_map(self.waypoint_store)
            maps = self.waypoint_store.list_maps()
        except Exception as exc:  # noqa: BLE001 - нельзя подтвердить имя = нельзя говорить «загружена»
            return self._refuse(wanted, None, "map_lookup_failed", lookup_failed,
                                error=f"Не удалось проверить карту '{wanted}' в WaypointStore: {exc}"), None
        active_name = (active or {}).get("name") or None
        verdict = resolve_map_request(wanted, active, maps)
        if verdict == MAP_NOT_STORED:
            where = f" — «{active_name}»" if active_name else ", и это не она"
            return self._refuse(wanted, active_name, kn.MAP_NOT_STORED_REASON,
                                f"Карту «{wanted}» не храню: база карт у меня одна{where}. Ничего не загружал"), None
        if verdict == MAP_UNKNOWN:
            return self._refuse(wanted, active_name, "map_not_found",
                                f"Карты «{wanted}» нет, ничего не загружал"), None
        return None, active_name

    def execute(self, map_name: str = "") -> MCPToolResult:
        """Перезагрузить карту из базы и переключиться в режим локализации."""
        # Нет клиента (rtabmap_msgs не собран на этой Pi) — понятная ошибка, не исключение (#3789).
        if self.load_database_client is None:
            return MCPToolResult(
                success=False,
                error="RTABMap load_database service недоступен (нет rtabmap_msgs)",
                message="Сервис карт недоступен",
                data={"outcome": "unavailable", "reason": "unavailable"},
            )

        from rtabmap_msgs.srv import LoadDatabase  # type: ignore
        from std_srvs.srv import Empty

        # База одна (#4392): имя не той карты, что в ней, - честный отказ, ничего не перезагружаем.
        wanted = (map_name or "").strip()
        refusal, loaded_name = self._check_request(wanted)
        if refusal is not None:
            return refusal

        # Перезагрузить базу данных (clear=False — не стирать, просто reload)
        req = LoadDatabase.Request()
        req.database_path = SLAM_DB_PATH
        req.clear = False
        loaded = _call_service(self.load_database_client, req, 20.0, "RTABMap load_database")
        if not loaded.success:
            return _failed_call(self, loaded, "Не удалось загрузить карту")
        self.log_info("🗺️ База данных перезагружена")

        # Переключить в режим локализации
        switched = _call_service(
            self.set_mode_localization_client, Empty.Request(), 10.0, "RTABMap set_mode_localization"
        )
        if not switched.success:
            return _failed_call(self, switched, "Карта загружена, но режим навигации не включился")
        self.log_info("🧭 Режим локализации активирован")

        if self.mapping_state is not None:
            self.mapping_state.set_localization()
            self.log_info(f"🗺️  MappingState → localization via load_map (map='{loaded_name or ''}')")

        # Имя в фразе - карты, что реально в базе (из ``maps``), а не аргумент вызова.
        suffix = f" Карта: '{loaded_name}'." if loaded_name else ""
        return MCPToolResult(
            success=True,
            message=f"Карта загружена, переключаюсь в режим навигации.{suffix}",
            data={kn.MAP_NAME_FIELD: loaded_name},
        )


def _distinct_names(maps: List[Dict], skip: Optional[str]) -> List[str]:
    """Имена карт без повторов (по :func:`map_name_key`), кроме ``skip`` и безымянных, в порядке ``maps``."""
    seen = {map_name_key(skip)} if skip else set()
    names = []
    for m in maps:
        name = (m.get("name") or "").strip()
        if name and map_name_key(name) not in seen:
            seen.add(map_name_key(name))
            names.append(name)
    return names


def _maps_failed(error: str) -> MCPToolResult:
    return MCPToolResult(success=False, error=error, message="Не смог посмотреть карты",
                         data={"reason": "map_lookup_failed"})


class ListMapsTool(MCPTool):
    """Какие карты робот хранит (#4392): одна база - одна карта; остальные имена - без геометрии."""

    event_source = "inproc:waypoint_store"

    def __init__(self, node, waypoint_store: Optional["WaypointAdapter"] = None):
        super().__init__(node)
        self.waypoint_store = waypoint_store

    @property
    def name(self) -> str:
        return "list_maps"

    @property
    def description(self) -> str:
        return (
            "Какие карты помещений робот хранит: «какие карты знаешь», «на какой ты карте». "
            "Отвечай по данным тула: база карт одна, остальные имена — старые карты без геометрии."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.INSTANT

    def execute(self) -> MCPToolResult:
        if self.waypoint_store is None:
            return _maps_failed("WaypointStore недоступен")
        try:
            active = _active_map(self.waypoint_store)
            maps = self.waypoint_store.list_maps()
        except Exception as exc:  # noqa: BLE001 - не прочли БД карт: так и говорим
            return _maps_failed(f"WaypointStore: {exc}")
        active_name = (active or {}).get("name") or None
        stale = _distinct_names([m for m in maps if not m.get("is_active")], active_name)
        if active is None:
            head = "Готовой карты у меня нет."
        elif active_name:
            head = f"Храню одну карту — «{active_name}»: база карт у меня одна."
        else:
            head = "Храню одну карту, без имени: база карт у меня одна."
        tail = f" Старые названия без карты: {', '.join(stale)}." if stale else ""
        return MCPToolResult(
            success=True,
            message=head + tail,
            data={kn.MAP_NAME_FIELD: active_name, "has_map": active is not None, "stale_names": stale},
        )


class DeleteMapTool(MCPTool):
    """Забыть карту по имени (#4392): строки старых карт удаляются, карту в базе удалить нельзя."""

    event_source = "inproc:waypoint_store"

    def __init__(self, node, waypoint_store: Optional["WaypointAdapter"] = None):
        super().__init__(node)
        self.waypoint_store = waypoint_store

    @property
    def name(self) -> str:
        return "delete_map"

    @property
    def description(self) -> str:
        return (
            "Забыть (удалить) карту по имени: «удали карту офис», «забудь карту». "
            "Карту, что сейчас в базе, удалить нельзя — её можно только перезаписать новым картографированием."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(name="map_name", type="string", description="Название карты, которую забыть",
                             required=True),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.INSTANT

    @property
    def slice(self) -> str:
        return "personality"

    def _fail(self, wanted: str, reason: str, message: str, error: Optional[str] = None) -> MCPToolResult:
        return MCPToolResult(success=False, error=error or message, message=message,
                             data={"reason": reason, kn.MAP_NAME_FIELD: wanted or None})

    def execute(self, map_name: str = "") -> MCPToolResult:
        wanted = (map_name or "").strip()
        if not wanted:
            return self._fail(wanted, "map_not_found", "Какую карту забыть? Назови её")
        if self.waypoint_store is None:
            return self._fail(wanted, "map_lookup_failed", f"Не смог удалить карту «{wanted}»")
        try:
            active = _active_map(self.waypoint_store)
            if active is not None and map_name_key(active.get("name")) == map_name_key(wanted):
                return self._fail(wanted, kn.MAP_DELETE_ACTIVE_REASON,
                                  f"Карту «{wanted}» удалить не могу: она у меня в базе одна, "
                                  "её можно только перезаписать новой картой")
            removed = self.waypoint_store.delete_inactive_maps_by_name(wanted)
        except Exception as exc:  # noqa: BLE001 - БД карт упала: не говорим «удалил»
            return self._fail(wanted, "map_lookup_failed", f"Не смог удалить карту «{wanted}»",
                              error=f"WaypointStore: {exc}")
        if removed == 0:
            return self._fail(wanted, "map_not_found", f"Карты «{wanted}» у меня нет, удалять нечего")
        self.log_info(f"🗑️ Забыта карта '{wanted}' ({removed} строк maps)")
        return MCPToolResult(success=True, message=f"Забыл карту «{wanted}».",
                             data={kn.MAP_NAME_FIELD: wanted, "removed": removed})
