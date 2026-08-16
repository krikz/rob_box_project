#!/usr/bin/env python3
"""
system.py - Инструменты управления системой робота

Инструменты:
- SetVolumeTool: Установить громкость TTS
- SetPitchTool: Установить высоту голоса
- SetSpeedTool: Установить скорость речи
- GetRobotStatusTool: Получить статус робота
"""

import math
import threading
from typing import List, Optional, TYPE_CHECKING

# Ленивый импорт ROS 2 модулей для поддержки unit тестов
if TYPE_CHECKING:
    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType


def _wait_future(future, timeout_sec: float) -> bool:
    """Wait for an rclpy Future without touching the executor.

    ``rclpy.spin_until_future_complete()`` is UNSAFE to call from within a
    callback that is already executing under ``MultiThreadedExecutor`` — it
    internally tries to add the node to a *new* executor, which corrupts the
    existing one and silently breaks all subsequent subscription callbacks.

    This helper attaches a ``done_callback`` to the future so that a plain
    ``threading.Event`` is set when the future completes.  The calling thread
    blocks on the event, leaving the ROS 2 executor completely undisturbed.

    Returns True if the future completed within *timeout_sec*, False otherwise.
    """
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    return event.wait(timeout=timeout_sec)


class SetVolumeTool(MCPTool):
    """Инструмент для управления громкостью TTS."""

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from rcl_interfaces.srv import GetParameters, SetParameters

        self.get_params_client = node.create_client(GetParameters, "/tts_node/get_parameters")
        self.set_params_client = node.create_client(SetParameters, "/tts_node/set_parameters")

    @property
    def name(self) -> str:
        return "set_volume"

    @property
    def description(self) -> str:
        return "Установить громкость голоса робота. Используй для команд 'громче', 'тише', 'максимальная громкость'."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="action",
                type="string",
                description="Действие с громкостью",
                required=True,
                enum=["louder", "quieter", "max", "normal"],
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Set volume - FAST операция < 2s (ROS service call)."""
        return ToolExecutionType.FAST

    def execute(self, action: str) -> MCPToolResult:
        """Установить громкость."""
        # Динамический импорт во время выполнения
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters

        self.log_info(f"Изменение громкости: {action}")

        if not self.get_params_client.wait_for_service(timeout_sec=1.0):
            return MCPToolResult(success=False, error="TTS параметры недоступны")

        # Получить текущую громкость
        get_request = GetParameters.Request()
        get_request.names = ["volume_db"]
        future = self.get_params_client.call_async(get_request)
        if not _wait_future(future, timeout_sec=2.0):
            return MCPToolResult(success=False, error="Не удалось получить текущую громкость")

        if future.result() is None:
            return MCPToolResult(success=False, error="Не удалось получить текущую громкость")

        current_volume = future.result().values[0].double_value

        # Вычислить новую громкость
        if action == "louder":
            new_volume = min(current_volume + 3.0, 6.0)
            message = "Делаю громче"
        elif action == "quieter":
            new_volume = max(current_volume - 3.0, -20.0)
            message = "Делаю тише"
        elif action == "max":
            new_volume = 6.0
            message = "Максимальная громкость"
        elif action == "normal":
            new_volume = -3.0
            message = "Нормальная громкость"
        else:
            return MCPToolResult(success=False, error=f"Неизвестное действие: {action}")

        # Проверка на пределы
        if abs(new_volume - current_volume) < 0.1:
            if action == "louder":
                return MCPToolResult(success=True, message="Громкость уже максимальная")
            elif action == "quieter":
                return MCPToolResult(success=True, message="Громкость уже минимальная")

        # Установить новую громкость
        set_request = SetParameters.Request()
        param = Parameter()
        param.name = "volume_db"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = new_volume
        set_request.parameters = [param]

        future = self.set_params_client.call_async(set_request)
        if not _wait_future(future, timeout_sec=2.0):
            return MCPToolResult(success=False, error="Не удалось установить громкость (timeout)")

        if future.result() is None or not future.result().results[0].successful:
            return MCPToolResult(success=False, error="Не удалось установить громкость")

        self.log_info(f"Громкость: {current_volume:.1f} → {new_volume:.1f} dB")

        return MCPToolResult(
            success=True, data={"old_volume": current_volume, "new_volume": new_volume}, message=message
        )


class SetPitchTool(MCPTool):
    """Инструмент для управления высотой голоса."""

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from rcl_interfaces.srv import GetParameters, SetParameters

        self.get_params_client = node.create_client(GetParameters, "/tts_node/get_parameters")
        self.set_params_client = node.create_client(SetParameters, "/tts_node/set_parameters")

    @property
    def name(self) -> str:
        return "set_pitch"

    @property
    def description(self) -> str:
        return "Установить высоту голоса робота. Используй для команд 'говори выше', 'говори ниже'."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="action",
                type="string",
                description="Действие с высотой голоса",
                required=True,
                enum=["higher", "lower", "normal"],
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Set pitch - FAST операция < 2s (ROS service call)."""
        return ToolExecutionType.FAST

    def execute(self, action: str) -> MCPToolResult:
        """Установить высоту голоса."""
        # Динамический импорт во время выполнения
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters

        self.log_info(f"Изменение pitch: {action}")

        if not self.get_params_client.wait_for_service(timeout_sec=1.0):
            return MCPToolResult(success=False, error="TTS параметры недоступны")

        # Получить текущий pitch_shift
        get_request = GetParameters.Request()
        get_request.names = ["pitch_shift"]
        future = self.get_params_client.call_async(get_request)
        if not _wait_future(future, timeout_sec=2.0):
            return MCPToolResult(success=False, error="Не удалось получить текущий pitch")

        if future.result() is None:
            return MCPToolResult(success=False, error="Не удалось получить текущий pitch")

        current_pitch = future.result().values[0].double_value

        # Вычислить новый pitch
        if action == "higher":
            new_pitch = min(current_pitch + 0.2, 2.0)
            message = "Говорю выше"
        elif action == "lower":
            new_pitch = max(current_pitch - 0.2, 0.5)
            message = "Говорю ниже"
        elif action == "normal":
            new_pitch = 1.0
            message = "Нормальный голос"
        else:
            return MCPToolResult(success=False, error=f"Неизвестное действие: {action}")

        # Проверка на пределы
        if abs(new_pitch - current_pitch) < 0.01:
            if action == "higher":
                return MCPToolResult(success=True, message="Голос уже максимально высокий")
            elif action == "lower":
                return MCPToolResult(success=True, message="Голос уже минимально низкий")

        # Установить новый pitch
        set_request = SetParameters.Request()
        param = Parameter()
        param.name = "pitch_shift"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = new_pitch
        set_request.parameters = [param]

        future = self.set_params_client.call_async(set_request)
        if not _wait_future(future, timeout_sec=2.0):
            return MCPToolResult(success=False, error="Не удалось установить pitch (timeout)")

        if future.result() is None or not future.result().results[0].successful:
            return MCPToolResult(success=False, error="Не удалось установить pitch")

        self.log_info(f"Pitch: {current_pitch:.2f} → {new_pitch:.2f}")

        return MCPToolResult(success=True, data={"old_pitch": current_pitch, "new_pitch": new_pitch}, message=message)


class SetSpeedTool(MCPTool):
    """Инструмент для управления скоростью речи."""

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from rcl_interfaces.srv import GetParameters, SetParameters

        self.get_params_client = node.create_client(GetParameters, "/tts_node/get_parameters")
        self.set_params_client = node.create_client(SetParameters, "/tts_node/set_parameters")

    @property
    def name(self) -> str:
        return "set_speed"

    @property
    def description(self) -> str:
        return "Установить скорость речи робота. Используй для команд 'говори быстрее', 'говори медленнее'."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="action",
                type="string",
                description="Действие со скоростью речи",
                required=True,
                enum=["faster", "slower", "normal"],
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Set speed - FAST операция < 2s (ROS service call)."""
        return ToolExecutionType.FAST

    def execute(self, action: str) -> MCPToolResult:
        """Установить скорость речи."""
        # Динамический импорт во время выполнения
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters

        self.log_info(f"Изменение speed: {action}")

        if not self.get_params_client.wait_for_service(timeout_sec=1.0):
            return MCPToolResult(success=False, error="TTS параметры недоступны")

        # Получить текущую скорость
        get_request = GetParameters.Request()
        get_request.names = ["yandex_speed"]
        future = self.get_params_client.call_async(get_request)
        if not _wait_future(future, timeout_sec=2.0):
            return MCPToolResult(success=False, error="Не удалось получить текущую скорость")

        if future.result() is None:
            return MCPToolResult(success=False, error="Не удалось получить текущую скорость")

        current_speed = future.result().values[0].double_value

        # Вычислить новую скорость
        if action == "faster":
            new_speed = min(current_speed + 0.2, 2.0)
            message = "Говорю быстрее"
        elif action == "slower":
            new_speed = max(current_speed - 0.2, 0.5)
            message = "Говорю медленнее"
        elif action == "normal":
            new_speed = 1.0
            message = "Нормальная скорость"
        else:
            return MCPToolResult(success=False, error=f"Неизвестное действие: {action}")

        # Проверка на пределы
        if abs(new_speed - current_speed) < 0.01:
            if action == "faster":
                return MCPToolResult(success=True, message="Скорость уже максимальная")
            elif action == "slower":
                return MCPToolResult(success=True, message="Скорость уже минимальная")

        # Установить новую скорость
        set_request = SetParameters.Request()
        param = Parameter()
        param.name = "yandex_speed"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = new_speed
        set_request.parameters = [param]

        future = self.set_params_client.call_async(set_request)
        if not _wait_future(future, timeout_sec=2.0):
            return MCPToolResult(success=False, error="Не удалось установить скорость (timeout)")

        if future.result() is None or not future.result().results[0].successful:
            return MCPToolResult(success=False, error="Не удалось установить скорость")

        self.log_info(f"Speed: {current_speed:.2f} → {new_speed:.2f}")

        return MCPToolResult(
            success=True, data={"old_speed": current_speed, "new_speed": new_speed}, message=message
        )


class GetCurrentTimeTool(MCPTool):
    """Инструмент для получения текущего времени и даты.

    Не требует ROS-зависимостей — время берётся из системных часов Python.
    Вызывать когда пользователь спрашивает время, дату, день недели и т.д.
    """

    @property
    def name(self) -> str:
        return "get_current_time"

    @property
    def description(self) -> str:
        return (
            "Получить текущее время и дату. Используй когда пользователь спрашивает "
            "который час, какая дата, какой день недели, какое время суток, сколько сейчас времени."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        """Мгновенная операция — только Python datetime, никаких I/O."""
        return ToolExecutionType.INSTANT

    def execute(self) -> MCPToolResult:
        """Вернуть текущее время из системных часов."""
        import datetime
        import locale

        now = datetime.datetime.now()

        # Русские названия дней и месяцев без зависимости от locale
        WEEKDAYS_RU = ["понедельник", "вторник", "среда", "четверг", "пятница", "суббота", "воскресенье"]
        MONTHS_RU = [
            "января", "февраля", "марта", "апреля", "мая", "июня",
            "июля", "августа", "сентября", "октября", "ноября", "декабря"
        ]

        hour = now.hour
        if 5 <= hour < 12:
            period = "утро"
        elif 12 <= hour < 17:
            period = "день"
        elif 17 <= hour < 22:
            period = "вечер"
        else:
            period = "ночь"

        data = {
            "time": now.strftime("%H:%M"),
            "date": f"{now.day} {MONTHS_RU[now.month - 1]} {now.year}",
            "weekday": WEEKDAYS_RU[now.weekday()],
            "period": period,
            "iso": now.isoformat(timespec="seconds"),
        }

        message = (
            f"Сейчас {data['time']}, {data['weekday']}, {data['date']}, {data['period']}."
        )
        self.log_info(f"Текущее время: {message}")
        return MCPToolResult(success=True, data=data, message=message)


class GetRobotStatusTool(MCPTool):
    """Инструмент для получения статуса робота.

    Реальные данные читаются из ROS-топиков:
    - /odom          (nav_msgs/Odometry)     — позиция робота (x, y, theta)
    - /battery_state (sensor_msgs/BatteryState) — уровень заряда батареи (%)

    Подписки создаются при инициализации инструмента, execute() отдаёт
    последние полученные значения. Если топик не публикует данные — в ответе
    явная ошибка/пометка недоступности вместо hardcoded заглушки.
    """

    # Топики ROS для реальных данных
    ODOM_TOPIC = "/odom"
    BATTERY_TOPIC = "/battery_state"

    def __init__(self, node, wait_timeout_sec: float = 2.0):
        super().__init__(node)
        self._position: Optional[dict] = None  # {"x": float, "y": float, "theta": float}
        self._battery_level: Optional[float] = None  # проценты 0-100
        self._data_event = threading.Event()
        # Сколько ждать первое сообщение из топиков при «холодном» старте
        self._wait_timeout_sec = wait_timeout_sec

        if self.node is not None:
            # Динамический импорт ROS 2 модулей для поддержки unit тестов
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import BatteryState

            self.node.create_subscription(Odometry, self.ODOM_TOPIC, self._on_odom, 10)
            self.node.create_subscription(BatteryState, self.BATTERY_TOPIC, self._on_battery, 10)

    def _on_odom(self, msg):
        """Сохранить последнюю позицию из /odom (nav_msgs/Odometry)."""
        try:
            pos = msg.pose.pose.position
            q = msg.pose.pose.orientation
            # yaw (theta) из кватерниона ориентации
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)
            self._position = {"x": float(pos.x), "y": float(pos.y), "theta": theta}
            self._data_event.set()
        except Exception as e:
            self.log_warning(f"Ошибка обработки /odom: {e}")

    def _on_battery(self, msg):
        """Сохранить последний уровень заряда из /battery_state (sensor_msgs/BatteryState)."""
        try:
            # ROS: percentage == -1.0 означает «неизвестно»
            if msg.percentage is not None and float(msg.percentage) >= 0.0:
                self._battery_level = float(msg.percentage)
            else:
                self._battery_level = None
            self._data_event.set()
        except Exception as e:
            self.log_warning(f"Ошибка обработки /battery_state: {e}")

    @property
    def name(self) -> str:
        return "get_robot_status"

    @property
    def description(self) -> str:
        return "Получить текущий статус робота (позиция, батарея, состояние систем)."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        """Get robot status - MEDIUM операция 2-10s (множественные ROS queries)."""
        return ToolExecutionType.MEDIUM

    def execute(self) -> MCPToolResult:
        """Получить статус робота из реальных ROS-топиков /odom и /battery_state."""
        self.log_info("Запрос статуса робота")

        # Если данные ещё не приходили — ждём первое сообщение (топики обычно
        # публикуют с частотой 10-50 Гц, так что первое сообщение приходит быстро).
        if self._position is None or self._battery_level is None:
            self._data_event.wait(timeout=self._wait_timeout_sec)

        unavailable = []
        if self._position is None:
            unavailable.append(self.ODOM_TOPIC)
        if self._battery_level is None:
            unavailable.append(self.BATTERY_TOPIC)

        status = {
            "position": self._position,
            "battery_level": self._battery_level,
            "systems": {"navigation": "active", "vision": "active", "tts": "active"},
        }

        if unavailable:
            missing = ", ".join(unavailable)
            status["unavailable_topics"] = unavailable
            # Частичные данные: хотя бы один топик жив — отдаём реальные данные с пометкой
            if self._position is not None or self._battery_level is not None:
                return MCPToolResult(
                    success=True,
                    data=status,
                    message=f"Статус робота получен (недоступны топики: {missing})",
                )
            return MCPToolResult(
                success=False,
                data=status,
                error=(
                    "Статус робота недоступен: топики /odom и /battery_state не публикуют данные "
                    "(ROS-топики не запущены или данные ещё не получены)"
                ),
            )

        return MCPToolResult(success=True, data=status, message="Статус робота получен")
