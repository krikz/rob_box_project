"""
test_system.py - Unit тесты для системных инструментов

Тестирует:
- GetRobotStatusTool: чтение реальных данных из /odom и /battery_state,
  graceful обработка недоступности топиков.

Паттерн моков ROS-модулей такой же, как в test_animation.py — тесты
работают и без установленного ROS 2 (локально), и в CI-контейнере
(rob-box-ci:humble), где ROS 2 доступен.
"""

import importlib.util
import sys
from unittest.mock import Mock

import pytest

# ROS-модули, которые пакет импортирует на уровне модуля (через tools/__init__).
# Если ROS 2 не установлен — подменяем их моками, чтобы пакет импортировался.
# В CI-контейнере реальные модули доступны, поэтому моки НЕ ставятся.
_ROS_MODULE_SUBMODULES = {
    "rclpy": ["action", "node", "callback_groups", "qos"],
    "geometry_msgs": ["msg"],
    "nav2_msgs": ["action"],
    "action_msgs": ["srv", "msg"],
    "std_msgs": ["msg"],
    "std_srvs": ["srv"],
    "nav_msgs": ["msg"],
    "sensor_msgs": ["msg"],
}


def _install_ros_mocks_if_needed():
    """Подменить ROS-модули моками, только если они реально не установлены."""
    for parent, submodules in _ROS_MODULE_SUBMODULES.items():
        # Модуль уже импортирован (реальный ROS или замокан другим тест-файлом) —
        # не трогаем и не вызываем find_spec (на Mock без __spec__ он падает).
        if parent in sys.modules:
            continue
        if importlib.util.find_spec(parent) is not None:
            continue  # реальный ROS-модуль доступен — не трогаем
        sys.modules.setdefault(parent, Mock())
        for sub in submodules:
            sys.modules.setdefault(f"{parent}.{sub}", Mock())


_install_ros_mocks_if_needed()

from rob_box_mcp_tools.tools.system import GetCurrentTimeTool, GetRobotStatusTool


def _make_odom_msg(x=1.5, y=-2.25, qz=0.0, qw=1.0):
    """Создать мок nav_msgs/Odometry с заданной позицией и yaw-кватернионом."""
    msg = Mock()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    return msg


def _make_battery_msg(percentage=85.0):
    """Создать мок sensor_msgs/BatteryState с заданным процентом заряда."""
    msg = Mock()
    msg.percentage = percentage
    return msg


@pytest.mark.unit
class TestGetRobotStatusTool:
    """Тесты GetRobotStatusTool."""

    def test_tool_creation(self, mock_node):
        """Тест создания инструмента и подписок на ROS-топики."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        assert tool.name == "get_robot_status"
        assert "позиция" in tool.description
        assert len(tool.parameters) == 0
        assert tool.ODOM_TOPIC == "/odom"
        assert tool.BATTERY_TOPIC == "/battery_state"

        # Подписки созданы на оба топика
        assert mock_node.get_subscription("/odom") is not None
        assert mock_node.get_subscription("/battery_state") is not None

    def test_execute_without_data_returns_error(self, mock_node):
        """Без сообщений из топиков — понятная ошибка, а не hardcoded заглушка."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        result = tool.execute()

        assert result.success is False
        assert "/odom" in result.error
        assert "/battery_state" in result.error
        # Никаких фейковых значений
        assert result.data["position"] is None
        assert result.data["battery_level"] is None

    def test_execute_with_odom_only(self, mock_node):
        """Данные /odom приходят, /battery_state нет — позиция реальная, батарея помечена недоступной."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        odom_sub = mock_node.get_subscription("/odom")
        odom_sub.callback(_make_odom_msg(x=2.0, y=3.0))

        result = tool.execute()

        assert result.success is True
        assert result.data["position"]["x"] == 2.0
        assert result.data["position"]["y"] == 3.0
        assert result.data["battery_level"] is None
        assert "/battery_state" in result.data["unavailable_topics"]

    def test_execute_with_battery_only(self, mock_node):
        """Данные /battery_state приходят, /odom нет — батарея реальная, позиция помечена недоступной."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        battery_sub = mock_node.get_subscription("/battery_state")
        battery_sub.callback(_make_battery_msg(percentage=72.5))

        result = tool.execute()

        assert result.success is True
        assert result.data["battery_level"] == 72.5
        assert result.data["position"] is None
        assert "/odom" in result.data["unavailable_topics"]

    def test_execute_full_status(self, mock_node):
        """Оба топика публикуют — возвращаются реальные позиция и батарея."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        odom_sub = mock_node.get_subscription("/odom")
        # Кватернион 90° вокруг Z: qz=qw=0.7071 → theta ≈ π/2
        odom_sub.callback(_make_odom_msg(x=1.5, y=-2.25, qz=0.7071067811865476, qw=0.7071067811865476))

        battery_sub = mock_node.get_subscription("/battery_state")
        battery_sub.callback(_make_battery_msg(percentage=42.0))

        result = tool.execute()

        assert result.success is True
        assert "unavailable_topics" not in result.data
        assert result.data["position"]["x"] == 1.5
        assert result.data["position"]["y"] == -2.25
        assert abs(result.data["position"]["theta"] - (3.141592653589793 / 2.0)) < 1e-6
        assert result.data["battery_level"] == 42.0
        assert result.data["systems"]["navigation"] == "active"

    def test_battery_unknown_percentage(self, mock_node):
        """percentage == -1.0 (неизвестно по ROS-конвенции) — батарея считается недоступной."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        battery_sub = mock_node.get_subscription("/battery_state")
        battery_sub.callback(_make_battery_msg(percentage=-1.0))

        result = tool.execute()

        assert result.data["battery_level"] is None
        assert "/battery_state" in result.data.get("unavailable_topics", [])

    def test_execute_without_node(self):
        """Инструмент без node (например, в unit-тестах) — graceful ошибка недоступности."""
        tool = GetRobotStatusTool(None, wait_timeout_sec=0.0)

        result = tool.execute()

        assert result.success is False
        assert "/odom" in result.error


@pytest.mark.unit
class TestGetCurrentTimeTool:
    """Тесты Issue #1777 / #1763: ``get_current_time`` должен возвращать
    время в timezone робота (Europe/Moscow по умолчанию), а не naive UTC."""

    def test_tool_metadata(self):
        """name/description/parameters контракт совпадают с системным промптом LLM."""
        tool = GetCurrentTimeTool(None)
        assert tool.name == "get_current_time"
        assert "время" in tool.description.lower()
        assert tool.parameters == []
        assert tool.DEFAULT_TIMEZONE == "Europe/Moscow"

    def test_default_timezone_is_moscow(self, monkeypatch):
        """Без ROBOT_TIMEZONE env → Europe/Moscow (UTC+3)."""
        monkeypatch.delenv("ROBOT_TIMEZONE", raising=False)
        tool = GetCurrentTimeTool(None)
        tz = tool._resolve_timezone()
        import datetime
        from datetime import timedelta
        # Europe/Moscow всегда UTC+3 (без DST). Передаём конкретную дату
        # в ``utcoffset`` — с None оно возвращает None для zoneinfo.
        probe = datetime.datetime(2026, 1, 15, 12, 0, 0)
        assert tz.utcoffset(probe) == timedelta(hours=3)
        assert "Moscow" in str(tz)

    def test_uses_robot_timezone_env(self, monkeypatch):
        """ROBOT_TIMEZONE env проброшено в tz (Europe/Berlin → UTC+2 в летнее время)."""
        monkeypatch.setenv("ROBOT_TIMEZONE", "Europe/Berlin")
        tool = GetCurrentTimeTool(None)
        tz = tool._resolve_timezone()
        assert "Berlin" in str(tz)

    def test_unknown_zoneinfo_falls_back_to_utc(self, monkeypatch, caplog):
        """Неизвестная timezone (нет в tzdata) — graceful fallback в UTC, не падает."""
        monkeypatch.setenv("ROBOT_TIMEZONE", "Mars/Olympus_Mons")
        tool = GetCurrentTimeTool(None)
        tz = tool._resolve_timezone()
        from datetime import timezone
        assert tz == timezone.utc

    def test_execute_returns_moscow_time_when_system_tz_is_utc(self, monkeypatch):
        """Главный кейс #1763/#1777: при системном TZ=UTC инструмент возвращает MSK.

        Симулируем «UTC в контейнере»: os.environ['TZ']='UTC' + time.tzset()
        (если доступно). Главное — даже если бы now() вернул UTC, наш
        инструмент форсит Europe/Moscow.
        """
        import datetime
        monkeypatch.delenv("ROBOT_TIMEZONE", raising=False)
        # Принудительно говорим системе что TZ=UTC (best-effort, glibc only).
        monkeypatch.setenv("TZ", "UTC")
        try:
            import time as _time
            _time.tzset()
        except Exception:
            pass  # не glibc-платформа (Windows/macOS) — наш фикс всё равно работает

        tool = GetCurrentTimeTool(None)
        result = tool.execute()
        assert result.success is True
        assert result.data is not None  # mypy/pyright — data Optional, но тут точно dict

        # iso имеет timezone-offset +03:00 (Europe/Moscow), не naive и не +00:00.
        iso = result.data["iso"]
        assert iso.endswith("+03:00"), f"expected +03:00 (MSK), got iso={iso!r}"
        assert result.data["timezone"] == "Europe/Moscow"

        # Сверить с независимым datetime.now(MSK): дельта < 5 секунд.
        msk_now = datetime.datetime.now(tz=datetime.timezone(datetime.timedelta(hours=3)))
        tool_now = datetime.datetime.fromisoformat(iso)
        assert abs((msk_now - tool_now).total_seconds()) < 5.0

    def test_execute_uses_robot_timezone(self, monkeypatch):
        """ROBOT_TIMEZONE=Europe/Berlin пробрасывается в execute()."""
        monkeypatch.setenv("ROBOT_TIMEZONE", "Europe/Berlin")
        tool = GetCurrentTimeTool(None)
        result = tool.execute()
        assert result.success is True
        assert result.data is not None  # mypy/pyright — data Optional, но тут точно dict
        assert "Berlin" in result.data["timezone"]
        # ISO содержит +01:00 или +02:00 (зависит от DST), но НЕ +00:00 (UTC)
        assert not result.data["iso"].endswith("+00:00") or "Berlin" in result.data["iso"]


# Issue #1762 — formatted_time (русская пропись для дословного озвучивания)
# Эти тесты живут в test_issue_1762_formatted_time.py в нашем PR; здесь
# добавляем smoke-проверку, что поле не теряется после rebase.
@pytest.mark.unit
class TestGetCurrentTimeFormatted:
    """Issue #1762: ``formatted_time`` поле должно быть в data."""

    def test_formatted_time_field_present(self, monkeypatch):
        """Поле ``formatted_time`` присутствует в data (русская пропись)."""
        monkeypatch.delenv("ROBOT_TIMEZONE", raising=False)
        tool = GetCurrentTimeTool(None)
        result = tool.execute()
        assert result.success is True
        assert result.data is not None
        assert "formatted_time" in result.data, (
            f"formatted_time missing after rebase, got keys: {list(result.data.keys())}"
        )
        # Минимальная sanity-проверка: непустая строка с буквами (русский текст).
        ft = result.data["formatted_time"]
        assert isinstance(ft, str)
        assert len(ft) > 0
        # Не «22:37», а пропись: «двадцать два тридцать семь»
        assert ":" not in ft, f"expected Russian prose, got {ft!r}"
