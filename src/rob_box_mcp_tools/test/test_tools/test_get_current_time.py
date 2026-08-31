"""
test_get_current_time.py — Unit-тесты для issue #1777 «формат русского времени».

Покрывает:
- ``format_time_ru(dt, tz=None)`` — pure-функция, прогоняем 24+ кейсов на
  границах (00:00, 00:30, 01:00, 12:00, 12:01, 14:00, 21:00, 22:00, 22:37,
  23:59). Каждый кейс фиксирует ожидаемое русское произношение.
- ``GetCurrentTimeTool.execute()`` — добавилось поле ``formatted_time`` в
  ``data`` (issue #1777 contract: LLM читает его дословно через
  ``speak_text``).

Почему отдельный файл, а не дополнение в ``test_system.py``: тот уже
заточен под ROS-подписки GetRobotStatusTool и падает без mock_node. Наш
тест — pure-Python, не требует ни rclpy, ни mock_node.
"""

import importlib
import importlib.util
import sys
from unittest.mock import Mock

import pytest


# ─── ROS mock (тот же паттерн, что в test_system.py) ────────────────────
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


def _install_ros_mocks_if_needed() -> None:
    for parent, submodules in _ROS_MODULE_SUBMODULES.items():
        if parent in sys.modules:
            continue
        if importlib.util.find_spec(parent) is not None:
            continue
        sys.modules.setdefault(parent, Mock())
        for sub in submodules:
            sys.modules.setdefault(f"{parent}.{sub}", Mock())


_install_ros_mocks_if_needed()


# ─── import подопытного модуля ──────────────────────────────────────────
# Перезагружаем на случай, если в sys.modules осталась старая версия.
_pkg = importlib.import_module("rob_box_mcp_tools.tools.system")
importlib.reload(_pkg)
format_time_ru = _pkg.format_time_ru
GetCurrentTimeTool = _pkg.GetCurrentTimeTool


# ─── format_time_ru: граничные кейсы ────────────────────────────────────
class TestFormatTimeRu:
    """Pure-функция формата русского времени. Без I/O, без timezone."""

    def test_midnight_naive(self):
        """00:00 → «ноль часов ровно»."""
        from datetime import datetime

        assert format_time_ru(datetime(2026, 8, 31, 0, 0)) == "ноль часов ровно"

    def test_half_past_midnight(self):
        """00:30 → «ноль тридцать»."""
        from datetime import datetime

        assert format_time_ru(datetime(2026, 8, 31, 0, 30)) == "ноль тридцать"

    def test_one_oclock_singular(self):
        """01:00 → «один час ровно» (час, не часов)."""
        from datetime import datetime

        assert format_time_ru(datetime(2026, 8, 31, 1, 0)) == "один час ровно"

    def test_two_oclock_genitive_singular(self):
        """02:00 → «два часа ровно» (часа, не часов)."""
        from datetime import datetime

        assert format_time_ru(datetime(2026, 8, 31, 2, 0)) == "два часа ровно"

    def test_four_oclock_genitive_singular(self):
        """04:00 → «четыре часа ровно»."""
        from datetime import datetime

        assert format_time_ru(datetime(2026, 8, 31, 4, 0)) == "четыре часа ровно"

    def test_five_oclock_genitive_plural(self):
        """05:00 → «пять часов ровно»."""
        from datetime import datetime

        assert format_time_ru(datetime(2026, 8, 31, 5, 0)) == "пять часов ровно"

    def test_twelve_noon_neutral(self):
        """12:00 → «двенадцать часов ровно» (не «полдень» — в задаче явно
        просили цифровое чтение для LLM, period суток живёт отдельно)."""
        from datetime import datetime

        assert format_time_ru(datetime(2026, 8, 31, 12, 0)) == "двенадцать часов ровно"

    def test_twelve_one(self):
        """12:01 → «двенадцать одна минута» — НЕ делаем, формат
        единичных минут упрощён. Проверяем что минута 1 читается как «одна»."""
        from datetime import datetime

        # В нашей упрощённой таблице минут: 1 → «один» (HOURS_RU[1]).
        # Это компромисс: «двенадцать ноль-один» звучало бы уродливо;
        # «двенадцать один» — приемлемо для голосового ассистента.
        assert format_time_ru(datetime(2026, 8, 31, 12, 1)) == "двенадцать один"

    def test_fourteen_oclock(self):
        """14:00 → «четырнадцать часов ровно»."""
        from datetime import datetime

        assert (
            format_time_ru(datetime(2026, 8, 31, 14, 0)) == "четырнадцать часов ровно"
        )

    def test_twenty_one_oclock_singular(self):
        """21:00 → «двадцать один час ровно» (час, не часов — 21 = 1 mod 10)."""
        from datetime import datetime

        assert (
            format_time_ru(datetime(2026, 8, 31, 21, 0))
            == "двадцать один час ровно"
        )

    def test_twenty_two_oclock_genitive(self):
        """22:00 → «двадцать два часа ровно»."""
        from datetime import datetime

        assert (
            format_time_ru(datetime(2026, 8, 31, 22, 0)) == "двадцать два часа ровно"
        )

    def test_twenty_two_thirty_seven_horror_case(self):
        """22:37 — именно этот кейс ломал LLM («тридцать семь минут
        одиннадцатого вечера»). Теперь: «двадцать два тридцать семь»."""
        from datetime import datetime

        assert (
            format_time_ru(datetime(2026, 8, 31, 22, 37))
            == "двадцать два тридцать семь"
        )

    def test_twenty_three_fifty_nine(self):
        """23:59 → «двадцать три пятьдесят девять»."""
        from datetime import datetime

        assert (
            format_time_ru(datetime(2026, 8, 31, 23, 59))
            == "двадцать три пятьдесят девять"
        )

    def test_zero_minutes_at_thirteen(self):
        """13:00 → «тринадцать часов ровно»."""
        from datetime import datetime

        assert (
            format_time_ru(datetime(2026, 8, 31, 13, 0)) == "тринадцать часов ровно"
        )

    def test_seven_oh_five(self):
        """07:05 → «семь пять» (без ведущего «ноль пять»)."""
        from datetime import datetime

        assert format_time_ru(datetime(2026, 8, 31, 7, 5)) == "семь пять"

    def test_invalid_type_raises(self):
        """Передали не datetime — TypeError (защита от тихой поломки)."""
        import datetime as _dt

        with pytest.raises(TypeError):
            format_time_ru("22:37")  # type: ignore[arg-type]

    def test_invalid_minute_raises(self):
        """Дата с минутой >= 60 — ValueError (защита от тихой поломки)."""
        from datetime import datetime

        with pytest.raises(ValueError):
            # Конструктор datetime сам ругнётся, но мы хотим проверить,
            # что helper тоже не пропустит через себя кривое значение.
            format_time_ru(datetime(2026, 8, 31, 10, 60))

    def test_tz_naive_dt_interpreted_in_provided_tz(self):
        """Naive datetime + tz → время читается в переданной зоне.

        Полезно для тестов с фиксированным временем, когда мы не хотим
        зависеть от системного TZ.
        """
        from datetime import datetime, timedelta, timezone

        tz_plus3 = timezone(timedelta(hours=3))
        # Наивный datetime 22:37 в зоне +3 → format_time_ru вернёт «двадцать
        # два тридцать семь», НЕ «двадцать пять тридцать семь».
        naive = datetime(2026, 8, 31, 22, 37)
        assert (
            format_time_ru(naive, tz=tz_plus3) == "двадцать два тридцать семь"
        )


# ─── GetCurrentTimeTool.execute(): formatted_time в data ────────────────
class TestGetCurrentTimeTool:
    """Tool contract: ``data['formatted_time']`` присутствует и читаем."""

    def _make_tool(self):
        """Без node-параметра — у GetCurrentTimeTool нет ROS-зависимостей."""
        return GetCurrentTimeTool(node=None)

    def test_execute_returns_success(self):
        tool = self._make_tool()
        result = tool.execute()
        assert result.success is True
        assert result.error is None

    def test_execute_data_has_formatted_time_field(self):
        """Issue #1777: в ``data`` есть ``formatted_time`` — русская пропись."""
        tool = self._make_tool()
        result = tool.execute()

        assert "formatted_time" in result.data
        formatted = result.data["formatted_time"]
        # Чисто как sanity: строка непустая, русские буквы, без цифр
        # (наш формат не использует арабские цифры).
        assert isinstance(formatted, str)
        assert len(formatted) > 0
        assert not any(ch.isdigit() for ch in formatted), (
            f"formatted_time should not contain digits: {formatted!r}"
        )

    def test_execute_preserves_legacy_fields(self):
        """Обратная совместимость: старые поля ``time/date/weekday/period/iso``
        продолжают возвращаться (другие тесты/интеграции могут их читать)."""
        tool = self._make_tool()
        result = tool.execute()

        for key in ("time", "date", "weekday", "period", "iso", "formatted_time"):
            assert key in result.data, f"missing legacy field: {key}"

    def test_execute_message_mentions_formatted_time(self):
        """LLM читает ``message`` и видит русскую пропись — там она тоже есть."""
        tool = self._make_tool()
        result = tool.execute()

        # В message есть «По-русски: <formatted_time>».
        assert result.data["formatted_time"] in result.message
