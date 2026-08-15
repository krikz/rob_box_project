"""
test_sound.py - Unit тесты для инструментов звука

Тестирует:
- PlaySoundTool: загрузка каталога, воспроизведение, ошибки неизвестного звука
- GetSoundInfoTool: получение информации о звуках, фильтры, ошибки

Покрытие: success + error + invalid params.
ROS 2 модули мокаются — тесты работают без робота.
"""

import json
import sys
from unittest.mock import MagicMock, Mock, patch

import pytest

# Mock ROS 2 модулей перед импортом tools
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "std_msgs",
    "std_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.sound import (  # noqa: E402
    PlaySoundTool,
    GetSoundInfoTool,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warning(self, msg):
        self.messages.append(("warning", msg))

    def error(self, msg):
        self.messages.append(("error", msg))

    def debug(self, msg):
        self.messages.append(("debug", msg))


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()
        self.publishers = {}

    def get_logger(self):
        return self._logger

    def create_publisher(self, msg_type, topic, qos):
        pub = Mock()
        pub.msg_type = msg_type
        pub.topic = topic
        pub.qos = qos
        self.publishers[topic] = pub
        return pub


_CATALOG_JSON = {
    "sounds": {
        "robot_happy.mp3": {
            "trigger": "robot_happy",
            "category": "base",
            "duration": 2.5,
            "description": "Happy sound",
            "description_ru": "Радостный звук",
        },
        "ui_confirm.mp3": {
            "trigger": "ui_confirm",
            "category": "ui",
            "duration": 1.0,
            "description": "Confirm",
            "description_ru": "Подтверждение",
        },
        "robot_error.mp3": {
            "trigger": "robot_error",
            "category": "base",
            "duration": 1.8,
            "description": "Error",
            "description_ru": "Ошибка",
        },
    }
}


@pytest.fixture
def fake_node():
    return _FakeNode()


# ---------------------------------------------------------------------------
# PlaySoundTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestPlaySoundTool:
    def test_tool_metadata(self, fake_node):
        tool = PlaySoundTool(fake_node)
        assert tool.name == "play_sound"
        assert tool.execution_type.value == "instant"
        assert [p.name for p in tool.parameters] == ["sound"]
        # enum параметров = доступные звуки
        assert tool.parameters[0].enum == tool._available_sounds

    def test_load_catalog_fallback_when_no_file(self, fake_node):
        # Ни один из possible_paths не существует → fallback список
        with patch("rob_box_mcp_tools.tools.sound.Path.exists", return_value=False):
            tool = PlaySoundTool(fake_node)
        assert tool._available_sounds == PlaySoundTool.AVAILABLE_SOUNDS
        assert tool._duration_map == {}
        assert any("fallback" in m[1] for m in fake_node._logger.messages)

    def test_load_catalog_parses_json(self, fake_node, tmp_path):
        catalog_path = tmp_path / "sound_catalog.json"
        catalog_path.write_text(json.dumps(_CATALOG_JSON), encoding="utf-8")

        class _FakePath:
            """Подмена pathlib.Path: exists() всегда True, открывает tmp-файл.

            Модуль ищет каталог по фиксированным путям (/ws/... и т.п.),
            поэтому подменяем Path целиком: exists() возвращает True, а
            open() через __fspath__ читает реальный tmp-файл.
            """

            def __init__(self, p):
                self._p = p

            def exists(self):
                return True

            def __str__(self):
                return str(self._p)

            def __fspath__(self):
                return str(catalog_path)

        with patch("rob_box_mcp_tools.tools.sound.Path", _FakePath):
            tool = PlaySoundTool(fake_node)

        assert tool._available_sounds == ["robot_error", "robot_happy", "ui_confirm"]
        assert tool._duration_map["robot_happy"] == 2.5
        assert tool._duration_map["ui_confirm"] == 1.0

    def test_load_catalog_handles_corrupt_json(self, fake_node, tmp_path):
        catalog_path = tmp_path / "sound_catalog.json"
        catalog_path.write_text("{corrupt json", encoding="utf-8")

        class _FakePath:
            def __init__(self, p):
                self._p = p

            def exists(self):
                return True

            def __str__(self):
                return str(self._p)

            def __fspath__(self):
                return str(catalog_path)

        with patch("rob_box_mcp_tools.tools.sound.Path", _FakePath):
            tool = PlaySoundTool(fake_node)

        # Ошибка парсинга → fallback
        assert tool._available_sounds == PlaySoundTool.AVAILABLE_SOUNDS
        assert any("Ошибка загрузки" in m[1] for m in fake_node._logger.messages)

    def test_execute_unknown_sound_returns_error(self, fake_node):
        with patch("rob_box_mcp_tools.tools.sound.Path.exists", return_value=False):
            tool = PlaySoundTool(fake_node)

        result = tool.execute(sound="nonexistent_sound")

        assert result.success is False
        assert "Неизвестный звук" in result.error
        assert "Доступные" in result.message

    def test_execute_success_publishes(self, fake_node):
        with patch("rob_box_mcp_tools.tools.sound.Path.exists", return_value=False):
            tool = PlaySoundTool(fake_node)

        result = tool.execute(sound="robot_happy")

        assert result.success is True
        assert result.data["sound"] == "robot_happy"
        assert result.data["duration"] == 1.5  # fallback duration по умолчанию
        pub = fake_node.publishers["/voice/sound/trigger"]
        assert pub.publish.called
        msg = pub.publish.call_args[0][0]
        assert msg.data == "robot_happy"

    def test_execute_success_with_catalog_duration(self, fake_node, tmp_path):
        catalog_path = tmp_path / "sound_catalog.json"
        catalog_path.write_text(json.dumps(_CATALOG_JSON), encoding="utf-8")

        class _FakePath:
            def __init__(self, p):
                self._p = p

            def exists(self):
                return True

            def __str__(self):
                return str(self._p)

            def __fspath__(self):
                return str(catalog_path)

        with patch("rob_box_mcp_tools.tools.sound.Path", _FakePath):
            tool = PlaySoundTool(fake_node)

        result = tool.execute(sound="robot_happy")

        assert result.success is True
        assert result.data["duration"] == 2.5  # из каталога
        assert "2.5" in result.message


# ---------------------------------------------------------------------------
# GetSoundInfoTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestGetSoundInfoTool:
    def test_tool_metadata(self, fake_node):
        tool = GetSoundInfoTool(fake_node)
        assert tool.name == "get_sound_info"
        assert tool.execution_type.value == "fast"
        assert [p.name for p in tool.parameters] == ["sound_name", "category"]

    def test_execute_catalog_unavailable(self, fake_node):
        tool = GetSoundInfoTool(fake_node)
        tool._load_catalog = Mock(return_value=None)

        result = tool.execute()

        assert result.success is False
        assert "Не удалось загрузить каталог" in result.error

    def test_execute_all_sounds(self, fake_node):
        tool = GetSoundInfoTool(fake_node)
        tool._load_catalog = Mock(return_value=_CATALOG_JSON)

        result = tool.execute()

        assert result.success is True
        assert result.data["total_sounds"] == 3
        assert result.data["categories"]["base"] == 2
        assert result.data["categories"]["ui"] == 1
        assert "Всего доступно 3 звуков" in result.message

    def test_execute_sound_by_trigger(self, fake_node):
        tool = GetSoundInfoTool(fake_node)
        tool._load_catalog = Mock(return_value=_CATALOG_JSON)

        result = tool.execute(sound_name="robot_happy")

        assert result.success is True
        assert result.data["trigger"] == "robot_happy"
        assert result.data["category"] == "base"
        assert "Радостный звук" in result.message

    def test_execute_sound_by_filename(self, fake_node):
        tool = GetSoundInfoTool(fake_node)
        tool._load_catalog = Mock(return_value=_CATALOG_JSON)

        result = tool.execute(sound_name="robot_error")

        assert result.success is True
        assert result.data["trigger"] == "robot_error"

    def test_execute_sound_not_found(self, fake_node):
        tool = GetSoundInfoTool(fake_node)
        tool._load_catalog = Mock(return_value=_CATALOG_JSON)

        result = tool.execute(sound_name="nonexistent")

        assert result.success is False
        assert "не найден" in result.error
        assert "списка всех звуков" in result.message

    def test_execute_filter_by_category(self, fake_node):
        tool = GetSoundInfoTool(fake_node)
        tool._load_catalog = Mock(return_value=_CATALOG_JSON)

        result = tool.execute(category="base")

        assert result.success is True
        assert result.data["category"] == "base"
        assert set(result.data["sounds"]) == {"robot_happy", "robot_error"}
        assert "2" in result.message

    def test_execute_filter_by_empty_category(self, fake_node):
        tool = GetSoundInfoTool(fake_node)
        tool._load_catalog = Mock(return_value=_CATALOG_JSON)

        result = tool.execute(category="robot")

        assert result.success is True
        assert result.data["sounds"] == []
        assert "не найдены" in result.message

    def test_load_catalog_fallback_uses_available_sounds(self, fake_node):
        """Fallback-каталог строится из PlaySoundTool.AVAILABLE_SOUNDS (issue TASK-037)."""
        with patch("rob_box_mcp_tools.tools.sound.Path.exists", return_value=False):
            tool = GetSoundInfoTool(fake_node)

        catalog = tool._load_catalog()
        assert catalog is not None
        assert "sounds" in catalog
        triggers = [v["trigger"] for v in catalog["sounds"].values()]
        assert "robot_affirm" in triggers
        assert "ui_button" in triggers
