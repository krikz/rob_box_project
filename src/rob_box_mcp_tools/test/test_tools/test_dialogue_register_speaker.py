"""test_dialogue_register_speaker.py — Unit-тесты RegisterSpeakerTool.

Issue #1101 — без noise-name guard LLM передавала "Зовут" вместо реального
имени (на фразе «робот меня зовут Денис говорю»), и /data/speakers.db
замусоривался записями ``name='Зовут'``. Здесь проверяем, что:

1. ``name=None``/``""`` → ask_required=true (попросить пользователя).
2. Слишком короткое имя → name_too_short.
3. **Шумовое имя** ("зовут", "имя", "меня", ...) → noise_name, без публикации.
4. Корректное кириллическое имя → публикуется в /voice/speaker/register.

Тест изолирован от ``rob_box_mcp_tools.tools.__init__`` (тот тянет
``rclpy`` через ``navigation.py`` и падает на Windows dev-машине).
Решение — собираем минимальный ``MCPTool`` base-класс через
``importlib.util`` и подсовываем его в ``sys.modules`` до загрузки
``dialogue.py``.
"""

from __future__ import annotations

import importlib.util
import json
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest


# ---------------------------------------------------------------------------
# Изоляция от rclpy: подсовываем заглушки для ``rob_box_mcp_tools.base``,
# ``rob_box_mcp_tools.tools.__init__`` и ``std_msgs`` до импорта dialogue.py.
# ---------------------------------------------------------------------------

_REPO_ROOT = Path(__file__).resolve().parents[2]  # test_tools/ -> test/ -> rob_box_mcp_tools/
_PKG_ROOT = _REPO_ROOT / "rob_box_mcp_tools"
_BASE_PATH = _PKG_ROOT / "base.py"
_DIALOGUE_PATH = _PKG_ROOT / "tools" / "dialogue.py"


def _install_mcp_base_stub() -> types.ModuleType:
    """Построить урезанный ``rob_box_mcp_tools.base`` без rclpy.

    В рабочем образе этот модуль импортирует ``rclpy`` транзитивно. Для
    чисто-логического теста RegisterSpeakerTool достаточно только:
      * ``MCPToolResult`` (dataclass с success/data/message)
      * базовый ``MCPTool``, который ``RegisterSpeakerTool`` наследует
        и чей ``__init__`` пишет ``self.node = node`` + ``self._logger``.
    """
    base_module = types.ModuleType("rob_box_mcp_tools.base")

    @dataclass_like
    class MCPToolResult:
        def __init__(self, success, data=None, message=None):
            self.success = bool(success)
            self.data = data if data is not None else {}
            self.message = message

    class MCPTool:
        def __init__(self, node):
            self.node = node
            self._logger = getattr(node, "get_logger", lambda: _DummyLogger())()

        def log_info(self, msg):
            self._logger.info(msg)

        def log_warning(self, msg):
            self._logger.warning(msg)

        def log_error(self, msg):
            self._logger.error(msg)

    class _DummyLogger:
        def info(self, msg): pass
        def warning(self, msg): pass
        def error(self, msg): pass
        def debug(self, msg): pass

    class MCPToolParameter:  # noqa: D401 — stub
        def __init__(self, name, type, description="", required=False, enum=None):
            self.name = name
            self.type = type
            self.description = description
            self.required = required
            self.enum = enum

    base_module.MCPToolResult = MCPToolResult
    base_module.MCPTool = MCPTool
    base_module.MCPToolParameter = MCPToolParameter
    return base_module


def dataclass_like(cls):
    """Тривиальный «декоратор-метка» — оставляем класс как есть.

    Используется только для читаемости: MCPToolResult ведёт себя как
    dataclass, но в стенде нам хватает обычного ``__init__``.
    """
    return cls


def _load_dialogue_isolated():
    """Импортировать ``dialogue.py`` БЕЗ запуска ``tools/__init__.py``.

    В рабочем образе ``rob_box_mcp_tools.tools.__init__`` тянет
    ``rclpy`` через ``navigation.py``. Подменяем его на пустой модуль,
    чтобы ``from .dialogue import *`` (наследие __init__) не выполнялся.
    Затем грузим ``dialogue.py`` напрямую через importlib.
    """
    sys.modules.setdefault("rob_box_mcp_tools", types.ModuleType("rob_box_mcp_tools"))
    pkg_tools = types.ModuleType("rob_box_mcp_tools.tools")
    pkg_tools.__path__ = [str(_REPO_ROOT / "rob_box_mcp_tools" / "tools")]
    sys.modules["rob_box_mcp_tools.tools"] = pkg_tools
    sys.modules["rob_box_mcp_tools.base"] = _install_mcp_base_stub()
    # std_msgs.msg.String импортируется внутри RegisterSpeakerTool.execute();
    # мок ниже подменяет его в момент вызова.
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class _String:
        def __init__(self):
            self.data = ""

    std_msgs_msg.String = _String
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    spec = importlib.util.spec_from_file_location(
        "rob_box_mcp_tools.tools.dialogue", _DIALOGUE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_dialogue = _load_dialogue_isolated()
RegisterSpeakerTool = _dialogue.RegisterSpeakerTool


@pytest.fixture
def mock_node() -> MagicMock:
    """Минимальный мок ROS2-ноды для RegisterSpeakerTool.

    Нужен только ``create_publisher``; результат подписки/сообщения не нужны.
    """
    node = MagicMock()
    pub = MagicMock()
    node.create_publisher.return_value = pub
    return node


@pytest.fixture
def tool(mock_node: MagicMock) -> RegisterSpeakerTool:
    return RegisterSpeakerTool(mock_node)


# ---------------------------------------------------------------------------
# 1. name=None / пустое имя → ask_required
# ---------------------------------------------------------------------------


def test_name_none_returns_ask_required(tool: RegisterSpeakerTool) -> None:
    result = tool.execute(name=None)
    assert result.success is True
    assert result.data == {"ask_required": True, "name": None}


def test_empty_name_returns_ask_required(tool: RegisterSpeakerTool) -> None:
    result = tool.execute(name="   ")
    assert result.success is True
    assert result.data["ask_required"] is True


# ---------------------------------------------------------------------------
# 2. Слишком короткое имя → name_too_short
# ---------------------------------------------------------------------------


def test_single_char_name_rejected(tool: RegisterSpeakerTool) -> None:
    result = tool.execute(name="Д")
    assert result.success is False
    assert result.data["error"] == "name_too_short"


# ---------------------------------------------------------------------------
# 3. Шумовые имена из фразы "зовут" / "имя" / "меня" → noise_name
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "noise",
    [
        "зовут",
        "Зовут",          # capitalized → всё равно в нижнем регистре сверяем
        "имя",
        "меня",
        "это",
        "зовут-это",
        "зовут меня",
        "моё",
        "мое",
        "моё имя",
        "мое имя",
        "имя мне",
        "имя моё",
        "имя мое",
    ],
)
def test_noise_names_are_rejected(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
    noise: str,
) -> None:
    """Без guard LLM передаёт «зовут» / «имя» вместо реального имени."""
    result = tool.execute(name=noise)
    assert result.success is False, f"{noise!r} должен быть отклонён"
    assert result.data["error"] == "noise_name"
    assert result.data["received"].lower() == noise.lower()
    # КРИТИЧНО: на шумовых именах публикации быть не должно.
    mock_node.create_publisher.return_value.publish.assert_not_called()


# ---------------------------------------------------------------------------
# 4. Корректное кириллическое имя → публикация в /voice/speaker/register
# ---------------------------------------------------------------------------


def test_valid_cyrillic_name_is_published(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    result = tool.execute(name="Денис")
    assert result.success is True
    assert result.data["registered_name"] == "Денис"

    # Проверяем, что в /voice/speaker/register ушёл корректный JSON.
    pub = mock_node.create_publisher.return_value
    assert pub.publish.called
    call_args = pub.publish.call_args
    sent_msg = call_args[0][0]
    payload = json.loads(sent_msg.data)
    assert payload == {"name": "Денис"}


def test_lowercase_name_is_capitalized(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    """«денис» → «Денис» (capitalize), без потери кириллицы."""
    result = tool.execute(name="денис")
    assert result.success is True
    assert result.data["registered_name"] == "Денис"

    sent_msg = mock_node.create_publisher.return_value.publish.call_args[0][0]
    assert json.loads(sent_msg.data) == {"name": "Денис"}


def test_name_with_whitespace_is_trimmed(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    result = tool.execute(name="  Антон  ")
    assert result.success is True
    assert result.data["registered_name"] == "Антон"

    sent_msg = mock_node.create_publisher.return_value.publish.call_args[0][0]
    assert json.loads(sent_msg.data) == {"name": "Антон"}
