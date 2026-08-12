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
    # Force-override (НЕ setdefault): test_dialogue_speak_text_batch.py
    # подменяет sys.modules['std_msgs'] на Mock() на уровне модуля. Если
    # он импортируется РАНЬШЕ нас, setdefault не перезапишет Mock —
    # String() внутри execute() вернёт общий MagicMock, rename/register
    # payload'ы затрут друг друга в одном объекте и тесты упадут
    # (observed: rename_pub получил {name: ...} вместо {old_name,new_name}).
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    spec = importlib.util.spec_from_file_location(
        "rob_box_mcp_tools.tools.dialogue", _DIALOGUE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_dialogue = _load_dialogue_isolated()
RegisterSpeakerTool = _dialogue.RegisterSpeakerTool


@pytest.fixture(autouse=True)
def _ensure_std_msgs_stub():
    """Восстановить stub std_msgs перед каждым тестом.

    test_dialogue_speak_text_batch.py перезаписывает на уровне модуля
    ``sys.modules['std_msgs'] = Mock()``. При коллекции он импортируется
    ПОСЛЕ нас и затирает наш stub; без восстановления ``String()`` внутри
    ``execute()`` вернёт общий MagicMock, rename/register payload'ы
    затрут друг друга и тесты переименования упадут.
    """
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class _String:
        def __init__(self):
            self.data = ""

    std_msgs_msg.String = _String
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    yield


@pytest.fixture
def mock_node() -> MagicMock:
    """Минимальный мок ROS2-ноды для RegisterSpeakerTool.

    Нужен только ``create_publisher``; результат подписки/сообщения не нужны.
    Возвращает РАЗНЫЕ publisher'ы для /voice/speaker/register и
    /voice/speaker/rename — тесты переименования проверяют, что rename-пакет
    уходит именно на rename-топик (issue #1101).
    """
    node = MagicMock()
    register_pub = MagicMock()
    rename_pub = MagicMock()
    node.create_publisher.side_effect = lambda msg_type, topic, depth: (
        rename_pub if topic == "/voice/speaker/rename" else register_pub
    )
    # По умолчанию тесты, не знающие про rename, смотрят на register-паблишер.
    node.register_pub = register_pub
    node.rename_pub = rename_pub
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
# 1a. Issue #1101 (live 11.08) — литералы "null"/"None" из OpenAI tool-call.
# LLM иногда сериализует JSON null как строку "null"; без guard это
# превращается в запись ``name='Null'`` в /data/speakers.db.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("literal", ["null", "None", "NULL", "Null", " none "])
def test_null_literal_treated_as_ask_user(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
    literal: str,
) -> None:
    result = tool.execute(name=literal)
    assert result.success is True
    assert result.data == {"ask_required": True, "name": None}
    # На литералах публикации быть не должно.
    mock_node.register_pub.publish.assert_not_called()


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
    mock_node.register_pub.publish.assert_not_called()


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
    pub = mock_node.register_pub
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

    sent_msg = mock_node.register_pub.publish.call_args[0][0]
    assert json.loads(sent_msg.data) == {"name": "Денис"}


def test_name_with_whitespace_is_trimmed(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    result = tool.execute(name="  Антон  ")
    assert result.success is True
    assert result.data["registered_name"] == "Антон"

    sent_msg = mock_node.register_pub.publish.call_args[0][0]
    assert json.loads(sent_msg.data) == {"name": "Антон"}


# ---------------------------------------------------------------------------
# 5. Rename path (issue #1101) — «я не X, я Y» → old_name/new_name
# ---------------------------------------------------------------------------


def test_rename_publishes_to_rename_topic_not_register(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    """Rename-пакет ({old_name, new_name}) должен уходить на
    /voice/speaker/rename, где speaker_id_node._on_rename_request его
    обработает. Раньше публиковалось в /voice/speaker/register —
    register-хендлер читает только {"name": ...} и молча игнорировал
    rename (bug #1101 live 11.08)."""
    result = tool.execute(name="Денис", old_name="Эйджик")
    assert result.success is True

    # rename ушёл на rename-топик (первый вызов rename_pub).
    assert mock_node.rename_pub.publish.called
    rename_msg = mock_node.rename_pub.publish.call_args[0][0]
    assert json.loads(rename_msg.data) == {
        "old_name": "Эйджик",
        "new_name": "Денис",
    }

    # новый name зарегистрирован на register-топик (НЕ на rename).
    assert mock_node.register_pub.publish.called
    register_msg = mock_node.register_pub.publish.call_args[0][0]
    assert json.loads(register_msg.data) == {"name": "Денис"}


def test_rename_without_new_name_only_renames(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    """old_name задан, name пустой — переименование всё равно уходит
    (new_name = old_name — no-op rename), ответ сообщает «спроси»."""
    result = tool.execute(name=None, old_name="Эйджик")
    assert result.success is True
    assert result.data["renamed"] is True

    assert mock_node.rename_pub.publish.called
    rename_msg = mock_node.rename_pub.publish.call_args[0][0]
    assert json.loads(rename_msg.data) == {
        "old_name": "Эйджик",
        "new_name": "Эйджик",
    }
    # Без нового имени регистрация НЕ публикуется.
    mock_node.register_pub.publish.assert_not_called()


def test_rename_ignores_noise_old_name(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    """old_name='null'/'None' — не шлём rename (guard на литералы)."""
    result = tool.execute(name="Денис", old_name="null")
    assert result.success is True
    assert result.data["registered_name"] == "Денис"
    # rename не публиковался (old_name='null' отброшен).
    mock_node.rename_pub.publish.assert_not_called()
    # register как обычно.
    assert mock_node.register_pub.publish.called
