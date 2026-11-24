"""test_dialogue_register_speaker.py — Unit-тесты RegisterSpeakerTool.

Issue #1101 — без noise-name guard LLM передавала "Зовут" вместо реального
имени (на фразе «робот меня зовут Денис говорю»), и /data/speakers.db
замусоривался записями ``name='Зовут'``. Здесь проверяем, что:

1. ``name=None``/``""`` → отказ валидации ``name_required`` (#5235, не успех).
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
        def __init__(self, success, data=None, message=None, error=None):
            self.success = bool(success)
            self.data = data if data is not None else {}
            self.message = message
            self.error = error

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

    def shared_publisher(node, msg_type, topic, qos):
        # Issue #3108: dialogue.py берёт часть publisher'ов через
        # base.shared_publisher. Кэш здесь не нужен — сквозной вызов.
        return node.create_publisher(msg_type, topic, qos)

    base_module.MCPToolResult = MCPToolResult
    base_module.MCPTool = MCPTool
    base_module.MCPToolParameter = MCPToolParameter
    base_module.shared_publisher = shared_publisher
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
    pkg_root = sys.modules.setdefault(
        "rob_box_mcp_tools", types.ModuleType("rob_box_mcp_tools")
    )
    # Issue #2932 — dialogue.py делает `from ..voice_request import
    # outcome_phrase` (и `from ..animations import ...`), т.е. относительные
    # импорты СОСЕДНИХ модулей пакета, а не только ``base``. Без __path__
    # на стаб-пакете относительный импорт падает с "'rob_box_mcp_tools' is
    # not a package" — сами модули (voice_state.py/animations.py) не тянут
    # rclpy, поэтому безопасно резолвить их из РЕАЛЬНОГО каталога пакета.
    pkg_root.__path__ = [str(_PKG_ROOT)]
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


def _is_polluted_key(name: str) -> bool:
    return (
        name == "rob_box_mcp_tools"
        or name.startswith("rob_box_mcp_tools.")
        or name in ("std_msgs", "std_msgs.msg")
    )


def _load_dialogue_restoring_sys_modules():
    """Загрузить dialogue.py и вернуть ``sys.modules`` в исходное состояние.

    Стабы из ``_load_dialogue_isolated`` (урезанный ``rob_box_mcp_tools.base``
    без ``ToolExecutionType``, пустой ``rob_box_mcp_tools.tools``,
    ``std_msgs``) нужны только на время exec_module. Если оставить их в
    ``sys.modules``, файлы, собираемые ПОСЛЕ этого, импортируют стаб вместо
    настоящего модуля: при прогоне всего ``test/`` одним pytest
    test_get_current_time.py падал на коллекции с
    ``cannot import name 'ToolExecutionType' from 'rob_box_mcp_tools.base'``.
    """
    saved = {k: v for k, v in sys.modules.items() if _is_polluted_key(k)}
    real_pkg = saved.get("rob_box_mcp_tools")
    saved_path = getattr(real_pkg, "__path__", None)
    try:
        return _load_dialogue_isolated()
    finally:
        for name in [k for k in sys.modules if _is_polluted_key(k)]:
            del sys.modules[name]
        sys.modules.update(saved)
        if real_pkg is not None and saved_path is not None:
            real_pkg.__path__ = saved_path


_dialogue = _load_dialogue_restoring_sys_modules()
RegisterSpeakerTool = _dialogue.RegisterSpeakerTool
#: Фраза хода (фолбек ``_current_turn_utterance_id``): без неё запрос не уходит (#3793, no_utterance_context).
_UTT = "a1b2c3d4e5f6"


@pytest.fixture(autouse=True)
def _short_register_deadline(monkeypatch):
    """#3793: тул ждёт SpeakerResult владельца до дедлайна класса; здесь владельца нет — исход timeout сразу."""
    monkeypatch.setattr(_dialogue, "register_deadline_s", lambda: 0.01)


@pytest.fixture(autouse=True)
def _ensure_std_msgs_stub(monkeypatch):
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
    # monkeypatch, а не прямое присваивание: после теста вернуть то, что
    # было, иначе stub утекает в файлы, которые идут следом.
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
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
    # Issue #2932 — MagicMock авто-создаёт ЛЮБОЙ несуществующий атрибут как
    # новый MagicMock (а не бросает AttributeError), поэтому
    # ``getattr(self.node, "_current_turn_utterance_id", None)`` в
    # RegisterSpeakerTool.execute() (issue #2842 фолбек) получал бы
    # MagicMock вместо None и падал на ``json.dumps`` в publish-пейлоаде.
    # Явно фиксируем отсутствие атрибута — как на реальном mcp_server node.
    node._current_turn_utterance_id = _UTT
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
# 1. name=None / пустое имя → отказ валидации (#5235)
# Раньше success=True + ask_required без status: v2 писал событие done,
# и фраза кода звучала «Запомнил тебя, None.»
# ---------------------------------------------------------------------------


def test_name_none_is_validation_error(tool: RegisterSpeakerTool, mock_node: MagicMock) -> None:
    result = tool.execute(name=None)
    assert result.success is False
    assert result.data["status"] == "rejected"
    assert result.data["reason"] == "name_required"
    assert result.data["error"] == "name_required"
    assert "name" not in result.data
    assert "имени нет ни в аргументах, ни в реплике" in (result.error or "")
    mock_node.register_pub.publish.assert_not_called()
    mock_node.rename_pub.publish.assert_not_called()


def test_empty_name_is_validation_error(tool: RegisterSpeakerTool, mock_node: MagicMock) -> None:
    result = tool.execute(name="   ")
    assert result.success is False
    assert result.data["status"] == "rejected"
    assert result.data["reason"] == "name_required"
    mock_node.register_pub.publish.assert_not_called()


def test_missing_name_without_self_intro_stays_name_required(
    tool: RegisterSpeakerTool, mock_node: MagicMock
) -> None:
    """#5236 не откатывается: нет name и нет имени в реплике — отказ name_required."""
    result = tool.execute(name=None, self_intro_name=None, known_speaker_name="Нина")
    assert result.success is False
    assert result.data["status"] == "rejected"
    assert result.data["reason"] == "name_required"
    assert "имени нет ни в аргументах, ни в реплике" in (result.error or "")
    mock_node.register_pub.publish.assert_not_called()
    mock_node.rename_pub.publish.assert_not_called()


def test_blank_self_intro_does_not_substitute(tool: RegisterSpeakerTool, mock_node: MagicMock) -> None:
    """#5238: одни пробелы в self_intro_name — имени в реплике нет, подстановки нет."""
    result = tool.execute(name="  ", self_intro_name="   ")
    assert result.success is False
    assert result.data["reason"] == "name_required"
    mock_node.register_pub.publish.assert_not_called()


# ---------------------------------------------------------------------------
# 1a. Issue #1101 (live 11.08) — литералы "null"/"None" из OpenAI tool-call.
# LLM иногда сериализует JSON null как строку "null"; без guard это
# превращается в запись ``name='Null'`` в /data/speakers.db.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("literal", ["null", "None", "NULL", "Null", " none "])
def test_null_literal_is_validation_error(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
    literal: str,
) -> None:
    result = tool.execute(name=literal)
    assert result.success is False
    assert result.data["status"] == "rejected"
    assert result.data["reason"] == "name_required"
    assert result.data["error"] == "name_required"
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
    assert result.data["status"] == "timeout"  # #3793: владелец в тесте молчит
    assert result.data["name"] == "Денис"

    # Проверяем, что в /voice/speaker/register ушёл корректный JSON.
    pub = mock_node.register_pub
    assert pub.publish.called
    call_args = pub.publish.call_args
    sent_msg = call_args[0][0]
    payload = json.loads(sent_msg.data)
    # Issue #2829 (ADR-0131 PR-2) — payload несёт utterance_id (None в
    # тесте: скрытый аргумент не передан, а node-фолбек тоже None).
    assert payload == {"name": "Денис", "utterance_id": _UTT}


def test_lowercase_name_is_capitalized(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    """«денис» → «Денис» (capitalize), без потери кириллицы."""
    result = tool.execute(name="денис")
    assert result.data["status"] == "timeout"  # #3793: владелец в тесте молчит
    assert result.data["name"] == "Денис"

    sent_msg = mock_node.register_pub.publish.call_args[0][0]
    assert json.loads(sent_msg.data) == {"name": "Денис", "utterance_id": _UTT}


def test_name_with_whitespace_is_trimmed(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    result = tool.execute(name="  Антон  ")
    assert result.data["status"] == "timeout"  # #3793: владелец в тесте молчит
    assert result.data["name"] == "Антон"

    sent_msg = mock_node.register_pub.publish.call_args[0][0]
    assert json.loads(sent_msg.data) == {"name": "Антон", "utterance_id": _UTT}


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
    assert result.data["status"] == "timeout"  # #3793: владелец в тесте молчит

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
    assert json.loads(register_msg.data) == {"name": "Денис", "utterance_id": _UTT}


def test_rename_without_new_name_does_not_publish(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    """#5235: old_name задан, name пустой — это не переименование.

    Раньше уходил no-op rename (new_name = old_name) и success=True. Пустое
    новое имя ничего не публикует: ни rename, ни register.
    """
    result = tool.execute(name=None, old_name="Эйджик")
    assert result.success is False
    assert result.data["status"] == "rejected"
    assert result.data["reason"] == "name_required"
    assert result.data["old_name"] == "Эйджик"

    mock_node.rename_pub.publish.assert_not_called()
    mock_node.register_pub.publish.assert_not_called()


def test_rename_ignores_noise_old_name(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
) -> None:
    """old_name='null'/'None' — не шлём rename (guard на литералы)."""
    result = tool.execute(name="Денис", old_name="null")
    assert result.data["status"] == "timeout"  # #3793: владелец в тесте молчит
    assert result.data["name"] == "Денис"
    # rename не публиковался (old_name='null' отброшен).
    mock_node.rename_pub.publish.assert_not_called()
    # register как обычно.
    assert mock_node.register_pub.publish.called


# ---------------------------------------------------------------------------
# 6. Issue #2932 — заглушечные/служебные имена ("unknown", "гость", ...)
# LLM вызывала register_speaker(name="unknown") вместо того, чтобы спросить
# настоящее имя — тул публиковал регистрацию под именем "Unknown"
# (см. E2E акт2 run 35943180077, robot develop 06ac0d2).
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "placeholder",
    [
        "unknown",
        "Unknown",
        "UNKNOWN",
        "неизвестный",
        "Неизвестный",
        "неизвестная",
        "незнакомец",
        "Незнакомец",
        "незнакомка",
        "гость",
        "Гость",
        "user",
        "User",
        "пользователь",
        "Пользователь",
        "speaker",
        "Speaker",
        "name",
        "Name",
        "-",
        "?",
    ],
)
def test_placeholder_names_are_rejected(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
    placeholder: str,
) -> None:
    """Служебные/заглушечные имена (issue #2932) не должны публиковаться —
    ни "-", ни "?" (эти два короче 2 символов и отсеиваются веткой
    name_too_short, но публикации быть не должно в любом случае)."""
    result = tool.execute(name=placeholder)
    assert result.success is False, f"{placeholder!r} должен быть отклонён"
    assert result.data["error"] in {"noise_name", "name_too_short"}
    mock_node.register_pub.publish.assert_not_called()


@pytest.mark.parametrize(
    "real_name",
    [
        "Юзеф",       # содержит "юз" (user) как префикс — не должен отсеиваться
        "Гостомысл",  # содержит "гост" (гость) как префикс — не должен отсеиваться
        "Спикеров",   # содержит "спикер" (speaker) как префикс
        "Насть",      # содержит "я" не относится, но проверим общую целостность
    ],
)
def test_names_containing_noise_prefix_are_published(
    tool: RegisterSpeakerTool,
    mock_node: MagicMock,
    real_name: str,
) -> None:
    """Фильтр сравнивает ЦЕЛОЕ имя, а не префикс/substring — реальные имена,
    начинающиеся с заглушечного слова, проходят и публикуются."""
    result = tool.execute(name=real_name)
    assert result.data["status"] == "timeout", f"{real_name!r} не должен быть отклонён"
    assert result.data["name"] == real_name.strip().capitalize()
    assert mock_node.register_pub.publish.called


# ---------------------------------------------------------------------------
# 7. Issue #5238 — имя из реплики, когда модель не передала name.
# self_intro_name — решение кода (speaker_adapter.named_self), не догадка LLM.
# Подстановка до гейтов: чужой old_name всё равно ловит not_own_profile.
# ---------------------------------------------------------------------------


def test_name_from_self_intro_is_published(tool: RegisterSpeakerTool, mock_node: MagicMock) -> None:
    """#5238, акт 2c: register_speaker({}) + self_intro_name='тамара' → публикация «Тамара».

    profile_name('тамара') == 'Тамара': в профиль уходит именительный с заглавной,
    не сырой аргумент хода. Исход — не name_required (владелец в тесте молчит → timeout).
    """
    from rob_box_dialog.words import profile_name

    said = "тамара"
    assert profile_name(said) == "Тамара"

    result = tool.execute(name=None, self_intro_name=said, known_speaker_name="Нина")
    assert result.data.get("reason") != "name_required"
    assert result.data["status"] != "rejected"
    assert result.data["name"] == "Тамара"

    assert mock_node.register_pub.publish.called
    register_msg = mock_node.register_pub.publish.call_args[0][0]
    assert json.loads(register_msg.data) == {"name": "Тамара", "utterance_id": _UTT}
    mock_node.rename_pub.publish.assert_not_called()


def test_foreign_old_name_still_refused_when_name_comes_from_self_intro(
    tool: RegisterSpeakerTool, mock_node: MagicMock
) -> None:
    """#5238: подстановка до гейта. old_name='Саша' при голосе 'Нина' — not_own_profile, без публикации."""
    result = tool.execute(
        name=None, old_name="Саша", self_intro_name="тамара", known_speaker_name="Нина"
    )
    assert result.success is False
    assert result.data["status"] == "rejected"
    assert result.data["reason"] == "not_own_profile"
    assert result.data["error"] == "not_own_profile"
    mock_node.register_pub.publish.assert_not_called()
    mock_node.rename_pub.publish.assert_not_called()


def test_explicit_name_is_not_replaced_by_self_intro(
    tool: RegisterSpeakerTool, mock_node: MagicMock
) -> None:
    """#5238: name передан — подстановки нет, старый гейт not_self_introduction."""
    result = tool.execute(name="Игорь", self_intro_name="тамара", known_speaker_name="Нина")
    assert result.success is False
    assert result.data["status"] == "rejected"
    assert result.data["reason"] == "not_self_introduction"
    assert result.data["received"] == "Игорь"
    assert result.data.get("name") != "Тамара"
    mock_node.register_pub.publish.assert_not_called()
    mock_node.rename_pub.publish.assert_not_called()
