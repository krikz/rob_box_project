"""test_issue_2781_voice_memory_e2e_isolation.py — E2E-изоляция voice_facts.

Issue #2781 — изоляция БД дикторов (#2750, починена в #2763/#2770)
накрывала ТОЛЬКО ``speakers.db``. Долгосрочная память (``voice_facts``,
записанная через MCP-тул ``memory_save``) писалась прямо в боевую
``/data/voice_memory.db`` без всякой изоляции: акт «Знакомство» ночного
марафона и регистрирует голоса (``register_speaker`` — изолировано), и
называет LLM факты о себе (``memory_save`` — НЕ было изолировано). Замер
на Vision Pi 22.09.2026 нашёл 59 из 116 фактов боевой базы,
упоминающих синтезированный каст марафона ("Саша не ест лук", "Борис
болеет за Спартак") вперемешку с фактами живых людей мастерской.

Механизм повторяет ``speaker_id_node.e2e_mode``
(``test_e2e_db_isolation.py`` в ``src/rob_box_voice/test/unit/node/``) один
в один: bool-параметр узла с валидирующим ``parameters_callback``, синхронно
переоткрывающим ``self.voice_memory`` на другом SQLite-файле. Владелец узла
здесь — ``mcp_server`` (единственный процесс, который исполняет
``memory_save``, см. ``tools/memory.py:MemorySaveTool``), а не
``speaker_id_node``.

Приём тестирования — ``object.__new__(MCPServer)`` + минимальный набор
полей, которые реально трогают ``parameters_callback``/``_apply_e2e_mode``
(тот же приём, что в ``test_e2e_db_isolation.py``). ``mcp_server.py``
загружается через тот же fake-dependency loader, что и ``test_mcp_server.py``
(``rclpy``/``std_msgs``/``rcl_interfaces`` — стабы; ``rob_box_voice`` —
НАСТОЯЩИЙ пакет, потому что тест обязан доказать, что факты реально
попадают в правильный SQLite-файл, а не просто дергают мок).

Run:
  python -m pytest src/rob_box_mcp_tools/test/test_issue_2781_voice_memory_e2e_isolation.py -q --no-cov
"""

from __future__ import annotations

import importlib.util
import sys
import threading
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest


def _load_real_voice_memory_class():
    """Load the real ``VoiceMemory`` class straight off its source file.

    Issue #2781 — ``test_mcp_server_speaker_result.py`` (collected in the
    same pytest session) permanently overwrites
    ``sys.modules['rob_box_voice.core.voice_memory']`` with a bare stub
    class (no ``monkeypatch``, no cleanup — see that file's
    ``_load_mcp_server_module``), so a plain
    ``from rob_box_voice.core.voice_memory import VoiceMemory`` here would
    silently resolve to whichever one collection order happened to import
    first. This test's entire point is proving facts land in the correct
    SQLite file, so it cannot risk running against a stub — load the
    module by path instead of by name, bypassing ``sys.modules`` entirely.
    ``voice_memory.py`` has no package-relative imports, so this is safe.
    """
    src = (
        Path(__file__).resolve().parents[3]
        / "src" / "rob_box_voice" / "rob_box_voice" / "core" / "voice_memory.py"
    )
    spec = importlib.util.spec_from_file_location("_real_voice_memory_2781", src)
    mod = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod.VoiceMemory


VoiceMemory = _load_real_voice_memory_class()


class _FakeParam:
    """Заглушка ``rclpy.parameter.Parameter`` — колбэку нужны только
    ``.name``/``.value`` (см. ``mcp_server.parameters_callback``:
    ``for param in params: if param.name == "e2e_mode": ...``)."""

    def __init__(self, name: str, value) -> None:
        self.name = name
        self.value = value


def _e2e_mode_param(enabled: bool) -> list[_FakeParam]:
    return [_FakeParam("e2e_mode", enabled)]


def _install_fake_mcp_server_dependencies(monkeypatch) -> None:
    """Минимальный набор стабов, достаточный чтобы ``mcp_server.py``
    импортировался без реального ROS2/rclpy окружения.

    Копия из ``test_mcp_server.py:_install_fake_mcp_server_dependencies``,
    урезанная до того, что реально нужно этому файлу (та же причина, по
    которой у ``test_mcp_server_speaker_result.py`` есть СВОЯ независимая
    копия — общий helper-модуль здесь не заводили, чтобы три теста файла не
    зависели друг от друга при правках).

    Намеренно НЕ стабит ``rob_box_voice.core.voice_memory`` — этот тест
    использует НАСТОЯЩИЙ ``VoiceMemory``, чтобы проверить, что факты
    реально попадают в правильный SQLite-файл.
    """
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_callback_groups = types.ModuleType("rclpy.callback_groups")
    rclpy_qos = types.ModuleType("rclpy.qos")
    rcl_interfaces = types.ModuleType("rcl_interfaces")
    rcl_interfaces_msg = types.ModuleType("rcl_interfaces.msg")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    registry_module = types.ModuleType("rob_box_mcp_tools.registry")
    tools_module = types.ModuleType("rob_box_mcp_tools.tools")
    waypoint_store_module = types.ModuleType("rob_box_mcp_tools.waypoint_store")
    mapping_state_module = types.ModuleType("rob_box_mcp_tools.mapping_state")

    class Node:
        pass

    class ReentrantCallbackGroup:
        pass

    class QoSProfile:
        def __init__(self, *args, **kwargs):
            pass

    class ReliabilityPolicy:
        RELIABLE = 1

    class HistoryPolicy:
        KEEP_LAST = 1

    class DurabilityPolicy:
        TRANSIENT_LOCAL = 1

    class SetParametersResult:
        def __init__(self, successful: bool = True, reason: str = ""):
            self.successful = successful
            self.reason = reason

    class String:
        def __init__(self):
            self.data = ""

    class MCPToolRegistry:
        pass

    class WaypointStore:
        pass

    class MappingState:
        pass

    def _tools_module_fallback(name):
        if name.startswith("__") and name.endswith("__"):
            raise AttributeError(name)

        class _Tool:
            def __init__(self, *args, **kwargs):
                self.name = name

        return _Tool

    tools_module.__getattr__ = _tools_module_fallback

    class MusicManager:
        def __init__(self, *args, **kwargs):
            pass

    class TrackLibrary:
        def __init__(self, *args, **kwargs):
            raise FileNotFoundError("missing 004_music_library.sql")

    tools_module.MusicManager = MusicManager
    tools_module.TrackLibrary = TrackLibrary

    rclpy_node.Node = Node
    rclpy_callback_groups.ReentrantCallbackGroup = ReentrantCallbackGroup
    rclpy_qos.QoSProfile = QoSProfile
    rclpy_qos.ReliabilityPolicy = ReliabilityPolicy
    rclpy_qos.HistoryPolicy = HistoryPolicy
    rclpy_qos.DurabilityPolicy = DurabilityPolicy
    rcl_interfaces_msg.SetParametersResult = SetParametersResult
    std_msgs_msg.String = String
    registry_module.MCPToolRegistry = MCPToolRegistry
    waypoint_store_module.WaypointStore = WaypointStore
    mapping_state_module.MappingState = MappingState

    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)
    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", rclpy_callback_groups)
    monkeypatch.setitem(sys.modules, "rclpy.qos", rclpy_qos)
    monkeypatch.setitem(sys.modules, "rcl_interfaces", rcl_interfaces)
    monkeypatch.setitem(sys.modules, "rcl_interfaces.msg", rcl_interfaces_msg)
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.registry", registry_module)
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.tools", tools_module)
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.waypoint_store", waypoint_store_module)
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.mapping_state", mapping_state_module)


def _load_mcp_server_module(monkeypatch):
    _install_fake_mcp_server_dependencies(monkeypatch)
    module_path = Path(__file__).resolve().parents[1] / "rob_box_mcp_tools" / "mcp_server.py"
    spec = importlib.util.spec_from_file_location("rob_box_mcp_tools.mcp_server", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    # Issue #2781 — cross-test-file pollution guard: ``mcp_server.py`` does
    # ``from rob_box_voice.core.voice_memory import VoiceMemory as
    # _VoiceMemory`` at module scope, which resolves through
    # ``sys.modules['rob_box_voice.core.voice_memory']``.
    # ``test_mcp_server_speaker_result.py`` permanently replaces that cache
    # entry with a bare stub class (no ``monkeypatch``, no cleanup) so it
    # can import ``mcp_server.py`` without a real ``rob_box_voice``. When
    # pytest collects both files in one session, this file's own
    # ``exec_module`` above can silently pick up that stub instead of the
    # real class — this test needs the real, SQLite-backed ``VoiceMemory``
    # to prove facts land in the right file, so pin it explicitly rather
    # than trusting file collection order.
    module._VoiceMemory = VoiceMemory
    return module


@pytest.fixture()
def node(monkeypatch, tmp_path):
    module = _load_mcp_server_module(monkeypatch)

    prod_path = str(tmp_path / "voice_memory.db")
    e2e_path = str(tmp_path / "voice_memory.e2e.db")

    instance = object.__new__(module.MCPServer)
    instance._voice_memory_prod_db_path = prod_path
    instance._voice_memory_e2e_db_path = e2e_path
    instance._voice_memory_e2e_mode_active = False
    instance._voice_memory_ollama_url = None
    instance._voice_memory_lock = threading.Lock()
    instance.voice_memory = VoiceMemory(db_path=prod_path)
    instance.get_logger = MagicMock(return_value=MagicMock())

    yield instance, module
    instance.voice_memory.close()


def test_enable_switches_off_prod_db_without_touching_it(node):
    """Боевая voice_memory.db не открывается на запись, пока e2e_mode включён."""
    instance, _module = node
    instance.voice_memory.save_fact("живой человек мастерской пьёт кофе без сахара")
    instance.voice_memory.close()

    # Переоткрываем узел на боевой (как это делает __init__/_init_voice_memory).
    instance.voice_memory = VoiceMemory(db_path=instance._voice_memory_prod_db_path)

    result = instance.parameters_callback(_e2e_mode_param(True))

    assert result.successful is True
    assert instance._voice_memory_e2e_mode_active is True
    assert instance.voice_memory.db_path == instance._voice_memory_e2e_db_path

    # Боевая БД физически не тронута: открываем её НАПРЯМУЮ.
    prod_check = VoiceMemory(db_path=instance._voice_memory_prod_db_path)
    try:
        facts = [f["fact"] for f in prod_check.get_facts()]
        assert any("кофе без сахара" in f for f in facts)
    finally:
        prod_check.close()


def test_marathon_facts_never_reach_prod_db(node):
    """Регрессия #2781: memory_save во время e2e_mode не должен попадать
    в боевую /data/voice_memory.db."""
    instance, _module = node
    instance.parameters_callback(_e2e_mode_param(True))

    instance.voice_memory.save_fact("Саша не ест лук, даже жареный")
    instance.voice_memory.save_fact("Борис — друг Саши, приходит раз в неделю с пиццей")

    prod_check = VoiceMemory(db_path=instance._voice_memory_prod_db_path)
    try:
        assert prod_check.get_facts() == [], (
            "факты каста ночного марафона попали в боевую voice_memory.db — "
            "изоляция сломана (issue #2781)"
        )
    finally:
        prod_check.close()

    e2e_check = VoiceMemory(db_path=instance._voice_memory_e2e_db_path)
    try:
        facts = {f["fact"] for f in e2e_check.get_facts()}
        assert "Саша не ест лук, даже жареный" in facts
        assert any("Борис" in f for f in facts)
    finally:
        e2e_check.close()


def test_disable_returns_to_prod_db(node):
    instance, _module = node
    instance.parameters_callback(_e2e_mode_param(True))
    instance.voice_memory.save_fact("факт только для e2e")

    result = instance.parameters_callback(_e2e_mode_param(False))

    assert result.successful is True
    assert instance._voice_memory_e2e_mode_active is False
    assert instance.voice_memory.db_path == instance._voice_memory_prod_db_path
    assert instance.voice_memory.get_facts() == []


def test_enable_wipes_leftover_e2e_file_from_previous_marathon(node):
    """Как и у speaker_id_node (issue #2750): узел сам гарантирует пустую
    e2e-базу при каждом включении — оператор не должен полагаться на
    дисциплину вызывающего кода."""
    instance, _module = node
    stale = VoiceMemory(db_path=instance._voice_memory_e2e_db_path)
    stale.save_fact("призрак с прошлого марафона")
    stale.close()

    instance.parameters_callback(_e2e_mode_param(True))

    assert instance.voice_memory.get_facts() == [], (
        "старый факт пережил включение e2e_mode — гарантия чистой базы не держит"
    )


def test_repeated_enable_same_value_does_not_wipe_mid_marathon(node):
    """Повторный ``set_parameters`` с ТЕМ ЖЕ значением (true поверх уже
    включённого true) не должен чистить e2e_db_path второй раз — иначе
    повторный вызов посреди акта обнулил бы уже накопленные факты."""
    instance, _module = node
    instance.parameters_callback(_e2e_mode_param(True))
    instance.voice_memory.save_fact("Саша не ест лук")
    memory_after_first = instance.voice_memory

    result = instance.parameters_callback(_e2e_mode_param(True))

    assert result.successful is True
    assert instance.voice_memory is memory_after_first, "повторный True пересоздал соединение — не no-op"
    facts = {f["fact"] for f in instance.voice_memory.get_facts()}
    assert facts == {"Саша не ест лук"}


def test_unrelated_param_change_does_not_touch_db(node):
    instance, _module = node
    memory_before = instance.voice_memory

    result = instance.parameters_callback([_FakeParam("tts_provider", "minimax")])

    assert result.successful is True
    assert instance.voice_memory is memory_before
    assert instance._voice_memory_e2e_mode_active is False


def test_no_prod_path_target_fails_fatally_instead_of_silently_passing(node):
    """Issue #2781 — если активен harness-адаптер
    (``MCP_USE_HARNESS_VOICE_MEMORY=1``) или ``rob_box_voice`` недоступен,
    ``_voice_memory_prod_db_path`` остаётся ``None`` (см.
    ``_init_voice_memory``). Переключение обязано ОТКАЗАТЬ явно
    (``successful=False``), а не молча продолжить писать в боевую БД —
    ровно то, чего требует харнесс от ``activate_e2e_speaker_db``-аналога."""
    instance, _module = node
    instance._voice_memory_prod_db_path = None

    result = instance.parameters_callback(_e2e_mode_param(True))

    assert result.successful is False
    assert instance._voice_memory_e2e_mode_active is False


def test_search_finds_marathon_fact_only_in_e2e_db_not_prod(node):
    """Issue #2793 — ``memory_search`` должен смотреть в ту же активную БД,
    что и ``memory_save``. До фикса #2793 ``VoiceMemory.search()`` читал
    только ``voice_turns`` и никогда ``voice_facts``, поэтому эта проверка
    падала независимо от изоляции; теперь ``search()`` находит факт сразу
    после ``save_fact`` в рамках ОДНОГО активного инстанса, и он не течёт
    в боевую БД, пока e2e_mode включён (та же гарантия, что у get_facts)."""
    instance, _module = node
    instance.parameters_callback(_e2e_mode_param(True))

    instance.voice_memory.save_fact("Борис любит зелёный чай без сахара")

    # Тот же активный инстанс сразу находит только что сохранённый факт.
    hits = instance.voice_memory.search("чай", limit=5)
    assert any(h["kind"] == "fact" and "чай" in h["content"] for h in hits)

    # В боевую БД факт не попал вовсе.
    prod_check = VoiceMemory(db_path=instance._voice_memory_prod_db_path)
    try:
        assert prod_check.search("чай", limit=5) == [], (
            "e2e-факт нашёлся в боевой БД поиском — изоляция search() сломана"
        )
    finally:
        prod_check.close()


def test_db_switch_failure_returns_unsuccessful_result(node, monkeypatch):
    """Провал переключения (диск недоступен и т. п.) обязан дойти до
    вызывающего ``ros2 param set`` как ``successful=False``."""
    instance, module = node

    def _boom(*args, **kwargs):
        raise OSError("simulated disk failure")

    monkeypatch.setattr(module, "_VoiceMemory", _boom)

    result = instance.parameters_callback(_e2e_mode_param(True))

    assert result.successful is False
    assert instance._voice_memory_e2e_mode_active is False, "провал не должен был поменять состояние"


def test_e2e_path_pointing_at_prod_never_wipes_prod(node):
    """Issue #2890 — e2e_db_path, указывающий на боевую voice_memory.db,
    не должен её стереть: включение отклоняется, факты живых людей на месте."""
    instance, _module = node
    instance.voice_memory.save_fact("живой человек мастерской пьёт кофе без сахара")
    instance._voice_memory_e2e_db_path = instance._voice_memory_prod_db_path

    result = instance.parameters_callback(_e2e_mode_param(True))

    assert result.successful is False
    assert instance._voice_memory_e2e_mode_active is False
    facts = [f["fact"] for f in instance.voice_memory.get_facts()]
    assert any("кофе без сахара" in f for f in facts), "боевая voice_memory.db стёрта"


def test_enable_logs_explicit_memory_wipe_line(node):
    """Issue #2890 — сброс e2e-памяти фактов перед актом виден в логе робота."""
    instance, _module = node

    instance.parameters_callback(_e2e_mode_param(True))

    logger = instance.get_logger.return_value
    lines = [str(c.args[0]) for c in logger.warning.call_args_list if c.args]
    assert any("e2e-память фактов сброшена" in line for line in lines), lines
