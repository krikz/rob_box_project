"""Проводка владельца плеера v2 в ``mcp_server`` (ADR-0149 PR-4b, #3312).

С PR-13b старый путь удалён, с PR-15 снят и флаг выбора движка: ``PlayerOwner`` — единственный
писатель снимка, ``/fail`` идёт к нему; ``mcp_server.yaml`` в обеих копиях совпадает и объявлен нодой
с теми же типами.
"""

import json
import re
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from .test_mcp_server import _FakeLogger, _FakePublisher, _load_mcp_server_module

pytestmark = pytest.mark.unit


@pytest.fixture(autouse=True)
def _history_in_memory(monkeypatch):
    """``music_history`` сета (#3399) — не в общую ``/data`` хоста."""
    monkeypatch.setenv("VOICE_MEMORY_DB_PATH", ":memory:")


REPO = Path(__file__).resolve().parents[3]
CONFIG_DIRS = [REPO / "src/rob_box_voice/config", REPO / "docker/vision/config/voice_assistant"]


class _Node:
    def __init__(self):
        self.params = {"music_v2_clock_latency": 0.5, "music_v2_reasoner": True,
                       "music_v2_reasoner_deadline_s": 45.0, "music_v2_dj_lines": True, "music_v2_tracks_dir": ""}
        self.music_state_pub = _FakePublisher()
        self.created = []
        self._logger = _FakeLogger()
        self.registry = SimpleNamespace(tools=[], execute=lambda name, **kw: None)
        self.registry.register = self.registry.tools.append

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def get_logger(self):
        return self._logger

    def create_publisher(self, msg_type, topic, qos):
        self.created.append(topic)
        return _FakePublisher()


def _manager():
    return SimpleNamespace(_renardo_context={}, known_synth_names=lambda: frozenset(),
                           _send_osc_raw=lambda *a: None)


def test_owner_is_attached_unconditionally(monkeypatch):
    """PR-13b/PR-15: старый путь и флаг удалены — владелец плеера v2 единственный."""
    module = _load_mcp_server_module(monkeypatch)
    node, manager = _Node(), _manager()
    assert module._attach_player_owner_v2(node, manager) is not None
    assert [t.name for t in node.registry.tools] == ["dj_set", "request_music"]


def test_no_manager_attaches_nothing_loudly(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    node = _Node()
    assert module._attach_player_owner_v2(node, None) is None
    assert node.created == [] and node.registry.tools == []
    assert any("MusicManager" in m for m in node._logger.error_messages)


def test_v2_owner_is_the_only_state_writer(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    node, manager = _Node(), _manager()
    state_pub = node.music_state_pub
    owner = module._attach_player_owner_v2(node, manager)
    assert owner is not None and node.created == ["/voice/music/event"]
    assert manager.osc_fail_listener == owner.on_server_fail
    # PR-5: сет v2 поверх владельца; PR-6: одиночный club-трек v2
    assert [t.name for t in node.registry.tools] == ["dj_set", "request_music"]
    first = json.loads(state_pub.published[0])
    assert first["state"] == "idle" and first["dj"] == {"enabled": False}
    assert node.music_state_pub is None  # публикатор снимка — только у владельца
    assert not hasattr(module.MCPServer, "publish_music_state")  # старого писателя нет (PR-13)
    assert node._dj_set_tool is node.registry.tools[0]


def _load(name):
    docs = [yaml.safe_load((d / name).read_text(encoding="utf-8")) for d in CONFIG_DIRS]
    assert docs[0] == docs[1], f"копии {name} разошлись"
    return docs[0]


def test_yaml_copies_match_and_are_declared_with_the_same_types():
    # ADR-0004: один файл — одна секция; у mcp_server.yaml — только своя.
    assert _load("mcp_server.yaml") == {"mcp_server": {"ros__parameters": {
        "music_v2_clock_latency": 0.5, "music_v2_reasoner": True, "music_v2_reasoner_deadline_s": 45.0,
        "music_v2_dj_lines": True, "music_v2_tracks_dir": "/data/music_v2/tracks"}}}
    src = (REPO / "src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py").read_text(encoding="utf-8")
    assert re.search(r'declare_parameter\("music_v2_clock_latency", V2_CLOCK_LATENCY_S\)', src)
    assert re.search(r'declare_parameter\("music_v2_reasoner", True\)', src)
    assert re.search(r'declare_parameter\("music_v2_reasoner_deadline_s", V2_REASONER_DEADLINE_S\)', src)
    assert re.search(r'declare_parameter\("music_v2_dj_lines", True\)', src)  # В2 (06.10): реплика на переходе


def test_music_engine_flag_is_gone():
    """PR-15: флага выбора движка нет ни в нодах, ни в launch-файлах, ни в конфигах."""
    paths = [
        "src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py",
        "src/rob_box_voice/rob_box_voice/dialogue_node.py",
        "docker/vision/config/voice_assistant/voice_assistant_headless.launch.py",
        "src/rob_box_voice/launch/voice_assistant.launch.py",
    ]
    for rel in paths:
        assert "music_engine" not in (REPO / rel).read_text(encoding="utf-8"), rel
    for config_dir in CONFIG_DIRS:
        assert not (config_dir / "music_engine.yaml").exists(), config_dir
