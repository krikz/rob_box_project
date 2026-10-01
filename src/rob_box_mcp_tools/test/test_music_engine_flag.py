"""Флаг ``music_engine`` и проводка владельца плеера v2 в ``mcp_server`` (ADR-0149 PR-4b, #3312).

При v1 — ничего нового (нет ``/voice/music/event``, снимок пишет старый путь); при v2 —
``PlayerOwner`` единственный писатель снимка, ``/fail`` идёт к нему, yaml в обеих копиях
совпадает и объявлен нодой с теми же типами.
"""

import json
import re
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from .test_mcp_server import _FakeLogger, _FakePublisher, _load_mcp_server_module

pytestmark = pytest.mark.unit

REPO = Path(__file__).resolve().parents[3]
YAMLS = [REPO / "src/rob_box_voice/config/mcp_server.yaml",
         REPO / "docker/vision/config/voice_assistant/mcp_server.yaml"]


class _Node:
    def __init__(self, engine):
        self.params = {"music_engine": engine, "music_v2_clock_latency": 0.5}
        self.music_state_pub = _FakePublisher()
        self.created = []
        self._logger = _FakeLogger()
        self.registry = SimpleNamespace(tools=[])
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


def test_v1_attaches_nothing(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    node, manager = _Node("v1"), _manager()
    assert module._attach_player_owner_v2(node, manager) is None
    assert node.created == [] and node.music_state_pub.published == []
    assert not hasattr(manager, "osc_fail_listener")
    assert node.registry.tools == []  # dj_set при v1 не регистрируется


def test_v2_owner_is_the_only_state_writer(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    node, manager = _Node("v2"), _manager()
    state_pub = node.music_state_pub
    owner = module._attach_player_owner_v2(node, manager)
    assert owner is not None and node.created == ["/voice/music/event"]
    assert manager.osc_fail_listener == owner.on_server_fail
    assert [t.name for t in node.registry.tools] == ["dj_set"]  # PR-5: сет v2 поверх владельца
    first = json.loads(state_pub.published[0])
    assert first["state"] == "idle" and first["dj"] == {"enabled": False}
    assert node.music_state_pub is None  # у старого пути публикатора снимка больше нет
    node._music_manager = manager
    module.MCPServer.publish_music_state(node)  # старый путь при v2 молчит
    assert len(state_pub.published) == 1


def test_unknown_engine_stays_v1_loudly(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    node = _Node("v3")
    assert module._attach_player_owner_v2(node, _manager()) is None
    assert any("v3" in m for m in node._logger.error_messages)


def test_yaml_copies_match_and_are_declared_with_the_same_types():
    docs = [yaml.safe_load(p.read_text(encoding="utf-8")) for p in YAMLS]
    assert docs[0] == docs[1]
    assert docs[0]["/**"]["ros__parameters"] == {"music_engine": "v1"}  # одно значение на две ноды
    assert docs[0]["mcp_server"]["ros__parameters"] == {"music_v2_clock_latency": 0.5}
    src = (REPO / "src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py").read_text(encoding="utf-8")
    assert re.search(r'declare_parameter\("music_engine", "v1"\)', src)
    assert re.search(r'declare_parameter\("music_v2_clock_latency", V2_CLOCK_LATENCY_S\)', src)
    dialogue = (REPO / "src/rob_box_voice/rob_box_voice/dialogue_node.py").read_text(encoding="utf-8")
    assert re.search(r'declare_parameter\("music_engine", "v1"\)', dialogue)


@pytest.mark.parametrize("launch", [
    "docker/vision/config/voice_assistant/voice_assistant_headless.launch.py",
    "src/rob_box_voice/launch/voice_assistant.launch.py",
])
def test_dialogue_node_reads_the_same_music_engine_file(launch):
    """PR-5: при v2 ``dialogue_node`` не заводит тик DJ — флаг должен доехать до него из того же файла."""
    text = (REPO / launch).read_text(encoding="utf-8")
    block = text[text.index("executable='dialogue_node'"):]
    block = block[:block.index("output=")]
    assert "mcp_server" in block
