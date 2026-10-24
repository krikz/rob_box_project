"""Флаг ``dialog_engine`` (ADR-0150 §11, PR-8 #3702): значения, чтение yaml, две копии, выбор нод launch-ом.

Launch-файлы грузятся с подменой ``launch``/``launch_ros`` (записывающие заглушки): тест видит ровно тот список нод,
который launch отдаёт ROS, и их порядок. При ``v1`` он обязан совпасть со списком до флага.
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest
import yaml

from rob_box_core import dialog_engine as de

REPO = Path(__file__).resolve().parents[3]
CONFIG_COPIES = (
    REPO / "docker" / "sound" / "config" / "voice_assistant" / de.FILE_NAME,
    REPO / "src" / "rob_box_voice" / "config" / de.FILE_NAME,
)
LAUNCHES = {
    "headless": REPO / "docker" / "sound" / "config" / "voice_assistant" / "voice_assistant_headless.launch.py",
    "src": REPO / "src" / "rob_box_voice" / "launch" / "voice_assistant.launch.py",
}
#: Ноды launch-файлов до флага (develop @ 881374044), по порядку.
NODES_BEFORE = {
    "headless": ["audio_node", "led_node", "dialogue_node", "tts_node", "stt_node", "sound_node", "command_node",
                 "speaker_id_node", "mcp_server"],
    "src": ["audio_node", "led_node", "dialogue_node", "tts_node", "stt_node", "sound_node", "command_node",
            "speaker_id_node", "mcp_server"],
}


def test_engine_accepts_only_known_values():
    assert de.engine("v1") == "v1"
    assert de.engine(" V2 ") == "v2"
    for bad in ("", None, "v3", "true"):
        with pytest.raises(ValueError):
            de.engine(bad)


def test_both_config_copies_are_identical_and_default_v2():
    texts = [p.read_text(encoding="utf-8") for p in CONFIG_COPIES]
    assert texts[0] == texts[1], "docker/sound и src/rob_box_voice: dialog_engine.yaml разошлись"
    data = yaml.safe_load(texts[0])
    assert data == {"/**": {"ros__parameters": {"dialog_engine": de.DEFAULT}}}
    assert de.from_yaml(CONFIG_COPIES[0]) == "v2"


def test_from_yaml_missing_file_is_default(tmp_path):
    assert de.from_yaml(tmp_path / "nope.yaml") == de.DEFAULT


def test_from_yaml_typo_is_loud(tmp_path):
    bad = tmp_path / de.FILE_NAME
    bad.write_text('/**:\n  ros__parameters:\n    dialog_engine: "v3"\n', encoding="utf-8")
    with pytest.raises(ValueError):
        de.from_yaml(bad)
    bad.write_text('/**:\n  ros__parameters:\n    other: 1\n', encoding="utf-8")
    with pytest.raises(ValueError):
        de.from_yaml(bad)


def test_declare_reads_node_parameter():
    class Node:
        def __init__(self, value):
            self.value = value
            self.declared = {}

        def declare_parameter(self, name, default):
            self.declared[name] = default

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self.value if self.value is not None else self.declared[name])

    assert de.declare(Node(None)) == "v2"
    assert de.declare(Node("v2")) == "v2"
    with pytest.raises(ValueError):
        de.declare(Node("v9"))


# ── launch ──────────────────────────────────────────────────────────────


class _Sub:
    def __init__(self, *parts):
        self.parts = parts


class _LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class _Node:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.name = kwargs.get("name")


class _Opaque:
    def __init__(self, function, args=()):
        self.function, self.args = function, list(args)


def _fake_launch(monkeypatch):
    mods = {name: types.ModuleType(name) for name in (
        "launch", "launch.actions", "launch.substitutions", "launch_ros", "launch_ros.actions",
        "launch_ros.substitutions")}
    mods["launch"].LaunchDescription = lambda entities: list(entities)
    mods["launch.actions"].DeclareLaunchArgument = lambda *a, **k: ("arg", a[0] if a else k.get("name"))
    mods["launch.actions"].ExecuteProcess = lambda **k: _Node(name=k.get("name", "mcp_server"), **k)
    mods["launch.actions"].OpaqueFunction = _Opaque
    mods["launch.substitutions"].LaunchConfiguration = _LaunchConfiguration
    mods["launch.substitutions"].PathJoinSubstitution = lambda parts: _Sub(*parts)
    mods["launch_ros.actions"].Node = _Node
    mods["launch_ros.substitutions"].FindPackageShare = lambda pkg: _Sub(pkg)
    for name, mod in mods.items():
        monkeypatch.setitem(sys.modules, name, mod)


def _nodes(monkeypatch, which, config_dir):
    _fake_launch(monkeypatch)
    spec = importlib.util.spec_from_file_location(f"launch_under_test_{which}", LAUNCHES[which])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    names, tts = [], None
    for entity in module.generate_launch_description():
        if isinstance(entity, _Opaque):
            for node in entity.function({"config_dir": str(config_dir)}, *entity.args):
                names.append(node.name)
                tts = node if node.name == "tts_node" else tts
        elif isinstance(entity, _Node):
            names.append(entity.name)
    return names, tts


def _config(tmp_path, value):
    (tmp_path / de.FILE_NAME).write_text(f'/**:\n  ros__parameters:\n    dialog_engine: "{value}"\n', encoding="utf-8")
    return tmp_path


@pytest.mark.parametrize("which", sorted(LAUNCHES))
def test_launch_v1_keeps_node_list_and_order(monkeypatch, tmp_path, which):
    names, tts = _nodes(monkeypatch, which, _config(tmp_path, "v1"))
    assert names == NODES_BEFORE[which]
    assert {"dialog_engine": "v1"} in tts.kwargs["parameters"]


@pytest.mark.parametrize("which", sorted(LAUNCHES))
def test_launch_without_flag_file_is_default_v2(monkeypatch, tmp_path, which):
    names, _ = _nodes(monkeypatch, which, tmp_path)
    assert names == ["dialog_node" if n == "dialogue_node" else n for n in NODES_BEFORE[which]]


@pytest.mark.parametrize("which", sorted(LAUNCHES))
def test_launch_v2_starts_dialog_node_instead_of_dialogue_node(monkeypatch, tmp_path, which):
    names, tts = _nodes(monkeypatch, which, _config(tmp_path, "v2"))
    assert names == ["dialog_node" if n == "dialogue_node" else n for n in NODES_BEFORE[which]]
    assert {"dialog_engine": "v2"} in tts.kwargs["parameters"]


def test_launch_v2_dialog_node_gets_its_yaml_and_flag(monkeypatch, tmp_path):
    _fake_launch(monkeypatch)
    cfg = _config(tmp_path, "v2")
    tts_args = {"package": "rob_box_voice", "executable": "tts_node", "name": "tts_node", "namespace": "",
                "parameters": ["tts_node.yaml"]}
    dialog, tts = de.launch_nodes({"config_dir": str(cfg)}, _Node(name="dialogue_node"), tts_args)
    assert (dialog.kwargs["executable"], dialog.name) == ("dialog_node", "dialog_node")
    assert dialog.kwargs["parameters"] == [str(cfg / "dialog_node.yaml"), {"dialog_engine": "v2"}]


def test_flag_value_lives_only_in_dialog_engine_module():
    """Значения флага не копируются: launch-файлы и ноды берут их из ``rob_box_core.dialog_engine``."""
    users = [LAUNCHES["headless"], LAUNCHES["src"],
             REPO / "src" / "rob_box_voice" / "rob_box_voice" / "core" / "speech_compat.py",
             REPO / "src" / "rob_box_telegram" / "rob_box_telegram" / "telegram_node.py",
             REPO / "src" / "rob_box_supervisor" / "rob_box_supervisor" / "supervisor_node.py"]
    for path in users:
        text = path.read_text(encoding="utf-8")
        assert '"v2"' not in text and "'v2'" not in text and '"v1"' not in text and "'v1'" not in text, path
