"""Ноды диалога в launch voice_assistant (ADR-0150 §11; флаг v1|v2 снят в #4354 A3, метка у dialog_node — в A4).

Launch-файлы грузятся с подменой ``launch``/``launch_ros`` (записывающие заглушки): тест видит ровно тот список нод,
который launch отдаёт ROS, и их порядок.
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
CONFIG_DIRS = (
    REPO / "docker" / "sound" / "config" / "voice_assistant",
    REPO / "src" / "rob_box_voice" / "config",
)
LAUNCHES = {
    "headless": REPO / "docker" / "sound" / "config" / "voice_assistant" / "voice_assistant_headless.launch.py",
    "src": REPO / "src" / "rob_box_voice" / "launch" / "voice_assistant.launch.py",
}
#: Ноды launch-файлов по порядку (нода диалога — ``dialog_node`` на месте удалённой ноды v1, PR-15 15p).
NODES = {
    "headless": ["audio_node", "led_node", "dialog_node", "tts_node", "stt_node", "sound_node", "command_node",
                 "speaker_id_node", "mcp_server"],
    "src": ["audio_node", "led_node", "dialog_node", "tts_node", "stt_node", "sound_node", "command_node",
            "speaker_id_node", "mcp_server"],
}


def test_engine_label_param_is_gone():
    """#4354 A4: метки ``dialog_engine`` у ``dialog_node`` нет — её читала только ветка v1 e2e-харнесса."""
    assert not hasattr(de, "declare") and not hasattr(de, "PARAM") and not hasattr(de, "V2")
    node_py = REPO / "src" / "rob_box_voice" / "rob_box_voice" / "dialog_node.py"
    assert "dialog_engine.declare" not in node_py.read_text(encoding="utf-8")


def test_no_engine_flag_file_in_config_dirs():
    """#4354 A3: флага движка нет — файла ``dialog_engine.yaml`` нет ни в одной копии каталога конфигов."""
    for config_dir in CONFIG_DIRS:
        assert not (config_dir / "dialog_engine.yaml").exists(), config_dir


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


@pytest.mark.parametrize("which", sorted(LAUNCHES))
def test_launch_starts_dialog_node_and_tts_without_engine_param(monkeypatch, tmp_path, which):
    """``tts_node`` параметр ``dialog_engine`` не объявляет — launch его больше не передаёт (#4354 A3)."""
    names, tts = _nodes(monkeypatch, which, tmp_path)
    assert names == NODES[which]
    assert "dialogue_node" not in names
    assert not any(isinstance(p, dict) and "dialog_engine" in p for p in tts.kwargs["parameters"])


def test_launch_dialog_node_gets_its_yaml(monkeypatch, tmp_path):
    _fake_launch(monkeypatch)
    cfg = tmp_path
    tts_args = {"package": "rob_box_voice", "executable": "tts_node", "name": "tts_node", "namespace": "",
                "parameters": ["tts_node.yaml"]}
    dialog, tts = de.launch_nodes({"config_dir": str(cfg)}, tts_args)
    assert (dialog.kwargs["executable"], dialog.name) == ("dialog_node", "dialog_node")
    assert dialog.kwargs["parameters"] == [str(cfg / "dialog_node.yaml"), {}]


def test_launch_dialog_node_gets_the_music_chooser_of_the_mcp_server(monkeypatch, tmp_path):
    """Ревью #4071 N4: путь пения хода (``dialog_node.build_sing``) выбирает тем же выбирателем, что путь тула музыки —
    значение ``mcp_server.music_chooser`` одно, launch передаёт его ``dialog_node`` (своей копии в yaml нет)."""
    _fake_launch(monkeypatch)
    cfg = tmp_path
    (cfg / "mcp_server.yaml").write_text('mcp_server:\n  ros__parameters:\n    music_chooser: "deterministic"\n',
                                         encoding="utf-8")
    tts_args = {"package": "rob_box_voice", "executable": "tts_node", "name": "tts_node", "namespace": "",
                "parameters": ["tts_node.yaml"]}
    dialog, _tts = de.launch_nodes({"config_dir": str(cfg)}, tts_args)
    assert dialog.kwargs["parameters"][1] == {"music_chooser": "deterministic"}


def test_shared_params_of_both_config_copies_are_the_live_music_chooser():
    """Обе копии каталога конфигов дают ``dialog_node`` режим выбирателя из своего ``mcp_server.yaml``."""
    for config_dir in CONFIG_DIRS:
        mcp = yaml.safe_load((config_dir / "mcp_server.yaml").read_text(encoding="utf-8"))
        assert de.shared_params(config_dir) == {"music_chooser": mcp["mcp_server"]["ros__parameters"]["music_chooser"]}
    assert de.shared_params(Path("нет-такого-каталога")) == {}


def test_engine_flag_is_gone_from_supervisor_telegram_and_launch():
    """#4354 A3: ТАРС и Telegram не выбирают движок (ветки v1 удалены), launch не читает флаг."""
    users = [LAUNCHES["headless"], LAUNCHES["src"],
             REPO / "src" / "rob_box_telegram" / "rob_box_telegram" / "telegram_node.py",
             REPO / "src" / "rob_box_supervisor" / "rob_box_supervisor" / "supervisor_node.py",
             REPO / "src" / "rob_box_supervisor" / "rob_box_supervisor" / "operator_turns.py",
             REPO / "src" / "rob_box_supervisor" / "rob_box_supervisor" / "operator_prompt.py"]
    for path in users:
        text = path.read_text(encoding="utf-8")
        assert "dialog_engine.declare" not in text and "dialog_engine.V1" not in text, path
        assert '"v1"' not in text and "'v1'" not in text, path
