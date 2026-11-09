"""Хосты робота берут вейки из профиля робота и без него не стартуют (ADR-0167 §2.4, шаг Ш1б-2).

``dialog_node``, ``stt_node`` и сессия ТАРС супервизора грузят профиль строго (``robot_profile.load``): нейтральные
умолчания ядра (вейк выключен, адресовано всё) роботу нельзя - он отвечал бы на каждую фразу мастерской.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.profile import ProfileError

robot_profile = pytest.importorskip("rob_box_voice.robot_profile")

REPO = Path(__file__).resolve().parents[3]
CONFIG = REPO / "docker" / "sound" / "config"
VOICE = REPO / "src" / "rob_box_voice"
HOSTS = {
    "dialog_node": VOICE / "rob_box_voice" / "dialog_node.py",
    "stt_node": VOICE / "rob_box_voice" / "stt_node.py",
}
YAMLS = {
    "dialog_node": (CONFIG / "voice_assistant" / "dialog_node.yaml", VOICE / "config" / "dialog_node.yaml"),
    "stt_node": (CONFIG / "voice_assistant" / "stt_node.yaml", VOICE / "config" / "stt_node.yaml"),
}
OPERATOR_TURNS = REPO / "src" / "rob_box_supervisor" / "rob_box_supervisor" / "operator_turns.py"

pytestmark = pytest.mark.skipif(not (CONFIG / "assistant.yaml").is_file(), reason="нет docker/ (без профиля робота)")


def _calls(path: Path):
    return [n for n in ast.walk(ast.parse(path.read_text(encoding="utf-8-sig"))) if isinstance(n, ast.Call)]


def _src(node: ast.AST) -> str:
    return ast.unparse(node)


def test_robot_profile_path_and_env(monkeypatch):
    monkeypatch.delenv(robot_profile.ENV, raising=False)
    assert robot_profile.path() == robot_profile.PATH == "/config/assistant.yaml"
    monkeypatch.setenv(robot_profile.ENV, "  ")
    assert robot_profile.path() == robot_profile.PATH
    monkeypatch.setenv(robot_profile.ENV, str(CONFIG / "assistant.yaml"))
    loaded = robot_profile.load()
    assert loaded.name == "РОББОКС" and list(loaded.wake_personality) == list(kn.DEFAULT_WAKE_WORDS)
    assert list(loaded.wake_operator) == list(kn.DEFAULT_OPERATOR_WAKE_WORDS)


def test_robot_profile_is_strict(tmp_path, monkeypatch):
    monkeypatch.setenv(robot_profile.ENV, str(tmp_path / "nope.yaml"))
    with pytest.raises(ProfileError, match="файла нет"):
        robot_profile.load()


@pytest.mark.parametrize("host", sorted(HOSTS))
def test_host_declares_the_robot_profile_and_loads_it_strictly(host):
    calls = _calls(HOSTS[host])
    declared = {
        n.args[0].value: _src(n.args[1])
        for n in calls
        if getattr(n.func, "attr", "") == "declare_parameter"
        and len(n.args) == 2
        and isinstance(n.args[0], ast.Constant)
    }
    assert declared.get("profile_path") == "robot_profile.path()"
    assert not {"wake_words", "wake_words_file", "wake_words_path"} & set(declared)
    loads = [_src(n.func) for n in calls]
    assert "robot_profile.load" in loads
    assert not {"resolve_profile", "address.wake_words", "wake_words"} & set(loads)
    for yaml_file in YAMLS[host]:
        text = yaml_file.read_text(encoding="utf-8")
        assert "profile_path:" in text and "/config/assistant.yaml" in text, yaml_file
        assert "wake_words_file" not in text and "wake_words_path" not in text, yaml_file


def test_operator_session_gets_the_robot_wake_table():
    sessions = [n for n in _calls(OPERATOR_TURNS) if _src(n.func) == "Session"]
    assert sessions and all(
        any(k.arg == "wake_words" and _src(k.value) == "robot_profile.load().wake_personality" for k in s.keywords)
        for s in sessions
    )
