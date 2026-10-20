"""Таблица знания: вейки совпадают с YAML робота, Tier-1, громкость, классы действий, экспрессия и схема команды
согласованы с каталогом тулов и лабораторией лица (ADR-0150 PR-1)."""

import ast
import pathlib

import pytest
import yaml

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import knowledge as kn

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
WAKE_YAML = REPO_ROOT / "docker" / "sound" / "config" / "wake_words.yaml"


def test_wake_words_yaml_matches_code():
    data = yaml.safe_load(WAKE_YAML.read_text(encoding="utf-8"))
    assert data["personality"] == list(kn.DEFAULT_WAKE_WORDS)
    assert data["operator"] == list(kn.DEFAULT_OPERATOR_WAKE_WORDS)
    expected = (list(kn.DEFAULT_WAKE_WORDS), list(kn.DEFAULT_OPERATOR_WAKE_WORDS))
    assert kn.resolve_wake_word_namespaces(str(WAKE_YAML)) == expected


def test_wake_words_loader_is_safe(tmp_path):
    broken = tmp_path / "w.yaml"
    broken.write_text("personality: [unclosed", encoding="utf-8")
    assert kn.load_wake_word_namespaces(str(broken)) == {}
    assert kn.load_wake_word_namespaces(str(tmp_path / "missing.yaml")) == {}
    assert kn.load_wake_word_namespaces("") == {}
    assert kn.resolve_wake_word_namespaces(None, ["робот"], ["тарс"]) == (["робот"], ["тарс"])


def test_wake_words_loader_keeps_file_order(tmp_path):
    custom = tmp_path / "w.yaml"
    custom.write_text("personality: [робби, робот]\noperator: [тарс]\n", encoding="utf-8")
    assert kn.load_wake_word_namespaces(str(custom)) == {"personality": ["робби", "робот"], "operator": ["тарс"]}


def test_longer_wake_variants_come_before_their_prefixes():
    words = kn.DEFAULT_WAKE_WORDS
    for i, short in enumerate(words):
        for longer in words[i + 1:]:
            assert not longer.startswith(short), f"«{longer}» стоит после своего префикса «{short}»"


def test_tier1_policy_uses_known_policies():
    for group in kn.TIER1_POLICY.values():
        assert group.policy in kn.POLICIES and group.boundary in kn.BOUNDARIES and group.words


def test_volume_table():
    assert {intent for intent, _ in kn.VOLUME_WORDS} == set(kn.VOLUME_INTENTS)
    assert set(kn.VOLUME_CHANNELS) == {"tts", "music", "sfx"}
    catalog = {t.name for t in TOOL_CATALOG}
    assert {c.tool for c in kn.VOLUME_CHANNELS.values() if c.tool} <= catalog


def _literal(path: pathlib.Path, name: str):
    """Значение присваивания ``name = <литерал>`` модуля без его импорта (тянет numpy/ROS); у dict — ключи."""
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        targets = node.targets if isinstance(node, ast.Assign) else [getattr(node, "target", None)]
        if any(isinstance(t, ast.Name) and t.id == name for t in targets):
            if isinstance(node.value, ast.Dict):
                return [ast.literal_eval(k) for k in node.value.keys]
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} не найден в {path}")


def test_every_catalog_tool_has_an_action_class():
    names = {t.name for t in TOOL_CATALOG}
    classified = set(kn.TOOL_CLASSES) | set(kn.SPEECH_CHANNEL_TOOLS)
    assert names - classified == set(), "тул каталога без класса действия"
    assert classified - names == set(), "класс у тула, которого нет в каталоге"
    assert set(kn.TOOL_CLASSES.values()) <= set(kn.ACTION_CLASSES)


def test_action_classes_follow_adr_5_1():
    for name, cls in kn.ACTION_CLASSES.items():
        assert cls.timeout_s is None or cls.timeout_s > 0, name
        assert set(cls.phrases) <= set(kn.ACTION_STATUSES), name
        assert set(cls.agents) <= set(kn.AGENTS), name
    motion = kn.ACTION_CLASSES["action.motion"]
    assert (motion.timeout_s, motion.interruptible) == (120.0, False)
    assert kn.ACTION_CLASSES["action.media"].timeout_s == 6.0
    assert kn.ACTION_CLASSES["action.local"].timeout_s == 3.0
    assert (kn.ACTION_CLASSES["query"].timeout_s, kn.ACTION_CLASSES["query"].interruptible) == (15.0, True)
    assert kn.ACTION_CLASSES["action.speak_aloud"].agents == ("operator", "telegram")
    assert motion.phrases["pending"] == "motion.started"
    assert set(kn.SINGLE_ACTION_CLASSES) <= set(kn.ACTION_CLASSES)


def test_emotions_and_gestures_are_scenes_of_player_or_lab():
    known = set(_literal(REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "animations.py",
                         "KNOWN_ANIMATIONS"))
    lab = set(_literal(REPO_ROOT / "tools" / "led_face_lab" / "states.py", "CLIPS"))
    assert set(kn.LAB_SCENES) <= lab
    scenes = known | set(kn.LAB_SCENES)
    assert set(kn.EMOTIONS.values()) <= scenes
    assert set(kn.GESTURES) <= scenes


def _walk(schema):
    yield schema
    for sub in schema.get("properties", {}).values():
        yield from _walk(sub)
    if isinstance(schema.get("items"), dict):
        yield from _walk(schema["items"])


def test_command_schema_modes():
    schema = kn.command_schema("json")
    assert schema["properties"]["speech"]["maxLength"] == kn.SPEECH_MAX_CHARS
    assert schema["properties"]["actions"]["maxItems"] == kn.ACTIONS_MAX
    face = schema["properties"]["expression"]["properties"]["face"]["properties"]
    assert {k: tuple(v["enum"]) for k, v in face.items() if "enum" in v} == dict(kn.FACE_SCENE_SLOTS)
    for node in _walk(schema):
        assert node.get("type") in {"object", "array", "string", "integer", "number", "boolean"}
        if "enum" in node:
            assert node["enum"], node
    tool = kn.command_schema("tool")
    assert tool["function"]["name"] == kn.COMMAND_TOOL and tool["function"]["parameters"] == schema
    with pytest.raises(ValueError):
        kn.command_schema("xml")


def test_prompt_fragment_names_every_enum_value():
    text = kn.prompt_fragment()
    values = [*kn.EMOTIONS, *kn.GESTURES, *kn.EARCONS, *kn.PERFORM_KINDS, *kn.WHEN, *kn.ASK_EXPECTS, *kn.PASSED,
              *(v for vs in kn.FACE_SCENE_SLOTS.values() for v in vs)]
    assert [v for v in values if v not in text] == []
    assert str(kn.SPEECH_MAX_CHARS) in text and str(kn.ACTIONS_MAX) in text
