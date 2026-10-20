"""Таблица знания: вейк-слова совпадают с YAML робота, Tier-1 и громкость согласованы (ADR-0150 PR-1)."""

import pathlib

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
