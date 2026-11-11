#!/usr/bin/env python3
"""test_wake_words_config.py — SSoT wake_words.yaml (issue #1990).

Проверяет, что ``docker/sound/config/wake_words.yaml`` — единственный
источник истины в проде — несёт оба namespace, а ``personality`` байт-в-байт
совпадает (и по составу, и по порядку) с кодовым фолбеком
``ROBOT_WAKE`` (иначе personality в проде и в dev-env разъедутся —
ровно класс ошибки #1252).

Заодно проверяет безопасный загрузчик ``load_wake_word_namespaces`` /
``resolve_wake_word_namespaces``: отсутствующий/битый файл → фолбек, без падения.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from rob_box_voice.core.dialogue_text import load_wake_word_namespaces, resolve_wake_word_namespaces
from voice_robot_wake import ROBOT_OPERATOR_WAKE, ROBOT_WAKE  # noqa: E402 — тестовый профиль робота

REPO_ROOT = Path(__file__).resolve().parents[5]
SSOT = REPO_ROOT / "docker" / "sound" / "config" / "wake_words.yaml"


def _load_ssot() -> dict:
    assert SSOT.exists(), f"SSoT файл не найден: {SSOT}"
    data = yaml.safe_load(SSOT.read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


class TestWakeWordsSSoT:
    def test_ssot_has_exactly_two_namespaces(self):
        # + переключатель рискованных вейков В18 (ADR-0150 PR-5)
        assert set(_load_ssot()) == {"personality", "operator", "risky_enabled"}

    def test_personality_matches_code_defaults_byte_for_byte(self):
        """Порядок важен для strip_wake_word (regex leftmost-first)."""
        data = _load_ssot()
        plain = [w for w in data["personality"] if isinstance(w, str)]  # risky: true (В18) — отдельно
        assert plain == list(ROBOT_WAKE), (
            "docker/sound/config/wake_words.yaml personality разошёлся с "
            "ROBOT_WAKE — это ровно класс ошибки #1252. Правь оба "
            "места в одном коммите."
        )

    def test_operator_matches_code_default(self):
        data = _load_ssot()
        assert data["operator"] == list(ROBOT_OPERATOR_WAKE)

    def test_resolve_real_ssot_file_uses_file_values(self):
        personality, operator = resolve_wake_word_namespaces(str(SSOT))
        assert personality == list(ROBOT_WAKE)
        assert operator == list(ROBOT_OPERATOR_WAKE)


class TestWakeWordsLoaderFallback:
    def test_load_missing_file_returns_empty(self, tmp_path):
        assert load_wake_word_namespaces(str(tmp_path / "nope.yaml")) == {}

    def test_load_broken_yaml_returns_empty(self, tmp_path):
        broken = tmp_path / "broken.yaml"
        broken.write_text("personality: [unclosed", encoding="utf-8")
        assert load_wake_word_namespaces(str(broken)) == {}

    def test_load_non_dict_yaml_returns_empty(self, tmp_path):
        scalar = tmp_path / "scalar.yaml"
        scalar.write_text("просто строка\n", encoding="utf-8")
        assert load_wake_word_namespaces(str(scalar)) == {}

    def test_resolve_empty_path_is_neutral_or_given_fallbacks(self):
        # ADR-0167 Ш1б-2б: кодовых вейков нет - без файла нейтрально или то, что передал хост.
        assert resolve_wake_word_namespaces(None) == ([], [])
        personality, operator = resolve_wake_word_namespaces(None, ROBOT_WAKE, ROBOT_OPERATOR_WAKE)
        assert personality == list(ROBOT_WAKE)
        assert operator == list(ROBOT_OPERATOR_WAKE)

    def test_resolve_ignores_unknown_namespaces(self, tmp_path):
        f = tmp_path / "w.yaml"
        f.write_text("personality:\n  - робокс\nunknown_ns:\n  - x\n", encoding="utf-8")
        personality, operator = resolve_wake_word_namespaces(str(f))
        assert personality == ["робокс"]
        # operator отсутствует в файле → фолбек (кодовых вейков нет - пусто)
        assert operator == []

    def test_loader_preserves_authoring_order(self, tmp_path):
        f = tmp_path / "w.yaml"
        f.write_text(
            "personality:\n  - роб бокс\n  - робокс\noperator:\n  - тарс\n",
            encoding="utf-8",
        )
        personality, operator = resolve_wake_word_namespaces(str(f))
        # длинный вариант первым — порядок файла, от него зависит strip
        assert personality == ["роб бокс", "робокс"]
        assert operator == ["тарс"]
