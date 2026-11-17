"""Тестовый профиль робота в тестах ядра совпадает с настоящим профилем робота (ADR-0167 Ш1б-2б, Ш1б-3).

Вейки и персона робота живут в профиле робота (``docker/sound/config/assistant.yaml`` → ``wake_words.yaml``,
``persona.txt``); у ядра их нет.
Тесты ядра (``src/rob_box_dialog/test``) - правила над речью робота - держат копию профиля в
``fixtures/robot_profile``, чтобы жить и без репо робота (будущий репо ядра). Копия обязана совпадать с оригиналом:
расхождение - тесты ядра проверяют не тот список, что слышит робот (класс ошибки #1252).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from rob_box_dialog.profile import load_profile

REPO = Path(__file__).resolve().parents[3]
ROBOT = REPO / "docker" / "sound" / "config"
FIXTURE = REPO / "src" / "rob_box_dialog" / "test" / "fixtures" / "robot_profile"

pytestmark = pytest.mark.skipif(not (ROBOT / "assistant.yaml").is_file(), reason="нет docker/ (без профиля робота)")


def test_fixture_wake_table_is_the_robot_wake_table():
    robot = yaml.safe_load((ROBOT / "wake_words.yaml").read_text(encoding="utf-8"))
    fixture = yaml.safe_load((FIXTURE / "wake_words.yaml").read_text(encoding="utf-8"))
    assert fixture == robot


def test_fixture_profile_is_the_robot_profile():
    robot, fixture = load_profile(ROBOT), load_profile(FIXTURE)
    assert (
        fixture.name,
        fixture.wake_personality,
        fixture.wake_operator,
        fixture.persona,
        fixture.operator_persona,
        fixture.owners,
    ) == (
        robot.name,
        robot.wake_personality,
        robot.wake_operator,
        robot.persona,
        robot.operator_persona,
        robot.owners,
    )
