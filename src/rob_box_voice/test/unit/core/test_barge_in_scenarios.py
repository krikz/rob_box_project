#!/usr/bin/env python3
"""Unit tests for barge-in scenario YAML coverage (TASK-047 regression 37527df).

Инвариант, который проверяют интеграционные сценарии (docker scenario_runner,
00_barge_in.yaml): после нового dialogue_id — speak_text от старого диалога
НЕ должен воспроизводиться. Эти unit-тесты проверяют, что YAML-сценарии
реально покрывают acceptance criteria TASK-047:

  - Сценарий A: прерывание во время ответа (анекдот → память)
  - Сценарий B: быстрое тройное прерывание, обрабатывается только последнее
  - Сценарий C: прерывание во время listen_for_response
  - Верификация: после каждого сценария dialogue_node возвращается в idle

Статическая проверка структуры YAML (без ROS/rclpy — CI-friendly).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

# В CI workspace docker/ копируется в test_ws/docker — parents[5] указывает
# на корень репозитория/workspace (src/rob_box_voice/test/unit/core/ → root).
REPO_ROOT = Path(__file__).resolve().parents[5]
SCENARIOS_FILE = (
    REPO_ROOT / "docker/vision/test/scenario_runner/scenarios/00_barge_in.yaml"
)

# Шаги, которые должны присутствовать в каждом barge-in сценарии (AC TASK-047)
REQUIRED_STEP_KEYS = {
    # Прерывающий запрос (новый dialogue_id)
    "inject_stt",
    # Ключевой инвариант: после прерывания НЕ должно прийти ни одного
    # нового response от старого диалога (speak_text старого dialogue_id
    # не воспроизводится)
    "assert_quiet_after_ms",
    # Верификация: dialogue_node возвращается в state=idle без зависания
    "assert_idle",
}


def _load_scenarios() -> list[dict]:
    if not SCENARIOS_FILE.exists():
        pytest.skip(f"scenario file not found: {SCENARIOS_FILE}")
    with open(SCENARIOS_FILE, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data.get("scenarios", [])


def _step_keys(scenario: dict) -> set[str]:
    keys: set[str] = set()
    for step in scenario.get("steps", []):
        keys.update(step.keys())
    return keys


def test_scenario_file_exists() -> None:
    assert SCENARIOS_FILE.exists(), f"missing {SCENARIOS_FILE}"


def test_has_three_barge_in_scenarios() -> None:
    scenarios = _load_scenarios()
    names = {s.get("name") for s in scenarios}
    # AC TASK-047: три сценария A/B/C
    assert "barge_in_joke_then_memory" in names, f"missing scenario A in {names}"
    assert "barge_in_rapid_triple_interrupt" in names, (
        f"missing scenario B (rapid triple interrupt) in {names}"
    )
    assert "barge_in_during_listen_for_response" in names, (
        f"missing scenario C in {names}"
    )


@pytest.mark.parametrize(
    "scenario_name",
    [
        "barge_in_joke_then_memory",
        "barge_in_rapid_triple_interrupt",
        "barge_in_during_listen_for_response",
    ],
)
def test_each_scenario_covers_ac(scenario_name: str) -> None:
    """Каждый сценарий обязан содержать прерывание + проверку ключевого
    инварианта (нет stale responses) + возврат в idle."""
    scenarios = {s.get("name"): s for s in _load_scenarios()}
    scenario = scenarios.get(scenario_name)
    assert scenario is not None, f"scenario {scenario_name!r} missing"

    keys = _step_keys(scenario)
    missing = REQUIRED_STEP_KEYS - keys
    assert not missing, (
        f"scenario {scenario_name!r} does not cover AC — missing step keys: "
        f"{sorted(missing)}"
    )


def test_key_invariant_quiet_after_barge_in() -> None:
    """Ключевой инвариант 37527df: после прерывающего STT должен идти шаг
    assert_quiet_after_ms — иначе тест не ловит досказывание старого диалога."""
    scenarios = {s.get("name"): s for s in _load_scenarios()}
    for name in ("barge_in_joke_then_memory", "barge_in_rapid_triple_interrupt"):
        scenario = scenarios.get(name)
        assert scenario is not None
        for step in scenario.get("steps", []):
            if "assert_quiet_after_ms" in step:
                assert int(step["assert_quiet_after_ms"]) >= 1000, (
                    f"{name}: assert_quiet_after_ms слишком мал — не ловит "
                    f"досказывание старого диалога"
                )
