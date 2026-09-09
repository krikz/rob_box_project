"""Regression tests for SHA-tag spam skip on round branches (issue #1826).

Контекст (issue #1826):
L-Build-All-Services триггерится каждые ~3-4 мин на z-{e2e}/test-round-N
ветке. Цикл:
  1. agent-flow триггерит L-Build → CI коммитит 'ci: main SHA tags → ...'
     в round → agent-flow видит новый HEAD → триггерит ещё build → loop.

Решение (двухслойное):
  1) CI side (root): L-Build Main/Vision Services.yml — skip SHA-tag commit
     для веток z-{e2e}/test-round-* и z-{e2e}/wip-*.
  2) agent-flow side (defense in depth): если последний коммит round —
     SHA-tag noise и для parent уже есть SUCCESS build → resume.

Тесты покрывают:
  - YAML guard присутствует в L-Build Main Pi Services.yml
  - YAML guard присутствует в L-Build Vision Pi Services.yml
  - Regex корректно матчит test-round-* и wip-* ветки (positive)
  - Regex НЕ матчит develop/main/feature/* ветки (negative)
  - Regex НЕ матчит произвольные строки (negative)
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]


# ----------------------------------------------------------------------- #
# Паттерн из workflow — должен быть синхронизирован с yml-guard'ом.
# ВАЖНО: regex в bash [[ =~ ]] требует \{ экранирования, тут берём без экрана
# для python-проверки (мы проверяем ту же логику через re.match).
# ----------------------------------------------------------------------- #

ROUND_BRANCH_RE = re.compile(r"^z-\{e2e\}/test-round-")
WIP_BRANCH_RE = re.compile(r"^z-\{e2e\}/wip-")


def _is_round_or_wip(branch: str) -> bool:
    """Python-эквивалент bash-guard из workflow (для unit-теста)."""
    return bool(ROUND_BRANCH_RE.match(branch) or WIP_BRANCH_RE.match(branch))


# ----------------------------------------------------------------------- #
# Regex match tests
# ----------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "branch",
    [
        "z-{e2e}/test-round-1",
        "z-{e2e}/test-round-310",
        "z-{e2e}/test-round-9999",
        "z-{e2e}/wip-1826-bug-ci",
        "z-{e2e}/wip-something-else",
    ],
)
def test_round_or_wip_branch_matches(branch: str) -> None:
    """Round/wip ветки ДОЛЖНЫ матчиться — guard их пропускает."""
    assert _is_round_or_wip(branch), (
        f"{branch} должен матчиться guard'ом (skip SHA-tag push)"
    )


@pytest.mark.parametrize(
    "branch",
    [
        "develop",
        "main",
        "feature/voice-fix",
        "fix/some-bug",
        "release/v1.0.0",
        "hotfix/critical",
        "z-{agent}/1826-bug-ci",
        "z-{agent}/something",
        "z-{e2e}/test-round",  # без номера
        "z-{e2e}/wip",  # без -
        "z-{e2e}/test-something",  # не round-
        "copilot/some-feature",
    ],
)
def test_non_round_or_wip_branch_does_not_match(branch: str) -> None:
    """Не-round/wip ветки НЕ должны матчиться — guard пропускает SHA-tag push."""
    assert not _is_round_or_wip(branch), (
        f"{branch} НЕ должен матчиться guard'ом (SHA-tag push НУЖЕН)"
    )


def test_test_round_branch_with_non_digit_suffix_matches() -> None:
    """test-round-foo (без цифр) ТОЖЕ матчит — на всякий случай.

    В реальности counter инкрементит N (число), но guard оставляем liberal —
    пусть ЛЮБАЯ ветка с префиксом z-{e2e}/test-round- пропускает SHA-tag.
    Это безопаснее чем узкий паттерн (если кто-то создаст round вручную).
    """
    assert _is_round_or_wip("z-{e2e}/test-round-foo")


# ----------------------------------------------------------------------- #
# YAML guard presence tests
# ----------------------------------------------------------------------- #


def _read_yaml(path: Path) -> str:
    assert path.exists(), f"workflow file not found: {path}"
    return path.read_text()


def test_main_build_workflow_has_skip_guard() -> None:
    """L-Build Main Pi Services.yml ДОЛЖЕН содержать guard от SHA-tag push на round."""
    yml = _read_yaml(REPO_ROOT / ".github" / "workflows" / "L-Build Main Pi Services.yml")
    assert "#1826" in yml or "issue #1826" in yml.lower(), (
        "L-Build Main Pi Services.yml должен иметь ссылку на issue #1826 в guard-комментарии"
    )
    assert r"^z-\{e2e\}/test-round-" in yml, (
        "L-Build Main Pi Services.yml должен иметь regex guard для test-round-*"
    )
    assert r"^z-\{e2e\}/wip-" in yml, (
        "L-Build Main Pi Services.yml должен иметь regex guard для wip-*"
    )
    assert "Issue #1826: skip SHA-tag push" in yml, (
        "Main: должен быть echo с skip-сообщением для round-ветки"
    )


def test_vision_build_workflow_has_skip_guard() -> None:
    """L-Build Vision Pi Services.yml ДОЛЖЕН содержать guard от SHA-tag push на round."""
    yml = _read_yaml(REPO_ROOT / ".github" / "workflows" / "L-Build Vision Pi Services.yml")
    assert "#1826" in yml or "issue #1826" in yml.lower(), (
        "L-Build Vision Pi Services.yml должен иметь ссылку на issue #1826 в guard-комментарии"
    )
    assert r"^z-\{e2e\}/test-round-" in yml, (
        "L-Build Vision Pi Services.yml должен иметь regex guard для test-round-*"
    )
    assert r"^z-\{e2e\}/wip-" in yml, (
        "L-Build Vision Pi Services.yml должен иметь regex guard для wip-*"
    )
    assert "Issue #1826: skip SHA-tag push" in yml, (
        "Vision: должен быть echo с skip-сообщением для round-ветки"
    )


def test_main_build_workflow_preserves_commit_for_non_round() -> None:
    """Проверяем что оригинальная логика commit сохранена для develop/main/feature/*."""
    yml = _read_yaml(REPO_ROOT / ".github" / "workflows" / "L-Build Main Pi Services.yml")
    # В else-ветке должен остаться git commit.
    assert 'git commit -m "ci: main SHA tags' in yml, (
        "Main: должна сохраняться оригинальная логика commit для non-round веток"
    )


def test_vision_build_workflow_preserves_commit_for_non_round() -> None:
    yml = _read_yaml(REPO_ROOT / ".github" / "workflows" / "L-Build Vision Pi Services.yml")
    assert 'git commit -m "ci: vision SHA tags' in yml, (
        "Vision: должна сохраняться оригинальная логика commit для non-round веток"
    )


# ----------------------------------------------------------------------- #
# agent-flow defense-in-depth guard
# ----------------------------------------------------------------------- #


def test_agent_flow_e2e_process_has_sha_spam_skip() -> None:
    """agent-flow-e2e-process.sh должен иметь detection SHA-tag spam + skip trigger."""
    script_path = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-process.sh"
    assert script_path.exists(), f"script not found: {script_path}"
    script = script_path.read_text()
    assert "#1826" in script, (
        "agent-flow-e2e-process.sh должен ссылаться на issue #1826 в guard-комментарии"
    )
    # Acceptance requirement: «diagnostic log "CI SHA-tag spam, skipping"».
    assert "CI SHA-tag spam, skipping" in script, (
        "agent-flow: должен логировать «CI SHA-tag spam, skipping» при обнаружении noise"
    )
    # Regex для SHA-tag коммитов в subject.
    assert "SHA" in script and "tags" in script, (
        "agent-flow: должен матчить SHA-tag коммиты по subject"
    )
    # После guard'а _round_head переназначается на parent (для downstream _existing_build).
    assert '_round_head="$_round_parent"' in script, (
        "agent-flow: должен переназначить _round_head на parent после детекта spam"
    )


def test_agent_flow_defense_in_depth_does_not_break_normal_flow() -> None:
    """Проверяем что новый guard не блокирует нормальный build-триггер когда это нужно.

    Конкретно: если последний коммит round — НЕ SHA-tag, новый код не должен
    менять _round_head. Это контракт: guard только для spam, не для нормальных
    merge-коммитов.
    """
    script_path = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-process.sh"
    script = script_path.read_text()
    # Контракт: новый код лежит между двумя anchor'ами.
    assert "_round_head=\"$(git -C \"$WORKTREE_DIR\" rev-parse HEAD" in script
    # Переназначение _round_head="$_round_parent" должно быть ВНУТРИ if-блока,
    # и if-блок должен быть guard'ом (не безусловным).
    # Проверим что присвоение _round_head="$_round_parent" следует после if-строки.
    # Грубая эвристика: между guard-логикой и _consec_fail_count.
    start_idx = script.find("_round_head=\"$(git -C \"$WORKTREE_DIR\" rev-parse HEAD")
    consec_idx = script.find("_consec_fail_count=\"$(gh run list", start_idx)
    parent_assign_idx = script.find('_round_head="$_round_parent"', start_idx)
    assert start_idx > 0
    assert consec_idx > start_idx, "structure broken: consec_check missing"
    assert parent_assign_idx > start_idx, "structure broken: parent assignment missing"
    assert parent_assign_idx < consec_idx, (
        "structure broken: parent assignment should be inside guard, before consec_check"
    )
