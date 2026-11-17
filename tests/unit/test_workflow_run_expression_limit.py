"""Гард: ни один `run:` с `${{ ... }}` не длиннее 21000 байт UTF-8.

Почему. GitHub считает `run:`-блок, внутри которого есть `${{`, одной строкой-выражением с лимитом 21000
(по байтам; кириллица в комментариях = 2 байта). Превысил — workflow не парсится: вместо прогона создаётся
«пустой» прогон с именем-путём файла, conclusion=failure, 0 джоб, и ни один чек не идёт. Так 23.10.2026 после
влития #3955 шаг «Run unit tests» в `G-Run Tests.yml` (19226 -> 22210 байт) положил тесты на develop
(runs 39965049542, 39965919381). actionlint и `yaml.safe_load` это не ловят.

Лечится выносом тела в скрипт (`source`/вызов) или сокращением комментариев.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
# Жёсткий лимит GitHub и рабочий потолок с запасом: упёрлись в потолок — выноси тело в скрипт, не растяни.
GITHUB_LIMIT_BYTES = 21000
CEILING_BYTES = 20000


def _yaml_files() -> list[Path]:
    gh = REPO_ROOT / ".github"
    files = list(gh.glob("workflows/*.yml")) + list(gh.glob("workflows/*.yaml"))
    files += list(gh.glob("actions/**/action.yml")) + list(gh.glob("actions/**/action.yaml"))
    return sorted(files)


def _run_blocks(node):
    """Все строки под ключом `run` в произвольно вложенном YAML."""
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "run" and isinstance(value, str):
                yield value
            else:
                yield from _run_blocks(value)
    elif isinstance(node, list):
        for item in node:
            yield from _run_blocks(item)


def _offenders(ceiling: int) -> list[str]:
    bad = []
    for path in _yaml_files():
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        for run in _run_blocks(data):
            size = len(run.encode("utf-8"))
            if "${{" in run and size > ceiling:
                head = next((ln.strip() for ln in run.splitlines() if ln.strip()), "")[:60]
                bad.append(f"{path.relative_to(REPO_ROOT)}: {size} байт (потолок {ceiling}), начало: {head!r}")
    return bad


def test_there_are_workflows_to_check() -> None:
    assert _yaml_files(), "не найден ни один workflow/action — гард ничего не проверяет"


def test_no_run_block_with_expression_exceeds_ceiling() -> None:
    bad = _offenders(CEILING_BYTES)
    assert not bad, (
        "run-блок с `${{` длиннее потолка: GitHub отвергнет workflow при "
        f"{GITHUB_LIMIT_BYTES} байтах (0 джоб, чеки не идут). Вынеси тело в скрипт.\n" + "\n".join(bad)
    )


def test_guard_detects_oversized_block(tmp_path: Path) -> None:
    """Самопроверка: блок с `${{` за лимитом попадает в нарушители, без `${{` — нет."""
    big = "echo ${{ github.sha }}\n" + "# я\n" * 6000
    plain = "echo hi\n" + "# я\n" * 6000
    assert len(big.encode("utf-8")) > GITHUB_LIMIT_BYTES
    assert any("${{" in r and len(r.encode("utf-8")) > CEILING_BYTES for r in _run_blocks({"steps": [{"run": big}]}))
    assert not any(
        "${{" in r and len(r.encode("utf-8")) > CEILING_BYTES for r in _run_blocks({"steps": [{"run": plain}]})
    )


@pytest.mark.parametrize("run", ["x" * 10])
def test_run_blocks_walks_nested(run: str) -> None:
    doc = {"jobs": {"a": {"steps": [{"run": run}, {"uses": "x"}]}}}
    assert list(_run_blocks(doc)) == [run]
