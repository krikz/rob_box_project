"""
test_issue_2763_speaker_db_isolation_anchor.py — якорь изоляции БД дикторов

Инцидент 22.09.2026 (run 35729958215): акт «Знакомство» записал синтетических
«Сашу» и «Бориса» в боевую /data/speakers.db рядом с профилями живых людей,
хотя изоляция из #2759 + #2763 была задеплоена и вручную на роботе работала.

Причина — не в самой изоляции, а в том, КАК решается, нужна ли она. Матчер
смотрел на имя файла сценария:

    case "$1" in *night_marathon_act2_acquaintance*) return 0 ;;

А воркфлоу копирует сценарий на билд-машину под фиксированным именем:

    CMD="/tmp/e2e_voice_test.sh --scenario /tmp/e2e_scenario.json"

Совпадения не было НИКОГДА. Отказ тихий: activate_e2e_speaker_db просто не
вызывался, поэтому в логе прогона нет ни «🧪 e2e_mode=true», ни E2E_FATAL —
отсутствие ОБЕИХ строк и есть диагноз.

Эти тесты держат три вещи:
  1. якорь берётся из СОДЕРЖИМОГО сценария (сценарий регистрирует дикторов),
     а не из его имени — то есть переживает переименование в /tmp;
  2. реальный сценарий акта 2 опознаётся ПОД ИМЕНЕМ, КОТОРОЕ ЕМУ ДАЁТ
     ВОРКФЛОУ — это и есть регрессионный тест на сам инцидент;
  3. сценарий без регистрации дикторов изоляции не требует.

Run:
  python3 -m pytest tests/unit/e2e_scripts/test_issue_2763_speaker_db_isolation_anchor.py -v --no-cov
"""

import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
E2E_SCRIPT = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"
SCENARIO_DIR = REPO_ROOT / ".github" / "e2e" / "scenarios" / "night"

#: Имя, под которым сценарий доезжает до харнесса на билд-машине. Захардкожено
#: в «L-E2E Voice Test.yml» — харнесс НИКОГДА не видит исходный путь.
WORKFLOW_SCENARIO_NAME = "e2e_scenario.json"

bash_required = pytest.mark.skipif(
    shutil.which("bash") is None, reason="bash недоступен в этом окружении"
)


def _extract_function(name: str) -> str:
    """Вырезать тело bash-функции из харнесса (конвенция остальных тестов)."""
    src = E2E_SCRIPT.read_text(encoding="utf-8")
    m = re.search(rf"^{name}\(\) \{{.*?^\}}", src, re.S | re.M)
    assert m, f"функция {name}() не найдена в {E2E_SCRIPT.name}"
    return m.group(0)


def _ask(scenario_path: Path) -> int:
    """rc функции-якоря для данного файла сценария."""
    body = _extract_function("scenario_registers_speakers")
    script = f'{body}\nscenario_registers_speakers "{scenario_path.as_posix()}"'
    return subprocess.run(
        ["bash", "-c", script], capture_output=True, text=True, timeout=10
    ).returncode


def _require_bash_sees(path: Path) -> None:
    """Пропустить тест, если найденный bash не видит путь pytest-tmpdir.

    На Windows `shutil.which("bash")` нередко указывает на WSL, которому
    путь вида ``C:/Users/...`` не виден вообще: ``[ -f ]`` отвечает «нет»,
    и тест краснеет по причине, не имеющей отношения к харнессу. В CI
    (Linux-раннер) проверка проходит и тесты работают в полную силу.
    """
    probe = subprocess.run(
        ["bash", "-c", f'[ -f "{path.as_posix()}" ]'],
        capture_output=True,
        text=True,
        timeout=10,
    )
    if probe.returncode != 0:
        pytest.skip(f"bash не видит {path} (вероятно WSL на Windows) — проверка только в CI")


@bash_required
def test_act2_scenario_detected_under_workflow_filename(tmp_path: Path) -> None:
    """Регрессия инцидента: акт 2 опознаётся под именем e2e_scenario.json.

    Файл кладётся ровно под тем именем, которое даёт воркфлоу — если якорь
    снова начнёт смотреть на имя, этот тест покраснеет.
    """
    src = SCENARIO_DIR / "night_marathon_act2_acquaintance_v1.json"
    assert src.exists(), f"сценарий акта 2 не найден: {src}"
    renamed = tmp_path / WORKFLOW_SCENARIO_NAME
    renamed.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
    _require_bash_sees(renamed)

    assert _ask(renamed) == 0, (
        "акт «Знакомство» обязан требовать изоляции БД дикторов даже после "
        "переименования в /tmp/e2e_scenario.json — иначе он пишет голосовые "
        "профили в боевую /data/speakers.db (issue #2750/#2763)"
    )


@bash_required
def test_scenario_without_registration_needs_no_isolation(tmp_path: Path) -> None:
    """Сценарий без register_speaker изоляции не требует."""
    src = SCENARIO_DIR / "night_marathon_act1_wakeup_v1.json"
    assert src.exists(), f"сценарий акта 1 не найден: {src}"
    renamed = tmp_path / WORKFLOW_SCENARIO_NAME
    renamed.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
    _require_bash_sees(renamed)

    assert _ask(renamed) != 0


@bash_required
def test_missing_scenario_file_is_not_treated_as_registering(tmp_path: Path) -> None:
    """Пропавший файл — якорь отвечает «нет», решение принимает вызывающий."""
    probe = tmp_path / WORKFLOW_SCENARIO_NAME
    probe.write_text("{}", encoding="utf-8")
    _require_bash_sees(probe)
    assert _ask(tmp_path / "нет-такого.json") != 0


def test_isolation_anchor_does_not_depend_on_scenario_filename() -> None:
    """Гвоздь в крышку: решение об изоляции не смотрит на имя файла.

    Без bash — чистая проверка текста харнесса, работает в любом окружении.
    """
    src = E2E_SCRIPT.read_text(encoding="utf-8")
    body = _extract_function("scenario_registers_speakers")

    assert "grep" in body and "register_speaker" in body, (
        "якорь обязан смотреть в СОДЕРЖИМОЕ сценария: имя файла назначает "
        "воркфлоу (/tmp/e2e_scenario.json), а не автор сценария"
    )
    assert "night_marathon" not in body, (
        "имя конкретного акта в якоре — это возврат к багу #2763: харнесс "
        "видит только /tmp/e2e_scenario.json, совпадения не будет никогда"
    )
    assert "is_acquaintance_scenario" not in src, (
        "старый матчер по имени файла должен быть удалён целиком, "
        "иначе его снова начнут звать"
    )
