"""
test_issue_2781_voice_memory_isolation_anchor.py — якорь изоляции voice_facts

Изоляция БД дикторов (#2750, починена в #2763/#2770) накрывает только
``speakers.db``. Долгосрочная память (``voice_facts``, записывается через
MCP-тул ``memory_save``) писалась прямо в боевую ``/data/voice_memory.db``
без всякой изоляции: акт «Знакомство» ночного марафона и регистрирует
голоса (``register_speaker`` — изолировано), и называет LLM факты о себе
через ``memory_save`` (НЕ было изолировано). Замер на Vision Pi 22.09.2026
нашёл 59 из 116 фактов боевой базы, упоминающих синтезированный каст
марафона ("Саша не ест лук", "Борис болеет за Спартак").

Этот файл держит для ``scenario_writes_memory`` те же три инварианта, что
``test_issue_2763_speaker_db_isolation_anchor.py`` держит для
``scenario_registers_speakers`` — якорь по СОДЕРЖИМОМУ сценария, а не по
имени файла (issue #2763: воркфлоу копирует сценарий на билд-машину под
фиксированным ``/tmp/e2e_scenario.json``, имя не несёт информации об
авторе):
  1. якорь смотрит на СОДЕРЖИМОЕ сценария (сценарий вызывает memory_save);
  2. реальный сценарий акта 2 опознаётся под именем, которое ему даёт
     воркфлоу;
  3. сценарий без memory_save изоляции не требует.

Run:
  python -m pytest tests/unit/e2e_scripts/test_issue_2781_voice_memory_isolation_anchor.py -v --no-cov
"""

import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
E2E_SCRIPT = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"
SCENARIO_DIR = REPO_ROOT / ".github" / "e2e" / "scenarios" / "night"

#: Имя, под которым сценарий доезжает до харнесса на билд-машине (issue #2763).
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
    body = _extract_function("scenario_writes_memory")
    script = f'{body}\nscenario_writes_memory "{scenario_path.as_posix()}"'
    return subprocess.run(
        ["bash", "-c", script], capture_output=True, text=True, timeout=10
    ).returncode


def _require_bash_sees(path: Path) -> None:
    """Пропустить тест, если найденный bash не видит путь pytest-tmpdir.

    На Windows ``shutil.which("bash")`` нередко указывает на WSL, которому
    путь вида ``C:/Users/...`` не виден вообще — см. тот же приём в
    ``test_issue_2763_speaker_db_isolation_anchor.py``.
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
    """Акт 2 (memory_save встречается в тексте) опознаётся под именем,
    которое ему даёт воркфлоу — регрессия того же класса инцидента, что
    #2763 (якорь не должен зависеть от имени файла)."""
    src = SCENARIO_DIR / "night_marathon_act2_acquaintance_v1.json"
    assert src.exists(), f"сценарий акта 2 не найден: {src}"
    assert "memory_save" in src.read_text(encoding="utf-8"), (
        "сценарий акта 2 обязан упоминать memory_save — иначе этот тест "
        "не проверяет ничего осмысленного"
    )
    renamed = tmp_path / WORKFLOW_SCENARIO_NAME
    renamed.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
    _require_bash_sees(renamed)

    assert _ask(renamed) == 0, (
        "акт «Знакомство» обязан требовать изоляции долгосрочной памяти даже "
        "после переименования в /tmp/e2e_scenario.json — иначе он пишет "
        "факты memory_save в боевую /data/voice_memory.db (issue #2781)"
    )


@bash_required
def test_scenario_without_memory_save_needs_no_isolation(tmp_path: Path) -> None:
    """Сценарий без memory_save изоляции памяти не требует."""
    src = SCENARIO_DIR / "night_marathon_act1_wakeup_v1.json"
    assert src.exists(), f"сценарий акта 1 не найден: {src}"
    assert "memory_save" not in src.read_text(encoding="utf-8")
    renamed = tmp_path / WORKFLOW_SCENARIO_NAME
    renamed.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
    _require_bash_sees(renamed)

    assert _ask(renamed) != 0


@bash_required
def test_missing_scenario_file_is_not_treated_as_writing_memory(tmp_path: Path) -> None:
    """Пропавший файл — якорь отвечает «нет», решение принимает вызывающий."""
    probe = tmp_path / WORKFLOW_SCENARIO_NAME
    probe.write_text("{}", encoding="utf-8")
    _require_bash_sees(probe)
    assert _ask(tmp_path / "нет-такого.json") != 0


def test_isolation_anchor_does_not_depend_on_scenario_filename() -> None:
    """Гвоздь в крышку: решение об изоляции памяти не смотрит на имя файла.

    Без bash — чистая проверка текста харнесса, работает в любом окружении.
    """
    body = _extract_function("scenario_writes_memory")

    assert "grep" in body and "memory_save" in body, (
        "якорь обязан смотреть в СОДЕРЖИМОЕ сценария: имя файла назначает "
        "воркфлоу (/tmp/e2e_scenario.json), а не автор сценария (issue #2763)"
    )
    assert "night_marathon" not in body, (
        "имя конкретного акта в якоре — это возврат к багу #2763: харнесс "
        "видит только /tmp/e2e_scenario.json, совпадения не будет никогда"
    )


def test_mcp_server_e2e_mode_activation_is_fatal_not_a_warning() -> None:
    """Issue #2781 — отказ включить e2e_mode у mcp_server обязан быть
    ФАТАЛОМ (``E2E_FATAL`` + ``exit``), как и у speaker_id_node
    (``activate_e2e_speaker_db``), а не предупреждением: лучше не прогнать
    акт, чем засорить боевую долгосрочную память."""
    body = _extract_function("activate_e2e_memory_db")

    assert "E2E_FATAL" in body, (
        "activate_e2e_memory_db обязан фейлить прогон фатально при отказе "
        "e2e_mode — предупреждение позволило бы сценарию тихо писать факты "
        "в боевую /data/voice_memory.db"
    )
    assert re.search(r"exit\s+2", body), (
        "activate_e2e_memory_db обязан завершать скрипт (exit 2) при отказе, "
        "по образцу activate_e2e_speaker_db"
    )


def test_deactivate_memory_db_is_wired_into_exit_trap() -> None:
    """Возврат mcp_server на боевую БД обязан происходить при ЛЮБОМ
    завершении прогона (PASS/FAIL/обрыв) — тот же trap EXIT, что уже
    гарантирует это для speaker_id_node."""
    src = E2E_SCRIPT.read_text(encoding="utf-8")
    trap_lines = [line for line in src.splitlines() if line.strip().startswith("trap ")]
    assert any(
        "deactivate_e2e_speaker_db" in line and "deactivate_e2e_memory_db" in line
        for line in trap_lines
    ), (
        "deactivate_e2e_memory_db должен быть в том же `trap ... EXIT`, что "
        "и deactivate_e2e_speaker_db — иначе робот может остаться на "
        "e2e-базе памяти после прерванного прогона"
    )
