"""Regression tests for ``scripts/setup/reinstall_vision_pi_systemd.sh``.

Issue #3018 — Vision Pi не восстанавливает контейнеры после падения, потому что
на нём остался устаревший ``robbox-vision.service`` без ``Restart=on-failure``
и без health-timer ``robbox-vision-health.timer``. Setup-скрипт
``setup_vision_pi.sh`` это всё умеет ставить, но целиком он делает лишние
шаги (apt install, clone repo, zram-swap), которые не нужны на этапе CI-деплоя.

Helper ``reinstall_vision_pi_systemd.sh`` делает ТОЛЬКО:
  1. ``setup_autostart`` — переустановка robbox-vision.service.
  2. ``setup_health_monitor`` — переустановка robbox-vision-health.timer/service.

Контракт:
  - Скрипт идемпотентен: повторный запуск безопасен.
  - Запускается без sudo-интерактива (на CI sudo подаётся через sshpass + TTY).
  - source'ит setup_vision_pi.sh (вспомогательные функции), но НЕ вызывает main().
"""
from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
HELPER_SCRIPT = REPO_ROOT / "scripts" / "setup" / "reinstall_vision_pi_systemd.sh"
SETUP_SCRIPT = REPO_ROOT / "scripts" / "setup" / "setup_vision_pi.sh"


def _read(p: Path) -> str:
    assert p.exists(), f"missing: {p}"
    return p.read_text(encoding="utf-8")


def test_helper_script_exists_and_is_executable():
    """Helper должен существовать и быть исполняемым (CI запускает напрямую)."""
    assert HELPER_SCRIPT.exists(), f"not found: {HELPER_SCRIPT}"
    mode = HELPER_SCRIPT.stat().st_mode
    assert mode & 0o111, f"helper not executable: mode={oct(mode)}"


def test_helper_sources_setup_vision_pi_without_running_main():
    """Helper должен source'ить setup_vision_pi.sh, но НЕ вызывать main().

    awk-трюк: всё до первой строки '^main() {' (включительно) пропускается,
    дальше — функции. main() не вызывается явно — это предотвращает
    apt install / clone repo на CI-этапе.
    """
    txt = _read(HELPER_SCRIPT)
    # source + awk-фильтр
    assert "source <(awk" in txt, \
        "helper должен source'ить setup_vision_pi.sh через awk-фильтр (исключая main)"
    assert "/^main\\\\(\\\\) \\{/" in txt or "/^main\\(\\) \\{/" in txt, \
        "awk должен резать на строке '^main() {'"
    # main() НЕ должен вызываться явно
    # (если helper случайно вызовет main — мы переустановим ВЕСЬ Pi)
    assert not re.search(r"^\s*main\b", txt, re.MULTILINE), \
        "helper не должен вызывать main() — это полная переустановка Pi"


def test_helper_calls_setup_autostart_and_setup_health_monitor():
    """Helper должен явно вызывать только эти две функции."""
    txt = _read(HELPER_SCRIPT)
    assert "setup_autostart" in txt, "helper должен вызывать setup_autostart"
    assert "setup_health_monitor" in txt, "helper должен вызывать setup_health_monitor"


def test_helper_passes_shellcheck():
    """Helper должен проходить shellcheck на уровне error.

    У CI есть gate: bash-скрипты с shellcheck error-level fail.
    """
    r = subprocess.run(
        ["shellcheck", "--severity=error", str(HELPER_SCRIPT)],
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0, (
        f"shellcheck failed:\n{r.stdout}\n{r.stderr}\n"
        f"(helper: {HELPER_SCRIPT})"
    )


def test_setup_vision_pi_sh_has_setup_autostart_and_setup_health_monitor():
    """Sanity-check: setup_vision_pi.sh всё ещё содержит обе функции.

    Если кто-то их переименовал/удалил — helper сломается.
    """
    txt = _read(SETUP_SCRIPT)
    assert "setup_autostart()" in txt, "setup_vision_pi.sh: нет функции setup_autostart"
    assert "setup_health_monitor()" in txt, "setup_vision_pi.sh: нет функции setup_health_monitor"


def test_helper_does_not_call_apt_install_or_clone_repository():
    """Helper не должен триггерить тяжёлые шаги CI-deploy (apt install, clone).

    Защита от регрессии: если кто-то добавит в helper вызов main() — тест
    укажет на лишние шаги.
    """
    # Разделяем код и комментарии — чтобы комментарий-упоминание не ловилось.
    # Простой способ: убираем все строки-комментарии (#…) и пустые строки,
    # потом ищем запрещённые токены в оставшемся коде.
    code_lines = [
        ln for ln in _read(HELPER_SCRIPT).splitlines()
        if ln.strip() and not ln.lstrip().startswith("#")
    ]
    code_only = "\n".join(code_lines)
    forbidden = ["apt install", "apt-get install", "git clone", "curl -sSL"]
    for f in forbidden:
        assert f not in code_only, \
            f"helper (код, не комментарии) не должен содержать '{f}' "\
            f"(тяжёлый шаг CI-deploy)"


def test_setup_vision_pi_sh_has_main_function_at_end():
    """awk-фильтр в helper режет на строке '^main() {'. Если setup_vision_pi.sh
    переименует/удалит main — фильтр вырежет ВСЁ, и helper ничего не сделает.

    Защита: проверяем, что main() присутствует в setup_vision_pi.sh.
    """
    txt = _read(SETUP_SCRIPT)
    assert re.search(r"^main\s*\(\s*\)\s*\{", txt, re.MULTILINE), \
        "setup_vision_pi.sh должен содержать функцию main() в конце файла"


def test_helper_bash_syntax_is_valid():
    """Helper должен проходить ``bash -n`` (синтаксическая проверка).

    Это простой guard от опечаток; CI запускает настоящий shellcheck на
    уровне error, но bash -n даёт дешёвый первый барьер.
    """
    r = subprocess.run(
        ["bash", "-n", str(HELPER_SCRIPT)],
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0, (
        f"bash -n failed:\n{r.stderr}\n"
        f"(helper: {HELPER_SCRIPT})"
    )


def test_helper_awk_filter_excludes_main_from_source():
    """awk '/^main\\\\(\\\\) \\{/{exit} {print}' должен вырезать строку с main().

    Реальный сценарий: setup_vision_pi.sh источник, awk-фильтр работает.
    Если кто-то сломает regex в awk — main() выполнится на CI и переустановит
    весь Pi (apt install, clone, zram-swap).

    Проверяем в dry-run: запускаем awk и считаем, что main() { в выводе нет.
    """
    awk_script = '/^main\\(\\) \\{/ { exit } { print }'
    r = subprocess.run(
        ["awk", awk_script, str(SETUP_SCRIPT)],
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0
    # В выводе не должно быть main() {
    assert "main() {" not in r.stdout, (
        "awk-фильтр не вырезал main() — helper случайно выполнит полную "
        "переустановку Pi (apt install, clone repo) на CI!"
    )
    # И в выводе должны быть setup_autostart/setup_health_monitor (мы их зовём).
    assert "setup_autostart" in r.stdout, (
        "awk-фильтр вырезал ВСЁ — helper не сможет вызвать setup_autostart"
    )
    assert "setup_health_monitor" in r.stdout, (
        "awk-фильтр вырезал ВСЁ — helper не сможет вызвать setup_health_monitor"
    )
