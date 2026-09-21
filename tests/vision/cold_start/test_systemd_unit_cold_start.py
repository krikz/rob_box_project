"""Regression tests for ``robbox-vision.service`` systemd unit (cold-start contract).

Issue #2610 + PR #2635 — после ребута Vision Pi systemd-юнит
``robbox-vision.service`` ОБЯЗАН:

1. Поднять весь compose-стек с локального кэша (``--pull never``),
   даже если katana build-registry offline.
2. Best-effort pre-pull (``--ignore-pull-failures``), чтобы при
   доступном registry сначала обновить локальный кэш, а потом стартовать.
3. Не падать на ``docker compose ps`` post-action — там нужен
   ``-`` префикс, чтобы диагностический вывод не валил unit.
4. Рестартовать при сбое (``Restart=on-failure`` + ``RestartSec``
   + ``StartLimitIntervalSec`` + ``StartLimitBurst``), но не
   зацикливаться.
5. Корректно завершаться (``ExecStop=docker compose down``) при
   ``systemctl stop`` или reboot.

Эти тесты жёстко привязаны к heredoc-блоку ``<< SERVICEEOF`` внутри
``scripts/setup/setup_vision_pi.sh``. Если кто-то уберёт один из
этих ключей (например, вернёт ``--pull always`` или уберёт
``Restart=on-failure``) — тесты красные.

Refs:
    * issue #2610 — корневая проблема
    * PR #2635 — фикс юнита
    * ADR-0111 — voice-resources как image-based init
"""
from __future__ import annotations

import re
import subprocess
import tempfile
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "setup" / "setup_vision_pi.sh"


# --------------------------------------------------------------------------- #
# Извлечение heredoc-тела
# --------------------------------------------------------------------------- #


_HEREDOC_RE = re.compile(
    r"<<\s*SERVICEEOF\s*\n(?P<body>.*?)\nSERVICEEOF",
    re.DOTALL,
)


def _extract_unit_template() -> str:
    """Извлечь systemd-unit (как heredoc-тело) из setup_vision_pi.sh.

    Используется тот же подход, что и в
    ``tests/unit/scripts/test_setup_vision_pi_systemd_unit.py`` —
    фиксированный маркер ``SERVICEEOF``.
    """
    text = SCRIPT.read_text(encoding="utf-8")
    match = _HEREDOC_RE.search(text)
    assert match is not None, (
        f"could not find SERVICEEOF heredoc in {SCRIPT}; "
        "the systemd-unit block was renamed or removed"
    )
    return match.group("body")


def _materialize_unit(template: str, compose_dir: str = "/tmp/vision", user: str = "ros2") -> str:
    """Подставить ``$COMPOSE_DIR`` и ``$USER``, как это делает сам скрипт."""
    return template.replace("$COMPOSE_DIR", compose_dir).replace("$USER", user)


# --------------------------------------------------------------------------- #
# 1. ExecStart: --pull never — обязателен
# --------------------------------------------------------------------------- #


def test_exec_start_uses_pull_never() -> None:
    """``ExecStart`` обязан использовать ``--pull never``.

    Без этого флага compose v2 при ``up -d`` попытается pull'нуть
    образ из registry. Если registry offline — стек зависает на
    pull-attempt и systemd падает. См. PR #2635 acceptance #1.
    """
    body = _extract_unit_template()
    assert re.search(r"^ExecStart=.*--pull\s+never", body, re.MULTILINE), (
        "ExecStart должен запускать стек с --pull never, иначе при "
        "недоступном registry (katana 10.1.1.249:5000) systemd упадёт "
        "с exit 1 и контейнеры не поднимутся (issue #2610)."
    )


def test_exec_start_does_not_use_pull_always() -> None:
    """Анти-регрессия: ``ExecStart`` НЕ должен использовать ``--pull always``.

    ``--pull always`` форсит pull при каждом старте → cold-start
    после reboot обречён на failure при offline registry.
    """
    body = _extract_unit_template()
    match = re.search(r"^ExecStart=(.+)$", body, re.MULTILINE)
    assert match is not None, "ExecStart отсутствует"
    exec_start = match.group(1)
    assert "--pull always" not in exec_start, (
        f"ExecStart содержит --pull always — регрессия cold-start. "
        f"Текущее: {exec_start!r}"
    )


# --------------------------------------------------------------------------- #
# 2. ExecStartPre: best-effort pull с --ignore-pull-failures
# --------------------------------------------------------------------------- #


def test_exec_start_pre_is_best_effort_pull() -> None:
    """``ExecStartPre`` обязан быть best-effort (``-`` prefix + ``--ignore-pull-failures``).

    Если registry доступен — pre-pull обновит локальный кэш. Если
    offline — не должен валить unit (Type=oneshot упадёт, а с ним
    весь стек не поднимется).
    """
    body = _extract_unit_template()
    match = re.search(r"^ExecStartPre=(.+)$", body, re.MULTILINE)
    assert match is not None, "ExecStartPre отсутствует — нет best-effort pull"
    pre_cmd = match.group(1)
    assert pre_cmd.startswith("-"), (
        f"ExecStartPre должен начинаться с '-' (ignore-error), "
        f"иначе ненулевой exit от pull уронит unit. Текущее: {pre_cmd!r}"
    )
    assert "pull" in pre_cmd.lower(), (
        f"ExecStartPre должен вызывать 'docker compose pull'. Текущее: {pre_cmd!r}"
    )
    assert "--ignore-pull-failures" in pre_cmd, (
        f"ExecStartPre должен игнорировать pull-фейлы (best-effort), иначе "
        f"при недоступном registry unit опять упадёт. Текущее: {pre_cmd!r}"
    )


# --------------------------------------------------------------------------- #
# 3. ExecStartPost: docker compose ps (диагностика) с ignore-error
# --------------------------------------------------------------------------- #


def test_exec_start_post_logs_ps_for_diagnostics() -> None:
    """``ExecStartPost`` обязан логировать ``docker compose ps``.

    Это acceptance #4 карточки t_708c5c05 — без post-action невозможно
    увидеть, что именно не поднялось и почему (exit codes). С
    ``-`` префиксом exit≠0 от ps не валит unit (Type=oneshot).
    """
    body = _extract_unit_template()
    match = re.search(r"^ExecStartPost=(.+)$", body, re.MULTILINE)
    assert match is not None, (
        "ExecStartPost отсутствует — нет логирования итогов docker compose ps. "
        "Без этого непонятно, какие именно сервисы упали и почему."
    )
    post_cmd = match.group(1)
    assert "docker compose ps" in post_cmd, (
        f"ExecStartPost должен запускать docker compose ps. Текущее: {post_cmd!r}"
    )
    assert "--format" in post_cmd, (
        f"ExecStartPost должен использовать --format (json) для парсимого "
        f"лога. Текущее: {post_cmd!r}"
    )
    assert post_cmd.startswith("-"), (
        f"ExecStartPost должен начинаться с '-' (ignore-error), иначе exit≠0 "
        f"от docker compose ps уронит unit. Текущее: {post_cmd.lstrip('-')!r}"
    )


# --------------------------------------------------------------------------- #
# 4. Restart=on-failure + RestartSec + rate limit
# --------------------------------------------------------------------------- #


def test_restart_policy_present() -> None:
    """``Restart=on-failure`` + ``RestartSec=`` обязательны (issue #2610 root fix)."""
    body = _extract_unit_template()
    assert re.search(r"^Restart=on-failure$", body, re.MULTILINE), (
        "Restart=on-failure отсутствует. Без него Type=oneshot после "
        "первого фейла больше не рестартует (issue #2610)."
    )
    match = re.search(r"^RestartSec=(\d+)", body, re.MULTILINE)
    assert match is not None, "RestartSec= не задан — нет задержки между рестартами"
    sec = int(match.group(1))
    # 5-120s — разумный диапазон. Меньше 5s — слишком агрессивно
    # (compose pull retry storm); больше 120s — слишком долго ждать
    # recovery после сбоя.
    assert 5 <= sec <= 120, (
        f"RestartSec={sec} вне разумного диапазона [5, 120]. "
        f"Слишком часто — retry storm; слишком редко — долго ждать recovery."
    )


def test_start_limit_keys_in_unit_section_not_service() -> None:
    """``StartLimitIntervalSec`` / ``StartLimitBurst`` — в ``[Unit]``, не в ``[Service]``.

    Справочная проверка против известной регрессии: если их положить
    в ``[Service]``, systemd молча игнорирует ("Unknown key name … in
    section Service, ignoring"), и rate-limit не работает → infinite
    restart loop при хронически сломанном registry.
    """
    body = _extract_unit_template()
    unit_section = re.search(r"\[Unit\](.*?)(?=\n\[)", body, re.DOTALL)
    service_section = re.search(r"\[Service\](.*?)(?=\n\[)", body, re.DOTALL)
    assert unit_section is not None, "нет секции [Unit]"
    assert service_section is not None, "нет секции [Service]"

    unit_text = unit_section.group(1)
    service_text = service_section.group(1)

    assert re.search(r"^StartLimitIntervalSec=\d+", unit_text, re.MULTILINE), (
        "StartLimitIntervalSec= должен быть в [Unit] (не в [Service], "
        "иначе systemd молча игнорирует)"
    )
    assert re.search(r"^StartLimitBurst=\d+", unit_text, re.MULTILINE), (
        "StartLimitBurst= должен быть в [Unit] (не в [Service])"
    )
    # Анти-регрессия: убедимся, что их нет в [Service].
    assert "StartLimitIntervalSec" not in service_text, (
        "StartLimitIntervalSec найден в [Service] — это регрессия! "
        "Должен быть только в [Unit]."
    )
    assert "StartLimitBurst" not in service_text, (
        "StartLimitBurst найден в [Service] — это регрессия! "
        "Должен быть только в [Unit]."
    )


def test_start_limit_values_are_sane() -> None:
    """``StartLimitIntervalSec`` / ``StartLimitBurst`` — в разумных пределах.

    Burst > 10 за минуту — это слишком агрессивно (compose каждый раз
    пытается pull'нуть 17 образов, что даже при online registry
    занимает ~минуту). Burst < 2 — systemd не даст восстановиться
    при транзиентном сбое.
    """
    body = _extract_unit_template()
    interval = re.search(r"^StartLimitIntervalSec=(\d+)", body, re.MULTILINE)
    burst = re.search(r"^StartLimitBurst=(\d+)", body, re.MULTILINE)
    assert interval is not None, "StartLimitIntervalSec= не задан"
    assert burst is not None, "StartLimitBurst= не задан"

    interval_val = int(interval.group(1))
    burst_val = int(burst.group(1))

    assert 60 <= interval_val <= 3600, (
        f"StartLimitIntervalSec={interval_val} вне диапазона "
        f"[60, 3600]. Меньше минуты — слишком агрессивно для compose."
    )
    assert 2 <= burst_val <= 10, (
        f"StartLimitBurst={burst_val} вне диапазона [2, 10]. "
        f"Меньше 2 — нет места на recovery; больше 10 — retry storm."
    )


# --------------------------------------------------------------------------- #
# 5. ExecStop — корректное завершение при рестарте/reboot
# --------------------------------------------------------------------------- #


def test_exec_stop_present() -> None:
    """``ExecStop=`` обязателен — иначе при рестарте/reboot контейнеры не завершатся корректно."""
    body = _extract_unit_template()
    match = re.search(r"^ExecStop=(.+)$", body, re.MULTILINE)
    assert match is not None, (
        "ExecStop= отсутствует — при рестарте/reboot контейнеры не "
        "завершатся корректно, могут остаться зомби-сокеты / volume locks."
    )
    stop_cmd = match.group(1)
    assert "docker compose" in stop_cmd and "down" in stop_cmd, (
        f"ExecStop должен вызывать 'docker compose down'. Текущее: {stop_cmd!r}"
    )


# --------------------------------------------------------------------------- #
# 6. systemd-analyze verify (материализованный unit)
# --------------------------------------------------------------------------- #


@pytest.mark.skipif(
    subprocess.run(["which", "systemd-analyze"], capture_output=True).returncode != 0,
    reason="systemd-analyze не установлен (типично для CI без systemd)",
)
def test_materialized_unit_passes_systemd_analyze_verify() -> None:
    """``systemd-analyze verify`` должен пройти без ошибок на нашем unit-е.

    Если кто-то занесёт синтаксически неверный ключ или поставит
    ``StartLimit*`` в ``[Service]`` — verify это поймает.
    """
    template = _extract_unit_template()
    unit_text = _materialize_unit(template)

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".service", delete=False, encoding="utf-8"
    ) as f:
        f.write(unit_text)
        path = f.name

    try:
        result = subprocess.run(
            ["systemd-analyze", "verify", path],
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert result.returncode == 0, (
            f"systemd-analyze verify завершился с rc={result.returncode}.\n"
            f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )
        # Анти-регрессия: убедимся, что systemd НЕ игнорировал наши ключи.
        # 'Unknown key name ... in section Service' — типичный симптом
        # регрессии StartLimit* в [Service].
        assert "Unknown key name" not in result.stderr or "robbox-vision" not in result.stderr, (
            f"systemd проигнорировал ключ в нашем unit-е:\n{result.stderr}"
        )
    finally:
        Path(path).unlink(missing_ok=True)


# --------------------------------------------------------------------------- #
# 7. shellcheck -S error на setup_vision_pi.sh
# --------------------------------------------------------------------------- #


@pytest.mark.skipif(
    subprocess.run(["which", "shellcheck"], capture_output=True).returncode != 0,
    reason="shellcheck не установлен",
)
def test_setup_vision_pi_sh_passes_shellcheck_error_level() -> None:
    """shellcheck -S error не должен находить ошибок в setup_vision_pi.sh."""
    result = subprocess.run(
        ["shellcheck", "-S", "error", str(SCRIPT)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, (
        f"shellcheck нашёл ошибки в {SCRIPT}:\n"
        f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )
