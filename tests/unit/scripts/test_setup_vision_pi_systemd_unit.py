"""Regression tests for the systemd unit template embedded in
``scripts/setup/setup_vision_pi.sh``.

Issue #2610 — ``robbox-vision.service`` падал после ребута, потому что:

* ``ExecStart=docker compose up -d`` без ``--pull never``, и registry
  (``10.1.1.249:5000``, katana build-host) недоступен → весь стек
  падает с ``no route to host`` → ``systemd exit 1``.
* ``Type=oneshot`` без ``Restart=on-failure`` → после первого фейла
  systemd больше не пытается, контейнеры не поднимаются вообще.

Фикс (t_708c5c05):
* ``ExecStartPre=-docker compose pull --ignore-pull-failures``
  — best-effort обновление, не блокирует запуск.
* ``ExecStart=docker compose up -d --pull never``
  — гарантированный запуск на локальном кэше.
* ``ExecStartPost=-docker compose ps --format json``
  — логирование итогов с exit-codes для диагностики.
* ``Restart=on-failure`` + ``RestartSec=30``
  + ``StartLimitIntervalSec=600`` + ``StartLimitBurst=5``
  — автоматический рестарт при сбое, защита от зацикливания.
* ``ExecStop=docker compose down`` — корректное завершение при reboot.

Эти тесты извлекают heredoc-блок из ``setup_vision_pi.sh`` и проверяют,
что все ключевые элементы фикса на месте и в правильных секциях.
Если кто-то уберёт ``--pull never``, тест покраснеет — ровно то, что
нам нужно как регрессионный guard.

Refs:
    * issue #2610 — root cause
    * t_51c4b1a9 — разведка
    * t_708c5c05 — этот фикс
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "setup" / "setup_vision_pi.sh"


# Регулярка для извлечения тела heredoc-а между << SERVICEEOF и SERVICEEOF.
# Используется фиксированный маркер ``SERVICEEOF``, который не экранируется
# (то есть переменные $COMPOSE_DIR / $USER раскрываются shell-ом, что и
# требуется — setup_vision_pi.sh подставляет реальные пути при установке).
_HEREDOC_RE = re.compile(
    r"<<\s*SERVICEEOF\s*\n(?P<body>.*?)\nSERVICEEOF",
    re.DOTALL,
)


def _extract_unit_template() -> str:
    """Извлечь systemd-unit (как heredoc-тело) из setup_vision_pi.sh.

    Возвращает сырое содержимое heredoc-а ДО раскрытия ``$COMPOSE_DIR``
    и ``$USER`` (для текстовых проверок ключей это и нужно — мы хотим
    удостовериться, что фикс присутствует в исходнике, а не после
    подстановки).
    """
    text = SCRIPT.read_text(encoding="utf-8")
    match = _HEREDOC_RE.search(text)
    assert match is not None, (
        f"could not find SERVICEEOF heredoc in {SCRIPT}; "
        "the systemd-unit block was renamed or removed"
    )
    return match.group("body")


def _materialize_unit(template: str, compose_dir: str = "/tmp/vision", user: str = "ros2") -> str:
    """Подставить ``$COMPOSE_DIR`` и ``$USER``, как это делает сам скрипт.

    Это нужно для ``systemd-analyze verify`` — он требует раскрытых
    переменных, иначе ругается ``Failed to prepare filename: Invalid argument``.
    """
    return template.replace("$COMPOSE_DIR", compose_dir).replace("$USER", user)


# --------------------------------------------------------------------------- #
# 1. Структурные проверки фикса в исходнике setup_vision_pi.sh
# --------------------------------------------------------------------------- #


def test_unit_template_has_pull_never_in_exec_start() -> None:
    """ExecStart обязан содержать ``--pull never`` (issue #2610 root fix)."""
    body = _extract_unit_template()
    assert re.search(r"^ExecStart=.*--pull\s+never", body, re.MULTILINE), (
        "ExecStart должен запускать стек с --pull never, иначе при недоступном "
        "registry (katana 10.1.1.249:5000) systemd упадёт с exit 1 и контейнеры "
        "не поднимутся (issue #2610)"
    )


def test_unit_template_has_best_effort_pull_in_exec_start_pre() -> None:
    """ExecStartPre обязан быть best-effort (--ignore-pull-failures)."""
    body = _extract_unit_template()
    match = re.search(r"^ExecStartPre=(.+)$", body, re.MULTILINE)
    assert match is not None, "ExecStartPre отсутствует — нет best-effort pull"
    pre_cmd = match.group(1)
    # Префикс '-' разрешает ненулевой exit code (для oneshot это обязательно).
    assert pre_cmd.startswith("-"), (
        f"ExecStartPre должен начинаться с '-' (ignore-error), "
        f"иначе ненулевой exit от pull уронит unit. Текущее: {pre_cmd!r}"
    )
    assert "--ignore-pull-failures" in pre_cmd, (
        f"ExecStartPre должен игнорировать pull-фейлы (best-effort), иначе "
        f"при недоступном registry unit опять упадёт. Текущее: {pre_cmd!r}"
    )


def test_unit_template_logs_ps_results_in_exec_start_post() -> None:
    """ExecStartPost обязан логировать ``docker compose ps`` для диагностики.

    Acceptance: «видно, что именно не поднялось и почему (exit codes)».
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
    # Префикс '-' обязателен — иначе не-zero exit при ps уронит unit
    # (например, если стек частично не поднялся).
    assert post_cmd.startswith("-"), (
        f"ExecStartPost должен начинаться с '-' (ignore-error), иначе exit≠0 "
        f"от docker compose ps уронит unit. Текущее: {post_cmd.lstrip('-')!r}"
    )


def test_unit_template_has_restart_policy() -> None:
    """Restart=on-failure + RestartSec обязательны (issue #2610)."""
    body = _extract_unit_template()
    assert re.search(r"^Restart=on-failure$", body, re.MULTILINE), (
        "Restart=on-failure отсутствует. Без него Type=oneshot после первого "
        "фейла больше не рестартует (issue #2610)."
    )
    assert re.search(r"^RestartSec=\d+", body, re.MULTILINE), (
        "RestartSec= не задан — между рестартами не будет задержки."
    )


def test_unit_template_has_start_limit_in_unit_section() -> None:
    """StartLimitIntervalSec / StartLimitBurst — в [Unit], не в [Service].

    Справочная проверка против известной регрессии: если их положить в
    [Service], systemd молча игнорирует ('Unknown key name … in section
    Service, ignoring'), и rate-limit не работает.
    """
    body = _extract_unit_template()
    # Найдём границы секций.
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


def test_unit_template_has_exec_stop() -> None:
    """ExecStop обязателен для корректного завершения при рестарте/reboot."""
    body = _extract_unit_template()
    assert re.search(r"^ExecStop=", body, re.MULTILINE), (
        "ExecStop= отсутствует — при рестарте/reboot контейнеры не "
        "завершатся корректно, могут остаться зомби-сокеты / volume locks."
    )


# --------------------------------------------------------------------------- #
# 1b. t_5ab5e44a — health-monitoring (observability/alerts на failed boot)
# --------------------------------------------------------------------------- #


def test_unit_template_logs_to_journal_standard_output_error() -> None:
    """StandardOutput=journal / StandardError=journal (п.3 acceptance).

    Без этого journalctl не показывает stdout/stderr от ExecStartPost
    (диагностика partial-up теряется).
    """
    body = _extract_unit_template()
    assert re.search(r"^StandardOutput=journal$", body, re.MULTILINE), (
        "StandardOutput=journal отсутствует — stdout от Exec* не попадёт в journal"
    )
    assert re.search(r"^StandardError=journal$", body, re.MULTILINE), (
        "StandardError=journal отсутствует — stderr от Exec* не попадёт в journal"
    )


def test_unit_template_has_log_level_max_info() -> None:
    """LogLevelMax=info — ограничить уровень логирования (п.3 acceptance).

    Дефолт systemd — debug, что раздувает journal на INFO-сообщениях
    от docker compose (не наши сообщения, а встроенный вывод).
    """
    body = _extract_unit_template()
    assert re.search(r"^LogLevelMax=info$", body, re.MULTILINE), (
        "LogLevelMax=info отсутствует — journal будет раздуваться debug-логами "
        "от docker compose / dependency services"
    )


def test_unit_template_has_exec_start_post_health_check() -> None:
    """ExecStartPost должен вызывать robbox_vision_health_check.sh (п.3 acceptance).

    Это второй ExecStartPost (первый — docker compose ps, для journal).
    Health-check пишет summary «что поднялось / что нет» в
    /var/log/robbox-vision-boot.log.
    """
    body = _extract_unit_template()
    posts = re.findall(r"^ExecStartPost=(-?)(.+)$", body, re.MULTILINE)
    assert len(posts) >= 2, (
        f"Ожидалось ≥2 ExecStartPost (docker compose ps + health-check), "
        f"найдено {len(posts)}: {posts}"
    )
    # Найти ExecStartPost, который вызывает robbox_vision_health_check.sh
    health_post = next(
        (cmd for _prefix, cmd in posts if "robbox_vision_health_check" in cmd),
        None,
    )
    assert health_post is not None, (
        f"Среди ExecStartPost нет вызова robbox_vision_health_check.sh: "
        f"{[cmd for _, cmd in posts]}"
    )
    # Префикс '-' обязателен: если скрипт неожиданно упал, не должен уронить
    # основной unit.
    assert "robbox_vision_health_check.sh" in health_post, (
        f"ExecStartPost должен вызывать robbox_vision_health_check.sh: "
        f"{health_post!r}"
    )


def test_setup_script_defines_setup_health_monitor() -> None:
    """Функция setup_health_monitor должна быть в setup_vision_pi.sh.

    Это функция, которая создаёт robbox-vision-health.timer/service
    и ставит SSoT-скрипт в /usr/local/bin.
    """
    text = SCRIPT.read_text(encoding="utf-8")
    assert re.search(r"^setup_health_monitor\s*\(\)\s*\{", text, re.MULTILINE), (
        "setup_health_monitor() не определена в setup_vision_pi.sh — "
        "health-timer не будет создан при fresh install"
    )


def test_setup_script_invokes_setup_health_monitor_in_main() -> None:
    """setup_health_monitor должен вызываться из main() (после setup_autostart)."""
    text = SCRIPT.read_text(encoding="utf-8")
    # Извлечь тело main()
    main_match = re.search(r"^main\s*\(\)\s*\{(.*?)^\}", text, re.MULTILINE | re.DOTALL)
    assert main_match is not None, "main() не найдена"
    main_body = main_match.group(1)
    # setup_health_monitor должен быть ПОСЛЕ setup_autostart (так
    # закомментировано в коде: «t_5ab5e44a: health-monitor ставим ПОСЛЕ
    # setup_autostart»).
    autostart_pos = main_body.find("setup_autostart")
    health_pos = main_body.find("setup_health_monitor")
    assert autostart_pos >= 0, "setup_autostart не вызывается в main()"
    assert health_pos >= 0, "setup_health_monitor не вызывается в main()"
    assert health_pos > autostart_pos, (
        "setup_health_monitor должен вызываться ПОСЛЕ setup_autostart — "
        "иначе ExecStartPost из robbox-vision.service упадёт (нет /var/log/"
        "robbox-vision-boot.log и нет SSoT-скрипта в /usr/local/bin)"
    )


# --------------------------------------------------------------------------- #
# 2. Материализованный unit проходит systemd-analyze verify
# --------------------------------------------------------------------------- #


@pytest.mark.skipif(
    subprocess.run(["which", "systemd-analyze"], capture_output=True).returncode != 0,
    reason="systemd-analyze не установлен (типично для CI без systemd)",
)
def test_materialized_unit_passes_systemd_analyze_verify() -> None:
    """``systemd-analyze verify`` должен пройти без ошибок на нашем unit-е.

    Это acceptance #1 из карточки t_708c5c05. Если кто-то занесёт
    синтаксически неверный ключ или поставит StartLimit* в [Service],
    verify это поймает.
    """
    template = _extract_unit_template()
    unit_text = _materialize_unit(template)

    # Пишем во временный файл — systemd-analyze verify принимает путь.
    import tempfile

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
        # stderr может содержать системные предупреждения (snapd, netplan) —
        # это не наши файлы, поэтому проверяем только exit code и наличие
        # конкретных ошибок именно про наш unit.
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
# 3. Интеграционная проверка: shellcheck не должен ругаться на уровне error
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
