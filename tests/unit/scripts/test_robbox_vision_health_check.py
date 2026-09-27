"""Юнит-тесты для scripts/monitoring/robbox_vision_health_check.sh.

Карточка t_5ab5e44a (наблюдаемость и алертинг на failed boot Vision Pi).
Тесты изолированы от реальной машины: подменяем PATH так, чтобы ``docker``
и ``systemctl`` указывали на fakes, а ``-d`` / ``-e`` каталоги BOOT_LOG
и METRICS_FILE — на временные файлы. Так тесты работают и на CI без
Docker / systemd, и локально на builder.

Acceptance покрытие:
  * exit 0 при running > 0
  * exit 0 в grace-периоде (running=0, grace_elapsed < threshold)
  * exit 1 при ALERT (running=0, grace_elapsed >= threshold, service started)
  * exit 0 в режиме inactive (service never started)
  * exit 0 при infra_error (docker daemon лежит)
  * JSON формат валиден и содержит все обязательные поля
  * boot-log + metrics file пишутся при non-dry-run
  * dry-run НЕ пишет на диск
  * shellcheck -S error чисто
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "monitoring" / "robbox_vision_health_check.sh"


# --------------------------------------------------------------------------- #
# Фикстуры
# --------------------------------------------------------------------------- #


@pytest.fixture()
def sandbox(tmp_path: Path) -> "Sandbox":
    """Изолированное окружение: fake PATH, tmp BOOT_LOG / METRICS_FILE / ALERT_LOG.

    Возвращает dataclass-объект с путями и helper'ом для запуска скрипта.
    """
    fake_bin = tmp_path / "fakebin"
    fake_bin.mkdir()
    # fake docker — управляем через JSON-файл в tmp_path
    fake_docker = tmp_path / "fakebin" / "docker"
    fake_systemctl = tmp_path / "fakebin" / "systemctl"
    # date / wc / head оставляем системные (через realpath).

    boot_log = tmp_path / "boot.log"
    metrics_file = tmp_path / "health.prom"
    alert_log = tmp_path / "alerts.log"

    sb = Sandbox(
        tmp_path=tmp_path,
        fake_bin=fake_bin,
        boot_log=boot_log,
        metrics_file=metrics_file,
        alert_log=alert_log,
    )
    return sb


class Sandbox:
    def __init__(self, tmp_path: Path, fake_bin: Path, boot_log: Path,
                 metrics_file: Path, alert_log: Path) -> None:
        self.tmp_path = tmp_path
        self.fake_bin = fake_bin
        self.boot_log = boot_log
        self.metrics_file = metrics_file
        self.alert_log = alert_log

    def install_fake_docker(self, running_names: list[str]) -> Path:
        """Создать fake-``docker``, который ``docker ps --filter status=running --format '{{.Names}}'``
        выводит имена из ``running_names`` (по одному в строке), а
        ``docker ps -a --format '{{.Names}}\\t{{.Status}}\\t{{.State}}'`` —
        те же имена со статусом ``Up 5 minutes``.

        Возвращает путь к fake'у (для дебага)."""
        fake = self.fake_bin / "docker"
        # Скрипт читает имена из env $RUNNING_NAMES (newline-separated) и
        # печатает их. Совпадает с поведением реального `docker ps` — newline
        # между именами, без trailing newline-extra.
        fake.write_text(
            "#!/bin/bash\n"
            "if [ \"$1\" = \"ps\" ]; then\n"
            "  # docker ps --filter status=running --format '{{.Names}}'\n"
            "  if echo \"$*\" | grep -q -- '--filter status=running'; then\n"
            "    if [ -n \"$RUNNING_NAMES\" ]; then\n"
            "      printf '%s\\n' \"$RUNNING_NAMES\"\n"
            "    fi\n"
            "    exit 0\n"
            "  fi\n"
            "  # docker ps -a --format '{{.Names}}\\t{{.Status}}\\t{{.State}}'\n"
            "  # (для boot-summary log). Определяем по флагу -a. Если задан\n"
            "  # $PS_A_OUTPUT — используем его (для кастомных сценариев),\n"
            "  # иначе выводим $RUNNING_NAMES со статусом 'Up 5 minutes'.\n"
            "  if echo \"$*\" | grep -q -- '-a'; then\n"
            "    if [ -n \"$PS_A_OUTPUT\" ]; then\n"
            "      printf '%s\\n' \"$PS_A_OUTPUT\"\n"
            "    elif [ -n \"$RUNNING_NAMES\" ]; then\n"
            "      printf '%s\\n' \"$RUNNING_NAMES\" | \\\n"
            "        while IFS= read -r name; do\n"
            "          [ -z \"$name\" ] && continue\n"
            "          printf '%s\\tUp 5 minutes\\trunning\\n' \"$name\"\n"
            "        done\n"
            "    fi\n"
            "    exit 0\n"
            "  fi\n"
            "fi\n"
            "exit 0\n"
        )
        fake.chmod(0o755)
        # Запомнить список имён в env, который наш run() подсунет.
        # ВАЖНО: newline-separated, не пробелами — иначе `wc -l` в скрипте
        # вернёт 1 даже для пустого списка (а не 0).
        self._running_env = "\n".join(running_names)
        return fake

    def install_fake_docker_fail(self) -> Path:
        """docker, который падает с exit≠0 (имитация daemon down)."""
        fake = self.fake_bin / "docker"
        fake.write_text(
            "#!/bin/bash\n"
            "echo 'Cannot connect to the Docker daemon' >&2\n"
            "exit 1\n"
        )
        fake.chmod(0o755)
        self._running_env = ""
        return fake

    def install_fake_systemctl(self, service_state: str) -> Path:
        """service_state ∈ {'never', 'inactive', 'started', 'notstarted', 'fail'}.

        ``never``    → ActiveEnterTimestamp пустой (юнит ни разу не запущен, no output).
        ``inactive`` → ActiveEnterTimestamp = "n/a" (реальный systemctl show --value
                       на неактивном сервисе; фикс t_5ab5e44a round-2).
        ``started``  → ActiveEnterTimestamp = «30 минут назад» (grace_elapsed > 300).
        ``notstarted`` → systemctl падает с rc≠0 (systemd не установлен).
        ``fail``     → ActiveEnterTimestamp = «30 секунд назад» (grace_elapsed < 300, ещё в grace).
        """
        fake = self.fake_bin / "systemctl"
        if service_state == "never":
            body = "#!/bin/bash\necho ''\nexit 0\n"
        elif service_state == "inactive":
            body = "#!/bin/bash\necho 'n/a'\nexit 0\n"
        elif service_state == "started":
            body = (
                "#!/bin/bash\n"
                "if [ \"$1\" = show ]; then\n"
                "  # ActiveEnterTimestamp = 30 минут назад (ISO-формат чтобы\n"
                "  # избежать локалезависимости `date -d \"Tue ...\"`).\n"
                "  ts=$(date -u -d '-30 minutes' '+%Y-%m-%d %H:%M:%S UTC')\n"
                "  printf 'ActiveEnterTimestamp=%s\\n' \"$ts\"\n"
                "  exit 0\n"
                "fi\n"
            )
        elif service_state == "fail":
            body = (
                "#!/bin/bash\n"
                "if [ \"$1\" = show ]; then\n"
                "  # ActiveEnterTimestamp = 30 секунд назад (внутри grace 300s)\n"
                "  ts=$(date -u -d '-30 seconds' '+%Y-%m-%d %H:%M:%S UTC')\n"
                "  printf 'ActiveEnterTimestamp=%s\\n' \"$ts\"\n"
                "  exit 0\n"
                "fi\n"
            )
        elif service_state == "notstarted":
            body = "#!/bin/bash\nexit 1\n"
        else:
            raise ValueError(service_state)
        fake.write_text(body)
        fake.chmod(0o755)
        return fake

    def run(self, *args: str, extra_env: dict | None = None) -> "Run":
        """Запустить SCRIPT в изолированной среде.

        Подменяем PATH на self.fake_bin + системные bin/sbin, чтобы
        наши fake-бинари нашлись первыми (но docker/systemctl из PATH
        по-прежнему доступны как fallback, если нужно).
        """
        env = os.environ.copy()
        # PATH: сначала фейки, потом /usr/bin:/bin (для date/wc/head).
        env["PATH"] = f"{self.fake_bin}:/usr/bin:/bin"
        env["ROBBOX_VISION_BOOT_LOG"] = str(self.boot_log)
        env["ROBBOX_VISION_METRICS_FILE"] = str(self.metrics_file)
        env["ROBBOX_VISION_ALERT_LOG"] = str(self.alert_log)
        # Подменить HOME, чтобы дефолтные пути textfile не уходили в $HOME.
        env["HOME"] = str(self.tmp_path)
        if hasattr(self, "_running_env"):
            env["RUNNING_NAMES"] = self._running_env
        if extra_env:
            env.update(extra_env)
        proc = subprocess.run(
            ["bash", str(SCRIPT), *args],
            capture_output=True,
            text=True,
            env=env,
            timeout=20,
        )
        return Run(proc.returncode, proc.stdout, proc.stderr)


class Run:
    def __init__(self, rc: int, stdout: str, stderr: str) -> None:
        self.rc = rc
        self.stdout = stdout
        self.stderr = stderr


# --------------------------------------------------------------------------- #
# 1. Структурные тесты (синтаксис, shellcheck)
# --------------------------------------------------------------------------- #


def test_script_exists_and_executable() -> None:
    """SSoT-скрипт существует, executable, проходит ``bash -n``."""
    assert SCRIPT.exists(), f"missing {SCRIPT}"
    assert SCRIPT.stat().st_mode & 0o111, f"{SCRIPT} должен быть executable"
    res = subprocess.run(["bash", "-n", str(SCRIPT)], capture_output=True, text=True)
    assert res.returncode == 0, f"bash -n failed: {res.stderr}"


@pytest.mark.skipif(
    subprocess.run(["which", "shellcheck"], capture_output=True).returncode != 0,
    reason="shellcheck не установлен",
)
def test_script_passes_shellcheck_error_level() -> None:
    """shellcheck -S error не должен находить ошибок."""
    res = subprocess.run(
        ["shellcheck", "-S", "error", str(SCRIPT)],
        capture_output=True, text=True, timeout=30,
    )
    assert res.returncode == 0, (
        f"shellcheck ошибки:\nSTDOUT:\n{res.stdout}\nSTDERR:\n{res.stderr}"
    )


def test_help_flag_exits_zero() -> None:
    """--help выходит с rc=0 и показывает заголовок (не empty)."""
    res = subprocess.run(
        ["bash", str(SCRIPT), "--help"],
        capture_output=True, text=True, timeout=5,
    )
    assert res.returncode == 0
    assert "robbox_vision_health_check" in res.stdout


# --------------------------------------------------------------------------- #
# 2. Поведенческие тесты (verdict / exit code / JSON)
# --------------------------------------------------------------------------- #


def test_ok_when_containers_running(sandbox: Sandbox) -> None:
    """3 running-контейнера, service стартовал давно → verdict=ok, exit=0."""
    sandbox.install_fake_docker(["zenoh-router-vision", "voice-assistant", "supercollider"])
    sandbox.install_fake_systemctl("started")
    run = sandbox.run("--dry-run", "--json")
    assert run.rc == 0, f"expected rc=0 (ok), got {run.rc}\nstderr: {run.stderr}"
    payload = json.loads(run.stdout.strip().splitlines()[-1])
    assert payload["verdict"] == "ok"
    assert payload["running_containers"] == 3
    assert payload["grace_elapsed_seconds"] >= 300


def test_grace_when_zero_running_but_fresh_start(sandbox: Sandbox) -> None:
    """0 контейнеров, но service стартовал 30 секунд назад → grace, exit=0."""
    sandbox.install_fake_docker([])
    sandbox.install_fake_systemctl("fail")  # 30s ago — внутри grace 300s
    run = sandbox.run("--dry-run", "--json")
    assert run.rc == 0, f"expected rc=0 (grace), got {run.rc}\nstderr: {run.stderr}"
    payload = json.loads(run.stdout.strip().splitlines()[-1])
    assert payload["verdict"] == "grace", payload
    assert payload["running_containers"] == 0
    assert payload["grace_elapsed_seconds"] < payload["grace_threshold_seconds"]


def test_alert_when_zero_running_past_grace(sandbox: Sandbox) -> None:
    """0 контейнеров, service стартовал 30 минут назад → ALERT, exit=1."""
    sandbox.install_fake_docker([])
    sandbox.install_fake_systemctl("started")
    run = sandbox.run("--dry-run", "--json")
    assert run.rc == 1, f"expected rc=1 (alert), got {run.rc}\nstderr: {run.stderr}"
    payload = json.loads(run.stdout.strip().splitlines()[-1])
    assert payload["verdict"] == "alert"
    assert payload["running_containers"] == 0
    assert "past grace" in payload["reason"]


def test_alert_suppressed_in_warn_only_mode(sandbox: Sandbox) -> None:
    """С --warn-only даже ALERT даёт rc=0 (для cron'ов, которые шумят)."""
    sandbox.install_fake_docker([])
    sandbox.install_fake_systemctl("started")
    run = sandbox.run("--dry-run", "--json", "--warn-only")
    assert run.rc == 0, f"expected rc=0 (warn-only), got {run.rc}\nstderr: {run.stderr}"
    payload = json.loads(run.stdout.strip().splitlines()[-1])
    assert payload["verdict"] == "alert"  # verdict не меняется


def test_inactive_when_service_never_started(sandbox: Sandbox) -> None:
    """Service ни разу не стартовал (пустой вывод) → verdict=inactive, НЕ алертим.

    Эмулирует случай «systemctl show --value вернул пустую строку» —
    это бывает, когда юнит не зарегистрирован в systemd вовсе.
    """
    sandbox.install_fake_docker([])
    sandbox.install_fake_systemctl("never")
    run = sandbox.run("--dry-run", "--json")
    assert run.rc == 0
    payload = json.loads(run.stdout.strip().splitlines()[-1])
    assert payload["verdict"] == "inactive"
    assert "never started" in payload["reason"]


def test_inactive_when_service_shows_na(sandbox: Sandbox) -> None:
    """systemctl show --value вернул 'n/a' (inactive сервис) → verdict=inactive.

    Реальный bug, найденный round-2 (t_5ab5e44a): на машине без active
    ``robbox-vision.service`` ``systemctl show --property=ActiveEnterTimestamp --value``
    возвращает буквальную строку ``n/a``, а не пустую строку. Скрипт должен
    корректно маппить это в grace_elapsed_seconds=-1, verdict=inactive,
    exit=0 (НЕ алерт: мы не можем отличить «cold boot» от «выключен»).
    """
    sandbox.install_fake_docker([])
    sandbox.install_fake_systemctl("inactive")
    run = sandbox.run("--dry-run", "--json")
    assert run.rc == 0, (
        f"verdict=inactive не должен давать ALERT exit, got rc={run.rc}\n"
        f"stderr: {run.stderr}"
    )
    payload = json.loads(run.stdout.strip().splitlines()[-1])
    assert payload["verdict"] == "inactive"
    assert payload["grace_elapsed_seconds"] == -1
    assert "never started" in payload["reason"]


def test_inactive_suppressed_when_running_containers_present(sandbox: Sandbox) -> None:
    """verdict=inactive + running>0 → не downgrade до inactive, остаётся ok.

    Если на хосте уже крутятся контейнеры (например, мы запустили скрипт
    на builder вне Vision Pi), факт «service не стартовал» не должен
    менять verdict: «есть running» перебивает «service inactive».
    """
    sandbox.install_fake_docker(["zenoh-router-vision"])
    sandbox.install_fake_systemctl("inactive")
    run = sandbox.run("--dry-run", "--json")
    assert run.rc == 0
    payload = json.loads(run.stdout.strip().splitlines()[-1])
    assert payload["verdict"] == "ok", payload
    assert payload["running_containers"] == 1


def test_infra_error_when_docker_down(sandbox: Sandbox) -> None:
    """docker daemon лежит → verdict=infra_error, exit=0 (это не failed-boot)."""
    sandbox.install_fake_docker_fail()
    sandbox.install_fake_systemctl("started")
    run = sandbox.run("--dry-run", "--json")
    assert run.rc == 0, (
        f"docker-инфра проблема не должна алертить как failed-boot, "
        f"got rc={run.rc}\nstderr: {run.stderr}"
    )
    payload = json.loads(run.stdout.strip().splitlines()[-1])
    assert payload["verdict"] == "infra_error"
    assert payload["running_containers"] == -1


# --------------------------------------------------------------------------- #
# 3. Side-effects: dry-run vs реальная запись
# --------------------------------------------------------------------------- #


def test_dry_run_does_not_write_files(sandbox: Sandbox) -> None:
    """--dry-run НЕ создаёт boot-log и metrics file."""
    sandbox.install_fake_docker(["zenoh-router-vision"])
    sandbox.install_fake_systemctl("started")
    run = sandbox.run("--dry-run", "--json")
    assert run.rc == 0
    assert not sandbox.boot_log.exists(), "dry-run не должен писать boot-log"
    assert not sandbox.metrics_file.exists(), "dry-run не должен писать metrics"


def test_real_run_writes_metrics_in_prometheus_format(sandbox: Sandbox) -> None:
    """Без --dry-run: METRICS_FILE имеет валидный Prometheus textfile-format."""
    sandbox.install_fake_docker(["zenoh-router-vision", "voice-assistant"])
    sandbox.install_fake_systemctl("started")
    run = sandbox.run("--json")
    assert run.rc == 0
    assert sandbox.metrics_file.exists(), "metrics file должен быть создан"
    text = sandbox.metrics_file.read_text(encoding="utf-8")
    # Все 4 метрики должны присутствовать с правильными типами
    assert re.search(r"^# HELP robbox_vision_running_containers \S", text, re.MULTILINE)
    assert re.search(r"^# TYPE robbox_vision_running_containers gauge", text, re.MULTILINE)
    assert re.search(r"^robbox_vision_running_containers \d+", text, re.MULTILINE)
    assert re.search(r"^robbox_vision_grace_elapsed_seconds -?\d+", text, re.MULTILINE)
    assert re.search(r"^robbox_vision_health_alert [01]", text, re.MULTILINE)
    assert re.search(r"^robbox_vision_health_check_timestamp_seconds \d+", text, re.MULTILINE)


def test_real_run_appends_to_boot_log(sandbox: Sandbox) -> None:
    """Без --dry-run: BOOT_LOG получает summary с маркером времени."""
    sandbox.install_fake_docker(["zenoh-router-vision"])
    sandbox.install_fake_systemctl("started")
    run = sandbox.run()
    assert run.rc == 0
    assert sandbox.boot_log.exists(), "boot log должен быть создан"
    text = sandbox.boot_log.read_text(encoding="utf-8")
    assert "robbox-vision boot summary" in text
    assert "running_containers=1" in text
    # Заголовок секции контейнеров пишется всегда (даже если список пустой)
    assert "container list" in text


def test_real_run_writes_alert_on_failure(sandbox: Sandbox) -> None:
    """При ALERT в ALERT_LOG пишется строка с running_containers=0."""
    sandbox.install_fake_docker([])
    sandbox.install_fake_systemctl("started")
    run = sandbox.run()
    assert run.rc == 1
    text = sandbox.alert_log.read_text(encoding="utf-8")
    assert "ALERT" in text
    assert "running_containers=0" in text


# --------------------------------------------------------------------------- #
# 4. JSON output
# --------------------------------------------------------------------------- #


def test_json_payload_has_all_required_fields(sandbox: Sandbox) -> None:
    """JSON содержит timestamp, running, grace, threshold, verdict, reason."""
    sandbox.install_fake_docker(["zenoh-router-vision"])
    sandbox.install_fake_systemctl("started")
    run = sandbox.run("--dry-run", "--json")
    payload = json.loads(run.stdout.strip().splitlines()[-1])
    required = {
        "timestamp", "running_containers", "grace_elapsed_seconds",
        "grace_threshold_seconds", "verdict", "reason",
    }
    missing = required - payload.keys()
    assert not missing, f"missing keys: {missing}; payload={payload}"


def test_unknown_flag_exits_with_code_2(sandbox: Sandbox) -> None:
    """Неизвестный флаг → rc=2 (usage error), как сказано в шапке."""
    sandbox.install_fake_docker([])
    sandbox.install_fake_systemctl("never")
    run = sandbox.run("--bogus-flag")
    assert run.rc == 2, f"expected rc=2, got {run.rc}"
    assert "Unknown arg" in run.stderr
