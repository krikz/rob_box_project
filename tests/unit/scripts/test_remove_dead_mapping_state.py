"""Фолбэк удаления мёртвого mapping_state.json (#5045).

На Main Pi файл root:root, sudo -n просит пароль, обычный rm получает
Permission denied (run 44948634286, job 136981418751, step 30) — и шаг
валил деплой. Приказ Шифу 22.11: мёртвый файл не критичен, шаг не должен
падать, но и молча съедать отказ нельзя. Скрипт
``scripts/deploy/remove_dead_mapping_state.sh`` пробует rm, затем
``sudo -n rm`` (пароль не спрашивает), затем docker (exec rtabmap, если
inspect показывает /maps на этот каталог, иначе docker run из локального
кэша, без pull). exit 0 во всех ветках; если файл остался — строка
«файл остался».

Docker и sudo здесь не настоящие: фейки в PATH. Полный разбор веток —
bash-регресс ``scripts/deploy/tests/test_remove_dead_mapping_state.sh``,
его гоняет ``test_bash_regression_passes``.
"""

from __future__ import annotations

import os
import stat
import subprocess
import textwrap
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "deploy" / "remove_dead_mapping_state.sh"
BASH_REGRESSION = (
    REPO_ROOT / "scripts" / "deploy" / "tests" / "test_remove_dead_mapping_state.sh"
)

FAKE_RM = textwrap.dedent(
    """\
    #!/usr/bin/env bash
    # Фейк rm: -f <путь>. Файл .rm-deny рядом — Permission denied, не трогаем.
    set -u
    target=""
    for arg in "$@"; do
        case "$arg" in
            -*) ;;
            *) target="$arg" ;;
        esac
    done
    dir="$(dirname "$target")"
    if [ -n "$target" ] && [ -f "$dir/.rm-deny" ]; then
        echo "rm: cannot remove '$target': Permission denied" >&2
        exit 1
    fi
    if [ -n "$target" ]; then
        /bin/rm -f -- "$target"
    fi
    exit 0
    """
)

FAKE_SUDO = textwrap.dedent(
    """\
    #!/usr/bin/env bash
    # Фейк sudo -n rm. Файл .sudo-deny рядом — «a password is required».
    set -u
    if [ "${1:-}" != "-n" ]; then
        echo "sudo: a password is required" >&2
        exit 1
    fi
    shift
    target=""
    for arg in "$@"; do
        case "$arg" in
            -*) ;;
            *) target="$arg" ;;
        esac
    done
    dir="$(dirname "$target")"
    if [ -n "$target" ] && [ -f "$dir/.sudo-deny" ]; then
        echo "sudo: a password is required" >&2
        exit 1
    fi
    if [ -n "$target" ]; then
        /bin/rm -f -- "$target"
    fi
    exit 0
    """
)

FAKE_DOCKER = textwrap.dedent(
    """\
    #!/usr/bin/env bash
    # Фейк docker. Сценарий — $FAKE_SCENARIO (key=value), argv — в $FAKE_DOCKER_LOG.
    set -u
    log="${FAKE_DOCKER_LOG:?}"
    printf '%s\\n' "$*" >> "$log"
    scenario="${FAKE_SCENARIO:?}"
    val() { sed -n "s/^$1=//p" "$scenario" | head -1; }

    cmd="${1:-}"
    shift || true
    case "$cmd" in
        inspect)
            fmt=""
            while [ $# -gt 0 ]; do
                case "$1" in
                    -f) fmt="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done
            case "$fmt" in
                *State.Running*) printf '%s\\n' "$(val running)" ;;
                *Destination*)
                    src="$(val source)"
                    if [ -n "$src" ]; then
                        printf '/config\\n/config-src\\n/maps\\n%s\\n' "$src"
                    fi
                    ;;
                *Config.Image*)
                    img="$(val image)"
                    if [ -n "$img" ]; then
                        printf '%s\\n' "$img"
                    fi
                    ;;
                *) exit 1 ;;
            esac
            exit 0
            ;;
        image)
            sub="${1:-}"
            shift || true
            if [ "$sub" = "inspect" ]; then
                if [ "$(val image_present)" = "yes" ]; then
                    exit 0
                fi
                echo "Error: No such image" >&2
                exit 1
            fi
            exit 1
            ;;
        images)
            line="$(val images_line)"
            if [ -n "$line" ]; then
                printf '%s\\n' "$line"
            fi
            exit 0
            ;;
        exec)
            if [ "$(val exec_removes)" = "yes" ]; then
                /bin/rm -f -- "$(val host_target)"
            fi
            exit 0
            ;;
        run)
            if [ "$(val run_removes)" = "yes" ]; then
                /bin/rm -f -- "$(val host_target)"
            fi
            exit 0
            ;;
        *)
            echo "fake docker: неизвестная команда $cmd" >&2
            exit 1
            ;;
    esac
    """
)


def _write_exe(path: Path, body: str) -> None:
    path.write_text(body)
    path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def _scenario(path: Path, **fields: str) -> None:
    path.write_text("".join(f"{key}={value}\n" for key, value in fields.items()))


def _run(
    tmp_path: Path, maps: Path, scenario: Path, log: Path
) -> subprocess.CompletedProcess[str]:
    fake = tmp_path / "bin"
    fake.mkdir()
    _write_exe(fake / "rm", FAKE_RM)
    _write_exe(fake / "sudo", FAKE_SUDO)
    _write_exe(fake / "docker", FAKE_DOCKER)
    env = os.environ.copy()
    env["PATH"] = f"{fake}{os.pathsep}{env.get('PATH', '')}"
    env["FAKE_SCENARIO"] = str(scenario)
    env["FAKE_DOCKER_LOG"] = str(log)
    return subprocess.run(
        ["bash", str(SCRIPT), str(maps)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=env,
        timeout=30,
    )


def _plant(maps: Path, deny_rm: bool, deny_sudo: bool) -> Path:
    maps.mkdir()
    target = maps / "mapping_state.json"
    target.write_text('{"map_name":"квартира"}\n')
    (maps / "rtabmap.db").write_text("rtabmap-db-sentinel\n")
    if deny_rm:
        (maps / ".rm-deny").write_text("1\n")
    if deny_sudo:
        (maps / ".sudo-deny").write_text("1\n")
    return target


def test_bash_regression_passes():
    """Полный bash-регресс #5045, включая фолбэк docker и exit 0, зелёный."""
    proc = subprocess.run(
        ["bash", str(BASH_REGRESSION)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=60,
    )
    combined = proc.stdout + proc.stderr
    assert proc.returncode == 0, combined
    assert "FAIL=0" in combined


def test_missing_file_exits_zero(tmp_path: Path):
    """(а) файла нет — ℹ️ и exit 0, docker не нужен."""
    maps = tmp_path / "maps"
    maps.mkdir()
    log = tmp_path / "docker.log"
    log.write_text("")
    proc = _run(tmp_path, maps, tmp_path / "unused.scenario", log)
    assert proc.returncode == 0, proc.stderr
    assert "отсутствует" in proc.stdout
    assert log.read_text() == ""


def test_rm_removes_file_exits_zero(tmp_path: Path):
    """(б) обычный rm удалил — exit 0, sudo и docker не званы, соседний db жив."""
    maps = tmp_path / "maps"
    target = _plant(maps, deny_rm=False, deny_sudo=False)
    log = tmp_path / "docker.log"
    log.write_text("")
    proc = _run(tmp_path, maps, tmp_path / "unused.scenario", log)
    assert proc.returncode == 0, proc.stderr
    assert not target.exists()
    assert "удалён" in proc.stdout
    assert (maps / "rtabmap.db").read_text() == "rtabmap-db-sentinel\n"
    assert log.read_text() == ""
    assert "sudo" not in proc.stderr


def test_rm_denied_docker_exec_removes(tmp_path: Path):
    """(в) rm и sudo -n отказали, docker exec с mount на этот каталог — exit 0.

    Вызов узкий: ``exec rtabmap rm -f /maps/mapping_state.json``. docker run
    не зовём, раз exec уже убрал файл.
    """
    maps = tmp_path / "maps"
    target = _plant(maps, deny_rm=True, deny_sudo=True)
    scenario = tmp_path / "scenario"
    log = tmp_path / "docker.log"
    log.write_text("")
    _scenario(
        scenario,
        running="true",
        source=str(maps.resolve()),
        image="ghcr.io/krikz/rob_box:rtabmap-humble-local",
        image_present="yes",
        images_line="",
        exec_removes="yes",
        run_removes="no",
        host_target=str(target),
    )
    proc = _run(tmp_path, maps, scenario, log)
    calls = log.read_text().splitlines()
    assert proc.returncode == 0, proc.stderr + "\n" + "\n".join(calls)
    assert not target.exists()
    assert "exec rtabmap rm -f /maps/mapping_state.json" in calls
    assert not any(line.startswith("run ") for line in calls)
    assert "a password is required" in proc.stderr
    assert (maps / "rtabmap.db").read_text() == "rtabmap-db-sentinel\n"


def test_rm_and_docker_fail_still_exits_zero(tmp_path: Path):
    """(г) rm, sudo -n и docker не смогли — exit 0, файл на месте, «файл остался».

    Проверка отсутствия выполнена: строка печатается только если файл жив.
    Отказ rm не съеден молча (Permission denied в stderr).
    """
    maps = tmp_path / "maps"
    target = _plant(maps, deny_rm=True, deny_sudo=True)
    scenario = tmp_path / "scenario"
    log = tmp_path / "docker.log"
    log.write_text("")
    _scenario(
        scenario,
        running="false",
        source="",
        image="",
        image_present="no",
        images_line="",
        exec_removes="no",
        run_removes="no",
        host_target=str(target),
    )
    proc = _run(tmp_path, maps, scenario, log)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert target.is_file()
    assert "файл остался" in proc.stderr
    assert "Permission denied" in proc.stderr
    assert "удалён" not in proc.stdout
