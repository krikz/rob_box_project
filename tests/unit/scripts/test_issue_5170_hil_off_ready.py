"""#5170: hil.sh off судит возврат по свежему healthcheck, не по running.

Две ветки товарища Шифу на фейковом docker (робота не трогаем):
1. свежий штатный healthcheck после off = healthy → выход 0, «боевой режим»;
2. до HIL healthy, после off running и за окно так и не healthy → выход ≠ 0,
   причина в логе, замок на месте, строки «боевой режим» нет.
"""

from __future__ import annotations

import json
import os
import stat
import subprocess
import textwrap
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
HIL = REPO / "scripts" / "hil" / "hil.sh"
OFF_READY = REPO / "scripts" / "hil" / "checks" / "off_ready.py"

NOW = datetime(2026, 11, 23, 10, 23, 30, tzinfo=timezone.utc)
RETURNED = NOW - timedelta(seconds=200)  # окно 180 с уже закрыто
STARTED = NOW - timedelta(seconds=30)
PROBE = "controller_server SIGILL Configuring fails=13"


def _iso(moment: datetime) -> str:
    return moment.strftime("%Y-%m-%dT%H:%M:%SZ")


def _iso_us(moment: datetime) -> str:
    """RFC3339 до микросекунд — для лога границы секунды, не для сравнения в коде."""
    return moment.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def _inspect(status: str, health: dict | None, started: datetime = STARTED) -> dict:
    state: dict = {"Status": status, "StartedAt": _iso(started)}
    if health is not None:
        state["Health"] = health
    return {"Name": "/nav2", "State": state}


def _health(status: str, start: datetime, output: str = PROBE) -> dict:
    return {"Status": status, "Log": [{"Start": _iso(start), "Output": output}]}


def _decide(
    before: str, inspect: dict, returned: datetime = RETURNED, now: datetime = NOW
) -> tuple[str, str]:
    raw = json.dumps(inspect)
    proc = subprocess.run(
        [
            "python3",
            "-X",
            "utf8",
            str(OFF_READY),
            "--inspect",
            "-",
            "--before",
            before,
            "--returned-at",
            _iso(returned),
            "--now",
            _iso(now),
        ],
        input=raw,
        text=True,
        capture_output=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    lines = proc.stdout.splitlines()
    word = lines[0] if lines else ""
    reason = lines[1] if len(lines) > 1 else ""
    return word, reason


def test_fresh_healthy_is_ready() -> None:
    word, reason = _decide(
        "running:healthy",
        _inspect("running", _health("healthy", STARTED, "nav2 healthcheck ok")),
    )
    assert word == "ready"
    assert reason == ""


def test_running_unhealthy_after_window_fails_with_probe_text() -> None:
    word, reason = _decide(
        "running:healthy", _inspect("running", _health("unhealthy", STARTED))
    )
    assert word == "fail"
    assert "running/unhealthy" in reason
    assert PROBE in reason


def test_no_healthcheck_running_is_ready() -> None:
    word, _reason = _decide("running:none", _inspect("running", None))
    assert word == "ready"


def test_previous_life_healthy_is_not_ready() -> None:
    stale = _health("healthy", STARTED - timedelta(hours=2), "healthy прошлой жизни")
    word, reason = _decide("running:healthy", _inspect("running", stale))
    assert word == "fail"
    assert "healthy прошлой жизни" in reason


def test_probe_one_second_before_started_at_is_previous_life() -> None:
    """Граница секунды (#5176): Start на 1 с раньше StartedAt — прошлая жизнь, не ready.

    Docker и заглушка режут метки до целой секунды. Строгое ``Start < StartedAt``
    (off_ready.py) тогда считает пробу чужой. Это верный отказ: запись старше
    старта контейнера готовностью не является. Свежая проба этого запуска
    обязана иметь Start не раньше StartedAt — это пишет заглушка compose up,
    не ослабление порога.
    """
    started = STARTED
    probe_start = STARTED - timedelta(seconds=1)
    print(
        "SECOND_BOUNDARY "
        f"Health.Log.Start={_iso(probe_start)} ({_iso_us(probe_start)}) "
        f"StartedAt={_iso(started)} ({_iso_us(started)}) "
        f"delta_s={(started - probe_start).total_seconds()}"
    )
    word, reason = _decide(
        "running:healthy",
        _inspect(
            "running", _health("healthy", probe_start, "nav2 healthcheck ok"), started
        ),
    )
    assert word == "fail", reason
    assert "nav2 healthcheck ok" in reason


def test_starting_inside_window_waits() -> None:
    inside = NOW - timedelta(seconds=10)
    word, _reason = _decide(
        "running:healthy",
        _inspect("running", {"Status": "starting", "Log": []}),
        returned=inside,
        now=NOW,
    )
    assert word == "wait"


def test_starting_after_window_fails() -> None:
    word, reason = _decide(
        "running:healthy", _inspect("running", {"Status": "starting", "Log": []})
    )
    assert word == "fail"
    assert "пробы healthcheck нет" in reason


def test_old_pre_state_without_health_is_unknown_not_pass() -> None:
    word, reason = _decide(
        "running", _inspect("running", _health("unhealthy", STARTED))
    )
    assert word == "unknown"
    assert "неизвестна" in reason
    assert PROBE in reason


def _write(path: Path, text: str, mode: int | None = None) -> None:
    path.write_text(text)
    if mode is not None:
        path.chmod(mode)


def _fake_tree(tmp: Path) -> dict[str, str]:
    """Минимальный фейк ssh/docker: только то, что зовёт hil.sh off после взятого замка."""
    bindir = tmp / "bin"
    state = tmp / "state"
    remote = tmp / "remote"
    lockdir = tmp / "tmp"
    for folder in (
        bindir,
        state,
        remote,
        lockdir,
        tmp / "main",
        tmp / "vision",
        tmp / "sound",
        tmp / "sbx" / "docker" / "sim",
    ):
        folder.mkdir(parents=True)
    (tmp / "main" / ".env").write_text("ROBOT_ID=RBXTEST01\n")
    (tmp / "sbx" / "docker" / "sim" / "compose.hil.yaml").write_text("services: {}\n")
    (state / "prod_maps").write_text("1|aaaa|мастерская|1.0|1\n")
    (lockdir / "hil.lock").write_text(
        "HIL on since earlier token=t maps=/tmp/hilmaps\n"
    )
    services = (
        "ros2-control lslidar robot-state-publisher twist-mux proximity-guard "
        "followpath-watchdog rtabmap imu-filter nav2 oak-d ceiling-camera voice-assistant"
    )
    for name in services.split():
        (state / name).write_text("running\n")
    ssh = bindir / "ssh"
    _write(
        ssh,
        textwrap.dedent(
            """\
            #!/usr/bin/env bash
            while [ "$#" -gt 0 ]; do case "$1" in -o) shift 2 ;; *) break ;; esac; done
            host="$1"; shift
            echo "$host|$*" >> "$FAKE_SSH_LOG"
            exec bash -c "$*"
            """
        ),
        stat.S_IRWXU,
    )
    docker = bindir / "docker"
    _write(
        docker,
        textwrap.dedent(
            """\
            #!/usr/bin/env bash
            echo "docker $*" >> "$FAKE_DOCKER_LOG"
            S="$FAKE_STATE"
            last() { printf '%s' "${@: -1}"; }
            case "$1" in
              inspect)
                name=$(last "$@")
                _fmt=0
                for _a in "$@"; do [ "$_a" = -f ] && _fmt=1; done
                if [ "$_fmt" = 0 ]; then
                  [ -f "$S/$name" ] || { printf '{"Name":"/%s","State":{"Status":"absent"}}\\n' "$name"; exit 0; }
                  st=$(cat "$S/$name")
                  started=$(cat "$S/$name.started" 2>/dev/null || echo 2026-11-23T10:23:00Z)
                  if [ -f "$S/$name.health" ]; then
                    printf '{"Name":"/%s","State":{"Status":"%s","StartedAt":"%s","Health":%s}}\\n' \\
                      "$name" "$st" "$started" "$(cat "$S/$name.health")"
                  else
                    printf '{"Name":"/%s","State":{"Status":"%s","StartedAt":"%s"}}\\n' "$name" "$st" "$started"
                  fi
                  exit 0
                fi
                case "$*" in
                  *Config.Env*) echo PATH=/x ;;
                  *State.Status*) cat "$S/$name" 2>/dev/null || echo absent ;;
                  *State.Running*) echo true ;;
                esac ;;
              ps)
                for s in ros2-control lslidar robot-state-publisher twist-mux proximity-guard \\
                         followpath-watchdog rtabmap imu-filter nav2 oak-d ceiling-camera voice-assistant; do
                  echo "$s ghcr.io/krikz/rob_box:$s-humble-dev-abc"
                done ;;
              compose)
                shift; verb=""; svcs=()
                while [ "$#" -gt 0 ]; do
                  case "$1" in
                    -f|-p) shift 2 ;;
                    --pull) shift 2 ;;
                    up|down) verb="$1"; shift ;;
                    -*) shift ;;
                    *) svcs+=("$1"); shift ;;
                  esac
                done
                if [ "$verb" = up ]; then
                  for s in "${svcs[@]}"; do
                    echo running > "$S/$s"
                    # Наносекунды, в файл — целая секунда, как Docker StartedAt.
                    # Health.Log.Start пишем ПОСЛЕ StartedAt и из той же метки:
                    # проба, снятая до hil.sh, на границе секунды старше старта
                    # и off_ready.py честно зовёт её прошлой жизнью (#5176).
                    stamp=$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)
                    sec="${stamp%.*}Z"
                    printf '%s\n' "$sec" > "$S/$s.started"
                    if [ -f "$S/$s.health.template" ]; then
                      python3 - "$S/$s.health.template" "$S/$s.health" "$sec" "$stamp" "$S/$s.started" <<'PY'
import json, sys
src, dst, sec, wall, started_file = sys.argv[1:]
health = json.loads(open(src, encoding="utf-8").read())
log = health.get("Log") or []
if log:
    log[-1]["Start"] = sec
open(dst, "w", encoding="utf-8").write(json.dumps(health))
open(dst + ".stamp", "w", encoding="utf-8").write(
    f"COMPOSE_UP Health.Log.Start={sec} wall={wall} file={dst}\\n"
    f"COMPOSE_UP StartedAt={sec} wall={wall} file={started_file}\\n"
)
PY
                    fi
                    printf 'COMPOSE_UP name=%s StartedAt=%s wall=%s file=%s\n' \
                      "$s" "$sec" "$stamp" "$S/$s.started" >> "$FAKE_STAMP_LOG"
                  done
                fi ;;
              exec) exit 1 ;;
            esac
            exit 0
            """
        ),
        stat.S_IRWXU,
    )
    return {
        "PATH": f"{bindir}:{os.environ.get('PATH', '')}",
        "FAKE_SSH_LOG": str(tmp / "ssh.log"),
        "FAKE_DOCKER_LOG": str(tmp / "docker.log"),
        "FAKE_STATE": str(state),
        "HIL_MAIN_HOST": "main",
        "HIL_VISION_HOST": "vision",
        "HIL_SOUND_HOST": "sound",
        "HIL_SANDBOX_HOST": "sbx",
        "HIL_MAIN_DIR": str(tmp / "main"),
        "HIL_VISION_DIR": str(tmp / "vision"),
        "HIL_SOUND_DIR": str(tmp / "sound"),
        "HIL_SANDBOX_REPO": str(tmp / "sbx"),
        "HIL_MAPS_SNAPSHOT": str(tmp / "sndstate" / "prod_maps.before.txt"),
        "HIL_LOCK": str(lockdir / "hil.lock"),
        "HIL_REMOTE_DIR": str(remote),
        "HIL_MAPS_DIR": str(tmp / "hilmaps"),
        "HIL_OFF_READY_POLL_S": "0",
        "FAKE_STAMP_LOG": str(tmp / "stamps.log"),
    }


def _run_off(
    tmp: Path, extra: dict[str, str] | None = None
) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.update(_fake_tree(tmp))
    if extra:
        env.update(extra)
    (tmp / "ssh.log").write_text("")
    (tmp / "docker.log").write_text("")
    # PRE_STATE: до HIL nav2 был healthy, остальные без HEALTHCHECK.
    pre = tmp / "remote" / "pre_on_state.txt"
    lines = []
    for name in (
        "ros2-control",
        "lslidar",
        "robot-state-publisher",
        "twist-mux",
        "proximity-guard",
        "followpath-watchdog",
        "rtabmap",
        "imu-filter",
        "nav2",
        "oak-d",
        "ceiling-camera",
        "voice-assistant",
    ):
        health = "healthy" if name == "nav2" else "none"
        lines.append(f"{name} running:{health}")
    pre.write_text("\n".join(lines) + "\n")
    return subprocess.run(
        ["bash", str(HIL), "off"],
        env=env,
        text=True,
        capture_output=True,
        check=False,
        cwd=str(REPO),
    )


def _log_boundary_stamps(state: Path, stamp_log: Path, label: str) -> None:
    """Обе метки наружу: что записала заглушка и что вернул compose up.

    Файл состояния — ``$FAKE_STATE/<name>.started`` и ``<name>.health``.
    Точность лога — секунда (как в inspect) и микросекунды (как сняли часы).
    """
    health_path = state / "nav2.health"
    started_path = state / "nav2.started"
    health_raw = health_path.read_text() if health_path.exists() else ""
    started_raw = started_path.read_text().strip() if started_path.exists() else ""
    probe = ""
    if health_raw:
        try:
            probe = str(json.loads(health_raw)["Log"][-1]["Start"])
        except (KeyError, IndexError, json.JSONDecodeError, TypeError):
            probe = ""
    wall = _iso_us(datetime.now(timezone.utc))
    detail = ""
    detail_path = state / "nav2.health.stamp"
    if detail_path.exists():
        detail = " ".join(detail_path.read_text().splitlines())
    compose_at = "COMPOSE_UP name=nav2 (не было)"
    if stamp_log.exists():
        compose_lines = [
            line
            for line in stamp_log.read_text().splitlines()
            if line.startswith("COMPOSE_UP name=nav2 ")
        ]
        if compose_lines:
            compose_at = compose_lines[-1]
    print(
        f"{label} wall={wall} "
        f"Health.Log.Start={probe or '(нет)'} file={health_path} "
        f"state.StartedAt={started_raw or '(нет)'} file={started_path} "
        f"{compose_at} {detail}"
    )


def test_hil_off_fresh_healthy_prints_combat_and_exits_0(tmp_path: Path) -> None:
    # Шаблон пробы без Start: compose up допишет Start = StartedAt этого запуска.
    # Писать Start до hil.sh нельзя — за время off секунда сменяется, и строгий
    # off_ready.py честно зовёт такую пробу прошлой жизнью (#5176).
    proc_env_dir = tmp_path
    env = os.environ.copy()
    env.update(_fake_tree(proc_env_dir))
    template = {"Status": "healthy", "Log": [{"Output": "nav2 healthcheck ok"}]}
    (proc_env_dir / "state" / "nav2.health.template").write_text(json.dumps(template))
    (proc_env_dir / "stamps.log").write_text("")
    pre = proc_env_dir / "remote" / "pre_on_state.txt"
    names = (
        "ros2-control lslidar robot-state-publisher twist-mux proximity-guard "
        "followpath-watchdog rtabmap imu-filter nav2 oak-d ceiling-camera voice-assistant"
    ).split()
    pre.write_text(
        "".join(f"{n} running:{'healthy' if n == 'nav2' else 'none'}\n" for n in names)
    )
    proc = subprocess.run(
        ["bash", str(HIL), "off"],
        env=env,
        text=True,
        capture_output=True,
        check=False,
        cwd=str(REPO),
    )
    _log_boundary_stamps(
        proc_env_dir / "state", proc_env_dir / "stamps.log", "FRESH_HEALTHY"
    )
    out = proc.stdout + proc.stderr
    assert proc.returncode == 0, out
    assert "робот в боевом режиме" in out
    assert not (proc_env_dir / "tmp" / "hil.lock").exists()


def test_hil_off_unhealthy_keeps_lock_and_hides_combat(tmp_path: Path) -> None:
    env = os.environ.copy()
    env.update(_fake_tree(tmp_path))
    env["HIL_OFF_READY_S"] = "0"
    started = datetime.now(timezone.utc) - timedelta(seconds=5)
    (tmp_path / "state" / "nav2.started").write_text(_iso(started))
    (tmp_path / "state" / "nav2.health").write_text(
        json.dumps(_health("unhealthy", started))
    )
    pre = tmp_path / "remote" / "pre_on_state.txt"
    names = (
        "ros2-control lslidar robot-state-publisher twist-mux proximity-guard "
        "followpath-watchdog rtabmap imu-filter nav2 oak-d ceiling-camera voice-assistant"
    ).split()
    pre.write_text(
        "".join(f"{n} running:{'healthy' if n == 'nav2' else 'none'}\n" for n in names)
    )
    proc = subprocess.run(
        ["bash", str(HIL), "off"],
        env=env,
        text=True,
        capture_output=True,
        check=False,
        cwd=str(REPO),
    )
    out = proc.stdout + proc.stderr
    assert proc.returncode != 0, out
    assert "робот в боевом режиме" not in out
    assert PROBE in out
    assert (tmp_path / "tmp" / "hil.lock").exists()


@pytest.mark.parametrize("ready", ["healthy", "none"])
def test_svc_words_match_module(ready: str) -> None:
    health = _health("healthy", STARTED, "ok") if ready == "healthy" else None
    word, _reason = _decide(f"running:{ready}", _inspect("running", health))
    assert word == "ready"
