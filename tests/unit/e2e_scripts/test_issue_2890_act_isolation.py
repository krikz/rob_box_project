"""
test_issue_2890_act_isolation.py — изоляция АКТА E2E, а не только БД дикторов.

Issue #2890: e2e_mode при старте акта стирал ``/data/speakers.e2e.db``, но
окно диалога dialogue_node и e2e-память фактов переживали переход между
актами. Прогон акта 2b 35903232434 сразу после акта 2 35900007906: в
контексте LLM стоял ход акта 2 «[Spkr:Саша] поищи … про чай», и новой Саше
(профиль 5baad325; старая 243092d7 стёрта) робот сказал факт старой.

Здесь исполняется НАСТОЯЩИЙ ``isolate_act_start`` и его помощники, вырезанные
из ``e2e_voice_test.sh`` (конвенция ``test_issue_2846_registration_verdict.py``).
``robot_ros`` подменён фейковым роботом с состоянием: параметры узлов лежат
файлами, ``e2e_mode`` false→true пишет событие WIPE (как узел стирает
e2e-базу), ``e2e_session_reset_token`` пишет событие RESET (или отклоняется,
как отклонил бы колбэк при провале сброса).

Run:
  python -m pytest tests/unit/e2e_scripts/test_issue_2890_act_isolation.py -v --no-cov
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
E2E_SCRIPT = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"
SCENARIOS = REPO_ROOT / ".github" / "e2e" / "scenarios"
NIGHT = SCENARIOS / "night"

#: Полный путь, а не "bash": на Windows CreateProcess ищет System32 раньше
#: PATH и находит WSL-заглушку, которой пути C:/... не видны.
BASH = shutil.which("bash")
bash_required = pytest.mark.skipif(BASH is None, reason="bash недоступен в этом окружении")

FUNCTIONS = (
    "scenario_registers_speakers",
    "scenario_writes_memory",
    "scenario_inherits_previous_act",
    "_read_e2e_mode",
    "_set_e2e_mode",
    "_read_e2e_applied",
    "_wait_e2e_followers",
    "_set_e2e_release_after",
    "_e2e_followers_or_fatal",
    "enter_e2e_mode",
    "adopt_e2e_mode",
    "marathon_e2e_mode",
    "leave_e2e_mode",
    "_read_dialogue_reset_token",
    "reset_dialogue_session_for_act",
    "_read_dialog_param",
    "detect_dialog_engine",
    "reset_dialog_v2_session_for_act",
    "activate_e2e_dialog_db",
    "deactivate_e2e_dialog_db",
    "isolate_act_start",
)

# Фейковый робот: ``robot_ros "ros2 param set|get /node name [value] --no-daemon"``.
FAKE_ROBOT = r'''
log() { echo ">>> $*"; }
_pfile() { echo "$STATE/$(echo "$1$2" | tr '/' '_')"; }
robot_ros() {
    # v2 (ADR-0150 PR-8 #3702): сброс сессии — сервис /dialog/control.
    case "$*" in
        *"service call /dialog/control "*"action: 2"*"set_by: tars"*)
            if [ "${FAKE_V2_RESET_REJECT:-0}" = 1 ]; then
                echo "rob_box_dialog_msgs.srv.DialogControl_Response(ok=False, epoch=0, scenes=[])"
            else
                echo "RESET /dialog_node" >> "$STATE/events"
                echo "rob_box_dialog_msgs.srv.DialogControl_Response(ok=True, epoch=1, scenes=[])"
            fi
            return 0
            ;;
    esac
    # shellcheck disable=SC2086
    set -- $*
    local verb="$3" node="$4" name="$5" value="$6" f
    f="$(_pfile "$node" "$name")"
    case "$verb" in
        set)
            case "$name" in
                e2e_mode)
                    # #3904: режим один — владелец /speaker_id_node; последователь
                    # /mcp_server на том же переходе стирает свою e2e-память.
                    if [ "$(cat "$f" 2>/dev/null || echo false)" = false ] && [ "$value" = true ]; then
                        echo "WIPE $node" >> "$STATE/events"
                        [ "${FAKE_FOLLOWER_STUCK:-0}" = 1 ] || echo "WIPE /mcp_server" >> "$STATE/events"
                    fi
                    echo "$value" > "$f"
                    ;;
                e2e_release_after_s)
                    # Владелец отклоняет таймер снятия, когда режим выключен (E2EModeOwner.on_params).
                    owner_mode="$(cat "$(_pfile /speaker_id_node e2e_mode)" 2>/dev/null || echo false)"
                    if [ "$value" != 0.0 ] && { [ "$owner_mode" != true ] || [ "${FAKE_TIMER_REJECT:-0}" = 1 ]; }; then
                        echo "Set parameter failed"
                        return 0
                    fi
                    echo "$value" > "$f"
                    ;;
                e2e_session_reset_token)
                    if [ "${FAKE_DIALOGUE_REJECT:-0}" = 1 ]; then
                        echo "Set parameter failed: issue 2890: e2e session reset failed"
                        return 0
                    fi
                    echo "RESET $node" >> "$STATE/events"
                    echo "$value" > "$f"
                    ;;
                e2e_db_path)
                    [ "$value" = "''" ] && value=""
                    echo "$value" > "$f"
                    ;;
            esac
            ;;
        get)
            case "$name" in
                e2e_mode_applied)
                    # Последователь применяет режим владельца; FAKE_FOLLOWER_STUCK=1 — отказ apply.
                    if [ "${FAKE_FOLLOWER_STUCK:-0}" != 1 ] \
                       && [ "$(cat "$(_pfile /speaker_id_node e2e_mode)" 2>/dev/null || echo false)" = true ]; then
                        echo "Boolean value is: True"
                    else
                        echo "Boolean value is: False"
                    fi
                    ;;
                e2e_release_after_s)
                    echo "Double value is: $(cat "$f" 2>/dev/null || echo 0.0)"
                    ;;
                e2e_mode)
                    if [ "$(cat "$f" 2>/dev/null || echo false)" = true ]; then
                        echo "Boolean value is: True"
                    else
                        echo "Boolean value is: False"
                    fi
                    ;;
                e2e_session_reset_token | e2e_db_path)
                    echo "String value is: $(cat "$f" 2>/dev/null)"
                    ;;
                dialog_engine)
                    # /dialog_node есть только при v2; при v1 ros2cli печатает «Node not found».
                    if [ "$node" = /dialog_node ] && [ -n "${FAKE_ENGINE:-}" ]; then
                        echo "String value is: $FAKE_ENGINE"
                    else
                        echo "Node not found"
                    fi
                    ;;
            esac
            ;;
    esac
}
'''


def _extract_function(name: str) -> str:
    src = E2E_SCRIPT.read_text(encoding="utf-8")
    m = re.search(rf"^{name}\(\) \{{.*?^\}}", src, re.S | re.M)
    assert m, f"функция {name}() не найдена в {E2E_SCRIPT.name} (issue #2890)"
    return m.group(0)


def _bash_path(p: Path) -> str:
    return p.as_posix()


def _require_bash_sees(path: Path) -> None:
    probe = subprocess.run(
        [BASH, "-c", f'[ -e "{_bash_path(path)}" ]'],
        capture_output=True,
        text=True,
        timeout=10,
    )
    if probe.returncode != 0:
        pytest.skip(f"bash не видит {path} (вероятно WSL на Windows) — проверка только в CI")


def _run_isolation(tmp_path: Path, scenario: Path, *, reject=False, preset=None, engine=None,
                   v2_reject=False, db_path="/tmp/e2e_dialog/dialog_events_test.db", tail="",
                   hold=False, follower_stuck=False, phase="", timer_reject=False):
    state = tmp_path / "state"
    state.mkdir(exist_ok=True)
    for key, value in (preset or {}).items():
        (state / key).write_text(value + "\n", encoding="utf-8")
    _require_bash_sees(state)
    body = "\n".join(_extract_function(n) for n in FUNCTIONS)
    # Под Windows ``python3`` бывает заглушкой Microsoft Store — подставляем
    # интерпретатор, под которым идёт pytest (в CI это тот же python3).
    py = _bash_path(Path(sys.executable))
    script = (
        f'STATE="{_bash_path(state)}"\n'
        f'python3() {{ "{py}" "$@"; }}\n'
        f'E2E_DIALOG_ENGINE=v1\nE2E_DIALOG_DB_PATH="{db_path}"\nE2E_DIALOG_DB_ACTIVATED=0\n'
        f'E2E_MODE_FOLLOWERS="/mcp_server"\nE2E_MODE_HELD=0\nE2E_HOLD_FOR_NEXT_ACT={1 if hold else 0}\n'
        'E2E_HOLD_RELEASE_S=1800\nE2E_FOLLOWER_WAIT_S=1\n'
        f'E2E_MARATHON_PHASE="{phase}"\nE2E_MARATHON_HOLD_S=10800\n'
        f"{FAKE_ROBOT}\n{body}\n"
        f'isolate_act_start "{_bash_path(scenario)}"\n'
        f"{tail}\n"
        'echo "EXIT_OK"\n'
    )
    env = {"PATH": os.environ.get("PATH", ""), "PYTHONIOENCODING": "utf-8"}
    if reject:
        env["FAKE_DIALOGUE_REJECT"] = "1"
    if engine:
        env["FAKE_ENGINE"] = engine
    if v2_reject:
        env["FAKE_V2_RESET_REJECT"] = "1"
    if follower_stuck:
        env["FAKE_FOLLOWER_STUCK"] = "1"
    if timer_reject:
        env["FAKE_TIMER_REJECT"] = "1"
    # Скрипт — через stdin, не ``-c``: на Windows склейка аргумента
    # командной строки для msys-bash портит кавычки в длинном скрипте.
    proc = subprocess.run(
        [BASH, "-s"],
        input=script,
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=60,
        env=env,
    )
    events_file = state / "events"
    events = events_file.read_text(encoding="utf-8").split("\n") if events_file.exists() else []
    if events_file.exists():
        events_file.unlink()  # следующий акт того же марафона начинает свой журнал
    return proc, [e for e in events if e]


def _scenario(tmp_path: Path, name: str, **top) -> Path:
    data = {"name": name, "steps": [{"label": "s1", "text": "Робот, привет"}]}
    data.update(top)
    path = tmp_path / "e2e_scenario.json"
    path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    return path


def _copy_real(tmp_path: Path, src: Path) -> Path:
    """Реальный сценарий под именем, которое ему даёт воркфлоу (issue #2763)."""
    dst = tmp_path / "e2e_scenario.json"
    dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
    return dst


# ── поведение isolate_act_start ─────────────────────────────────────────────


@bash_required
def test_act2b_resets_dialogue_and_wipes_speakers_and_memory(tmp_path):
    """Сам инцидент: акт 2b (регистрирует дикторов, memory_save в тексте нет)
    обязан начаться с пустого окна диалога и пустой e2e-памяти фактов."""
    scenario = _copy_real(tmp_path, NIGHT / "night_marathon_act2b_identity_question_v1.json")
    assert "memory_save" not in scenario.read_text(encoding="utf-8")

    proc, events = _run_isolation(tmp_path, scenario)

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "WIPE /speaker_id_node" in events
    assert "WIPE /mcp_server" in events, (
        "акт стёр каст дикторов, но оставил факты о нём — факт старой Саши "
        "доживает до новой (issue #2890)"
    )
    assert "RESET /dialogue_node" in events, (
        "окно диалога не сброшено — LLM акта 2b увидит ходы акта 2 (issue #2890)"
    )
    assert "окно диалога сброшено перед актом" in proc.stdout
    assert "E2E_PRECONDITION speaker_db=e2e" in proc.stdout


@bash_required
def test_plain_act_resets_dialogue_without_touching_dbs(tmp_path):
    """Акт без регистрации и memory_save: только сброс окна диалога."""
    proc, events = _run_isolation(tmp_path, _scenario(tmp_path, "plain_v1"))

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert events == ["RESET /dialogue_node"]
    assert "E2E_PRECONDITION speaker_db=live" in proc.stdout


@bash_required
def test_inheriting_act_keeps_dialogue_window(tmp_path):
    """Акт 3 марафона намеренно продолжает разговор — сброса нет, но в логе
    это сказано явно. Режим e2e держит прошлый акт (#3904)."""
    scenario = _copy_real(tmp_path, NIGHT / "night_marathon_act3_backlog_diarization_v1.json")

    proc, events = _run_isolation(tmp_path, scenario, preset={"_speaker_id_nodee2e_mode": "true"})

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "RESET /dialogue_node" not in events
    assert "inherits_previous_act=true" in proc.stdout


# ── #3904: режим e2e — один владелец на весь марафон ────────────────────────


def _owner(tmp_path: Path, name: str) -> str:
    f = tmp_path / "state" / ("/speaker_id_node" + name).replace("/", "_")
    return f.read_text(encoding="utf-8").strip() if f.exists() else ""


@bash_required
def test_marathon_act2_then_inheriting_act3_stays_on_e2e_db(tmp_path):
    """Сам инцидент #3904 (run 39910805388): акт 2 → акт 3 (наследует). Раньше
    trap акта 2 снимал режим, и акт 3 опознавал голоса по боевой speakers.db.
    Теперь акт 2 держит режим для следующего, акт 3 подхватывает его без
    стирания каста, последний акт снимает."""
    act2 = _copy_real(tmp_path, NIGHT / "night_marathon_act2_acquaintance_v1.json")
    proc, events = _run_isolation(tmp_path, act2, hold=True, tail="leave_e2e_mode")
    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert events == ["WIPE /speaker_id_node", "WIPE /mcp_server", "RESET /dialogue_node"]
    assert "E2E_PRECONDITION speaker_db=e2e" in proc.stdout
    assert "E2E_MODE held_for_next_act release_after_s=1800" in proc.stdout
    assert _owner(tmp_path, "e2e_mode") == "true", "trap акта 2 снял режим — акт 3 пойдёт на боевую БД"
    assert _owner(tmp_path, "e2e_release_after_s") == "1800.0", "без таймера обрыв марафона оставил бы робота на e2e"

    act3 = _copy_real(tmp_path, NIGHT / "night_marathon_act3_backlog_diarization_v1.json")
    proc, events = _run_isolation(tmp_path, act3, tail="leave_e2e_mode")
    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "E2E_PRECONDITION speaker_db=e2e" in proc.stdout
    assert not [e for e in events if e.startswith("WIPE")], "акт 3 стёр каст акта 2 — «Борис» станет незнакомым"
    assert "E2E_MODE off" in proc.stdout
    assert _owner(tmp_path, "e2e_mode") == "false", "последний акт марафона обязан вернуть робота на боевые БД"


@bash_required
def test_inheriting_act_without_e2e_mode_is_fatal_not_live(tmp_path):
    """Наследующий акт без включённого режима (ACTS=3 отдельно, таймер истёк)
    не идёт по боевой БД дикторов: E2E_PRECONDITION speaker_db=live + фатал."""
    scenario = _copy_real(tmp_path, NIGHT / "night_marathon_act3_backlog_diarization_v1.json")

    proc, events = _run_isolation(tmp_path, scenario)

    assert proc.returncode == 2
    assert "E2E_PRECONDITION speaker_db=live" in proc.stdout
    assert "E2E_FATAL" in proc.stderr and "#3904" in proc.stderr
    assert "EXIT_OK" not in proc.stdout
    assert events == []


@bash_required
def test_inheriting_act_cancels_release_timer(tmp_path):
    preset = {"_speaker_id_nodee2e_mode": "true", "_speaker_id_nodee2e_release_after_s": "1800.0"}
    scenario = _copy_real(tmp_path, NIGHT / "night_marathon_act3_backlog_diarization_v1.json")

    proc, _ = _run_isolation(tmp_path, scenario, preset=preset)

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert _owner(tmp_path, "e2e_release_after_s") == "0.0", "таймер снял бы режим посреди акта 3"


@bash_required
def test_act_without_hold_returns_robot_to_live_dbs(tmp_path):
    """Отдельный прогон (ротация, ручной) — trap выключает режим сразу."""
    scenario = _copy_real(tmp_path, NIGHT / "night_marathon_act2_acquaintance_v1.json")

    proc, _ = _run_isolation(tmp_path, scenario, tail="leave_e2e_mode")

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "E2E_MODE off" in proc.stdout
    assert _owner(tmp_path, "e2e_mode") == "false"


@bash_required
def test_memory_save_only_act_turns_on_the_single_mode(tmp_path):
    """memory_save без register_speaker включает тот же режим (не отдельный
    флаг памяти): память и дикторы не расходятся (#3904)."""
    scenario = _scenario(tmp_path, "mem_v1", steps=[
        {"label": "s1", "text": "запомни", "acceptance": {"expected_tool_calls": ["memory_save"]}}])

    proc, events = _run_isolation(tmp_path, scenario)

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "WIPE /speaker_id_node" in events and "WIPE /mcp_server" in events
    assert "E2E_PRECONDITION speaker_db=e2e" in proc.stdout


@bash_required
def test_follower_that_did_not_apply_mode_is_fatal(tmp_path):
    """mcp_server не применил режим владельца — акт не прогоняется: memory_save
    писал бы в боевую voice_memory.db."""
    scenario = _copy_real(tmp_path, NIGHT / "night_marathon_act2_acquaintance_v1.json")

    proc, _ = _run_isolation(tmp_path, scenario, follower_stuck=True)

    assert proc.returncode == 2
    assert "E2E_FATAL" in proc.stderr and "/mcp_server" in proc.stderr
    assert "EXIT_OK" not in proc.stdout


@bash_required
def test_inherit_flag_on_cast_wiping_act_is_fatal(tmp_path):
    """Нельзя стереть каст и продолжить разговор о нём."""
    scenario = _scenario(
        tmp_path,
        "bad_v1",
        inherits_previous_act=True,
        steps=[{"label": "s1", "text": "я Саша", "acceptance": {"expected_tool_calls": ["register_speaker"]}}],
    )

    proc, events = _run_isolation(tmp_path, scenario)

    assert proc.returncode == 2
    assert "E2E_FATAL" in proc.stderr
    assert "EXIT_OK" not in proc.stdout
    assert events == [], "до фатала ничего не должно было переключиться"


@bash_required
def test_non_bool_inherit_flag_is_schema_error(tmp_path):
    proc, _ = _run_isolation(tmp_path, _scenario(tmp_path, "bad_v1", inherits_previous_act="yes"))

    assert proc.returncode == 2
    assert "inherits_previous_act" in proc.stderr


@bash_required
def test_rejected_reset_is_fatal_not_a_warning(tmp_path):
    """Узел не подтвердил сброс — акт не прогоняется: вердикт по ходам чужого
    акта нечитаем."""
    proc, events = _run_isolation(tmp_path, _scenario(tmp_path, "plain_v1"), reject=True)

    assert proc.returncode == 2
    assert "E2E_FATAL" in proc.stderr and "dialogue_node" in proc.stderr
    assert "EXIT_OK" not in proc.stdout
    assert "RESET /dialogue_node" not in events


@bash_required
def test_stale_e2e_mode_from_crashed_run_is_rewiped(tmp_path):
    """Прошлый прогон не вернул узлы на боевую: e2e_mode уже true. Раньше
    включение было no-op и акт наследовал чужой каст — теперь выкл/вкл со
    стиранием."""
    scenario = _copy_real(tmp_path, NIGHT / "night_marathon_act2b_identity_question_v1.json")
    preset = {"_speaker_id_nodee2e_mode": "true", "_mcp_servere2e_mode": "true"}

    proc, events = _run_isolation(tmp_path, scenario, preset=preset)

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "WIPE /speaker_id_node" in events
    assert "WIPE /mcp_server" in events


# ── контракт сценариев ──────────────────────────────────────────────────────


def _all_scenarios():
    for path in sorted(SCENARIOS.rglob("*.json")):
        if "acceptance" in path.name or path.name.endswith("manifest.json"):
            continue
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if isinstance(data, dict) and "steps" in data:
            yield path, data


def test_no_scenario_inherits_while_wiping_the_cast():
    """Харнесс зафейлит такой сценарий на старте — ловим это до стенда."""
    bad = [
        p.name
        for p, d in _all_scenarios()
        if d.get("inherits_previous_act") is True and "register_speaker" in json.dumps(d)
    ]
    assert bad == []


def test_inherit_flag_is_bool_everywhere():
    bad = [
        p.name
        for p, d in _all_scenarios()
        if "inherits_previous_act" in d and not isinstance(d["inherits_previous_act"], bool)
    ]
    assert bad == []


@pytest.mark.parametrize(
    "act_file, inherits",
    [
        ("night_marathon_act1_wakeup_v1.json", False),
        ("night_marathon_act2_acquaintance_v1.json", False),
        ("night_marathon_act2b_identity_question_v1.json", False),
        ("night_marathon_act2c_voice_conflict_question_v1.json", False),
        ("night_marathon_act3_backlog_diarization_v1.json", True),
        ("night_marathon_act10_finale_memory_v1.json", True),
    ],
)
def test_marathon_acts_declare_dependency_explicitly(act_file, inherits):
    """2b/2c заявлены САМОДОСТАТОЧНЫМИ — сбрасываются; 3-10 продолжают
    историю ночи (манифест, акт 10 пересказывает ночь) — наследуют."""
    data = json.loads((NIGHT / act_file).read_text(encoding="utf-8"))
    assert data.get("inherits_previous_act", False) is inherits


def test_trap_is_armed_before_act_isolation():
    """E2E_FATAL сброса сессии после включённой e2e-БД обязан вернуть узлы на
    боевые БД — trap EXIT ставится ДО isolate_act_start."""
    src = E2E_SCRIPT.read_text(encoding="utf-8")
    # Issue #3248: перед restore_node_params в trap стоит
    # restore_robot_tts_default — ищем по самой связке restore/deactivate.
    trap_at = src.index("restore_node_params; leave_e2e_mode")
    assert src.rfind("trap '", 0, trap_at) != -1
    call_at = src.index('isolate_act_start "$SCENARIO_FILE"')
    assert trap_at < call_at


# ── ADR-0150 PR-8 (#3702): харнесс понимает dialog_engine v2 ────────────────

V2_DB = "/tmp/e2e_dialog/dialog_events_test.db"


def _param(tmp_path: Path, node: str, name: str) -> str:
    f = tmp_path / "state" / (node + name).replace("/", "_")
    return f.read_text(encoding="utf-8").strip() if f.exists() else ""


@bash_required
def test_v1_robot_keeps_v1_path(tmp_path):
    """/dialog_node нет — движок v1, сброс через dialogue_node как раньше, e2e-БД ходов не трогается."""
    proc, events = _run_isolation(tmp_path, _scenario(tmp_path, "plain_v1"))

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "E2E_DIALOG_ENGINE v1" in proc.stdout
    assert events == ["RESET /dialogue_node"]
    assert _param(tmp_path, "/dialog_node", "e2e_db_path") == ""


@bash_required
def test_v2_resets_via_dialog_control_and_opens_e2e_db(tmp_path):
    """v2: сброс — /dialog/control reset (не параметр dialogue_node), e2e-БД ходов — вне /data."""
    proc, events = _run_isolation(tmp_path, _scenario(tmp_path, "plain_v1"), engine="v2")

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "E2E_DIALOG_ENGINE v2" in proc.stdout
    assert "RESET /dialog_node" in events
    assert "RESET /dialogue_node" not in events
    assert "epoch=1" in proc.stdout
    assert f"E2E_DIALOG_DB {V2_DB}" in proc.stdout
    assert _param(tmp_path, "/dialog_node", "e2e_mode") == "", "#3904: своего e2e_mode у dialog_node нет"
    assert _param(tmp_path, "/dialog_node", "e2e_db_path") == V2_DB


@bash_required
def test_v2_rejected_reset_is_fatal(tmp_path):
    proc, events = _run_isolation(tmp_path, _scenario(tmp_path, "plain_v1"), engine="v2", v2_reject=True)

    assert proc.returncode == 2
    assert "E2E_FATAL" in proc.stderr and "dialog_node (v2)" in proc.stderr
    assert "ok=False" in proc.stderr
    assert "EXIT_OK" not in proc.stdout
    assert "RESET /dialog_node" not in events


@bash_required
def test_v2_inheriting_act_skips_reset_but_records_turns(tmp_path):
    scenario = _copy_real(tmp_path, NIGHT / "night_marathon_act3_backlog_diarization_v1.json")

    proc, events = _run_isolation(tmp_path, scenario, engine="v2", preset={"_speaker_id_nodee2e_mode": "true"})

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "RESET /dialog_node" not in events and "RESET /dialogue_node" not in events
    assert f"E2E_DIALOG_DB {V2_DB}" in proc.stdout


@bash_required
def test_v2_e2e_db_inside_data_is_fatal(tmp_path):
    proc, _ = _run_isolation(tmp_path, _scenario(tmp_path, "plain_v1"), engine="v2", db_path="/data/dialog.db")

    assert proc.returncode == 2
    assert "E2E_FATAL" in proc.stderr and "/data" in proc.stderr
    assert _param(tmp_path, "/dialog_node", "e2e_db_path") == ""


@bash_required
def test_v2_deactivate_stops_recording(tmp_path):
    proc, _ = _run_isolation(tmp_path, _scenario(tmp_path, "plain_v1"), engine="v2", tail="deactivate_e2e_dialog_db")

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert _param(tmp_path, "/dialog_node", "e2e_db_path") == ""
    assert "e2e_db_path=''" in proc.stdout


def test_trap_returns_dialog_node_from_e2e_db():
    src = E2E_SCRIPT.read_text(encoding="utf-8")
    trap_line = next(line for line in src.splitlines() if line.startswith("trap 'restore_robot_tts_default"))
    assert "deactivate_e2e_dialog_db" in trap_line


def test_v2_log_lines_match_harness_patterns():
    """Контракт «лог dialog_node v2 ↔ паттерны харнесса»: строки берутся из исходников v2."""
    node = (REPO_ROOT / "src/rob_box_voice/rob_box_voice/dialog_node.py").read_text(encoding="utf-8")
    turn = (REPO_ROOT / "src/rob_box_dialog/rob_box_dialog/turn.py").read_text(encoding="utf-8")
    address = (REPO_ROOT / "src/rob_box_dialog/rob_box_dialog/address.py").read_text(encoding="utf-8")
    grammar = (REPO_ROOT / "src/rob_box_dialog/rob_box_dialog/grammar.py").read_text(encoding="utf-8")
    assert 'f"[backlog] accumulated reason={decision.reason} entries="' in node
    assert re.search(r'NO_WAKE, [A-Z_, ]+= "no_wake"', address)
    assert 'f"{LOG_MARK} tier1 kind={command.kind} ' in turn and 'Tier1Command("new_session")' in grammar

    harness = E2E_SCRIPT.read_text(encoding="utf-8")
    backlog_re = re.search(r"check_backlog_accumulated\(\) \{.*?grep -qE '([^']+)'", harness, re.S).group(1)
    assert re.search(backlog_re, "[backlog] accumulated reason=no_wake entries=3")
    assert re.search(backlog_re, "🗒️ [backlog] accumulated (no_wake_word) doa=12.0 speaker='Саша'")

    act1 = json.loads((NIGHT / "night_marathon_act1_wakeup_v1.json").read_text(encoding="utf-8"))
    reset_pat = next(s for s in act1["steps"] if s["label"] == "n102_session_reset")["patterns"][0]
    assert re.search(reset_pat, "🧭 [turn-v2] tier1 kind=new_session args={} turn=ab12")
