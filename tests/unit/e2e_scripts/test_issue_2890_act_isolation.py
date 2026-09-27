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
    "_read_mcp_e2e_mode",
    "activate_e2e_speaker_db",
    "activate_e2e_memory_db",
    "_read_dialogue_reset_token",
    "reset_dialogue_session_for_act",
    "isolate_act_start",
)

# Фейковый робот: ``robot_ros "ros2 param set|get /node name [value] --no-daemon"``.
FAKE_ROBOT = r'''
log() { echo ">>> $*"; }
_pfile() { echo "$STATE/$(echo "$1$2" | tr '/' '_')"; }
robot_ros() {
    # shellcheck disable=SC2086
    set -- $*
    local verb="$3" node="$4" name="$5" value="$6" f
    f="$(_pfile "$node" "$name")"
    case "$verb" in
        set)
            case "$name" in
                e2e_mode)
                    if [ "$(cat "$f" 2>/dev/null || echo false)" = false ] && [ "$value" = true ]; then
                        echo "WIPE $node" >> "$STATE/events"
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
            esac
            ;;
        get)
            case "$name" in
                e2e_mode)
                    if [ "$(cat "$f" 2>/dev/null || echo false)" = true ]; then
                        echo "Boolean value is: True"
                    else
                        echo "Boolean value is: False"
                    fi
                    ;;
                e2e_session_reset_token)
                    echo "String value is: $(cat "$f" 2>/dev/null)"
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


def _run_isolation(tmp_path: Path, scenario: Path, *, reject=False, preset=None):
    state = tmp_path / "state"
    state.mkdir()
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
        f"{FAKE_ROBOT}\n{body}\n"
        f'isolate_act_start "{_bash_path(scenario)}"\n'
        'echo "EXIT_OK"\n'
    )
    env = {"PATH": os.environ.get("PATH", ""), "PYTHONIOENCODING": "utf-8"}
    if reject:
        env["FAKE_DIALOGUE_REJECT"] = "1"
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


@bash_required
def test_plain_act_resets_dialogue_without_touching_dbs(tmp_path):
    """Акт без регистрации и memory_save: только сброс окна диалога."""
    proc, events = _run_isolation(tmp_path, _scenario(tmp_path, "plain_v1"))

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert events == ["RESET /dialogue_node"]


@bash_required
def test_inheriting_act_keeps_dialogue_window(tmp_path):
    """Акт 3 марафона намеренно продолжает разговор — сброса нет, но в логе
    это сказано явно."""
    scenario = _copy_real(tmp_path, NIGHT / "night_marathon_act3_backlog_diarization_v1.json")

    proc, events = _run_isolation(tmp_path, scenario)

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "RESET /dialogue_node" not in events
    assert "inherits_previous_act=true" in proc.stdout


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
    trap_at = src.index("trap 'restore_node_params; deactivate_e2e_speaker_db")
    call_at = src.index('isolate_act_start "$SCENARIO_FILE"')
    assert trap_at < call_at
