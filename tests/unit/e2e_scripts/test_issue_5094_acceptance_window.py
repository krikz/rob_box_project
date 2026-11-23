"""#5094: окно приёмки (сценарий) и предпроверка блокировки привода.

Робота нет: сценарий читается с диска и прогоняется через штатный парсер
харнесса (``parse_scenario_to_tsv``), предпроверка — на синтетических снимках
``ros2 topic echo`` / ``ros2 lifecycle get``. Скрипт ничего не публикует.
"""

from __future__ import annotations

import json
import subprocess
import textwrap
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
SCENARIO = REPO / ".github" / "e2e" / "scenarios" / "5094_speech_and_animation_v1.json"
ACCEPTANCE = (
    REPO
    / ".github"
    / "e2e"
    / "scenarios"
    / "5094_speech_and_animation_acceptance_v1.json"
)
SCRIPT = REPO / "scripts" / "e2e" / "check_drive_locked.py"
LIB = REPO / "scripts" / "ci" / "e2e" / "e2e_voice_lib.sh"

PHRASE = "Робот, расскажи анекдот и покажи анимацию."
SPEECH = "SpeechEvent started speech_id"
ANIMATION = "лицо команды на done"

EMERGENCY_ZERO = textwrap.dedent(
    """\
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
    """
)
EMERGENCY_MOVING = EMERGENCY_ZERO.replace("x: 0.0", "x: 0.1", 1)
LOCK_TRUE = "data: true\n"
LOCK_FALSE = "data: false\n"
LIFECYCLE_INACTIVE = "Using ZID: abc\ninactive [1]\n"
LIFECYCLE_ACTIVE = "active [3]\n"

NAV_NODES = ("/controller_server", "/planner_server", "/bt_navigator")


def _run_file(blob: str) -> subprocess.CompletedProcess:
    import tempfile

    with tempfile.NamedTemporaryFile(
        "w", suffix=".json", encoding="utf-8", delete=False
    ) as handle:
        handle.write(blob)
        name = handle.name
    return subprocess.run(
        ["python3", str(SCRIPT), "--snapshots", name],
        capture_output=True,
        text=True,
        check=False,
    )


def _signs(output: str) -> dict:
    found = {}
    for line in output.splitlines():
        if line.startswith(("PASS ", "FAIL ")):
            mark, _, rest = line.partition(" ")
            name, _, raw = rest.partition(": ")
            found[name] = (mark, raw)
    return found


def test_scenario_is_one_synthetic_phrase_asking_speech_and_animation():
    scenario = json.loads(SCENARIO.read_text(encoding="utf-8"))
    assert scenario["inherits_previous_act"] is False
    assert scenario["cast"] == []
    steps = scenario["steps"]
    assert len(steps) == 1
    step = steps[0]
    assert step["text"] == PHRASE
    assert step["text"].startswith("Робот,")
    assert "анекдот" in step["text"] and "анимац" in step["text"]
    assert "silent_ok" not in step
    assert step["expect"] == "cycle"
    patterns = " ".join(step["patterns"])
    assert SPEECH in patterns
    assert ANIMATION in patterns
    assert "play_animation" in patterns
    # Окно само скорость не публикует и целей не шлёт.
    blob = json.dumps(scenario)
    assert "cmd_vel" not in blob
    assert "navigate" not in blob


def test_acceptance_file_sits_next_to_scenario():
    acceptance = json.loads(ACCEPTANCE.read_text(encoding="utf-8"))
    assert acceptance["issue"] == 5094
    assert "n5094_anekdot_anim" in acceptance["steps"]
    assert SCENARIO.parent == ACCEPTANCE.parent


def test_harness_parser_keeps_phrase_and_both_patterns():
    """Тот же парсер, что зовёт раннер: поля не съезжают, фраза доходит целиком."""
    out = Path("/tmp") / "5094_scenario_parsed.txt"
    completed = subprocess.run(
        ["bash", "-c", f'source "{LIB}" && parse_scenario_to_tsv "{SCENARIO}" "{out}"'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    row = out.read_text(encoding="utf-8").rstrip("\n")
    fields = row.split("\x1f")
    assert len(fields) == 16, fields
    assert fields[1] == "n5094_anekdot_anim"
    assert fields[2] == PHRASE
    assert fields[3] == "anton"
    patterns = json.loads(fields[4])
    assert any(SPEECH in pat for pat in patterns)
    assert any(ANIMATION in pat for pat in patterns)
    assert fields[6] == "cycle"
    assert fields[14] == "0"  # silent_ok: ход без речи не допустим


def test_phrase_fits_vad_window():
    completed = subprocess.run(
        [
            "python3",
            "-c",
            "import importlib.util, sys\n"
            f"spec = importlib.util.spec_from_file_location('phrase_budget', r'{REPO / 'scripts' / 'e2e' / 'phrase_budget.py'}')\n"
            "module = importlib.util.module_from_spec(spec)\n"
            "sys.modules[spec.name] = module\n"
            "spec.loader.exec_module(module)\n"
            f"judged = module.verdict('n5094_anekdot_anim', 'anton', {PHRASE!r}, {{}})\n"
            "assert judged.level == module.OK, judged\n"
            f"assert len({PHRASE!r}) <= module.char_limit('anton')\n",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr


def test_emergency_zero_alone_is_pass_and_prints_raw():
    result = _run_file(json.dumps({"cmd_vel_emergency": EMERGENCY_ZERO}))
    assert result.returncode == 0, result.stdout
    signs = _signs(result.stdout)
    assert signs["cmd_vel_emergency"][0] == "PASS"
    assert "0.0" in signs["cmd_vel_emergency"][1]
    assert signs["joystick_lock"][0] == "FAIL"
    assert signs["teleop_lock"][0] == "FAIL"
    for node in NAV_NODES:
        assert signs[f"nav2 {node}"][0] == "FAIL"
    assert "PASS drive locked by: cmd_vel_emergency" in result.stdout


def test_lock_true_alone_is_pass():
    result = _run_file(json.dumps({"teleop_lock": LOCK_TRUE}))
    assert result.returncode == 0, result.stdout
    signs = _signs(result.stdout)
    assert signs["teleop_lock"][0] == "PASS"
    assert "data: true" in signs["teleop_lock"][1]
    assert "PASS drive locked by: teleop_lock" in result.stdout


def test_joystick_lock_true_names_its_priority():
    result = _run_file(json.dumps({"joystick_lock": LOCK_TRUE}))
    assert result.returncode == 0, result.stdout
    assert "priority 51" in result.stdout
    assert "data: true" in result.stdout


def test_both_locks_false_and_no_emergency_is_fail():
    """False — «ехать можно» (lock_block), не блокировка. Молчание тоже не блокировка."""
    result = _run_file(
        json.dumps(
            {
                "cmd_vel_emergency": "",
                "joystick_lock": LOCK_FALSE,
                "teleop_lock": LOCK_FALSE,
            }
        )
    )
    assert result.returncode != 0
    assert "FAIL drive NOT locked" in result.stdout
    signs = _signs(result.stdout)
    assert signs["joystick_lock"][0] == "FAIL"
    assert signs["teleop_lock"][0] == "FAIL"
    assert "data: false" in signs["teleop_lock"][1]


def test_nonzero_emergency_is_not_a_stop():
    result = _run_file(json.dumps({"cmd_vel_emergency": EMERGENCY_MOVING}))
    assert result.returncode != 0
    signs = _signs(result.stdout)
    assert signs["cmd_vel_emergency"][0] == "FAIL"
    assert "0.1" in signs["cmd_vel_emergency"][1]


def test_nav_inactive_alone_is_pass_and_zenoh_prefix_is_ignored():
    snapshots = {node: LIFECYCLE_INACTIVE for node in NAV_NODES}
    result = _run_file(json.dumps(snapshots))
    assert result.returncode == 0, result.stdout
    signs = _signs(result.stdout)
    for node in NAV_NODES:
        assert signs[f"nav2 {node}"][0] == "PASS"
        assert "inactive" in signs[f"nav2 {node}"][1]
        assert "Using ZID" in signs[f"nav2 {node}"][1]
    assert "PASS drive locked by:" in result.stdout


def test_nav_active_is_not_a_lock():
    snapshots = {node: LIFECYCLE_ACTIVE for node in NAV_NODES}
    result = _run_file(json.dumps(snapshots))
    assert result.returncode != 0
    signs = _signs(result.stdout)
    for node in NAV_NODES:
        assert signs[f"nav2 {node}"][0] == "FAIL"
        assert "active" in signs[f"nav2 {node}"][1]


def test_empty_snapshots_fail_every_sign():
    result = _run_file("{}")
    assert result.returncode != 0
    signs = _signs(result.stdout)
    assert len(signs) == 7  # 6 признаков + итоговая строка
    assert signs["drive NOT locked"][0] == "FAIL"
    assert all(mark == "FAIL" for mark, _ in signs.values())


def test_capture_cmd_reads_fake_probe_and_does_not_publish():
    """Команда чтения подставляется снаружи. Скрипт только зовёт её и печатает сырьё."""
    probe = textwrap.dedent(
        """\
        #!/bin/sh
        echo "probe $1" >> /tmp/5094_probe_calls.txt
        case "$1" in
          teleop_lock) printf 'data: true\\n' ;;
          *) printf '' ;;
        esac
        """
    )
    import tempfile

    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "probe.sh"
        path.write_text(probe, encoding="utf-8")
        path.chmod(0o755)
        calls = Path("/tmp") / "5094_probe_calls.txt"
        if calls.exists():
            calls.unlink()
        completed = subprocess.run(
            ["python3", str(SCRIPT), "--capture-cmd", f"{path} {{name}}"],
            capture_output=True,
            text=True,
            check=False,
        )
        assert completed.returncode == 0, completed.stdout + completed.stderr
        assert "PASS teleop_lock" in completed.stdout
        assert "data: true" in completed.stdout
        called = calls.read_text(encoding="utf-8")
        assert "teleop_lock" in called
        assert "cmd_vel_emergency" in called
        # Пробник только дописывал файл вызовов — публикации в выводе нет.
        assert "publish" not in completed.stdout


def test_script_has_no_publish_and_no_robot_host():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "create_publisher" not in text
    assert "topic pub" not in text
    assert "10.1.1." not in text
    assert "ssh " not in text


@pytest.mark.parametrize(
    "raw, ok",
    [
        ("data: true\n", True),
        ("data: True\n", True),
        ("data: false\n", False),
        ("", False),
        ("не разобрано\n", False),
    ],
)
def test_lock_parser_is_strict(raw, ok):
    result = _run_file(json.dumps({"teleop_lock": raw}))
    signs = _signs(result.stdout)
    assert signs["teleop_lock"][0] == ("PASS" if ok else "FAIL")
    assert (result.returncode == 0) is ok
    assert (
        raw.strip() in signs["teleop_lock"][1] or "нет Bool" in signs["teleop_lock"][1]
    )
