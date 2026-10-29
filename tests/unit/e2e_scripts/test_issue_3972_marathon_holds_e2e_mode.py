"""
test_issue_3972_marathon_holds_e2e_mode.py - режим e2e держит раннер марафона, а не каждый акт.

Инцидент (разбор 23.10, §1.B): ``e2e_mode=ON`` 16:34 (акт 2) -> OFF 16:57 -> ON 17:00 (2b, БД стёрта) -> OFF 17:12 ->
ON 17:15 (2c). Каждый акт без ``inherits_previous_act`` включал режим заново, а включение стирает
``/data/speakers.e2e.db``: с акта 2b диктор был ``unknown``. Теперь раннер раздаёт актам фазу
(first|middle|last): стирает один first, middle/last каст не трогают, выключает last.

Два слоя:
  * харнесс - НАСТОЯЩИЙ ``isolate_act_start`` + ``marathon_e2e_mode`` на фейковом роботе (как в #2890);
  * раннер - НАСТОЯЩИЙ ``run_night_marathon.sh`` с фейковым ``gh``: смотрим, какие input'ы он диспатчит.

Run:
  python -m pytest tests/unit/e2e_scripts/test_issue_3972_marathon_holds_e2e_mode.py -v --no-cov
"""

from __future__ import annotations

import os
import stat
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from tests.unit.e2e_scripts.test_issue_2890_act_isolation import (
    BASH,
    NIGHT,
    REPO_ROOT,
    _bash_path,
    _copy_real,
    _owner,
    _require_bash_sees,
    _run_isolation,
    bash_required,
)

RUNNER = REPO_ROOT / "scripts" / "e2e" / "run_night_marathon.sh"
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "L-E2E Voice Test.yml"
HARNESS = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"

ACT2 = "night_marathon_act2_acquaintance_v1.json"
ACT2B = "night_marathon_act2b_identity_question_v1.json"
ACT2C = "night_marathon_act2c_voice_conflict_question_v1.json"
ACT3 = "night_marathon_act3_backlog_diarization_v1.json"


def _wipes(events):
    return [e for e in events if e.startswith("WIPE")]


# -- харнесс ------------------------------------------------------------------


@bash_required
def test_marathon_wipes_e2e_db_once_across_acts_2_2b_2c_3(tmp_path):
    """Сам инцидент: акты 2 -> 2b -> 2c (регистрируют дикторов) -> 3. Стирание ровно одно, на first."""
    plan = (("first", ACT2), ("middle", ACT2B), ("middle", ACT2C), ("last", ACT3))
    all_events = []
    for phase, name in plan:
        scenario = _copy_real(tmp_path, NIGHT / name)
        proc, events = _run_isolation(tmp_path, scenario, phase=phase, tail="leave_e2e_mode")
        assert "EXIT_OK" in proc.stdout, f"{name}: {proc.stderr}"
        assert "E2E_PRECONDITION speaker_db=e2e" in proc.stdout
        all_events.append((phase, events))

    wipes = [(phase, _wipes(ev)) for phase, ev in all_events]
    assert wipes[0][1] == ["WIPE /speaker_id_node", "WIPE /mcp_server"], "first обязан стереть e2e-БД (свежий каст)"
    for phase, w in wipes[1:]:
        assert w == [], f"акт phase={phase} стёр каст прошлых актов - в акте 3 диктор снова unknown (#3972)"


@bash_required
def test_first_and_middle_acts_do_not_turn_the_mode_off(tmp_path):
    for phase in ("first", "middle"):
        scenario = _copy_real(tmp_path, NIGHT / ACT2)
        if phase == "middle":  # режим уже включён актом first
            (tmp_path / "state").mkdir(exist_ok=True)
        preset = {"_speaker_id_nodee2e_mode": "true"} if phase == "middle" else None
        proc, _ = _run_isolation(tmp_path, scenario, phase=phase, preset=preset, tail="leave_e2e_mode")
        assert "EXIT_OK" in proc.stdout, proc.stderr
        assert "E2E_MODE off" not in proc.stdout, f"phase={phase} выключил режим - следующий акт пойдёт на боевую БД"
        assert _owner(tmp_path, "e2e_mode") == "true"
        assert _owner(tmp_path, "e2e_release_after_s") == "10800.0", "без таймера обрыв оставит робота на e2e-БД навсегда"


@bash_required
def test_last_act_turns_the_mode_off_without_wiping(tmp_path):
    scenario = _copy_real(tmp_path, NIGHT / ACT3)
    preset = {"_speaker_id_nodee2e_mode": "true"}

    proc, events = _run_isolation(tmp_path, scenario, phase="last", preset=preset, tail="leave_e2e_mode")

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert _wipes(events) == []
    assert "E2E_MODE off" in proc.stdout
    assert _owner(tmp_path, "e2e_mode") == "false", "последний акт обязан вернуть робота на боевые БД"


@bash_required
def test_last_act_ignores_legacy_hold_flag(tmp_path):
    scenario = _copy_real(tmp_path, NIGHT / ACT3)
    preset = {"_speaker_id_nodee2e_mode": "true"}

    proc, _ = _run_isolation(tmp_path, scenario, phase="last", hold=True, preset=preset, tail="leave_e2e_mode")

    assert "held_for_next_act" not in proc.stdout
    assert _owner(tmp_path, "e2e_mode") == "false"


@bash_required
@pytest.mark.parametrize("phase", ["middle", "last"])
def test_follow_up_act_without_mode_is_fatal_not_live(tmp_path, phase):
    """Режима нет (таймер истёк, нода перезапущена, first не гонялся) - честный фатал, а не прогон по боевой БД
    и не молчаливое включение со стиранием."""
    scenario = _copy_real(tmp_path, NIGHT / ACT2B)

    proc, events = _run_isolation(tmp_path, scenario, phase=phase)

    assert proc.returncode == 2
    assert "E2E_PRECONDITION speaker_db=live" in proc.stdout
    assert "E2E_FATAL" in proc.stderr and "#3972" in proc.stderr
    assert "EXIT_OK" not in proc.stdout
    assert _wipes(events) == [], "middle/last не имеют права включать режим и стирать БД"


@bash_required
def test_first_act_refuses_to_run_without_owner_release_timer(tmp_path):
    """Владелец не принял таймер -> оборванный марафон оставил бы e2e-режим без срока. Фатал; trap снимет режим."""
    scenario = _copy_real(tmp_path, NIGHT / ACT2)

    proc, _ = _run_isolation(tmp_path, scenario, phase="first", timer_reject=True, tail="leave_e2e_mode")

    assert proc.returncode == 2
    assert "E2E_FATAL" in proc.stderr and "таймер" in proc.stderr and "#3972" in proc.stderr
    assert "EXIT_OK" not in proc.stdout


@bash_required
def test_single_act_outside_marathon_behaves_as_before(tmp_path):
    """Без phase: акт сам включает, стирает и выключает (одиночный прогон, ротация)."""
    scenario = _copy_real(tmp_path, NIGHT / ACT2)

    proc, events = _run_isolation(tmp_path, scenario, tail="leave_e2e_mode")

    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert _wipes(events) == ["WIPE /speaker_id_node", "WIPE /mcp_server"]
    assert "E2E_MODE off" in proc.stdout
    assert _owner(tmp_path, "e2e_mode") == "false"
    assert _owner(tmp_path, "e2e_release_after_s") in ("", "0.0"), "одиночный акт не оставляет таймеров марафона"


@bash_required
def test_single_acts_each_wipe_as_before(tmp_path):
    """Контроль на ложный зелёный: без phase 2 -> 2b по-прежнему стирает дважды (поведение до #3972)."""
    wipes = 0
    for name in (ACT2, ACT2B):
        scenario = _copy_real(tmp_path, NIGHT / name)
        proc, events = _run_isolation(tmp_path, scenario, tail="leave_e2e_mode")
        assert "EXIT_OK" in proc.stdout, proc.stderr
        wipes += len([e for e in events if e == "WIPE /speaker_id_node"])
    assert wipes == 2


def test_invalid_phase_is_rejected_by_the_harness_args():
    src = HARNESS.read_text(encoding="utf-8")
    assert "--e2e-marathon-phase) E2E_MARATHON_PHASE=" in src
    assert 'first|middle|last) ;;' in src


# -- workflow -----------------------------------------------------------------


def _workflow():
    return yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))


def test_workflow_input_default_keeps_old_behavior():
    inputs = (_workflow().get(True) or _workflow()["on"])["workflow_dispatch"]["inputs"]
    assert inputs["e2e_marathon_phase"]["default"] == ""
    assert inputs["e2e_marathon_phase"]["required"] is False
    assert len(inputs) <= 25, "GitHub: не больше 25 inputs у workflow_dispatch"


def test_workflow_passes_phase_to_harness_only_when_valid():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "E2E_MARATHON_PHASE: ${{ inputs.e2e_marathon_phase }}" in text
    assert 'CMD="$CMD --e2e-marathon-phase $E2E_PHASE_CLEAN"' in text
    assert "first|middle|last)" in text


# -- раннер -------------------------------------------------------------------

FAKE_GH = r'''#!/usr/bin/env bash
# Фейковый gh: пишет диспатчи в $FAKE_GH_LOG; FAKE_GH_FAIL_DISPATCH_AT=N - N-й диспатч не проходит.
case "$1 $2" in
    "workflow run")
        n=$(( $(cat "$FAKE_GH_LOG.n" 2>/dev/null || echo 0) + 1 )); echo "$n" > "$FAKE_GH_LOG.n"
        if [ "${FAKE_GH_FAIL_DISPATCH_AT:-0}" = "$n" ]; then exit 1; fi
        echo "DISPATCH $*" >> "$FAKE_GH_LOG"; exit 0 ;;
    "run list") echo 4242; exit 0 ;;
    "run view")
        case "$*" in *--log*) exit 0 ;; esac
        echo "completed success"; exit 0 ;;
esac
exit 0
'''


def _run_runner(tmp_path: Path, acts: str = "", *, fail_dispatch_at: int = 0, release_cmd: str = ""):
    fake_dir = tmp_path / "bin"
    fake_dir.mkdir(exist_ok=True)
    gh = fake_dir / "gh"
    gh.write_text(FAKE_GH, encoding="utf-8", newline="\n")
    gh.chmod(gh.stat().st_mode | stat.S_IEXEC)
    log = tmp_path / "gh.log"
    log.write_text("", encoding="utf-8")
    (tmp_path / "gh.log.n").unlink(missing_ok=True)
    _require_bash_sees(tmp_path)
    env = {
        "PATH": f"{_bash_path(fake_dir)}{os.pathsep}{os.environ.get('PATH', '')}",
        "PYTHONIOENCODING": "utf-8", "PYTHONUTF8": "1",
        "SKIP_SENTINEL": "1", "POLL_SEC": "0", "DISPATCH_WAIT_SEC": "0", "MAX_WAIT": "5",
        "REPORT": _bash_path(tmp_path / "report"), "ACTS": acts, "PY_BIN": _bash_path(Path(sys.executable)),
        "FAKE_GH_LOG": _bash_path(log), "FAKE_GH_FAIL_DISPATCH_AT": str(fail_dispatch_at),
        "E2E_RELEASE_CMD": release_cmd, "HOME": os.environ.get("HOME", str(tmp_path)),
        "SYSTEMROOT": os.environ.get("SYSTEMROOT", ""),
    }
    proc = subprocess.run([BASH, _bash_path(RUNNER)], capture_output=True, text=True, encoding="utf-8",
                          timeout=120, env=env)
    dispatches = [ln for ln in log.read_text(encoding="utf-8").splitlines() if ln.startswith("DISPATCH")]
    return proc, dispatches


def _phase_of(dispatch: str) -> str:
    for token in dispatch.split():
        if token.startswith("e2e_marathon_phase="):
            return token.split("=", 1)[1]
    return ""


@bash_required
def test_runner_gives_first_middle_last_phases_and_no_legacy_hold(tmp_path):
    proc, dispatches = _run_runner(tmp_path)

    assert proc.returncode == 0, proc.stdout + proc.stderr
    phases = [_phase_of(d) for d in dispatches]
    assert len(dispatches) == 12
    assert phases[0] == "first" and phases[-1] == "last"
    assert phases.count("first") == 1 and phases.count("last") == 1
    assert set(phases[1:-1]) == {"middle"}
    assert not [d for d in dispatches if "e2e_hold_for_next_act" in d], "старый per-act hold не должен идти вместе с фазами"
    assert "не дойдя до акта last" not in proc.stdout


@bash_required
def test_runner_single_act_is_standalone(tmp_path):
    proc, dispatches = _run_runner(tmp_path, acts="2")

    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert len(dispatches) == 1
    assert _phase_of(dispatches[0]) == "" and "e2e_hold_for_next_act" not in dispatches[0]


@bash_required
def test_runner_plan_starting_with_inheriting_act_keeps_legacy_hold(tmp_path):
    """ACTS=3,4: первый акт наследует - без режима он обязан упасть на E2E_PRECONDITION, а не получить пустой каст."""
    proc, dispatches = _run_runner(tmp_path, acts="3,4")

    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert [_phase_of(d) for d in dispatches] == ["", ""]
    assert "e2e_hold_for_next_act=true" in dispatches[0] and "e2e_hold_for_next_act" not in dispatches[1]


@bash_required
def test_runner_dry_run_prints_phase_per_act(tmp_path):
    env = {"PATH": os.environ.get("PATH", ""), "DRY_RUN": "1", "ACTS": "1,2,3,4", "SKIP_SENTINEL": "1",
           "PYTHONUTF8": "1", "REPORT": _bash_path(tmp_path / "r"), "PY_BIN": _bash_path(Path(sys.executable)),
           "HOME": os.environ.get("HOME", str(tmp_path)), "SYSTEMROOT": os.environ.get("SYSTEMROOT", "")}
    proc = subprocess.run([BASH, _bash_path(RUNNER)], capture_output=True, text=True, encoding="utf-8",
                          timeout=60, env=env)
    out = proc.stdout
    assert out.count("режим e2e марафона: first") == 1 and out.count("режим e2e марафона: last") == 1
    assert out.count("режим e2e марафона: middle") == 2, out


@bash_required
def test_runner_aborted_before_last_act_releases_mode_via_hook(tmp_path):
    """Диспатч 2-го акта упал: last не отработал, режим остался на роботе - раннер обязан это сказать и вызвать хук."""
    marker = tmp_path / "released"
    proc, dispatches = _run_runner(tmp_path, acts="1,2", fail_dispatch_at=2,
                                   release_cmd=f"echo done > {_bash_path(marker)}")

    assert len(dispatches) == 1 and _phase_of(dispatches[0]) == "first"
    assert "не дойдя до акта last" in proc.stdout
    assert marker.exists(), "E2E_RELEASE_CMD не выполнена - робот остался бы на e2e-БД до таймера"


@bash_required
def test_runner_clean_marathon_does_not_run_release_hook(tmp_path):
    marker = tmp_path / "released"
    proc, _ = _run_runner(tmp_path, acts="1,2", release_cmd=f"echo done > {_bash_path(marker)}")

    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert not marker.exists(), "last отработал и сам снял режим - хук раннера лишний"
