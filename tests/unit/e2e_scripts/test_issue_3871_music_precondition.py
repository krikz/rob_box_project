"""Issue #3871 -- предусловие акта: музыка/DJ-сет глушатся ДО первого хода, не вышло -- E2E_FATAL.

22.10 замер акта 1 (run 39849541397) шёл под DJ-сетом, запущенным из Telegram: во всех ходах
``<music_state>{"playing":"yes",...,"dj":"on"}``. Здесь исполняется НАСТОЯЩАЯ ``ensure_music_stopped_for_act``
из ``e2e_voice_lib.sh`` и НАСТОЯЩИЙ python, который она отправляет в контейнер (подписанный /mcp/execute,
разбор снимка настоящим ``rob_box_voice.core.music_player_state``); робот -- ``fake_robot_ssh.sh`` с
файловым «плеером» (``ros_stubs/``).

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_3871_music_precondition.py -v
"""

from __future__ import annotations

import json
import shutil
import subprocess
import time
from pathlib import Path

import pytest

from tests.unit.e2e_scripts.test_issue_3857_run_step import extract_function

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
E2E_SCRIPT = SCRIPTS_DIR / "e2e_voice_test.sh"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
FAKE = Path(__file__).resolve().parent / "fake_robot_ssh.sh"

needs_bash = pytest.mark.skipif(shutil.which("bash") is None or shutil.which("python3") is None,
                                reason="нужны bash и python3")


def _snapshot(*, state="playing", dj=False, stops_at=None, track_id="set1:3:abc"):
    return json.dumps({"state": state, "track_id": track_id if state == "playing" else None,
                       "form_ends_at": None, "stops_at": stops_at, "dj": {"enabled": dj},
                       "finished_track_id": None, "ts": time.time()})


PLAYING_DJ = _snapshot(dj=True)
PLAYING_PLAIN = _snapshot(dj=False)
IDLE = _snapshot(state="idle")


def _run(tmp_path, *, snapshot=None, env=None, inherits="no", wait="3"):
    """-> (rc, stdout, stderr, state_dir). snapshot=None -- плеер ничего не публиковал."""
    state = tmp_path / "state"
    state.mkdir()
    if snapshot is not None:
        (state / "music_state.json").write_text(snapshot, encoding="utf-8")
    script = "\n".join([
        "set -u",
        'export FAKE_ROBOT_STATE="%s"' % state.as_posix(),
        *("export %s='%s'" % kv for kv in (env or {}).items()),
        'ROBOT_SSH="bash %s"' % FAKE.as_posix(),
        # маркер E2E_RECOVER строит e2e_events.recover_marker (ревью #4034), как в главном потоке харнесса
        'SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
        "E2E_MUSIC_IDLE_WAIT=%s" % wait,
        'log() { echo ">>> $*"; }',
        LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
        extract_function("robot_ros"),
        'ensure_music_stopped_for_act "%s"; echo "AFTER_RC=$?"' % inherits,
    ])
    p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=120)
    return p.returncode, p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace"), state


def _calls(state):
    f = state / "mcp_calls"
    return [json.loads(ln) for ln in f.read_text(encoding="utf-8").splitlines()] if f.exists() else []


@needs_bash
class TestMusicPrecondition:
    def test_dj_set_playing_is_stopped_then_idle(self, tmp_path):
        rc, out, err, state = _run(tmp_path, snapshot=PLAYING_DJ)
        assert rc == 0 and "AFTER_RC=0" in out, (out, err)
        assert "E2E_PRECONDITION music=stopped" in out
        calls = _calls(state)
        assert [(c["tool"], c["parameters"]) for c in calls] == [("dj_set", {"action": "stop"}), ("stop_music", {})]
        assert all(c["signed"] for c in calls), "стоп обязан идти подписанным /mcp/execute"
        assert json.loads((state / "music_state.json").read_text(encoding="utf-8"))["state"] == "idle"

    def test_plain_track_without_dj_is_stopped(self, tmp_path):
        rc, out, err, state = _run(tmp_path, snapshot=PLAYING_PLAIN)
        assert rc == 0 and "E2E_PRECONDITION music=stopped" in out, (out, err)
        assert "stop_music" in [c["tool"] for c in _calls(state)]

    def test_dj_enabled_between_tracks_is_stopped(self, tmp_path):
        """Сет включён, но трек в паузе между пьесами (state=idle, dj.enabled) -- сет следующий трек заведёт."""
        rc, out, err, state = _run(tmp_path, snapshot=_snapshot(state="idle", dj=True))
        assert rc == 0 and "E2E_PRECONDITION music=stopped" in out, (out, err)
        assert _calls(state)[0]["tool"] == "dj_set"

    def test_silence_is_was_idle_and_robot_not_touched(self, tmp_path):
        rc, out, err, state = _run(tmp_path, snapshot=IDLE)
        assert rc == 0 and "E2E_PRECONDITION music=was_idle" in out, (out, err)
        assert _calls(state) == [], "в тишине стоп-тулы не зовём"

    def test_player_never_published_is_was_idle(self, tmp_path):
        rc, out, err, state = _run(tmp_path, snapshot=None)
        assert rc == 0 and "E2E_PRECONDITION music=was_idle" in out, (out, err)
        assert "snapshot=нет" in out and _calls(state) == []

    def test_stale_latched_playing_past_stops_at_is_idle(self, tmp_path):
        """Конечный трек давно замолчал, а latched «playing» остался: парсер плеера считает это тишиной."""
        rc, out, err, state = _run(tmp_path, snapshot=_snapshot(stops_at=time.time() - 600))
        assert rc == 0 and "E2E_PRECONDITION music=was_idle" in out, (out, err)
        assert _calls(state) == []

    def test_music_that_will_not_stop_is_fatal(self, tmp_path):
        rc, out, err, state = _run(tmp_path, snapshot=PLAYING_DJ, env={"FAKE_MUSIC_STUCK": "1"}, wait="2")
        assert rc == 2, (rc, out, err)
        assert "E2E_FATAL: перед актом играет музыка" in err
        assert "AFTER_RC" not in out, "после FATAL акт идти не должен"
        assert "E2E_PRECONDITION" not in out
        assert len(_calls(state)) == 2, "стоп всё же отправлен"

    def test_slow_stop_within_wait_is_ok(self, tmp_path):
        rc, out, err, state = _run(tmp_path, snapshot=PLAYING_DJ, env={"FAKE_MUSIC_STOP_DELAY": "1"}, wait="6")
        assert rc == 0 and "E2E_PRECONDITION music=stopped" in out, (out, err)

    def test_stop_slower_than_wait_is_fatal(self, tmp_path):
        rc, out, err, state = _run(tmp_path, snapshot=PLAYING_DJ, env={"FAKE_MUSIC_STOP_DELAY": "6"}, wait="2")
        assert rc == 2 and "E2E_FATAL: перед актом играет музыка" in err, (rc, out, err)

    def test_no_secret_to_sign_is_fatal_not_silent(self, tmp_path):
        rc, out, err, state = _run(tmp_path, snapshot=PLAYING_DJ, env={"FAKE_NO_SIGN": "1"})
        assert rc == 2, (rc, out, err)
        assert "E2E_FATAL: перед актом не удалось проверить" in err and "secret" in err
        assert _calls(state) == []

    def test_inherited_act_stops_leaked_music_and_marks_it(self, tmp_path):
        """#4027: наследующий акт больше не пропускает проверку -- n712 не остановил музыку, и она шла весь акт 8.
        Играет на старте -- утечка прошлого акта: стоп и пометка E2E_RECOVER, а не тихий PASS под музыку."""
        rc, out, err, state = _run(tmp_path, snapshot=PLAYING_DJ, inherits="yes")
        assert rc == 0 and "E2E_RECOVER music at_act_start" in out, (out, err)
        assert "E2E_PRECONDITION music=skipped" not in out
        assert [c["tool"] for c in _calls(state)] == ["dj_set", "stop_music"]

    def test_inherited_act_in_silence_is_was_idle(self, tmp_path):
        rc, out, err, state = _run(tmp_path, snapshot=IDLE, inherits="yes")
        assert rc == 0 and "E2E_PRECONDITION music=was_idle" in out and "E2E_RECOVER" not in out, (out, err)
        assert _calls(state) == []


def test_main_script_runs_precondition_after_isolation_before_first_step():
    src = E2E_SCRIPT.read_text(encoding="utf-8").replace("\r\n", "\n")
    isolate = src.index('    isolate_act_start "$SCENARIO_FILE"\n')
    ensure = src.index('ensure_music_stopped_for_act "$_music_inherits"')
    node_params = src.index('    apply_node_params "$SCENARIO_FILE"\n')
    assert isolate < ensure < node_params
    assert 'scenario_inherits_previous_act "$SCENARIO_FILE" && _music_inherits=yes' in src


def test_restore_and_music_share_one_signing_implementation():
    """Подпись /mcp/execute -- в одном месте (_e2e_mcp_client_py), а не по копии на задачу."""
    lib = LIB.read_text(encoding="utf-8").replace("\r\n", "\n")
    assert lib.count('RequestAuthenticator.from_env(sender="harness")') == 1
    assert lib.count("auth.sign(request)") == 1
