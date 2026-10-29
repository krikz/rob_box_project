# -*- coding: utf-8 -*-
"""#3710 (эпик #3683, П5 #4077): n707 и n708 судят «эту песню» акта по id, а не по факту вызова ``music_library``.

Ложный PASS марафона-4: оба шага проверяли только ``Инструмент music_library выполнен успешно``. Робот мог рассказать
или включить ЛЮБОЙ трек библиотеки, и шаг зеленел. Теперь поле шага ``track_ref``:

* id песни акта называет сам заказ (``🎤 [song] … track=<id>``, ``engine/gen/song.py``); строки нет -- ``SKIP``, не PASS;
* вызов тула в окне шага (``📥 Запрос выполнения: music_library с параметрами {…}``) обязан нести ЭТОТ id при нужном
  ``action``; чужой id, пустой id и другой ``action`` -- красный.

Судит ``e2e_events`` (``track_ref*``); поле пишет генератор марафона (``TrackCall``, ``TRACK_REFS``).

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_3710_track_ref.py -v -o addopts=""
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from tests.unit.e2e_scripts.test_issue_3857_event_waits import _feed, _l, needs_bash
from tests.unit.e2e_scripts.test_issue_3989_recover import _act_start

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
EVENTS = SCRIPTS_DIR / "e2e_events.py"
NIGHT = REPO_ROOT / ".github" / "e2e" / "scenarios" / "night"
ACT7 = NIGHT / "night_marathon_act7_neuro_composer_v1.json"

for _dir in (SCRIPTS_DIR, REPO_ROOT / "scripts" / "e2e"):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))
import e2e_events as eev  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

SONG_ID = "c0ffee00-1111-4222-8333-444455556666"
OTHER_ID = "deadbeef-0000-4000-8000-000000000001"
#: Строка заказа песни как её пишет ``SongStudio.make`` (engine/gen/song.py).
SONG_LINE = ("🎤 [song] «Енот-механик» minimax:Calm_Woman жанр=pop task=t-77 credits=12 "
             "track=%s за 118 с")


def song(track=SONG_ID, ts="{NOW-100}"):
    return _l("mcp_server", ts, SONG_LINE % track)


def call(tool, params, ts="{NOW-20}"):
    return _l("mcp_server", ts, "📥 Запрос выполнения: %s с параметрами %s" % (tool, params))


def rule_of(label):
    scenario = json.loads(ACT7.read_text(encoding="utf-8"))
    step = next(s for s in scenario["steps"] if s["label"] == label)
    return eev.track_ref(step)


# -- генератор: поле у n707 и n708 и только у них ----------------------------------------------------------------


def test_n707_and_n708_carry_track_ref_from_the_generator():
    scenario = json.loads(ACT7.read_text(encoding="utf-8"))
    refs = {s["label"]: s["track_ref"] for s in scenario["steps"] if "track_ref" in s}
    assert sorted(refs) == ["n707_gen_track_info", "n708_gen_play"]
    assert refs["n707_gen_track_info"] == {
        "reason": "CASCADE_N702", "log": gen.SONG_TRACK_LOG, "tool": "music_library", "action": "info", "wait_sec": 240}
    assert refs["n708_gen_play"] == {
        "reason": "CASCADE_N702", "log": gen.SONG_TRACK_LOG, "tool": "music_library", "action": "play"}


def test_rule_is_a_step_field_not_a_label_match():
    """Правило едет полем шага: сценарий приходит на робота как ``/tmp/e2e_scenario.json``, по имени файла не работает,
    а по имени шага -- не должно: тот же шаг под другим label несёт то же правило."""
    steps = json.loads(ACT7.read_text(encoding="utf-8"))["steps"]
    n707 = next(s for s in steps if s["label"] == "n707_gen_track_info")
    renamed = {"steps": [{**n707, "label": "zzz"}]}
    assert [label for label, _ in eev.track_ref_rules(renamed)] == ["zzz"]
    assert [label for label, _ in eev.track_ref_rules({"steps": [{**n707, "label": "zzz", "track_ref": None}]})] == []


def test_no_other_generated_step_has_track_ref():
    for path in sorted(NIGHT.glob("night_marathon_act*_v1.json")):
        if path.name.endswith("_acceptance_v1.json"):
            continue
        labels = [label for label, _ in eev.track_ref_rules(json.loads(path.read_text(encoding="utf-8")))]
        assert labels == ([] if "act7" not in path.name else ["n707_gen_track_info", "n708_gen_play"]), path.name


def test_song_track_log_reads_the_real_song_line():
    rule = rule_of("n707_gen_track_info")
    got = eev.track_ref_bind(rule, _feed(song()))
    assert got["fetch"] == 1 and got["track"] == SONG_ID and "track=" + SONG_ID in got["evidence"]


# -- привязка: id из лога акта ------------------------------------------------------------------------------------


class TestBind:
    def test_no_song_line_means_no_id(self):
        rule = rule_of("n707_gen_track_info")
        for feed in (_feed(""), _feed(_l("mcp_server", "{NOW-10}", "🎤 [song] не вышло: задача t1 без треков"),),
                     _feed(call("music_library", "{'action': 'list'}"))):
            assert eev.track_ref_bind(rule, feed) == {"fetch": 1, "track": "-", "evidence": "-"}

    def test_unreadable_log_is_not_proof(self):
        rule = rule_of("n707_gen_track_info")
        assert eev.track_ref_bind(rule, "Traceback\n" + song())["fetch"] == 0

    def test_last_song_of_the_act_wins(self):
        rule = rule_of("n707_gen_track_info")
        got = eev.track_ref_bind(rule, _feed(song(OTHER_ID, "{NOW-300}") + song(SONG_ID, "{NOW-100}")))
        assert got["track"] == SONG_ID


# -- вердикт вызова -----------------------------------------------------------------------------------------------


class TestVerdict:
    @pytest.mark.parametrize("label,action", [("n707_gen_track_info", "info"), ("n708_gen_play", "play")])
    def test_call_with_the_acts_track_passes(self, label, action):
        rule = rule_of(label)
        log = call("music_library", "{'action': '%s', 'track_id': '%s'}" % (action, SONG_ID))
        got = eev.track_ref_verdict(rule, SONG_ID, _feed(log))
        assert got["ok"] == 1 and SONG_ID in got["detail"], got

    def test_param_order_does_not_matter(self):
        rule = rule_of("n707_gen_track_info")
        log = call("music_library", "{'track_id': '%s', 'action': 'info'}" % SONG_ID)
        assert eev.track_ref_verdict(rule, SONG_ID, _feed(log))["ok"] == 1

    def test_tool_called_without_any_id_is_the_old_false_green(self):
        rule = rule_of("n707_gen_track_info")
        log = (call("music_library", "{'action': 'info'}")
               + _l("mcp_server", "{NOW-19}", "✅ Инструмент music_library выполнен успешно"))
        got = eev.track_ref_verdict(rule, SONG_ID, _feed(log))
        assert got["ok"] == 0 and "(пусто)" in got["detail"]

    def test_other_track_fails_and_names_both(self):
        rule = rule_of("n708_gen_play")
        log = call("music_library", "{'action': 'play', 'track_id': '%s'}" % OTHER_ID)
        got = eev.track_ref_verdict(rule, SONG_ID, _feed(log))
        assert got["ok"] == 0 and OTHER_ID in got["detail"] and SONG_ID in got["detail"]

    def test_right_track_with_wrong_action_fails(self):
        rule = rule_of("n707_gen_track_info")
        log = call("music_library", "{'action': 'play', 'track_id': '%s'}" % SONG_ID)
        got = eev.track_ref_verdict(rule, SONG_ID, _feed(log))
        assert got["ok"] == 0 and "action=['play']" in got["detail"]

    def test_other_tool_with_the_id_does_not_count(self):
        rule = rule_of("n707_gen_track_info")
        log = call("stop_music", "{'action': 'info', 'track_id': '%s'}" % SONG_ID)
        assert eev.track_ref_verdict(rule, SONG_ID, _feed(log))["ok"] == 0

    def test_no_call_in_the_window_fails(self):
        rule = rule_of("n707_gen_track_info")
        got = eev.track_ref_verdict(rule, SONG_ID, _feed(""))
        assert got["ok"] == 0 and "нет вызова" in got["detail"]

    def test_id_in_another_field_or_substring_does_not_count(self):
        rule = rule_of("n707_gen_track_info")
        for params in ("{'action': 'info', 'query': '%s'}" % SONG_ID,
                       "{'action': 'info', 'track_id': '%s-x'}" % SONG_ID):
            assert eev.track_ref_verdict(rule, SONG_ID, _feed(call("music_library", params)))["ok"] == 0, params

    def test_garbled_params_line_is_not_a_call(self):
        rule = rule_of("n707_gen_track_info")
        log = call("music_library", "{'action': 'info', 'track_id': %s" % SONG_ID)
        assert eev.track_ref_verdict(rule, SONG_ID, _feed(log))["ok"] == 0

    def test_unreadable_log_is_not_a_pass(self):
        rule = rule_of("n707_gen_track_info")
        assert eev.track_ref_verdict(rule, SONG_ID, "Traceback")["fetch"] == 0


# -- схема поля: кривое поле не молча без сверки -------------------------------------------------------------------

GOOD = {"reason": "CASCADE_N702", "log": r"track=(\S+)", "tool": "music_library", "action": "info"}


class TestSchema:
    def test_good_field_parses_with_default_wait(self):
        rule = eev.track_ref({"label": "n707", "track_ref": GOOD})
        assert rule.reason == "CASCADE_N702" and rule.wait_sec == 0 and rule.action == "info"

    def test_step_without_field_has_no_rule(self):
        assert eev.track_ref({"label": "n711"}) is None

    @pytest.mark.parametrize("patch", [
        {"reason": "cascade"}, {"reason": 5}, {"log": ""}, {"log": "(unclosed"}, {"log": "track=\\S+"},
        {"log": "(a)(b)"}, {"tool": "Music Library"}, {"tool": ""}, {"action": ""}, {"action": None},
        {"wait_sec": -1}, {"wait_sec": 601}, {"wait_sec": "5"}, {"wait_sec": True},
    ])
    def test_bad_field_is_an_error(self, patch):
        with pytest.raises(ValueError):
            eev.track_ref({"label": "n707", "track_ref": {**GOOD, **patch}})

    @pytest.mark.parametrize("raw", ["CASCADE_N702", [], 7])
    def test_not_an_object_is_an_error(self, raw):
        with pytest.raises(ValueError):
            eev.track_ref({"label": "n707", "track_ref": raw})

    def test_generator_rejects_unknown_ref_key(self):
        with pytest.raises(KeyError):
            gen.step("n999", "Робот, привет.", track_ref=gen.TrackCall("NOPE", "music_library", "info"))


# -- CLI --------------------------------------------------------------------------------------------------------


def _cli(*args, stdin=""):
    return subprocess.run([sys.executable, str(EVENTS), *args], input=stdin, capture_output=True, text=True,
                          encoding="utf-8")


def test_cli_trackrefs_trackbind_trackcheck(tmp_path):
    listed = _cli("trackrefs", "--scenario", str(ACT7))
    assert listed.returncode == 0
    assert "n707_gen_track_info\tCASCADE_N702\t240" in listed.stdout.splitlines()
    assert "n708_gen_play\tCASCADE_N702\t0" in listed.stdout.splitlines()
    bound = _cli("trackbind", "--scenario", str(ACT7), "--label", "n707_gen_track_info", stdin=_feed(song())).stdout
    assert "track=" + SONG_ID in bound.splitlines() and "fetch=1" in bound.splitlines()
    none = _cli("trackbind", "--scenario", str(ACT7), "--label", "n707_gen_track_info", stdin=_feed("")).stdout
    assert "track=-" in none.splitlines()
    good = _cli("trackcheck", "--scenario", str(ACT7), "--label", "n708_gen_play", "--track", SONG_ID,
                stdin=_feed(call("music_library", "{'action': 'play', 'track_id': '%s'}" % SONG_ID))).stdout
    assert "ok=1" in good.splitlines()
    bad = _cli("trackcheck", "--scenario", str(ACT7), "--label", "n708_gen_play", "--track", SONG_ID,
               stdin=_feed(call("music_library", "{'action': 'play', 'track_id': '%s'}" % OTHER_ID))).stdout
    assert "ok=0" in bad.splitlines()
    no_track = _cli("trackcheck", "--scenario", str(ACT7), "--label", "n708_gen_play", stdin=_feed(""))
    assert no_track.returncode == 2
    broken = tmp_path / "bad.json"
    broken.write_text(json.dumps({"steps": [{"label": "a", "track_ref": {"reason": "X"}}]}), encoding="utf-8")
    err = _cli("trackrefs", "--scenario", str(broken))
    assert err.returncode == 2 and "track_ref" in err.stderr


# -- харнесс (bash): SKIP без песни, вердикт после шага ------------------------------------------------------------


def _scenario(tmp_path, wait_sec=0):
    path = tmp_path / "scenario.json"
    field = {"reason": "CASCADE_N702", "log": gen.SONG_TRACK_LOG, "tool": "music_library", "action": "info"}
    if wait_sec:
        field["wait_sec"] = wait_sec
    path.write_text(json.dumps({"steps": [{"label": "n707", "track_ref": field}, {"label": "n711"}]}), encoding="utf-8")
    return path


def _bind(tmp_path, log_text, label="n707", wait_sec=0, extra=""):
    log = tmp_path / "robot.log"
    log.write_text(log_text, encoding="utf-8")
    scenario = _scenario(tmp_path, wait_sec)
    body = "\n".join([
        'export FAKE_ROBOT_LOG="%s"' % log.as_posix(),
        'OUT_DIR="%s"; E2E_RUN_BEFORE=2026-01-01T00:00:00Z' % tmp_path.as_posix(),
        'emit_step() { echo "E2E_STEP $1"; }',
        extra,
        'load_track_refs "%s"' % scenario.as_posix(),
        'step_track_ref_skip %s "%s"; echo "RC=$? ID=${E2E_STEP_TRACK_ID}"' % (label, scenario.as_posix()),
        'check_track_ref %s "%s" 2026-01-01T00:00:00Z; echo "CHECK_RC=$?"' % (label, scenario.as_posix()),
    ])
    return _act_start(tmp_path, body)


@needs_bash
class TestHarness:
    def test_no_song_skips_the_step_not_passes(self, tmp_path):
        rc, out, err, _ = _bind(tmp_path, call("music_library", "{'action': 'list'}"))
        assert "RC=0 ID=" in out and "E2E_STEP n707 SKIP CASCADE_N702" in out and "НЕ играется" in out, (out, err)

    def test_song_found_plays_and_remembers_id(self, tmp_path):
        rc, out, err, _ = _bind(tmp_path, song() + call("music_library", "{'action': 'info', 'track_id': '%s'}" % SONG_ID))
        assert "RC=1 ID=" + SONG_ID in out and "E2E_STEP" not in out, (out, err)
        assert "CHECK_RC=0" in out and "PATTERN_OK: track_ref" in out, (out, err)

    def test_call_on_other_track_is_a_pattern_miss(self, tmp_path):
        rc, out, err, _ = _bind(tmp_path, song() + call("music_library", "{'action': 'info', 'track_id': '%s'}" % OTHER_ID))
        assert "CHECK_RC=1" in out and "PATTERN_MISS: track_ref" in out and OTHER_ID in out, (out, err)

    def test_call_without_id_is_a_pattern_miss(self, tmp_path):
        rc, out, err, _ = _bind(tmp_path, song() + call("music_library", "{'action': 'info'}"))
        assert "CHECK_RC=1" in out and "PATTERN_MISS: track_ref" in out, (out, err)

    def test_unreadable_log_skips_loudly(self, tmp_path):
        rc, out, err, _ = _bind(tmp_path, song(), extra='ROBOT_SSH="false"')
        assert "RC=0" in out and "E2E_STEP n707 SKIP track_ref_log_unreadable" in out, (out, err)

    def test_step_without_field_is_untouched(self, tmp_path):
        rc, out, err, state = _bind(tmp_path, "", label="n711", extra=': > "$FAKE_ROBOT_STATE/ssh_calls"')
        assert "RC=1 ID=" in out and "CHECK_RC=0" in out and "E2E_STEP" not in out, (out, err)
        assert (state / "ssh_calls").read_text(encoding="utf-8") == ""

    def test_waits_for_the_song_line_up_to_wait_sec(self, tmp_path):
        rc, out, err, _ = _bind(tmp_path, "", wait_sec=2, extra="sleep() { command sleep 1; }")
        assert "жду до 2 с" in out and "SKIP CASCADE_N702" in out, (out, err)

    def test_bad_field_is_fatal_before_first_step(self, tmp_path):
        bad = tmp_path / "bad.json"
        bad.write_text(json.dumps({"steps": [{"label": "a", "track_ref": {"reason": "X"}}]}), encoding="utf-8")
        rc, out, err, _ = _act_start(tmp_path, 'OUT_DIR="%s"; load_track_refs "%s"' % (tmp_path.as_posix(), bad.as_posix()))
        assert rc == 2 and "E2E_FATAL: поле track_ref сценария не разобрано" in err, (rc, out, err)

    def test_loop_wires_bind_before_play_and_check_after(self):
        text = (SCRIPTS_DIR / "e2e_voice_test.sh").read_text(encoding="utf-8")
        assert 'load_track_refs "$SCENARIO_FILE"' in text
        assert text.index("step_precondition_skip \"$label\"") < text.index("step_track_ref_skip \"$label\"")
        assert text.index("check_music_compare \"$label\"") < text.index("check_track_ref \"$label\"")
