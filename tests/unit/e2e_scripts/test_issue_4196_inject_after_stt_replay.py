# -*- coding: utf-8 -*-
"""#4196 + инъекция после stt_replay: последний заход -- топиком, не вторым звуком.

Марафон 20.11, акт 4, run 44802283781, шаг n411_pitch_bass: ожидалось
«Робот, говори басом, как диктор в старом кино.», распознано
«робот галибаса как диктор в старом кино». Звуковой повтор уже был, ключевое
слово снова не нашлось, а пути stt_keys не было последней попытки чистым текстом.
Шаг судился по фразе, которую человек не говорил.

Судья ``stt_replay_inject`` (e2e_events.py) решает, bash только исполняет:
после единственного звукового повтора, если слова шага снова не нашлись, --
``step_stt_inject``. Успех -- ``OK recovered_stt_inject after_recover_stt_replay``,
не чистый PASS. Инъекция не легла -- честный FAIL с меткой ``inject_failed``.
Потолок: один звуковой повтор и одна инъекция. Исход уже в окне (#4955),
повторная инъекция, запрет fallback и barge_in инъекцию не дают.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_4196_inject_after_stt_replay.py -v -o addopts=""
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from tests.unit.e2e_scripts.test_issue_2846_registration_verdict import extract_scenario_loop
from tests.unit.e2e_scripts.test_issue_3857_event_waits import _l
from tests.unit.e2e_scripts.test_issue_3857_run_step import extract_function
from tests.unit.e2e_scripts.test_issue_3989_recover import REPLY, SCRIPTS_DIR, _run_step, heard, needs_bash
from tests.unit.e2e_scripts.test_issue_4196_false_greens import KINDS, STEP, _scenario
from tests.unit.e2e_scripts.test_issue_4951_stt_inject_fallback import E2E_SCRIPT, EVENTS, EXTRA, LIB, _state

ROOT = Path(__file__).resolve().parents[3]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import e2e_events as eev  # noqa: E402

PHRASE = "Робот, говори басом, как диктор"
BAD = "робот гарри басом как диктор"
PATTERN = "Инструмент pitch_shift выполнен успешно"


def _cli(*args):
    return subprocess.run(
        [sys.executable, str(EVENTS), *args],
        capture_output=True,
        text=True,
        encoding="utf-8",
    )


class TestSttReplayInjectJudge:
    def test_distorted_replay_asks_for_one_topic_attempt(self):
        got = eev.stt_replay_inject(
            stt_ok=False, replayed=True, fallback=True, injected=False, barge_in=False, executed=False
        )
        assert got == {"inject": 1, "reason": "inject"}

    def test_keys_heard_fallback_off_already_injected_barge_and_executed_do_not_inject(self):
        base = dict(stt_ok=False, replayed=True, fallback=True, injected=False, barge_in=False, executed=False)
        assert eev.stt_replay_inject(**{**base, "stt_ok": True})["reason"] == "keys_heard"
        assert eev.stt_replay_inject(**{**base, "replayed": False})["reason"] == "not_replayed"
        assert eev.stt_replay_inject(**{**base, "fallback": False})["reason"] == "fallback_off"
        assert eev.stt_replay_inject(**{**base, "injected": True})["reason"] == "already_injected"
        assert eev.stt_replay_inject(**{**base, "barge_in": True})["reason"] == "barge_in"
        assert eev.stt_replay_inject(**{**base, "executed": True})["reason"] == "executed"
        for override in (
            {"stt_ok": True},
            {"replayed": False},
            {"fallback": False},
            {"injected": True},
            {"barge_in": True},
            {"executed": True},
        ):
            assert eev.stt_replay_inject(**{**base, **override})["inject"] == 0

    def test_cli_prints_inject_and_reason(self):
        yes = _cli("stt-replay-inject", "--stt-ok", "0", "--replayed", "1", "--fallback", "1")
        assert yes.returncode == 0, yes.stderr
        assert yes.stdout.splitlines() == ["inject=1", "reason=inject"]
        no = _cli("stt-replay-inject", "--stt-ok", "0", "--replayed", "1", "--fallback", "1", "--executed", "1")
        assert no.stdout.splitlines() == ["inject=0", "reason=executed"]


def _names(played):
    return [p.rsplit("/", 1)[-1] for p in played]


def _step_env(tmp_path, *, patterns=None, fallback="1"):
    env = _scenario(tmp_path) + KINDS + "; E2E_STT_INJECT_FALLBACK=%s" % fallback
    if patterns is not None:
        env += "; patterns_json=%s" % ("'" + json.dumps(patterns, ensure_ascii=False) + "'")
    return env


@needs_bash
class TestRunStepInjectAfterReplay:
    def test_distorted_replay_injects_once_and_marks_recovered_not_clean_pass(self, tmp_path):
        """Повтор снова без ключевого слова -- одна инъекция, второго звука нет.
        run_step помечает recovered_stt_inject; строку OK пишет scenario-цикл (отдельный тест)."""
        _state(tmp_path, reaction=True)
        first = heard(BAD, "{NOW-30}") + REPLY
        again = heard(BAD, "{NOW-11}") + REPLY
        out, err, _, played = _run_step(tmp_path, first, [again], STEP, _step_env(tmp_path), replace=True, extra=EXTRA)
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=2" in out
        assert out.count("E2E_RECOVER stt_replay a ") == 1, out
        assert out.count("E2E_RECOVER stt_inject a ") == 1, out
        assert "RECOVERED=stt_inject" in out
        assert "инъекция удалась" in out
        assert "ПОЛНЫЙ ЦИКЛ" in out
        assert "E2E_STEP a FAIL" not in out
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]
        journal = (tmp_path / "state" / "utterances").read_text(encoding="utf-8")
        assert journal.count('"provider": "e2e_harness"') == 1
        assert PHRASE in journal

    def test_inject_not_picked_up_is_honest_fail_with_inject_failed(self, tmp_path):
        """Топик не подхватил фразу (stt_inject_command rc 1, как нет подписчика) -- честный FAIL, не PASS.
        Фейк всегда отдаёт подписчика; глушение функции -- тот же приём, что в тесте #4951."""
        first = heard(BAD, "{NOW-30}") + REPLY
        again = heard(BAD, "{NOW-11}") + REPLY
        out, err, _, played = _run_step(
            tmp_path,
            first,
            [again],
            STEP,
            _step_env(tmp_path) + "; stt_inject_command() { return 1; }",
            replace=True,
            extra=EXTRA,
        )
        assert "RC=1" in out, (out, err)
        assert "PLAY_COUNT=2" in out
        assert "E2E_STEP a FAIL stt_replay_distorted after_recover_stt_replay inject_failed" in out, out
        assert "OK recovered" not in out
        assert "E2E_RECOVER stt_inject" not in out
        assert out.count("E2E_RECOVER stt_replay a ") == 1
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_a_second_injection_is_not_attempted(self, tmp_path):
        """Инъекция в шаге уже была -- судья говорит already_injected, второго захода нет."""
        _state(tmp_path, reaction=True)
        first = heard(BAD, "{NOW-30}") + REPLY
        again = heard(BAD, "{NOW-11}") + REPLY
        out, err, _, played = _run_step(
            tmp_path, first, [again], STEP, _step_env(tmp_path) + "; E2E_STEP_STT_INJECTED=1", replace=True, extra=EXTRA
        )
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=2" in out
        assert "already_injected" in out, out
        assert "E2E_RECOVER stt_inject" not in out
        assert not (tmp_path / "state" / "utterances").exists()
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_executed_window_is_not_injected(self, tmp_path):
        """#4955: исход шага уже в окне первой попытки -- ни повтора, ни инъекции."""
        _state(tmp_path, reaction=True)
        first = heard(BAD, "{NOW-30}") + _l("mcp_server", "{NOW-25}", PATTERN) + REPLY
        out, err, _, played = _run_step(tmp_path, first, [], STEP, _step_env(tmp_path, patterns=[PATTERN]), extra=EXTRA)
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=1" in out
        assert "команда исполнена, повтора нет" in out
        assert "E2E_RECOVER" not in out
        assert "/voice/stt/utterance" not in out
        assert not (tmp_path / "state" / "utterances").exists()
        assert _names(played) == ["cmd_a_eq.wav"]

    def test_fallback_off_does_not_inject_after_a_distorted_replay(self, tmp_path):
        first = heard(BAD, "{NOW-30}") + REPLY
        again = heard(BAD, "{NOW-11}") + REPLY
        out, err, _, played = _run_step(
            tmp_path, first, [again], STEP, _step_env(tmp_path, fallback="0"), replace=True, extra=EXTRA
        )
        assert "PLAY_COUNT=2" in out, (out, err)
        assert "fallback_off" in out, out
        assert "inject_failed" not in out
        assert "E2E_RECOVER stt_inject" not in out
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]


@needs_bash
def test_scenario_loop_marks_inject_after_stt_replay(tmp_path):
    """Строка шага несёт и вид восстановления, и то, что перед инъекцией был звуковой stt_replay."""
    scen = tmp_path / "scenario.json"
    scen.write_text(
        json.dumps(
            {
                "steps": [
                    {
                        "label": "n411_pitch_bass",
                        "text": PHRASE,
                        "expect": "cycle",
                        "stt_keys": ["говори", "бас"],
                        "recover": ["stt_replay"],
                    }
                ]
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    out_dir = tmp_path / "out"
    out_dir.mkdir()
    script = "\n".join(
        [
            "set -u",
            'OUT_DIR="%s"' % out_dir.as_posix(),
            'SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
            'ROBOT_SSH="true"; PASS=1; WAKE_GATE_SKIPPED_STEPS=0; E2E_RETRY_PAUSE=0; E2E_FAIL_KIND=""',
            'WAKE_GATE_PREFLIGHT_FILE="$OUT_DIR/wake_gate_preflight.json"',
            'log() { echo ">>> $*"; }',
            LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
            (SCRIPTS_DIR / "e2e_voice_wake_gate.sh").read_text(encoding="utf-8").replace("\r\n", "\n"),
            extract_function("ensure_outdir"),
            extract_function("emit_step"),
            extract_function("mark_fail_kind"),
            extract_function("check_music_compare"),
            extract_function("write_summary"),
            "run_step() { E2E_STEP_RECOVERED=stt_inject; E2E_STEP_STT_REPLAYED=1; return 0; }",
            "parse_transcript() { :; }",
            'load_recover_policy "%s"' % scen.as_posix(),
            'parse_scenario_to_tsv "%s" "$OUT_DIR/scenario_parsed.txt"' % scen.as_posix(),
            'LAST_STEP_SPEECH_FILE="$OUT_DIR/.last_step_speech.txt"; : > "$LAST_STEP_SPEECH_FILE"',
            extract_scenario_loop(),
        ]
    )
    p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=120)
    out, err = p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace")
    assert "E2E_STEP n411_pitch_bass OK recovered_stt_inject after_recover_stt_replay" in out, (out, err)
    assert "E2E_STEP n411_pitch_bass OK recovered_stt_inject\n" not in out
    # проводка: без звукового повтора суффикса нет (no_accept-инъекция #4951 не меняется)
    assert E2E_SCRIPT.read_text(encoding="utf-8").count('after_recover_stt_replay"') >= 1
