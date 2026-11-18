# -*- coding: utf-8 -*-
"""#4694 (17.11, Telegram 15:45 UTC): харнесс не ставит FAIL, если STT вернул не команду, а мусор.

n805b run 43636148539: ждали «Робот, говори.», STT принял «робот колории», судья сказал silenced, харнесс
написал ``E2E_STEP n805b_unsilence FAIL turn_silenced`` с первой попытки (``recover: no_wake``, ``stt_replay``
тогда звался только при ``E2E_WAIT_RC=0``). Решение Шифу: такой случай -- повтор хода, не FAIL. Повтор один
(всего ``STT_GARBAGE_MAX_ATTEMPTS``, считая первую). Второй промах -- честный FAIL, не зелёный. Словарь
диалога не правится.

Run:
    /home/ros2/rob_box_project/.venv-core/bin/pytest -v tests/unit/e2e_scripts/test_issue_4694_stt_garbage_retry.py
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from tests.unit.e2e_scripts.test_issue_3989_recover import SCRIPTS_DIR, _run_step, heard, needs_bash, tier1, turn_stage

ROOT = Path(__file__).resolve().parents[3]
for _dir in (ROOT / "scripts" / "e2e", SCRIPTS_DIR):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_events as eev  # noqa: E402

EVENTS = SCRIPTS_DIR / "e2e_events.py"

# Ход n805b: фраза принята, тишина её проглотила, unsilence нет (run 43636148539, «робот колории»).
GARBAGE = heard("робот колории", "{NOW-30}") + turn_stage("silenced", ts="{NOW-20}")
COMMAND = heard("робот говори", "{NOW-12}") + tier1("unsilence", "{NOW-11}") + turn_stage("done", ts="{NOW-10}")
GARBAGE_AGAIN = heard("робот колории", "{NOW-8}") + turn_stage("silenced", ts="{NOW-6}")

STEP = 'run_step "Робот, говори." anton a cycle; echo "RC=$?"'
KINDS = (
    "E2E_STEP_RECOVER_KINDS=no_wake,stt_replay; E2E_STEP_RECOVERED=''; E2E_STEP_STT_REPLAYED=''; "
    "E2E_STEP_STT_ATTEMPT=''; E2E_STEP_SILENT_OK=1"
)


def _scenario(tmp_path: Path) -> str:
    path = tmp_path / "scenario.json"
    path.write_text(
        json.dumps(
            {
                "steps": [
                    {
                        "label": "a",
                        "text": "Робот, говори.",
                        "stt_keys": ["говори"],
                        "silent_ok": True,
                        "recover": ["no_wake", "stt_replay"],
                    }
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return 'SCENARIO_FILE="%s"; STEP_BEFORE=initial; text="Робот, говори.";' % path.as_posix()


def _names(played):
    return [p.rsplit("/", 1)[-1] for p in played]


def test_attempt_budget_is_one_replay_and_the_only_constant():
    """Всего две попытки фразы, считая первую: один повтор, не бесконечный ретрай. Число -- одна константа."""
    assert eev.STT_GARBAGE_MAX_ATTEMPTS == 2
    src = (EVENTS).read_text(encoding="utf-8")
    assert src.count("STT_GARBAGE_MAX_ATTEMPTS = ") == 1


def test_kolorii_is_garbage_and_asks_for_exactly_one_replay():
    """«робот колории» -- не команда «говори»: первая попытка просит повтор, вторая -- нет."""
    first = eev.stt_garbage_retry("Робот, говори.", "робот колории", 1, missing="говори")
    assert first == {"retry": 1, "reason": "retry", "attempt": 1, "max": 2}
    second = eev.stt_garbage_retry("Робот, говори.", "робот колории", 2, missing="говори")
    assert second["retry"] == 0 and second["reason"] == "attempts_exhausted"
    assert second["max"] == eev.STT_GARBAGE_MAX_ATTEMPTS


def test_the_command_itself_is_not_garbage_and_is_not_replayed():
    assert eev.heard_matches_command("Робот, говори.", "робот говори")
    assert not eev.heard_matches_command("Робот, говори.", "робот колории")
    got = eev.stt_garbage_retry("Робот, говори.", "робот говори", 1, missing="-")
    assert got["retry"] == 0 and got["reason"] == "command_heard"


def test_empty_accept_and_a_step_without_replay_are_not_retried():
    """Пустое ПРИНЯТО судит другой судья. Шаг без stt_replay мусор не повторяет (настоящая тишина -- FAIL)."""
    assert eev.stt_garbage_retry("Робот, говори.", "", 1, missing="говори")["reason"] == "nothing_heard"
    denied = eev.stt_garbage_retry("Робот, говори.", "робот колории", 1, missing="говори", allowed=False)
    assert denied["retry"] == 0 and denied["reason"] == "not_allowed"


def test_keys_present_is_not_a_replay_even_if_the_wording_differs():
    """Слово шага на месте, но не все слова команды -- не мусор: ход судит прежний судья, повтора нет."""
    got = eev.stt_garbage_retry("Робот, говори басом.", "говори", 1, missing="-")
    assert not eev.heard_matches_command("Робот, говори басом.", "говори")
    assert got["retry"] == 0 and got["reason"] == "keys_present"


def test_cli_stt_retry_reports_the_constant(tmp_path):
    out = subprocess.run(
        [
            sys.executable,
            str(EVENTS),
            "stt-retry",
            "--text",
            "Робот, говори.",
            "--heard",
            "робот колории",
            "--attempt",
            "1",
            "--missing",
            "говори",
            "--allowed",
            "1",
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=True,
    ).stdout.splitlines()
    assert out == ["retry=1", "reason=retry", "attempt=1", "max=%d" % eev.STT_GARBAGE_MAX_ATTEMPTS]


@needs_bash
class TestRunStepSttGarbage:
    def test_garbage_on_a_silenced_turn_replays_once_and_is_not_a_first_try_fail(self, tmp_path):
        """«колории» + silenced: фраза ещё раз, окно с повтора, FAIL с первой попытки нет. Повтор услышан --
        итог recovered_stt_replay, не чистый OK."""
        out, err, _, played = _run_step(
            tmp_path,
            GARBAGE,
            [COMMAND],
            STEP,
            _scenario(tmp_path) + KINDS,
            replace=True,
        )
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=2" in out
        assert out.count("E2E_RECOVER stt_replay a ") == 1 and "missing=говори" in out
        assert "RECOVERED=stt_replay" in out
        assert "FAIL turn_silenced" not in out
        assert "STEP_BEFORE=initial" not in out
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_garbage_again_is_an_honest_fail_without_a_third_play(self, tmp_path):
        """Повтор тоже мусор: второго повтора нет, шаг красный. Зелёным без повтора не помечается."""
        out, err, _, played = _run_step(
            tmp_path,
            GARBAGE,
            [GARBAGE_AGAIN],
            STEP,
            _scenario(tmp_path) + KINDS,
            replace=True,
        )
        assert "RC=1" in out, (out, err)
        assert "PLAY_COUNT=2" in out
        assert out.count("E2E_RECOVER stt_replay a ") == 1
        assert "E2E_STEP a FAIL turn_silenced after_recover_stt_replay" in out
        assert "повтора нет (attempts_exhausted" in out
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_a_real_silence_of_the_command_is_still_a_fail_without_replay(self, tmp_path):
        """Команда узнана («говори»), а тишина её проглотила: это не мусор STT, повтора нет, FAIL сразу."""
        heard_right = heard("робот говори", "{NOW-30}") + turn_stage("silenced", ts="{NOW-20}")
        out, err, _, played = _run_step(
            tmp_path,
            heard_right,
            [],
            STEP,
            _scenario(tmp_path) + KINDS,
        )
        assert "RC=1" in out and "PLAY_COUNT=1" in out, (out, err)
        assert "E2E_RECOVER" not in out
        assert "E2E_STEP a FAIL turn_silenced" in out
        assert "повтора нет (command_heard" in out
        assert _names(played) == ["cmd_a_eq.wav"]

    def test_step_without_stt_replay_does_not_retry_garbage(self, tmp_path):
        out, err, _, _ = _run_step(
            tmp_path,
            GARBAGE,
            [],
            STEP,
            _scenario(tmp_path) + "E2E_STEP_RECOVER_KINDS=no_wake; E2E_STEP_RECOVERED=''; E2E_STEP_STT_REPLAYED=''",
        )
        assert "PLAY_COUNT=1" in out and "E2E_RECOVER" not in out, (out, err)
        assert "E2E_STEP a FAIL turn_silenced" in out


def test_n805b_still_excludes_silenced_recovery_and_keeps_stt_replay():
    """Решение 15.11 не отменено: n805b не лечится фразой «Робот, говори.» как восстановлением silenced."""
    step = {
        "label": "n805b_unsilence",
        "text": "Робот, говори.",
        "stt_keys": ["говори"],
        "recover": ["no_wake", "silenced", "stt_replay"],
    }
    assert eev.recover_kinds({}, step) == ("no_wake", "stt_replay")
    assert eev.stt_verdict(step, "робот колории")["missing"] == "говори"
