# -*- coding: utf-8 -*-
"""#4981: эхо собственного TTS -- не ответ шага, окно первой попытки не сдвигается.

Марафон 20.11, акт 7, run 44806961884, шаг n704: тул вызван, затем микрофон принял
колонку робота во время TTS. Харнесс брал последний ПРИНЯТО, считал его промахом и
сдвигал окно на повтор -- вызов тула выпадал из ``--since``.

Судья ``step_answer`` (e2e_events.py) решает, bash только спрашивает. Признак эха --
время TTS (SpeechEvent started до конца) или флаг hardware AEC «во время TTS», не текст.
``STEP_FIRST_BEFORE`` хранит окно первой попытки: приёмка после повтора ищет тул от него.
``stt_replay_inject`` не меняется.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_4981_tts_echo_not_step_answer.py -v -o addopts=""
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from tests.unit.e2e_scripts.test_issue_2846_registration_verdict import extract_scenario_loop
from tests.unit.e2e_scripts.test_issue_3857_event_waits import _l, speech
from tests.unit.e2e_scripts.test_issue_3857_run_step import extract_function
from tests.unit.e2e_scripts.test_issue_3989_recover import REPLY, SCRIPTS_DIR, _run_step, heard, needs_bash
from tests.unit.e2e_scripts.test_issue_4196_false_greens import KINDS
from tests.unit.e2e_scripts.test_issue_4951_stt_inject_fallback import E2E_SCRIPT, LIB

ROOT = Path(__file__).resolve().parents[3]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import e2e_events as eev  # noqa: E402

EVENTS = SCRIPTS_DIR / "e2e_events.py"
PHRASE = "Робот, библиотека"
ECHO = "семь шестьдесят"
TOOL = "music_library"
SID = "eeee4981"


def _cli(*args, stdin=""):
    return subprocess.run(
        [sys.executable, str(EVENTS), *args],
        input=stdin,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )


def _aec(ts):
    return _l("stt_node", ts, "🎤 [hardware AEC] Фраза 4.83с во время TTS — обрабатываем")


def _tool(ts, name=TOOL):
    return _l("mcp_server", ts, "📥 Запрос выполнения: %s с параметрами {}" % name)


def _done(ts):
    return _l("tts_node", ts, "✅ Воспроизведение завершено")


def _feed(body, now=1_790_000_100.0):
    """Как ``_feed`` тестов #3857, но без импорта тяжёлого модуля ради одной подстановки."""
    import re

    stamped = re.sub(r"\{NOW-([0-9.]+)\}", lambda m: "%.2f" % (now - float(m.group(1))), body)
    return "%.2f\n%s" % (now, stamped)


def _echo_then_nothing():
    """Правильный приём, тул, TTS, эхо колонки внутри TTS. Ответа после эха нет."""
    return (
        heard(PHRASE, "{NOW-40}")
        + _tool("{NOW-38}")
        + speech("started", SID, "{NOW-30}")
        + _aec("{NOW-24}")
        + heard(ECHO, "{NOW-23}")
        + speech("finished", SID, "{NOW-10}", "spoken_chars=20")
    )


class TestStepAnswerJudge:
    def test_echo_during_tts_is_not_the_step_answer(self):
        _now, lines = eev.split_input(_feed(_echo_then_nothing()))
        assert eev.step_answer(lines) == PHRASE
        echo_at = next(i for i, line in enumerate(lines) if ECHO in line and "ПРИНЯТО" in line)
        assert eev.accept_is_tts_echo(lines, echo_at) is True

    def test_accept_after_tts_is_the_answer(self):
        body = _echo_then_nothing() + heard(PHRASE, "{NOW-8}")
        _now, lines = eev.split_input(_feed(body))
        assert eev.step_answer(lines) == PHRASE
        accepts = [i for i, line in enumerate(lines) if "ПРИНЯТО" in line and PHRASE in line]
        assert len(accepts) == 2
        # последний приём -- после конца TTS, эхом не является; эхо колонки между ними -- да
        assert eev.accept_is_tts_echo(lines, accepts[-1]) is False

    def test_aec_flag_marks_echo_even_without_speech_event(self):
        """Флага источника хватает: SpeechEvent в окне нет, но AEC сказал «во время TTS»."""
        body = heard(PHRASE, "{NOW-40}") + _aec("{NOW-24}") + heard(ECHO, "{NOW-23}")
        _now, lines = eev.split_input(_feed(body))
        assert eev.step_answer(lines) == PHRASE

    def test_playback_done_closes_the_tts_window(self):
        body = (
            heard(PHRASE, "{NOW-40}")
            + speech("started", SID, "{NOW-30}")
            + _done("{NOW-12}")
            + heard(PHRASE, "{NOW-8}")
        )
        _now, lines = eev.split_input(_feed(body))
        assert eev.step_answer(lines) == PHRASE
        late = [i for i, line in enumerate(lines) if "ПРИНЯТО" in line][-1]
        assert eev.accept_is_tts_echo(lines, late) is False

    def test_same_words_outside_tts_are_still_the_answer(self):
        """Повтор фразы человеком вне TTS -- ответ, не эхо. Текст не сравнивается."""
        body = heard(PHRASE, "{NOW-40}") + heard(PHRASE, "{NOW-8}")
        _now, lines = eev.split_input(_feed(body))
        assert eev.step_answer(lines) == PHRASE
        assert eev.accept_is_tts_echo(lines, 0) is False

    def test_window_since_drops_the_previous_turn(self):
        body = heard("старый ход", "{NOW-80}") + heard(PHRASE, "{NOW-8}")
        now, lines = eev.split_input(_feed(body))
        assert eev.step_answer(lines, since=now - 20) == PHRASE

    def test_tool_called_since_sees_the_first_window_only(self):
        _now, lines = eev.split_input(_feed(_tool("{NOW-38}")))
        assert eev.tool_called_since(lines, 0.0, [TOOL]) is True
        assert eev.tool_called_since(lines, 1_790_000_100.0, [TOOL]) is False
        catalog = _l("dialog_node", "{NOW-38}", "tools: %s, speak_text" % TOOL)
        _now, listed = eev.split_input(_feed(catalog))
        assert eev.tool_called_since(listed, 0.0, [TOOL]) is False

    def test_cli_drops_echo_and_keeps_phrase(self):
        got = _cli("step-answer", stdin=_feed(_echo_then_nothing()))
        assert got.returncode == 0, got.stderr
        assert "heard=%s" % PHRASE in got.stdout.splitlines()
        assert "echo=1" in got.stdout.splitlines()

    def test_stt_replay_inject_decision_is_unchanged(self):
        """#4983: таблица инъекции после звукового повтора не двигается этим фиксом."""
        got = eev.stt_replay_inject(
            stt_ok=False,
            replayed=True,
            fallback=True,
            injected=False,
            barge_in=False,
            executed=False,
        )
        assert got == {"inject": 1, "reason": "inject"}


def _scenario(tmp_path, *, tools=True):
    step = {
        "label": "n704",
        "text": PHRASE,
        "expect": "cycle",
        "recover": ["stt_replay"],
    }
    if tools:
        step["acceptance"] = {"expected_tool_calls": [TOOL]}
    path = tmp_path / "scenario.json"
    path.write_text(json.dumps({"steps": [step]}, ensure_ascii=False), encoding="utf-8")
    return "SCENARIO_FILE=%s; E2E_STEP_RECOVER_KINDS=%s; E2E_STEP_RECOVERED=''; E2E_STT_INJECT_FALLBACK=0" % (
        path.as_posix(),
        KINDS,
    )


@needs_bash
class TestRunStepEcho:
    def test_echo_does_not_replay_or_shift_when_tool_already_ran(self, tmp_path):
        """Чужая фраза во время TTS -- эхо, не промах. Тул первой попытки уже есть: повтора нет.

        Если бы эхо считалось ответом, stt_miss_retry сказал бы retry и харнесс сдвинул бы окно.
        Судья отбрасывает эхо, остаётся настоящий приём, и промаха нет -- повтора тоже нет.
        """
        body = (
            heard(PHRASE, "{NOW-40}")
            + _tool("{NOW-38}")
            + speech("started", SID, "{NOW-30}")
            + _aec("{NOW-24}")
            + heard("не та фраза", "{NOW-23}")
            + speech("finished", SID, "{NOW-10}", "spoken_chars=20")
            + REPLY
        )
        out, err, _, played = _run_step(
            tmp_path,
            body,
            [],
            'run_step "%s" anton n704 cycle; echo "RC=$?"' % PHRASE,
            _scenario(tmp_path),
        )
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=1" in out
        assert "E2E_RECOVER stt_replay" not in out
        assert "услышано «%s»" % PHRASE in out
        assert "услышано «не та фраза»" not in out
        assert [p.rsplit("/", 1)[-1] for p in played] == ["cmd_n704_eq.wav"]

    def test_plain_answer_is_still_accepted(self, tmp_path):
        out, err, _, played = _run_step(
            tmp_path,
            heard(PHRASE, "{NOW-30}") + _tool("{NOW-28}") + REPLY,
            [],
            'run_step "%s" anton n704 cycle; echo "RC=$?"' % PHRASE,
            _scenario(tmp_path),
        )
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=1" in out
        assert "E2E_RECOVER" not in out
        assert [p.rsplit("/", 1)[-1] for p in played] == ["cmd_n704_eq.wav"]

    def test_repeated_phrase_outside_tts_still_replays_a_real_miss(self, tmp_path):
        """Чужая фраза вне TTS -- по-прежнему промах, один повтор. Повтор фразы вне TTS -- ответ."""
        first = heard("не та фраза", "{NOW-30}") + REPLY
        again = heard(PHRASE, "{NOW-11}") + REPLY
        out, err, _, played = _run_step(
            tmp_path,
            first,
            [again],
            'run_step "%s" anton n704 cycle; echo "RC=$?"' % PHRASE,
            _scenario(tmp_path, tools=False),
            replace=True,
        )
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=2" in out
        assert out.count("E2E_RECOVER stt_replay") == 1
        assert [p.rsplit("/", 1)[-1] for p in played] == [
            "cmd_n704_eq.wav",
            "cmd_n704_eq.wav",
        ]


@needs_bash
def test_acceptance_keeps_first_window_tool_after_replay(tmp_path):
    """Повтор всё же случился -- приёмка ищет тул от STEP_FIRST_BEFORE, не от сдвинутого окна.

    Фейк ``docker logs --since`` не режет, поэтому приёмка здесь зовётся с тем окном, которое
    цикл сценария выбрал сам (STEP_JUDGE_BEFORE). Тул старше сдвинутого STEP_BEFORE и младше
    первой попытки: от сдвинутого окна его не видно, от первого -- видно.
    """
    out_dir = tmp_path / "out"
    out_dir.mkdir()
    log = tmp_path / "robot.log"
    # Метки абсолютные: тул на 1790000012 (2026-09-21T14:13:12Z); первая попытка 14:13:10Z, повтор сдвинул окно на 14:13:40Z.
    tool_line = "[mcp_server-9] [INFO] [1790000012.00] [mcp_server]: 📥 Запрос выполнения: %s с параметрами {}\n" % TOOL
    log.write_text(tool_line, encoding="utf-8")
    acc = json.dumps({"expected_tool_calls": [TOOL]}, ensure_ascii=False)
    script = "\n".join(
        [
            "set -u",
            'OUT_DIR="%s"' % out_dir.as_posix(),
            'SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
            'export FAKE_ROBOT_STATE="%s/state" FAKE_ROBOT_LOG="%s"' % (tmp_path.as_posix(), log.as_posix()),
            'ROBOT_SSH="bash %s"' % (Path(__file__).resolve().parent / "fake_robot_ssh.sh").as_posix(),
            'log() { echo ">>> $*"; }',
            LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
            extract_function("ensure_outdir"),
            extract_function("check_acceptance"),
            # как цикл сценария после повтора: STEP_BEFORE сдвинут, STEP_FIRST_BEFORE -- первая попытка,
            # инъекции не было -- судить от первого окна.
            "STEP_BEFORE=2026-09-21T14:13:40Z",
            "STEP_FIRST_BEFORE=2026-09-21T14:13:10Z",
            "E2E_STEP_STT_INJECTED=",
            'STEP_JUDGE_BEFORE="$STEP_BEFORE"',
            'if [ -n "${STEP_FIRST_BEFORE:-}" ] && [ -z "${E2E_STEP_STT_INJECTED:-}" ]; then',
            '    STEP_JUDGE_BEFORE="$STEP_FIRST_BEFORE"',
            "fi",
            'echo "JUDGE=$STEP_JUDGE_BEFORE"',
            'check_acceptance n704 \'%s\' "$STEP_JUDGE_BEFORE"; echo "ACC_RC=$?"' % acc,
        ]
    )
    proc = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=60)
    out, err = proc.stdout.decode("utf-8", "replace"), proc.stderr.decode("utf-8", "replace")
    assert "JUDGE=2026-09-21T14:13:10Z" in out, (out, err)
    assert "ACC_RC=0" in out, (out, err)
    assert "expected tool calls not invoked" not in out
    # Фейк docker logs --since не режет, поэтому приёмка выше доказывает только выбор окна циклом.
    # Что тул виден от первой попытки и не виден от сдвинутого окна -- решает сам судья.
    now, lines = eev.split_input("1790000100.00\n" + tool_line)
    first = eev.epoch_of("2026-09-21T14:13:10Z")
    shifted = eev.epoch_of("2026-09-21T14:13:40Z")
    assert eev.tool_called_since(lines, first, [TOOL]) is True
    assert eev.tool_called_since(lines, shifted, [TOOL]) is False


def test_scenario_loop_pins_first_window(tmp_path):
    """Цикл сценария запоминает STEP_FIRST_BEFORE до run_step и не подменяет его инъекции."""
    assert 'STEP_FIRST_BEFORE=""' in E2E_SCRIPT.read_text(encoding="utf-8")
    loop = extract_scenario_loop()
    assert "STEP_FIRST_BEFORE" in loop
    assert "STEP_JUDGE_BEFORE" in loop
    assert "E2E_STEP_STT_INJECTED" in loop
