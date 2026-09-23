"""Issue #2846 — харнесс зелёный при сломанной регистрации голоса.

Живой прогон 35875477264 (акт 2, develop ``f9b826d``), лог харнесса::

    >>> ACCEPTANCE[n201_sasha_intro_long]: ❌ discovery tool 'register_speaker' invoked at pos 8021, AFTER verbal answer at pos 1282 (issue #2406: verbal-only LLM answer before required tool)
    >>> STEP n201_sasha_intro_long: ❌ проверка не прошла — retry 1/1
    ...
    >>> ACCEPTANCE[n201_sasha_intro_long]: ✅ all checks passed
    >>> STEP n201_sasha_intro_long: ✅ acceptance PASS
    E2E_STEP n201_sasha_intro_long OK

А в ``docker logs voice-assistant`` за то же время (цитата из issue #2846;
лог CI логов робота не содержит)::

    [speaker_id_node] ⚠️ [issue #2829] register_request for 'Саша' has no utterance_id -- honest refusal instead of registering whoever speaks next
    [dialogue_node]   ⚠️ [issue #2829] Регистрация 'Саша' отклонена: no_utterance_context (utterance_id=None)

Две дыры, обе проверяются здесь на РЕАЛЬНОМ коде харнесса (bash-функции
вырезаются из ``e2e_voice_test.sh`` и исполняются с подставным ssh, который
отдаёт фикстуру вместо ``docker logs``), а не на копии валидатора:

1. «❌» и «✅» на один шаг — это две ПОПЫТКИ ретрая (``retry_acceptance: 1``
   у n201), а не две проверки. Строки ACCEPTANCE теперь помечены попыткой,
   итог шага — одна строка «итог — …», а OK после проваленной попытки
   уходит в E2E_STEP как ``OK after_retry=N``.
2. ``register_speaker`` вызван != голос зарегистрирован. Для шагов, где
   ожидается регистрация, шаг валится на отказ speaker_id_node
   (``no_utterance_context`` / ``utterance_not_found`` / ``too_short``) и на
   ОТСУТСТВИЕ маркера ``✅ Speaker '<имя>' registered``.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_2846_registration_verdict.py -v --no-cov
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
E2E_SCRIPT = SCRIPTS_DIR / "e2e_voice_test.sh"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
WAKE = SCRIPTS_DIR / "e2e_voice_wake_gate.sh"

pytestmark = pytest.mark.skipif(
    shutil.which("bash") is None or shutil.which("python3") is None,
    reason="нужны bash и python3",
)

SCRIPT_LINES = E2E_SCRIPT.read_text(encoding="utf-8").split("\n")


def extract_function(name: str) -> str:
    """Вырезает bash-функцию, пропуская python-heredoc'и (как в
    test_e2e_quality_summary.py: внутри check_acceptance есть python с
    ``}`` в первой колонке)."""
    start = next(
        (i for i, ln in enumerate(SCRIPT_LINES) if ln.startswith(name + "() {")), None
    )
    assert start is not None, f"функция {name} не найдена в {E2E_SCRIPT}"
    i = start + 1
    heredoc = None
    while i < len(SCRIPT_LINES):
        line = SCRIPT_LINES[i]
        if heredoc is not None:
            if line.strip() == heredoc:
                heredoc = None
            i += 1
            continue
        m = re.search(r"<<-?'([A-Za-z_][A-Za-z0-9_]*)'", line)
        if m:
            heredoc = m.group(1)
        elif line == "}":
            return "\n".join(SCRIPT_LINES[start : i + 1])
        i += 1
    raise AssertionError(f"не найден конец функции {name}")


def extract_scenario_loop() -> str:
    """Вырезает scenario-цикл (while … done < scenario_parsed.txt) — тот
    самый код, что гоняется на стенде, с ретраями и итогом шага."""
    start = next(
        i for i, ln in enumerate(SCRIPT_LINES)
        if ln.startswith("    while IFS=$'\\x1f' eval \"$E2E_SCENARIO_ROW_READ\"; do")
    )
    end = next(
        i for i, ln in enumerate(SCRIPT_LINES)
        if i > start and ln == '    done < "$OUT_DIR/scenario_parsed.txt"'
    )
    return "\n".join(SCRIPT_LINES[start : end + 1])


def run_bash(script: str) -> tuple[int, str, str]:
    """bash со скриптом через stdin, байтами (на Windows текстовый режим
    переводит \\n в CRLF прямо в трубе)."""
    proc = subprocess.run(
        ["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=120
    )
    return (
        proc.returncode,
        proc.stdout.decode("utf-8", "replace"),
        proc.stderr.decode("utf-8", "replace"),
    )


# ── фикстуры лога voice-assistant ─────────────────────────────────────────
#
# Строки — копии f-строк из кода робота (speaker_id_node.py / dialogue_node.py),
# обёрнутые в префикс ROS-лога, как их печатает `docker logs voice-assistant`.

def _l(node: str, level: str, msg: str) -> str:
    return f"[{node}-9] [{level}] [1758638000.1] [{node}]: {msg}\n"


TOOLS_CATALOG = _l(
    "dialogue_node", "INFO",
    "  tools(56): clear_waypoints, get_music_state, register_speaker, "
    "speak_text, stop_music",
)
# Тул вызван ДО голосового ответа — discovery (#2406) проходит.
TOOL_CALLED = (
    _l("dialogue_node", "INFO", "📤 Отправлен запрос b5119339: register_speaker")
    + _l("mcp_server", "INFO",
         "📥 Запрос выполнения: register_speaker с параметрами {'name': 'Саша'}")
    + _l("mcp_server", "INFO", "✅ Инструмент register_speaker выполнен успешно")
)
VOICE_ANSWER = _l(
    "dialogue_node", "INFO",
    "✅ [turn] process_input returned: spoken='Приятно познакомиться, Саша!'[:60] "
    "tools=['register_speaker', 'speak_text'] finish_reason='stop'",
)
VOICE_ANSWER_NO_TOOL = _l(
    "dialogue_node", "INFO",
    "✅ [turn] process_input returned: spoken='Понял, не буду настаивать.'[:60] "
    "tools=['speak_text'] finish_reason='stop'",
)
# Ровно строки из issue #2846 (run 35875477264).
REJECT_NO_CONTEXT = (
    _l("speaker_id_node", "WARN",
       "⚠️ [issue #2829] register_request for 'Саша' has no utterance_id -- "
       "honest refusal instead of registering whoever speaks next")
    + _l("dialogue_node", "WARN",
         "⚠️ [issue #2829] Регистрация 'Саша' отклонена: no_utterance_context "
         "(utterance_id=None)")
    + _l("tts_node", "INFO",
         "🔊 TTS: text='Не расслышал — скажи, пожалуйста, ещё пару слов, "
         "чтобы я запомнил твой голос.'")
)
REJECT_NOT_FOUND = (
    _l("speaker_id_node", "WARN",
       "⚠️ [issue #2829] register_request for 'Саша': no embedding for "
       "utterance=u-42 within 1.5s -- honest refusal")
    + _l("dialogue_node", "WARN",
         "⚠️ [issue #2829] Регистрация 'Саша' отклонена: utterance_not_found "
         "(utterance_id=u-42)")
)
REJECT_TOO_SHORT = (
    _l("speaker_id_node", "WARN",
       "⚠️ [issue #2769] Registration of 'Саша' rejected — audio too short for "
       "a reliable anchor: 0.80s < 1.5s required. Профиль НЕ создан — прошу "
       "повторить фразу.")
    + _l("dialogue_node", "WARN",
         "⚠️ [issue #2769] Регистрация 'Саша' отклонена — реплика 0.8с короче "
         "требуемых 1.5с")
)
ACCEPTED = (
    _l("speaker_id_node", "INFO", "✅ Speaker 'Саша' registered (id=a1b2c3d4)")
    + _l("dialogue_node", "INFO",
         "✅ [issue 1077] Speaker registered: 'Саша' id=a1b2c3d4")
)
ACCEPTED_CONFLICT = _l(
    "speaker_id_node", "WARN",
    "⚠️ Speaker 'Борис' (id=f499c2f7) — голос похож на уже известного 'Саша' "
    "(id=a1b2c3d4, score=0.846 >= порога слияния), но имя другое: завожу "
    "ОТДЕЛЬНЫЙ профиль и НЕ переименовываю чужой (ADR-0127).",
)
MERGED = _l(
    "speaker_id_node", "INFO",
    "🔗 Speaker 'Борис' merged into existing profile (id=dc417cef) — voice "
    "matched an already-known speaker, no duplicate created",
)

LOG_REJECTED = TOOLS_CATALOG + TOOL_CALLED + VOICE_ANSWER + REJECT_NO_CONTEXT
LOG_ACCEPTED = TOOLS_CATALOG + TOOL_CALLED + ACCEPTED + VOICE_ANSWER
LOG_NO_OUTCOME = TOOLS_CATALOG + TOOL_CALLED + VOICE_ANSWER
# Попытка 1 прогона 35875477264: голос ДО тула (discovery #2406 FAIL).
LOG_VOICE_BEFORE_TOOL = TOOLS_CATALOG + VOICE_ANSWER + TOOL_CALLED + ACCEPTED

ACC_N201 = {
    "expected_tool_calls": ["register_speaker"],
    "discovery_tools": ["register_speaker"],
    "must_not_call": [],
}


# ── запуск реального check_acceptance ─────────────────────────────────────

def _stub_prelude(tmp: Path) -> str:
    """Общий пролог: log(), подставной ssh, пути. ROBOT_SSH читает
    $STUB/log_<N>.txt, где N — номер попытки из $STUB/attempt (или
    последовательность вызовов, если задан $STUB/seq_*)."""
    fake_ssh = tmp / "fake_ssh.sh"
    fake_ssh.write_bytes(
        (
            "#!/bin/bash\n"
            'case "$*" in\n'
            '  *"docker logs"*)\n'
            '    n="$(cat "$STUB/attempt" 2>/dev/null || echo 1)"\n'
            '    c="$(cat "$STUB/calls_$n" 2>/dev/null || echo 0)"; c=$((c + 1))\n'
            '    echo "$c" > "$STUB/calls_$n"\n'
            '    if [ -f "$STUB/log_${n}_call_${c}.txt" ]; then cat "$STUB/log_${n}_call_${c}.txt";\n'
            '    else cat "$STUB/log_${n}.txt" 2>/dev/null; fi ;;\n'
            '  *date*) date -u +%Y-%m-%dT%H:%M:%SZ ;;\n'
            "  *) : ;;\n"
            "esac\n"
        ).encode("utf-8")
    )
    fake_ssh.chmod(0o755)
    stub = tmp.as_posix()
    return "\n".join(
        [
            "set -u",
            f'STUB="{stub}"; export STUB',
            f'OUT_DIR="{stub}/out"; mkdir -p "$OUT_DIR"',
            f'SCRIPT_DIR_E2E="{SCRIPTS_DIR.as_posix()}"',
            f'ROBOT_SSH="bash {fake_ssh.as_posix()}"',
            'log() { echo ">>> $*"; }',
            'ensure_outdir() { mkdir -p "$OUT_DIR"; }',
            "E2E_TTS_PROVIDER=minimax",
            "",
        ]
    )


def run_check_acceptance(
    tmp_path: Path,
    acc: dict,
    logs: str | list[str],
    tag: str = "",
    wait_sec: int = 0,
) -> tuple[int, str, dict]:
    """``logs`` строкой — один и тот же лог на каждое чтение; списком —
    i-е чтение ``docker logs`` отдаёт i-й элемент (ack приходит позже)."""
    if isinstance(logs, str):
        (tmp_path / "log_1.txt").write_text(logs, encoding="utf-8")
    else:
        for i, chunk in enumerate(logs, 1):
            (tmp_path / f"log_1_call_{i}.txt").write_text(chunk, encoding="utf-8")
        (tmp_path / "log_1.txt").write_text(logs[-1], encoding="utf-8")
    acc_s = json.dumps(acc, ensure_ascii=False).replace("'", "'\\''")
    script = "\n".join(
        [
            _stub_prelude(tmp_path),
            f"E2E_REGISTRATION_ACK_WAIT_SEC={wait_sec}",
            extract_function("check_acceptance"),
            f"check_acceptance n201_sasha_intro_long '{acc_s}' "
            f"2026-09-23T14:37:48Z '{tag}'",
            'echo "RC=$?"',
            "",
        ]
    )
    rc, out, err = run_bash(script)
    assert "syntax error" not in err, err
    m = re.search(r"^RC=(\d+)$", out, re.M)
    assert m, (out, err)
    acc_json = json.loads(
        (tmp_path / "out" / "acceptance.json").read_text(encoding="utf-8")
    )
    return int(m.group(1)), out, acc_json


def acceptance_lines(out: str) -> list[str]:
    return [ln for ln in out.splitlines() if ln.startswith(">>> ACCEPTANCE[")]


class TestRegistrationVerdictInCheckAcceptance:
    """Дыра 2: вызов register_speaker без принятия регистрации — не PASS."""

    def test_issue_2846_rejection_fails_step(self, tmp_path):
        """Ровно прогон 35875477264, попытка 2: тул ДО голоса (discovery ok),
        но speaker_id_node отказал no_utterance_context. На develop было
        «✅ all checks passed»."""
        rc, out, acc = run_check_acceptance(tmp_path, ACC_N201, LOG_REJECTED)
        assert rc == 1, out
        assert acc["pass"] is False
        assert "no_utterance_context" in acc["reason"], acc["reason"]
        lines = acceptance_lines(out)
        assert len(lines) == 1, lines
        assert "❌" in lines[0] and "all checks passed" not in out

    @pytest.mark.parametrize(
        "reject_log,reason",
        [
            (REJECT_NOT_FOUND, "utterance_not_found"),
            (REJECT_TOO_SHORT, "too_short"),
        ],
        ids=["utterance_not_found", "too_short"],
    )
    def test_other_rejection_reasons_fail(self, tmp_path, reject_log, reason):
        logs = TOOLS_CATALOG + TOOL_CALLED + VOICE_ANSWER + reject_log
        rc, out, acc = run_check_acceptance(tmp_path, ACC_N201, logs)
        assert rc == 1, out
        assert reason in acc["reason"], acc["reason"]

    def test_no_registration_outcome_fails(self, tmp_path):
        """Тул «выполнен успешно», но ни принятия, ни отказа — не доказано,
        значит не принято (ADR-0018)."""
        rc, out, acc = run_check_acceptance(tmp_path, ACC_N201, LOG_NO_OUTCOME)
        assert rc == 1, out
        assert "NOT confirmed" in acc["reason"], acc["reason"]

    def test_accepted_registration_passes(self, tmp_path):
        rc, out, acc = run_check_acceptance(tmp_path, ACC_N201, LOG_ACCEPTED)
        assert rc == 0, out
        assert acc["pass"] is True
        assert acc["registration"]["accepted"] == ["Саша"]
        lines = acceptance_lines(out)
        assert len(lines) == 1 and "✅ all checks passed" in lines[0], lines

    def test_adr_0127_separate_profile_counts_as_accepted(self, tmp_path):
        """n722: голос похож на Сашу, имя другое — заведён ОТДЕЛЬНЫЙ профиль.
        Это принятая регистрация, не отказ."""
        logs = TOOLS_CATALOG + TOOL_CALLED + ACCEPTED_CONFLICT + VOICE_ANSWER
        rc, out, acc = run_check_acceptance(tmp_path, ACC_N201, logs)
        assert rc == 0, out
        assert acc["registration"]["accepted"] == ["Борис"]

    def test_late_ack_is_awaited(self, tmp_path):
        """Первое чтение лога — без исхода, второе — с принятием: харнесс
        дочитывает лог, а не красит шаг за гонку."""
        rc, out, acc = run_check_acceptance(
            tmp_path, ACC_N201, [LOG_NO_OUTCOME, LOG_ACCEPTED], wait_sec=3
        )
        assert rc == 0, out
        assert "ждали исход регистрации" in out

    def test_step_without_registration_ignores_rejection(self, tmp_path):
        """n210-подобный шаг (register_speaker запрещён) — отказ регистрации
        в окне шага к его вердикту отношения не имеет."""
        acc = {"expected_tool_calls": [], "must_not_call": ["register_speaker"]}
        logs = TOOLS_CATALOG + VOICE_ANSWER_NO_TOOL + REJECT_NO_CONTEXT
        rc, out, res = run_check_acceptance(tmp_path, acc, logs)
        assert rc == 0, out
        assert res["registration_expected"] is False

    def test_explicit_opt_out(self, tmp_path):
        acc = dict(ACC_N201, require_registration_accepted=False)
        rc, out, _ = run_check_acceptance(tmp_path, acc, LOG_REJECTED)
        assert rc == 0, out

    def test_explicit_opt_in_without_tool_in_expected(self, tmp_path):
        acc = {"expected_tool_calls": [], "must_not_call": [],
               "require_registration_accepted": True}
        rc, out, res = run_check_acceptance(tmp_path, acc, LOG_REJECTED)
        assert rc == 1, out
        assert "no_utterance_context" in res["reason"]

    def test_non_bool_flag_is_schema_error(self, tmp_path):
        acc = dict(ACC_N201, require_registration_accepted="yes")
        rc, out, res = run_check_acceptance(tmp_path, acc, LOG_ACCEPTED)
        assert rc == 1, out
        assert "require_registration_accepted must be true/false" in res["reason"]


class TestMergeIsPartOfTheSameVerdict:
    """Склейка профиля (run 35667281570) раньше проверялась bash-блоком ПОСЛЕ
    «ACCEPTANCE[…]: ✅ all checks passed» — две противоречивые строки."""

    def test_merge_fails_inside_check_acceptance(self, tmp_path):
        logs = TOOLS_CATALOG + TOOL_CALLED + MERGED + VOICE_ANSWER
        rc, out, acc = run_check_acceptance(tmp_path, ACC_N201, logs)
        assert rc == 1, out
        assert "merged into existing profile" in acc["reason"]
        assert "all checks passed" not in out
        merges = (tmp_path / "out" / "speaker_merges.log").read_text(encoding="utf-8")
        assert "n201_sasha_intro_long" in merges and "dc417cef" in merges

    def test_scenario_loop_has_no_second_merge_verdict(self):
        """В scenario-цикле не осталось собственной проверки склейки,
        печатавшей ❌ после ✅ той же попытки."""
        loop = extract_scenario_loop()
        assert "merged into existing profile" not in loop


class TestAttemptTaggedAcceptanceLine:
    """Дыра 1: строка ACCEPTANCE говорит, к какой попытке она относится."""

    def test_attempt_tag_in_line(self, tmp_path):
        rc, out, _ = run_check_acceptance(
            tmp_path, ACC_N201, LOG_VOICE_BEFORE_TOOL, tag="попытка 1/2"
        )
        assert rc == 1
        lines = acceptance_lines(out)
        assert lines == [ln for ln in lines if "] попытка 1/2: ❌" in ln], lines
        assert len(lines) == 1


# ── реальный scenario-цикл с ретраем ──────────────────────────────────────

def run_scenario(tmp_path: Path, attempt_logs: list[str]) -> str:
    """Один шаг n201 (retry_acceptance=1) через НАСТОЯЩИЙ scenario-цикл.
    run_step — заглушка «цикл прошёл», i-я попытка видит attempt_logs[i]."""
    for i, chunk in enumerate(attempt_logs, 1):
        (tmp_path / f"log_{i}.txt").write_text(chunk, encoding="utf-8")
    scenario = {
        "steps": [
            {
                "label": "n201_sasha_intro_long",
                "voice": "anton",
                "text": "Робот, привет, давай знакомиться как следует. Меня зовут Саша.",
                "expect": "cycle",
                "patterns": [],
                "acceptance": ACC_N201,
                "retry_acceptance": 1,
            }
        ]
    }
    scen = tmp_path / "scenario.json"
    scen.write_text(json.dumps(scenario, ensure_ascii=False), encoding="utf-8")
    script = "\n".join(
        [
            _stub_prelude(tmp_path),
            "E2E_REGISTRATION_ACK_WAIT_SEC=0",
            "E2E_RETRY_PAUSE=0",
            "PASS=1",
            "WAKE_GATE_SKIPPED_STEPS=0",
            'WAKE_GATE_PREFLIGHT_FILE="$OUT_DIR/wake_gate_preflight.json"',
            # Встраиваем текстом, а не `source`: read_text() нормализует
            # CRLF (чекаут на Windows с autocrlf), bash на \r спотыкается.
            LIB.read_text(encoding="utf-8"),
            WAKE.read_text(encoding="utf-8"),
            extract_function("emit_step"),
            extract_function("mark_fail_kind"),
            extract_function("check_acceptance"),
            # run_step: цикл прошёл; номер попытки → какой лог отдаст ssh.
            'run_step() { n="$(cat "$STUB/attempt" 2>/dev/null || echo 0)"; '
            'echo $((n + 1)) > "$STUB/attempt"; return 0; }',
            "parse_transcript() { :; }",
            "check_patterns() { return 0; }",
            f'parse_scenario_to_tsv "{scen.as_posix()}" "$OUT_DIR/scenario_parsed.txt"',
            'LAST_STEP_SPEECH_FILE="$OUT_DIR/.last_step_speech.txt"',
            ': > "$LAST_STEP_SPEECH_FILE"',
            extract_scenario_loop(),
            'echo "PASS=$PASS"',
            "",
        ]
    )
    rc, out, err = run_bash(script)
    assert "syntax error" not in err, err
    assert "PASS=" in out, (out, err)
    return out


def step_markers(out: str) -> list[str]:
    return [ln for ln in out.splitlines() if ln.startswith("E2E_STEP ")]


class TestScenarioLoopRetryVerdict:
    def test_issue_2846_run_35875477264_is_fail(self, tmp_path):
        """Попытка 1: голос ДО тула (#2406). Попытка 2: тул ДО голоса, но
        регистрация отклонена. На develop: E2E_STEP … OK."""
        out = run_scenario(tmp_path, [LOG_VOICE_BEFORE_TOOL, LOG_REJECTED])
        assert step_markers(out) == ["E2E_STEP n201_sasha_intro_long FAIL"], out
        assert "PASS=0" in out
        acc_lines = acceptance_lines(out)
        assert len(acc_lines) == 2, acc_lines
        assert "попытка 1/2: ❌" in acc_lines[0]
        assert "попытка 2/2: ❌" in acc_lines[1]
        assert "no_utterance_context" in acc_lines[1]
        assert "all checks passed" not in out
        verdicts = [ln for ln in out.splitlines() if "итог —" in ln]
        assert len(verdicts) == 1 and "❌ FAIL" in verdicts[0], verdicts

    def test_ok_after_retry_says_so(self, tmp_path):
        """Ретрай штатный (04d4ba2f6, LLM недетерминирован) — последняя
        попытка решает. Но OK после проваленной попытки обязан это сказать."""
        out = run_scenario(tmp_path, [LOG_VOICE_BEFORE_TOOL, LOG_ACCEPTED])
        assert step_markers(out) == [
            "E2E_STEP n201_sasha_intro_long OK after_retry=1"
        ], out
        assert "PASS=1" in out
        verdicts = [ln for ln in out.splitlines() if "итог —" in ln]
        assert len(verdicts) == 1, verdicts
        assert "✅ OK с попытки 2/2" in verdicts[0]
        assert "AFTER verbal answer" in verdicts[0]

    def test_first_attempt_ok_has_plain_marker(self, tmp_path):
        out = run_scenario(tmp_path, [LOG_ACCEPTED])
        assert step_markers(out) == ["E2E_STEP n201_sasha_intro_long OK"], out
        assert "попытка 1/2: ✅" in acceptance_lines(out)[0]


# ── чистые хелперы e2e_tool_match (импорт внутри: на develop их нет, и
#    падать должны тесты, а не сбор модуля) ─────────────────────────────────

def _tm():
    sys.path.insert(0, str(SCRIPTS_DIR))
    import e2e_tool_match  # noqa: E402

    return e2e_tool_match


class TestRegistrationHelpers:
    def test_outcome_dedups_rejection_from_both_nodes(self):
        out = _tm().registration_outcome(REJECT_NO_CONTEXT)
        assert out["rejected"] == [{"name": "Саша", "reason": "no_utterance_context"}]
        assert out["accepted"] == [] and out["merged"] == []

    def test_outcome_too_short_from_dialogue_line_only(self):
        """Если строку speaker_id_node переформулируют, отказ всё равно
        виден по dialogue_node (и наоборот, #2842 правит dialogue_node)."""
        line = _l("dialogue_node", "WARN",
                  "⚠️ [issue #2769] Регистрация 'Саша' отклонена — реплика 0.8с")
        out = _tm().registration_outcome(line)
        assert out["rejected"] == [{"name": "Саша", "reason": "too_short"}]

    def test_expected_auto_rule(self):
        tm = _tm()
        assert tm.registration_expected(ACC_N201) is True
        assert tm.registration_expected({"expected_tool_calls": ["register_speaker"]})
        assert tm.registration_expected({"discovery_tools": ["register_speaker"]})
        assert tm.registration_expected({"must_not_call": ["register_speaker"]}) is False
        assert tm.registration_expected({}) is False

    def test_dialogue_ack_alone_is_not_speaker_id_proof(self):
        """Маркер успеха — строка speaker_id_node, а не только ack в
        dialogue_node (его формат меняет #2842)."""
        ack_only = _l("dialogue_node", "INFO",
                      "✅ [issue 1077] Speaker registered: 'Саша' id=a1b2c3d4")
        assert _tm().registration_outcome(ack_only)["accepted"] == []
