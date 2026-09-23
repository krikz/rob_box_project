"""Issue #2902 — ложный FAIL акта 2c (n722, run 35912751803): два дефекта харнесса.

Лог робота, попытка 1 (цитата из issue #2902; робот на develop ``54d1a6f``)::

    [dialogue_node] 👥 [issue #2747] переспрашиваю про личность: twin=False conflict=True held_until_turn_end=True
    [dialogue_node] ✅ [turn] process_input returned: spoken='Здравствуй, Борис! Очень приятно познакомиться. Саша тут технику чинит…'
    [dialogue_node] 👥 [issue #2828] ответ хода заменён переспросом про личность: 'Здравствуй, Борис! Очень приятно позн…'
    [tts_node]      🔊 TTS: … text='Твой голос очень похож на голос, который я …'

Харнесс::

    >>> ACCEPTANCE[n722_boris_intro_conflict] попытка 1/2: ❌ forbidden phrases spoken by robot: ['Приятно познакомиться']

Попытка 2 (ретрай той же реплики) — Борис уже зарегистрирован, вопрос уже
задан, реплика прочитана как ответ::

    [dialogue_node] 👥 [issue #2828] ответ на переспрос: same=None kind=conflict new=d8ea7db8 known=8be4bc5e
    [tts_node]      🔊 TTS: … text='Здравствуй, Борис! Очень приятно познаком…'
    >>> ACCEPTANCE[n722_boris_intro_conflict] попытка 2/2: ❌ expected tool calls not invoked: ['register_speaker']; forbidden phrases spoken by robot: ['Приятно познакомиться'] …
    E2E_STEP n722_boris_intro_conflict FAIL

1. ``robot_speech()`` считал речью ``spoken=`` хода, чей ответ заменён
   переспросом и вслух НЕ звучал. Теперь ``spoken=`` — подстраховка по ходу:
   только если у хода нет TTS/speak_text и нет маркера «не озвучено».
2. Ретрай шага, изменившего состояние робота (регистрация принята, задан
   вопрос о личности, речь совпала с ``when_robot_asked`` следующего шага),
   запрещён: итог ``FAIL retry_blocked_state_changed`` одной строкой.

Строки лога — формат f-строк dialogue_node.py / tts_node.py / speaker_id_node.py
/ mcp_server.py в ROS-префиксе ``docker logs voice-assistant`` (обрезанные
«…» в issue — сокращения автора issue; здесь полный текст, как его печатает
tts_node с #1709). Исполняются НАСТОЯЩИЕ ``check_acceptance`` и scenario-цикл
из ``e2e_voice_test.sh`` (конвенция ``test_issue_2846_registration_verdict.py``).

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_2902_speech_and_retry.py -v --no-cov
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
ACT_2C = (
    REPO_ROOT / ".github" / "e2e" / "scenarios" / "night"
    / "night_marathon_act2c_voice_conflict_question_v1.json"
)

needs_bash = pytest.mark.skipif(
    shutil.which("bash") is None or shutil.which("python3") is None,
    reason="нужны bash и python3",
)

SCRIPT_LINES = E2E_SCRIPT.read_text(encoding="utf-8").split("\n")


def _tm():
    sys.path.insert(0, str(SCRIPTS_DIR))
    import e2e_tool_match  # noqa: E402

    return e2e_tool_match


# ── фикстуры лога voice-assistant ─────────────────────────────────────────

def _l(node: str, level: str, msg: str) -> str:
    return f"[{node}-9] [{level}] [1790193621.3] [{node}]: {msg}\n"


def _tts(text: str) -> str:
    # tts_node.py: f"🔊 TTS: speech_id=…, dialogue_id=…, batch=… i/n, voice=…, lang=…, text={text!r}"
    return _l(
        "tts_node", "INFO",
        "🔊 TTS: speech_id=5e1c2a9b, dialogue_id=None, batch=None None/None, "
        f"voice=default, lang=default, text={text!r}",
    )


def _turn(spoken: str, tools: list) -> str:
    # dialogue_node.py _run_turn: f"✅ [turn] process_input returned: spoken={…!r}[:60] tools=…"
    return _l(
        "dialogue_node", "INFO",
        f"✅ [turn] process_input returned: spoken={spoken!r}[:60] "
        f"tools={tools!r} finish_reason='stop' truncated_tool_args=False error=None",
    )


GREETING = (
    "Здравствуй, Борис! Очень приятно познакомиться. Саша тут технику чинит "
    "по вечерам, так что вы в хорошей компании."
)
QUESTION = (
    "Твой голос очень похож на голос, который я запомнил как Саша. "
    "Вы разные люди или это ты под другим именем?"
)

REGISTER_CALL = (
    _l("dialogue_node", "INFO", "📤 Отправлен запрос b5119339: register_speaker")
    + _l("mcp_server", "INFO",
         "📥 Запрос выполнения: register_speaker с параметрами {'name': 'Борис'}")
    + _l("mcp_server", "INFO", "✅ Инструмент register_speaker выполнен успешно")
)
# speaker_id_node _do_register (ADR-0127): отдельный профиль при конфликте.
CONFLICT_REGISTERED = _l(
    "speaker_id_node", "WARN",
    "⚠️ Speaker 'Борис' (id=d8ea7db8) — голос похож на уже известного 'Саша' "
    "(id=8be4bc5e, score=0.846 >= порога слияния), но имя другое: завожу "
    "ОТДЕЛЬНЫЙ профиль и НЕ переименовываю чужой (ADR-0127).",
)
ASK_HELD = _l(
    "dialogue_node", "INFO",
    "👥 [issue #2747] переспрашиваю про личность: twin=False conflict=True "
    "held_until_turn_end=True",
)
REPLACED = _l(
    "dialogue_node", "INFO",
    "👥 [issue #2828] ответ хода заменён переспросом про личность: "
    f"{GREETING[:80]!r}",
)

# Попытка 1 run 35912751803: ответ LLM заменён, вслух прозвучал ТОЛЬКО вопрос.
LOG_N722_ATTEMPT1 = (
    REGISTER_CALL + CONFLICT_REGISTERED + ASK_HELD
    + _turn(GREETING, ["register_speaker"]) + REPLACED + _tts(QUESTION)
)
# Попытка 2 того же прогона: реплика прочитана как ответ на переспрос,
# приветствие ПРОЗВУЧАЛО, register_speaker не вызван.
LOG_N722_ATTEMPT2 = (
    _l("dialogue_node", "INFO",
       "👥 [issue #2828] ответ на переспрос: same=None kind=conflict "
       "new=d8ea7db8 known=8be4bc5e")
    + _turn(GREETING, []) + _tts(GREETING)
)
# Поздний ack: приветствие ушло в TTS ДО вопроса — робот РЕАЛЬНО сказал
# запрещённое (оговорка _comment n722). Это честный FAIL, и состояние уже
# изменено: регистрация принята, вопрос задан.
LOG_N722_LATE_ACK = (
    REGISTER_CALL + _turn(GREETING, ["register_speaker"]) + _tts(GREETING)
    + CONFLICT_REGISTERED
    + _l("dialogue_node", "INFO",
         "👥 [issue #2747] переспрашиваю про личность: twin=False conflict=True "
         "held_until_turn_end=False")
    + _tts(QUESTION)
)
# n723: «мы разные» — вопрос разрешён, повторной регистрации нет.
LOG_N723_ANSWER = (
    _l("dialogue_node", "INFO",
       "👥 [issue #2828] ответ на переспрос: same=False kind=conflict "
       "new=d8ea7db8 known=8be4bc5e")
    + _turn("Понял, вы разные люди — так и запомню.", [])
    + _tts("Понял, вы разные люди — так и запомню.")
)

ACT = json.loads(ACT_2C.read_text(encoding="utf-8"))
STEPS = {s["label"]: s for s in ACT["steps"]}
N721 = STEPS["n721_sasha_intro"]
N722 = STEPS["n722_boris_intro_conflict"]
N723 = STEPS["n723_boris_answer_different"]
FORBIDDEN = N722["acceptance"]["must_not_say"]


# ── 1. robot_speech(): речь — только озвученное ──────────────────────────

class TestSpeechIsWhatWasVoiced:
    def test_issue_2902_replaced_answer_is_not_speech(self):
        """Ровно попытка 1 run 35912751803: приветствие LLM заменено
        переспросом, прозвучал только вопрос."""
        tm = _tm()
        speech = tm.robot_speech(LOG_N722_ATTEMPT1)
        assert "Твой голос очень похож" in speech
        assert "Здравствуй, Борис" not in speech
        assert [k for k in FORBIDDEN if tm.keyword_hit(LOG_N722_ATTEMPT1, k)] == []

    def test_voiced_forbidden_phrase_still_caught(self):
        """Не ослаблено: приветствие, реально ушедшее в TTS, ловится."""
        tm = _tm()
        for log in (LOG_N722_ATTEMPT2, LOG_N722_LATE_ACK):
            assert tm.keyword_hit(log, "Приятно познакомиться"), log

    def test_expected_keywords_and_when_robot_asked_use_same_channel(self):
        """expected_keywords / must_not_say / when_robot_asked — одна функция:
        вопрос из TTS виден всем трём, заменённое приветствие — никому."""
        tm = _tm()
        assert tm.keyword_hit(LOG_N722_ATTEMPT1, "разные люди")
        assert not tm.keyword_hit(LOG_N722_ATTEMPT1, "Борис")
        pattern = N723["when_robot_asked"]
        assert re.search(pattern, tm.robot_speech(LOG_N722_ATTEMPT1), re.I)

    def test_spoken_without_tts_is_still_speech(self):
        """Подстраховка жива: TTS в окно не доехал, маркера нет — spoken=
        считается речью, как раньше (строже, а не мягче)."""
        tm = _tm()
        log = _turn("Привет, Борис, рад знакомству", [])
        assert tm.keyword_hit(log, "Привет, Борис")

    @pytest.mark.parametrize(
        "marker_line",
        [
            REPLACED,
            _l("dialogue_node", "INFO",
               "🔇 LLM completion marker — skip auto-TTS: 'Привет, Борис'"),
            _l("dialogue_node", "INFO",
               "🔇 [issue 988] speak_text called — final text skipped "
               "(anti-duplicate): 'Привет, Борис'"),
            _l("dialogue_node", "INFO",
               "🔇 [issue 988] speak_text called in cycle — skipping auto-TTS "
               "of final text: 'Привет, Борис'"),
            _l("dialogue_node", "INFO",
               "🔇 [DJ] переход #3 — свободный текст НЕ озвучиваю (речь только "
               "через speak_text/хук): 'Привет, Борис'"),
            _l("dialogue_node", "WARN",
               "🤐 [issue 1882] planning-narration hard-mute: spoken matches "
               "planning pattern, tools empty, speaking nothing (head='Привет, Борис')"),
            _l("dialogue_node", "WARN",
               "🔇 Служебный текст LLM не озвучиваем: '[SYSTEM] Привет, Борис'"),
            _l("dialogue_node", "WARN",
               "🏷️ [issue 2760] LLM написала вызов тула ТЕКСТОМ вместо "
               "tool-call (head='Привет, Борис') — один ретрай, user_input='x', tools=[]"),
            _l("dialogue_node", "WARN",
               "🧾 [issue 2175] MiniMax regurgitates system-template в spoken "
               "(head='Привет, Борис') — один ретрай, user_input='x', tools=[]"),
            _l("dialogue_node", "INFO",
               "🔇 [issue 2874] ответ хода не озвучиваю (ретрай/отзыв гуардом, "
               "retry=True retracted=False): 'Привет, Борис'"),
        ],
        ids=["2828_replaced", "done_marker", "988_short", "988_verbose", "dj",
             "1882_planning", "service_text", "2760_markup", "2175_regurgitate",
             "2874_retracted"],
    )
    def test_not_voiced_markers_drop_spoken(self, marker_line):
        tm = _tm()
        log = _turn("Привет, Борис, рад знакомству", []) + marker_line
        assert not tm.keyword_hit(log, "Привет, Борис"), marker_line

    def test_marker_is_scoped_to_its_turn(self):
        """Маркер хода 2 не глушит spoken хода 1; #2888-вопрос, напечатанный
        ДО строки результата хода 2, не глушит spoken хода 1."""
        tm = _tm()
        log = (
            _turn("Привет, Борис, рад знакомству", [])
            + _l("dialogue_node", "INFO",
                 "👤 [issue #2888] identity question by robot: kind=tentative "
                 "held_until_turn_end=True")
            + _turn("Саша, это ты?", []) + REPLACED + _tts("Это ты, Саша?")
        )
        assert tm.keyword_hit(log, "Привет, Борис")
        assert tm.keyword_hit(log, "Это ты, Саша")
        assert not tm.keyword_hit(log, "Саша, это ты")

    def test_tts_text_with_apostrophe_is_captured(self):
        """repr() берёт двойные кавычки, если в тексте апостроф — такую
        фразу spoken= больше не страхует, TTS обязан её видеть."""
        tm = _tm()
        log = _turn("Rock'n'roll, Борис!", []) + _tts("Rock'n'roll, Борис!")
        assert tm.keyword_hit(log, "Rock'n'roll")


# ── 2. retry_block_reason(): чистое правило ──────────────────────────────

class TestRetryBlockReason:
    def test_issue_2902_attempt1_changed_state(self):
        reason = _tm().retry_block_reason(
            LOG_N722_ATTEMPT1, N723["when_robot_asked"]
        )
        assert "registration accepted: 'Борис'" in reason
        assert "identity question" in reason
        assert "when_robot_asked" in reason

    def test_rejected_registration_does_not_block(self):
        """#2846: отказ регистрации ничего не изменил — ретрай полезен."""
        log = REGISTER_CALL + _l(
            "speaker_id_node", "WARN",
            "⚠️ [issue #2829] register_request for 'Борис' has no utterance_id "
            "-- honest refusal instead of registering whoever speaks next",
        ) + _turn("Приятно познакомиться, Борис!", ["register_speaker"])
        assert _tm().retry_block_reason(log, "") == ""

    def test_next_step_question_alone_blocks(self):
        log = _turn("Вы разные люди?", []) + _tts("Вы разные люди?")
        assert "when_robot_asked" in _tm().retry_block_reason(
            log, N723["when_robot_asked"]
        )
        assert _tm().retry_block_reason(log, "") == ""

    def test_bad_pattern_is_not_a_crash(self):
        assert _tm().retry_block_reason(_tts("Привет"), "(") == ""


# ── 3. настоящий bash: check_acceptance и scenario-цикл ──────────────────

def extract_function(name: str) -> str:
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
    start = next(
        i for i, ln in enumerate(SCRIPT_LINES)
        if ln.startswith("    while IFS=$'\\x1f' eval \"$E2E_SCENARIO_ROW_READ\"; do")
    )
    end = next(
        i for i, ln in enumerate(SCRIPT_LINES)
        if i > start and ln == '    done < "$OUT_DIR/scenario_parsed.txt"'
    )
    return "\n".join(SCRIPT_LINES[start : end + 1])


def run_bash(script: str) -> tuple:
    proc = subprocess.run(
        ["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=180
    )
    return (
        proc.returncode,
        proc.stdout.decode("utf-8", "replace"),
        proc.stderr.decode("utf-8", "replace"),
    )


def _stub_prelude(tmp: Path) -> str:
    """ssh-заглушка: ``docker logs`` отдаёт $STUB/log_<N>.txt, где N — номер
    сыгранной реплики (его пишет заглушка run_step)."""
    fake_ssh = tmp / "fake_ssh.sh"
    fake_ssh.write_bytes(
        (
            "#!/bin/bash\n"
            'case "$*" in\n'
            '  *"docker logs"*)\n'
            '    n="$(cat "$STUB/attempt" 2>/dev/null || echo 1)"\n'
            '    cat "$STUB/log_${n}.txt" 2>/dev/null ;;\n'
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
            "E2E_REGISTRATION_ACK_WAIT_SEC=0",
            "",
        ]
    )


def run_check_acceptance(tmp_path: Path, label: str, acc: dict, log: str):
    (tmp_path / "log_1.txt").write_text(log, encoding="utf-8")
    acc_s = json.dumps(acc, ensure_ascii=False).replace("'", "'\\''")
    script = "\n".join(
        [
            _stub_prelude(tmp_path),
            extract_function("check_acceptance"),
            f"check_acceptance {label} '{acc_s}' 2026-09-23T14:37:48Z 'попытка 1/2'",
            'echo "RC=$?"',
            "",
        ]
    )
    rc, out, err = run_bash(script)
    assert "syntax error" not in err, err
    m = re.search(r"^RC=(\d+)$", out, re.M)
    assert m, (out, err)
    return int(m.group(1)), out


def run_scenario(tmp_path: Path, steps: list, logs: list) -> str:
    """Сценарий через НАСТОЯЩИЙ scenario-цикл; i-я сыгранная реплика
    (включая повторы) видит logs[i-1]."""
    for i, chunk in enumerate(logs, 1):
        (tmp_path / f"log_{i}.txt").write_text(chunk, encoding="utf-8")
    scen = tmp_path / "scenario.json"
    scen.write_text(json.dumps({"steps": steps}, ensure_ascii=False), encoding="utf-8")
    script = "\n".join(
        [
            _stub_prelude(tmp_path),
            "E2E_RETRY_PAUSE=0",
            "PASS=1",
            "WAKE_GATE_SKIPPED_STEPS=0",
            'WAKE_GATE_PREFLIGHT_FILE="$OUT_DIR/wake_gate_preflight.json"',
            LIB.read_text(encoding="utf-8"),
            WAKE.read_text(encoding="utf-8"),
            extract_function("emit_step"),
            extract_function("mark_fail_kind"),
            extract_function("check_acceptance"),
            'run_step() { n="$(cat "$STUB/attempt" 2>/dev/null || echo 0)"; '
            'echo $((n + 1)) > "$STUB/attempt"; return 0; }',
            "parse_transcript() { :; }",
            "check_patterns() { return 0; }",
            f'parse_scenario_to_tsv "{scen.as_posix()}" "$OUT_DIR/scenario_parsed.txt"',
            'LAST_STEP_SPEECH_FILE="$OUT_DIR/.last_step_speech.txt"',
            ': > "$LAST_STEP_SPEECH_FILE"',
            extract_scenario_loop(),
            'echo "PASS=$PASS"',
            'echo "PLAYED=$(cat "$STUB/attempt" 2>/dev/null || echo 0)"',
            "",
        ]
    )
    rc, out, err = run_bash(script)
    assert "syntax error" not in err, err
    assert "PASS=" in out, (out, err)
    return out


def step_markers(out: str) -> list:
    return [ln for ln in out.splitlines() if ln.startswith("E2E_STEP ")]


def acceptance_lines(out: str, label: str) -> list:
    return [ln for ln in out.splitlines() if ln.startswith(f">>> ACCEPTANCE[{label}]")]


def verdict_lines(out: str, label: str) -> list:
    return [ln for ln in out.splitlines()
            if ln.startswith(f">>> STEP {label}: итог —")]


@needs_bash
class TestCheckAcceptanceN722:
    def test_issue_2902_attempt1_passes_acceptance(self, tmp_path):
        """На develop: «❌ forbidden phrases spoken by robot:
        ['Приятно познакомиться']» — по тексту, который не звучал."""
        rc, out = run_check_acceptance(
            tmp_path, N722["label"], N722["acceptance"], LOG_N722_ATTEMPT1
        )
        assert rc == 0, out
        assert "forbidden phrases spoken" not in out
        acc = json.loads((tmp_path / "out" / "acceptance.json").read_text(encoding="utf-8"))
        assert "Здравствуй, Борис" not in acc["robot_speech"]
        assert "Твой голос очень похож" in acc["robot_speech"]

    def test_voiced_greeting_still_fails(self, tmp_path):
        rc, out = run_check_acceptance(
            tmp_path, N722["label"], N722["acceptance"], LOG_N722_LATE_ACK
        )
        assert rc == 1, out
        assert "forbidden phrases spoken by robot: ['Приятно познакомиться']" in out


@needs_bash
class TestScenarioLoopN722:
    def test_issue_2902_act_2c_replay_is_green(self, tmp_path):
        """run 35912751803 целиком: n722 проходит с первой попытки, повтора
        нет, n723 отвечает на заданный вопрос. На develop: n722 FAIL."""
        out = run_scenario(
            tmp_path, [N722, N723], [LOG_N722_ATTEMPT1, LOG_N723_ANSWER]
        )
        assert step_markers(out) == [
            "E2E_STEP n722_boris_intro_conflict OK",
            "E2E_STEP n723_boris_answer_different OK",
        ], out
        assert len(acceptance_lines(out, N722["label"])) == 1
        assert "PASS=1" in out and "PLAYED=2" in out

    def test_state_changing_attempt_is_not_retried(self, tmp_path):
        """Попытка 1 честно провалена (приветствие прозвучало), но Борис уже
        зарегистрирован и вопрос задан — повтор той же реплики проверял бы
        ответ на переспрос. Ретрая нет, итог — одна строка, реплика n723 не
        съедена повтором. На develop: повтор (2 строки ACCEPTANCE n722)."""
        out = run_scenario(
            tmp_path, [N722, N723], [LOG_N722_LATE_ACK, LOG_N723_ANSWER]
        )
        markers = step_markers(out)
        assert markers == [
            "E2E_STEP n722_boris_intro_conflict FAIL retry_blocked_state_changed",
            "E2E_STEP n723_boris_answer_different OK",
        ], out
        acc = acceptance_lines(out, N722["label"])
        assert len(acc) == 1 and "попытка 1/2: ❌" in acc[0], acc
        verdicts = verdict_lines(out, N722["label"])
        assert len(verdicts) == 1, verdicts
        assert "❌ FAIL на попытке 1/2" in verdicts[0]
        assert "ретрай запрещён" in verdicts[0]
        assert "registration accepted: 'Борис'" in verdicts[0]
        assert "retry 1/1" not in out
        assert "PASS=0" in out and "PLAYED=2" in out

    def test_retry_without_state_change_keeps_2855_verdict(self, tmp_path):
        """Регистрацию отклонили — состояние не менялось, ретрай штатный, и
        вердикт #2855 прежний: «OK after_retry=1», одна строка итога."""
        rejected = (
            REGISTER_CALL + _turn("Приятно познакомиться, Саша!", ["register_speaker"])
            + _tts("Приятно познакомиться, Саша!")
            + _l("speaker_id_node", "WARN",
                 "⚠️ [issue #2829] register_request for 'Саша' has no utterance_id "
                 "-- honest refusal instead of registering whoever speaks next")
        )
        accepted = (
            REGISTER_CALL
            + _l("speaker_id_node", "INFO", "✅ Speaker 'Саша' registered (id=a1b2c3d4)")
            + _turn("Приятно познакомиться, Саша!", ["register_speaker"])
            + _tts("Приятно познакомиться, Саша!")
        )
        out = run_scenario(tmp_path, [N721], [rejected, accepted])
        assert step_markers(out) == ["E2E_STEP n721_sasha_intro OK after_retry=1"], out
        verdicts = verdict_lines(out, N721["label"])
        assert len(verdicts) == 1 and "✅ OK с попытки 2/2" in verdicts[0], verdicts
        assert "ретрай НЕ делаю" not in out

    def test_next_step_question_blocks_retry(self, tmp_path):
        """Шаг без регистрации: робот уже задал вопрос, на который отвечает
        следующий шаг (when_robot_asked), а проверка провалилась по другой
        причине — повтор реплики стал бы ответом на вопрос."""
        step = {
            "label": "s1_hello",
            "voice": "ermil",
            "text": "Робот, привет.",
            "patterns": [],
            "acceptance": {"expected_tool_calls": [], "must_not_call": [],
                           "expected_keywords": ["погод"]},
            "retry_acceptance": 1,
        }
        asked = _turn("Вы разные люди?", []) + _tts("Вы разные люди?")
        out = run_scenario(tmp_path, [step, N723], [asked, LOG_N723_ANSWER])
        assert step_markers(out) == [
            "E2E_STEP s1_hello FAIL retry_blocked_state_changed",
            "E2E_STEP n723_boris_answer_different OK",
        ], out
        assert "when_robot_asked" in verdict_lines(out, "s1_hello")[0]
