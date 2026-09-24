"""Issue #2933 — харнесс пишет «нет акцепта» и переигрывает реплику, хотя
робот отработал полный цикл (ПРИНЯТО → LLM → TTS): run 35941885025, n704 ×3.

Лог робота (цитата из issue #2933, робот на develop ``06ac0d2``)::

    1790212630.78 [stt_node]      ✅ ПРИНЯТО (respeaker): робот а что ты обо
                                   мне запомнил из того что я говорил
    1790212639.04 [dialogue_node] ✅ [turn] process_input returned:
                                   spoken='Хм, тут пустовато — …'
    1790212639.05 [tts_node]      🔊 TTS: text='Хм, тут пустовато — …'

Харнесс (``check_cycle()``, ``.github/workflows/scripts/e2e_voice_test.sh``
~строка 1439) трижды написал ``нет акцепта`` и переиграл реплику n704, хотя
``docker logs voice-assistant`` за то же окно показывают полный цикл все три
раза (таймстемпы согласуются с PLAY attempt 1/2/3 из лога харнесса run
35941885025, см. анализ ниже) — робот получил три дубля одной команды.

Раскопанная причина (доказывается тестами ниже, класс
``TestCheckCycleAcceptedButUnconfirmed``): ``check_cycle()`` до фикса не
различал «ПРИНЯТО в логах ЕСТЬ, но цикл ещё не закрылся к моменту проверки»
(ретрай гарда — в этих ходах ДВА ``process_input returned`` и ДВА ``🔊 TTS:``
— может не уложиться в ``E2E_REACTION_WINDOW``, либо ``docker logs`` по SSH
не прочитались вовсе) от «робот реально не услышал команду» — обе ветки
схлопывались в один и тот же ``return 1``, и ``run_step()`` переигрывал
``paplay`` вслепую (класс ``TestRunStepDoesNotReplayAfterAccept``).

Фикс — ``check_cycle()`` теперь возвращает отдельный ``rc=3`` («ПРИНЯТО
есть, цикл не подтверждён — не переигрывать, читать логи того же окна ещё
раз»), а ``run_step()`` при ``rc=3`` пропускает повторный ``paplay`` и
переиспользует то же окно ``BEFORE``.

Строки лога — формат f-строк stt_node.py / dialogue_node.py / tts_node.py в
ROS-префиксе ``docker logs voice-assistant`` (конвенция
``test_issue_2902_speech_and_retry.py``: узел-pid, уровень, ROS-таймстемп в
квадратных скобках, узел, сообщение). Исполняются НАСТОЯЩИЕ ``check_cycle``
и ``run_step`` из ``e2e_voice_test.sh``.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_2933_false_no_accept.py -v --no-cov
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
E2E_SCRIPT = SCRIPTS_DIR / "e2e_voice_test.sh"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"

needs_bash = pytest.mark.skipif(
    shutil.which("bash") is None or shutil.which("python3") is None,
    reason="нужны bash и python3",
)

SCRIPT_LINES = E2E_SCRIPT.read_text(encoding="utf-8").split("\n")


# ── фикстуры лога voice-assistant (issue #2933, n704) ─────────────────────

def _l(node: str, level: str, ts: str, msg: str) -> str:
    return f"[{node}-9] [{level}] [{ts}] [{node}]: {msg}\n"


def _accept(ts: str, text: str) -> str:
    # stt_node.py:986 — f"✅ ПРИНЯТО ({source}): {text}"
    return _l("stt_node", "INFO", ts, f"✅ ПРИНЯТО (respeaker): {text}")


def _llm_input(ts: str, text: str) -> str:
    # dialogue_node.py:3047 — f"📥 LLM INPUT: {clean[:200]!r}"
    return _l("dialogue_node", "INFO", ts, f"📥 LLM INPUT: {text!r}")


def _turn(ts: str, spoken: str) -> str:
    # dialogue_node.py:4804 — f"✅ [turn] process_input returned: spoken=…"
    return _l(
        "dialogue_node", "INFO", ts,
        f"✅ [turn] process_input returned: spoken={spoken!r}[:60] "
        "tools=[] finish_reason='stop' truncated_tool_args=False error=None",
    )


def _tts_start(ts: str, text: str) -> str:
    # tts_node.py:2759 — f"🔊 TTS: speech_id=…, …, text={text!r}"
    return _l(
        "tts_node", "INFO", ts,
        f"🔊 TTS: speech_id=5e1c2a9b, dialogue_id=None, batch=None None/None, "
        f"voice=default, lang=default, text={text!r}",
    )


def _tts_finished(ts: str) -> str:
    # tts_node.py:5780-5784 — "✅ Воспроизведение завершено" +
    # "📢 Публикую TTS finished event: …, success=True, …"
    return (
        _l("tts_node", "INFO", ts, "✅ Воспроизведение завершено")
        + _l(
            "tts_node", "INFO", ts,
            "📢 Публикую TTS finished event: speech_id=5e1c2a9b..., "
            "success=True, duration=3.1s, batch=1/1",
        )
    )


QUESTION = "робот а что ты обо мне запомнил из того что я говорил"
ANSWER1 = "Хм, тут пустовато — "
ANSWER2 = "На самом деле пустовато — "

# Снимок docker logs НА МОМЕНТ проверки check_cycle (спустя
# E2E_REACTION_WINDOW=40с после PLAY, т.е. ~T+40): ПРИНЯТО есть, гард
# ретраит (2× process_input returned, 2× 🔊 TTS start), но ни один TTS ещё
# НЕ дошёл до "TTS finished" в этом окне — синтез двух реплик подряд не
# уложился в 40с. Это ровно то, что видел check_cycle в run 35941885025 на
# всех трёх попытках n704 (акцепт+TTS start в логе есть, харнесс всё равно
# написал "нет акцепта").
LOG_ACCEPTED_CYCLE_NOT_YET_CLOSED = (
    _accept("1790212630.78", QUESTION)
    + _llm_input("1790212630.90", QUESTION)
    + _turn("1790212635.00", ANSWER1)
    + _tts_start("1790212635.05", ANSWER1)
    + _turn("1790212639.04", ANSWER2)
    + _tts_start("1790212639.05", ANSWER2)
)

# То же окно, перечитанное чуть позже: второй TTS (после ретрая гарда)
# успел закрыться — "TTS finished" пришёл. Ход с 2 process_input / 2 TTS —
# это ПОЛНЫЙ цикл (issue #2933, п.3), не регресс.
LOG_ACCEPTED_CYCLE_CLOSED = LOG_ACCEPTED_CYCLE_NOT_YET_CLOSED + _tts_finished("1790212642.30")


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


def run_bash(script: str) -> tuple:
    proc = subprocess.run(
        ["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=180
    )
    return (
        proc.returncode,
        proc.stdout.decode("utf-8", "replace"),
        proc.stderr.decode("utf-8", "replace"),
    )


def _fake_ssh(tmp: Path, logs_by_call: list) -> Path:
    """SSH-заглушка: N-й вызов ``docker logs`` отдаёт logs_by_call[N-1];
    ``date`` отдаёт настоящий UTC. Счётчик считает вызовы docker logs, а не
    PLAY — так тест видит РЕАЛЬНОЕ число обращений к логам робота."""
    for i, chunk in enumerate(logs_by_call, 1):
        (tmp / f"log_{i}.txt").write_text(chunk, encoding="utf-8")
    fake_ssh = tmp / "fake_ssh.sh"
    fake_ssh.write_text(
        "#!/bin/bash\n"
        'case "$*" in\n'
        '  *"docker logs"*)\n'
        '    n=$(( $(cat "$STUB/dockercall" 2>/dev/null || echo 0) + 1 ))\n'
        '    echo "$n" > "$STUB/dockercall"\n'
        '    cat "$STUB/log_${n}.txt" 2>/dev/null ;;\n'
        "  *date*) date -u +%Y-%m-%dT%H:%M:%SZ ;;\n"
        "  *) : ;;\n"
        "esac\n",
        encoding="utf-8",
    )
    fake_ssh.chmod(0o755)
    return fake_ssh


def _fake_ssh_broken(tmp: Path) -> Path:
    """SSH-заглушка, имитирующая недоступность robot'а/логов: docker logs
    падает с ненулевым rc (сеть/ssh-сбой), а не отдаёт пустую строку с rc=0."""
    fake_ssh = tmp / "fake_ssh_broken.sh"
    fake_ssh.write_text(
        "#!/bin/bash\n"
        'case "$*" in\n'
        '  *"docker logs"*) echo "ssh: connect to host 10.1.1.21 port 22: Connection timed out" >&2; exit 255 ;;\n'
        "  *date*) date -u +%Y-%m-%dT%H:%M:%SZ ;;\n"
        "  *) : ;;\n"
        "esac\n",
        encoding="utf-8",
    )
    fake_ssh.chmod(0o755)
    return fake_ssh


def run_check_cycle(tmp_path: Path, before: str, fake_ssh: Path) -> tuple:
    script = "\n".join(
        [
            "set -u",
            f'STUB="{tmp_path.as_posix()}"; export STUB',
            f'OUT_DIR="{tmp_path.as_posix()}/out"; mkdir -p "$OUT_DIR"',
            f'ROBOT_SSH="bash {fake_ssh.as_posix()}"',
            'log() { echo ">>> $*" >&2; }',
            extract_function("check_cycle"),
            f'check_cycle "{before}"',
            'echo "RC=$?"',
            "",
        ]
    )
    rc, out, err = run_bash(script)
    assert "syntax error" not in err, err
    m = re.search(r"^RC=(\d+)$", out, re.M)
    assert m, (out, err)
    return int(m.group(1)), out, err


@needs_bash
class TestCheckCycleAcceptedButUnconfirmed:
    """Прямое доказательство: check_cycle() должен отличать «ПРИНЯТО есть,
    цикл ещё не закрылся» от «нет акцепта». До фикса обе ветки отдавали
    rc=1 — этот тест ловит регресс на develop и подтверждает фикс."""

    def test_accepted_but_no_tts_finished_yet_is_not_no_accept(self, tmp_path):
        """Снимок лога в момент проверки (run 35941885025, n704): ПРИНЯТО
        + 2×process_input + 2×TTS-start, TTS finished ещё не пришёл.
        Ожидание (issue #2933): rc=3 («не переигрывать»), НЕ rc=1."""
        fake_ssh = _fake_ssh(tmp_path, [LOG_ACCEPTED_CYCLE_NOT_YET_CLOSED])
        rc, out, err = run_check_cycle(tmp_path, "2026-09-24T01:17:01Z", fake_ssh)
        assert rc == 3, (
            f"check_cycle вернул rc={rc}, ожидался rc=3 (ПРИНЯТО есть, цикл "
            f"не подтверждён — не «нет акцепта»). stdout={out!r} stderr={err!r}"
        )

    def test_ssh_failure_is_not_no_accept(self, tmp_path):
        """SSH/docker logs недоступны (rc!=0) — раньше `|| echo ''` схлопывал
        это в тот же rc=1, что и «робот молчит». issue #2933: должно быть
        rc=3 («логи не прочитались», не «нет акцепта»)."""
        fake_ssh = _fake_ssh_broken(tmp_path)
        rc, out, err = run_check_cycle(tmp_path, "2026-09-24T01:17:01Z", fake_ssh)
        assert rc == 3, (
            f"check_cycle вернул rc={rc}, ожидался rc=3 (SSH/логи недоступны "
            f"— не «нет акцепта»). stdout={out!r} stderr={err!r}"
        )

    def test_genuinely_no_accept_is_still_rc1(self, tmp_path):
        """Не ослаблено: если ПРИНЯТО в логах правда нет — всё ещё rc=1
        (реплику можно переигрывать)."""
        fake_ssh = _fake_ssh(tmp_path, ["[stt_node-9] [INFO] [1790212630.78] [stt_node]: обычный лог без акцепта\n"])
        rc, out, err = run_check_cycle(tmp_path, "2026-09-24T01:17:01Z", fake_ssh)
        assert rc == 1, (out, err)

    def test_guard_retry_two_turns_two_tts_is_full_cycle_when_closed(self, tmp_path):
        """issue #2933 п.3: ход с ретраем гарда (2 process_input, 2 TTS) —
        это ПОЛНЫЙ цикл, если TTS finished в итоге пришёл. Не регресс."""
        fake_ssh = _fake_ssh(tmp_path, [LOG_ACCEPTED_CYCLE_CLOSED])
        rc, out, err = run_check_cycle(tmp_path, "2026-09-24T01:17:01Z", fake_ssh)
        assert rc == 0, (
            f"check_cycle вернул rc={rc}, ожидался rc=0 (полный цикл с "
            f"ретраем гарда). stdout={out!r} stderr={err!r}"
        )


def _paplay_stub(bindir: Path, counter: Path) -> None:
    p = bindir / "paplay"
    p.write_text(
        "#!/bin/bash\n"
        f'n=$(( $(cat "{counter.as_posix()}" 2>/dev/null || echo 0) + 1 ))\n'
        f'echo "$n" > "{counter.as_posix()}"\n'
        "exit 0\n",
        encoding="utf-8",
    )
    p.chmod(0o755)
    for name in ("ffmpeg", "pactl", "ffprobe"):
        q = bindir / name
        q.write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
        q.chmod(0o755)


@needs_bash
class TestRunStepDoesNotReplayAfterAccept:
    """run_step() не должен переигрывать paplay, если ПРИНЯТО уже есть в
    логах — иначе робот получает дубли (issue #2933)."""

    def test_no_duplicate_play_once_accepted(self, tmp_path):
        # 1-й вызов docker logs (attempt 1): ПРИНЯТО есть, цикл ещё не
        # закрылся → check_cycle rc=3 → run_step НЕ должен переиграть paplay.
        # 2-й вызов docker logs (attempt 2, то же окно): цикл закрылся →
        # rc=0 → шаг завершается успешно.
        fake_ssh = _fake_ssh(
            tmp_path,
            [LOG_ACCEPTED_CYCLE_NOT_YET_CLOSED, LOG_ACCEPTED_CYCLE_CLOSED],
        )
        bindir = tmp_path / "bin"
        bindir.mkdir()
        play_counter = tmp_path / "play_count"
        _paplay_stub(bindir, play_counter)
        cmd_wav = tmp_path / "out" / "cmd_n704_sasha_no_repeat_question_eq.wav"
        cmd_wav.parent.mkdir(parents=True, exist_ok=True)
        cmd_wav.write_bytes(b"RIFF....WAVEfmt ")

        script = "\n".join(
            [
                "set -u",
                f'export PATH="{bindir.as_posix()}:$PATH"',
                f'STUB="{tmp_path.as_posix()}"; export STUB',
                f'OUT_DIR="{tmp_path.as_posix()}/out"; mkdir -p "$OUT_DIR"',
                f'ROBOT_SSH="bash {fake_ssh.as_posix()}"',
                "E2E_MAX_ATTEMPTS=3",
                "E2E_REACTION_WINDOW=0",
                "E2E_RETRY_PAUSE=0",
                "E2E_SILENCE_WAIT=0",
                "E2E_SILENCE_WAIT_MAX=0",
                'log() { echo ">>> $*"; }',
                LIB.read_text(encoding="utf-8"),
                extract_function("ensure_outdir"),
                extract_function("emit_step"),
                extract_function("mark_fail_kind"),
                extract_function("vad_reject_reason"),
                extract_function("emit_step_fail_or_vad"),
                extract_function("check_cycle"),
                extract_function("run_step"),
                'synth_command() { : ; }',
                'run_step "робот а что ты обо мне запомнил из того что я говорил" '
                'anton n704_sasha_no_repeat_question cycle',
                'echo "RUN_STEP_RC=$?"',
                f'echo "PLAY_COUNT=$(cat "{play_counter.as_posix()}" 2>/dev/null || echo 0)"',
                "",
            ]
        )
        rc, out, err = run_bash(script)
        assert "syntax error" not in err, err

        m_rc = re.search(r"^RUN_STEP_RC=(\d+)$", out, re.M)
        m_play = re.search(r"^PLAY_COUNT=(\d+)$", out, re.M)
        assert m_rc and m_play, (out, err)
        run_step_rc = int(m_rc.group(1))
        play_count = int(m_play.group(1))

        assert run_step_rc == 0, (
            f"run_step() вернул {run_step_rc} — ожидался 0 (шаг в итоге "
            f"подтверждён полным циклом). stdout={out!r} stderr={err!r}"
        )
        assert play_count == 1, (
            f"issue #2933: paplay вызван {play_count} раз(а) — харнесс "
            f"переиграл реплику, хотя ПРИНЯТО было зафиксировано уже в "
            f"первом окне (робот получает дубли). stdout={out!r}"
        )
        assert "ПРИНЯТО уже было" in out, out
