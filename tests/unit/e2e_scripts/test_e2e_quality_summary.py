"""
test_e2e_quality_summary.py — сводка качества e2e-прогона.

Вердикт e2e остаётся БИНАРНЫМ (ADR-0015). Здесь проверяется не новая шкала, а
то, что доказательства прогона доезжают до человека:

  1. emit_step() — результат каждого шага копится в steps.jsonl
     (раньше «9 из 11 прошло» не сохранялось нигде, и FAIL на первом шаге был
     неотличим от FAIL на последнем).
  2. write_summary() — сводит шаги + GATE-1 + метрики звука в summary.json.
  3. parse_transcript() — transcript.json обязан быть ВАЛИДНЫМ JSON.
     В живом артефакте прогона 34928781542 лежало
     ``"expected": Робот, стоп музыку,`` — без кавычек. Файл читает
     e2e_baseline_diff.py под ``except: pass``, поэтому keyword_match_pct
     молча не считался.
  4. Порядок в main flow: запись конвертируется в wav ДО замеров. Раньше
     stop_recording висел только на ``trap EXIT`` и отрабатывал позже, а оба
     артефакта содержали ``{"error":"recording.wav not found"}`` — при том что
     сам wav лежал рядом в архиве.
  5. Шаг Step Summary в workflow рендерит цифры и стоит под ``if: always()``
     (на FAIL шаги без ``if:`` скипаются — именно поэтому отчёта не было
     ровно тогда, когда он нужен).

Run:
  python3 -m pytest tests/unit/e2e_scripts/test_e2e_quality_summary.py -v --no-cov
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
E2E_SCRIPT = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "L-E2E Voice Test.yml"

pytestmark = pytest.mark.skipif(shutil.which("bash") is None, reason="bash недоступен")

SCRIPT_LINES = E2E_SCRIPT.read_text(encoding="utf-8").split("\n")


def extract_function(name: str) -> str:
    """Вырезает bash-функцию, пропуская python-heredoc'и.

    Наивный поиск первой ``}`` в первой колонке не годится: внутри функций
    живут heredoc'и с python, где dict-литерал закрывается ``}`` тоже в первой
    колонке, и кусок функции теряется.
    """
    start = next(
        (i for i, l in enumerate(SCRIPT_LINES) if l.startswith(name + "() {")), None
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


def run_bash(script: str) -> tuple[int, str, str]:
    """bash со скриптом через stdin.

    Кормим БАЙТАМИ: в текстовом режиме Python на Windows переводит "\\n" в
    CRLF прямо в трубе, и bash спотыкается о ``$'\\r'``.
    """
    proc = subprocess.run(
        ["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=60
    )
    return (
        proc.returncode,
        proc.stdout.decode("utf-8", "replace"),
        proc.stderr.decode("utf-8", "replace"),
    )


STUBS = "\n".join(
    [
        "set -u",
        'OUT_DIR="$(mktemp -d)"',
        'ensure_outdir() { mkdir -p "$OUT_DIR"; }',
        'log() { echo ">>> $*"; }',
        "",
    ]
)


# --- emit_step + write_summary ---------------------------------------------


def run_summary(step_lines: list[str], extra: list[str] | None = None) -> dict:
    script = "\n".join(
        [
            STUBS,
            'PASS=0',
            'E2E_FAIL_KIND=feature',
            'E2E_TTS_PROVIDER_RESOLVED=silero',
            'RUN_ID=test1',
            extract_function("emit_step"),
            extract_function("write_summary"),
            *step_lines,
            *(extra or []),
            "write_summary",
            'echo "---JSON---"',
            'cat "$OUT_DIR/summary.json"',
            "",
        ]
    )
    rc, out, err = run_bash(script)
    assert "syntax error" not in err, err
    assert "---JSON---" in out, (out, err)
    return json.loads(out.split("---JSON---", 1)[1])


class TestStepTally:
    """Счётчик шагов: доказательство, а не новая шкала вердикта."""

    def test_counts_ok_fail_skip(self):
        data = run_summary(
            [
                'emit_step "s1 OK"',
                'emit_step "s2 FAIL no_accept"',
                'emit_step "s3 OK"',
                'emit_step "s4 SKIP wake-gate"',
            ]
        )
        assert data["steps"] == {
            "total": 4,
            "ok": 2,
            "fail": 1,
            "skip": 1,
            "failed_labels": ["s2"],
        }

    def test_duplicate_label_collapses_to_last(self):
        """run_step печатает свой FAIL, scenario-цикл — итог того же шага.

        Без схлопывания по label один упавший шаг считался бы дважды.
        """
        data = run_summary(
            [
                'emit_step "s1 FAIL no_accept"',
                'emit_step "s1 OK"',
            ]
        )
        assert data["steps"]["total"] == 1
        assert data["steps"]["ok"] == 1
        assert data["steps"]["fail"] == 0

    def test_steps_jsonl_is_valid_jsonl(self):
        script = "\n".join(
            [
                STUBS,
                extract_function("emit_step"),
                'emit_step "dj02_stop_music FAIL no_accept"',
                'echo "---JSONL---"',
                'cat "$OUT_DIR/steps.jsonl"',
                "",
            ]
        )
        rc, out, err = run_bash(script)
        assert rc == 0, err
        line = out.split("---JSONL---", 1)[1].strip()
        rec = json.loads(line)
        assert rec["label"] == "dj02_stop_music"
        assert rec["status"] == "FAIL"
        assert rec["detail"] == "no_accept"

    def test_stdout_marker_contract_preserved(self):
        """ADR-0015: пост-валидатор читает stdout-маркер, его ломать нельзя."""
        rc, out, err = run_bash(
            "\n".join([STUBS, extract_function("emit_step"), 'emit_step "s1 OK"', ""])
        )
        assert rc == 0, err
        assert "E2E_STEP s1 OK" in out

    def test_all_call_sites_go_through_emit_step(self):
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        # Единственный прямой echo — внутри самой emit_step.
        assert text.count('echo "E2E_STEP ') == 1, (
            "маркер шага печатается в обход emit_step — шаг не попадёт в сводку"
        )


class TestSummaryContent:
    """summary.json собирает то, что раньше лежало по разным zip-артефактам."""

    def test_verdict_and_provider(self):
        data = run_summary(['emit_step "s1 OK"'])
        assert data["verdict"] == "FAIL"  # PASS=0 в стабах
        assert data["fail_kind"] == "feature"
        assert data["tts_provider"] == "silero"

    def test_inlines_gate1_and_metrics(self):
        data = run_summary(
            ['emit_step "s1 OK"'],
            extra=[
                'printf \'%s\' \'{"pass": false, "reason": "no calls", '
                '"missing_expected_calls": ["stop_music"], "forbidden_calls": []}\''
                ' > "$OUT_DIR/acceptance.json"',
                # Ключи — ровно те, что пишет e2e_audio_metrics.py
                # (rms_dbfs/peak_dbfs). Первая версия сводки читала rms_db и
                # показывала null при живых метриках — поймано прогоном на 249.
                'printf \'%s\' \'{"rms_dbfs": -21.5, "peak_dbfs": -3.1, '
                '"silence_ratio": 0.42, "mic_working": true}\''
                ' > "$OUT_DIR/audio_metrics.json"',
                'printf \'%s\' \'{"pass": true, "keyword_match_pct": 87.5}\''
                ' > "$OUT_DIR/baseline_diff.json"',
            ],
        )
        assert data["gate1"]["missing_expected_calls"] == ["stop_music"]
        assert data["audio"]["rms_dbfs"] == -21.5
        assert data["baseline"]["keyword_match_pct"] == 87.5

    def test_audio_keys_match_the_producer_script(self):
        """Сводка обязана читать те же имена, что пишет e2e_audio_metrics.py.

        Первая версия читала ``rms_db``, а скрипт пишет ``rms_dbfs`` — сводка
        показывала null при полностью живых метриках. Тест сверяет две стороны
        напрямую, а не по памяти.
        """
        producer = (
            REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_audio_metrics.py"
        ).read_text(encoding="utf-8")
        consumer = E2E_SCRIPT.read_text(encoding="utf-8")
        for key in ("rms_dbfs", "peak_dbfs", "silence_ratio", "mic_working"):
            assert '"%s"' % key in producer, ("не пишется скриптом", key)
            assert 'audio.get("%s")' % key in consumer, ("не читается сводкой", key)

    def test_missing_metrics_surface_as_error_not_silence(self):
        """«Не посчиталось» должно быть видно, а не выглядеть как ноль."""
        data = run_summary(
            ['emit_step "s1 OK"'],
            extra=[
                'printf \'%s\' \'{"error":"recording.wav not found"}\''
                ' > "$OUT_DIR/audio_metrics.json"',
            ],
        )
        assert data["audio"]["error"] == "recording.wav not found"
        assert data["audio"]["rms_dbfs"] is None


# --- transcript.json --------------------------------------------------------


class TestTranscriptJson:
    """Файл обязан парситься — его читает e2e_baseline_diff.py."""

    def _run(self, expected: str, recognized: str) -> dict:
        logs = (
            "[stt_node] Получена "
            "фраза: 5.85с (187264 bytes)\n"
            f"[dialogue_node] ✅ ПРИНЯТО: {recognized}\n"
        )
        script = "\n".join(
            [
                STUBS,
                'FAKE_LOG="$(mktemp)"',
                "cat > \"$FAKE_LOG\" <<'FAKELOG'",
                logs.rstrip("\n"),
                "FAKELOG",
                'ROBOT_SSH="cat $FAKE_LOG #"',
                extract_function("parse_transcript"),
                f'parse_transcript "dj02" "2026-01-01T00:00:00Z" "{expected}"',
                'echo "---JSON---"',
                'cat "$OUT_DIR/transcript.json"',
                "",
            ]
        )
        rc, out, err = run_bash(script)
        assert rc == 0, err
        assert "---JSON---" in out, (out, err)
        return json.loads(out.split("---JSON---", 1)[1])

    def test_cyrillic_expected_with_commas(self):
        """Ровно та строка, на которой ломался живой артефакт."""
        data = self._run("Робот, стоп музыку", "ok")
        assert data["expected"] == "Робот, стоп музыку"

    def test_quotes_in_recognized_are_escaped(self):
        data = self._run("x", 'скажи "привет"')
        assert '"' in data["recognized"]

    def test_duration_is_number_not_string(self):
        data = self._run("x", "ok")
        assert data["duration_s"] == 5.85

    def test_empty_expected_is_null(self):
        data = self._run("", "ok")
        assert data["expected"] is None


# --- порядок в main flow ----------------------------------------------------


class TestMainFlowOrder:
    def test_recording_is_finalized_before_metrics(self):
        """stop_recording обязан стоять ДО write_artifacts_audio.

        Иначе audio_metrics/baseline_diff читают ещё не созданный
        recording.wav и пишут {"error": ...} — что и происходило в каждом
        прогоне, включая зелёные.
        """
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        i_stop = text.index("\nstop_recording\n")
        i_audio = text.index('\nwrite_artifacts_audio "$FINAL_VOICE_TEXT"')
        i_summary = text.index("\nwrite_summary\n")
        assert i_stop < i_audio < i_summary, (i_stop, i_audio, i_summary)

    def test_trap_still_installed_as_safety_net(self):
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        assert "trap 'stop_recording' EXIT" in text

    def test_summary_written_to_outdir(self):
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        assert 'os.path.join(out_dir, "summary.json")' in text


# --- workflow ---------------------------------------------------------------


def _workflow_steps():
    yaml = pytest.importorskip("yaml")
    data = yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))
    for job in data.get("jobs", {}).values():
        if "steps" in job:
            return job["steps"]
    raise AssertionError("no steps")


class TestWorkflowSurfacesNumbers:
    def test_quality_summary_step_exists_and_always_runs(self):
        steps = _workflow_steps()
        step = next(
            (s for s in steps if s.get("name") == "E2E quality summary (Step Summary)"),
            None,
        )
        assert step is not None, "шаг со сводкой качества не найден"
        assert "always()" in str(step.get("if", "")), (
            "без if: always() сводка скипается ровно на FAIL-прогонах"
        )
        assert "GITHUB_STEP_SUMMARY" in step["run"]

    @pytest.mark.parametrize(
        "name",
        ["E2E timing metrics (response speed)", "Summary"],
    )
    def test_reporting_steps_are_not_skipped_on_fail(self, name):
        """Скип-каскад: у шага без `if:` действует неявное `if: success()`."""
        steps = _workflow_steps()
        step = next((s for s in steps if s.get("name") == name), None)
        assert step is not None, name
        assert "always()" in str(step.get("if", "")), name
