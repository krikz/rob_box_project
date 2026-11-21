# -*- coding: utf-8 -*-
"""#5002: окно шага acceptance не должно судить ходы вне шага и должно сохраняться в артефакты.

Ночной марафон 21.11, акт 7 (run 44880526738): n701_silence_check получил ложный
``forbidden tool calls invoked: ['stop_music']``. Сам ход шага stop_music не вызывал;
в окно ``docker logs --since`` без верхней границы попал чужой ход до шага. Файл окна
``.acceptance_<label>.txt`` начинается с точки, и upload-artifact (``**/*.txt``, скрытые
файлы не берёт) его не увозит.

Тест гоняет настоящий ``check_acceptance`` (как test_issue_2902). Заглушка docker
отдаёт сырые строки с docker-префиксами timestamp и режет их по ``--since``/``--until``,
как docker 28.5 на роботе: ``--since`` включительно, ``--until`` включительно только
при наносекундной точности (секунда без дроби отсекает всю эту секунду).

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_5002_acceptance_step_window.py -v -o addopts=""
"""
from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
E2E_SCRIPT = ROOT / "scripts" / "ci" / "e2e" / "e2e_voice_test.sh"
SCRIPTS_DIR = ROOT / "scripts" / "ci" / "e2e"
SCRIPT_LINES = E2E_SCRIPT.read_text(encoding="utf-8").splitlines()
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
LIB_LINES = LIB.read_text(encoding="utf-8").splitlines()

# Синтетика, не цитаты. Шкала -- timestamp docker-лога (наносекунды), как --timestamps.
BEFORE = "2026-11-21T07:27:40.100000000Z"  # ход до шага (чужой stop_music)
STEP_START = "2026-11-21T07:31:45.200000000Z"  # начало окна шага
STEP_LINE = "2026-11-21T07:31:50.300000000Z"  # ход самого шага
STEP_END = "2026-11-21T07:31:53.400000000Z"  # конец окна = момент взятия конца шага
AFTER = "2026-11-21T07:32:10.500000000Z"  # ход следующего шага

LOG_LINES = [
    f"{BEFORE} [voice] Запрос выполнения: stop_music с параметрами {{}}",
    f"{BEFORE} [voice] tier1 kind=stop_all llm_calls=0 was_playing=True",
    f"{STEP_LINE} [voice] Запрос выполнения: get_current_time с параметрами {{}}",
    f'{STEP_LINE} [voice] Запрос выполнения: play_animation с параметрами {{"name": "idle"}}',
    f"{STEP_LINE} [voice] accepted: synthetic silence check",
    f"{AFTER} [voice] Запрос выполнения: stop_music с параметрами {{}}",
]

ACC = {"must_not_call": ["stop_music"], "expected_tool_calls": ["get_current_time"]}

needs_bash = pytest.mark.skipif(shutil.which("bash") is None, reason="bash нужен, чтобы исполнить check_acceptance")


def extract_function(name: str, lines=None, src=None) -> str:
    lines = SCRIPT_LINES if lines is None else lines
    src = E2E_SCRIPT if src is None else src
    start = next((i for i, ln in enumerate(lines) if ln.startswith(name + "() {")), None)
    assert start is not None, f"функция {name} не найдена в {src}"
    i = start + 1
    heredoc = None
    while i < len(lines):
        line = lines[i]
        if heredoc is not None:
            if line.strip() == heredoc:
                heredoc = None
            i += 1
            continue
        m = re.search(r"<<-?'([A-Za-z_][A-Za-z0-9_]*)'", line)
        if m:
            heredoc = m.group(1)
        elif line == "}":
            return "\n".join(lines[start : i + 1])
        i += 1
    raise AssertionError(f"не найден конец функции {name}")


def _stub(tmp: Path) -> str:
    """Заглушка робота: ``date`` отдаёт конец шага, ``docker logs`` режет RAW по since/until."""
    (tmp / "raw.log").write_text("\n".join(LOG_LINES) + "\n", encoding="utf-8")
    fake = tmp / "fake_ssh.sh"
    fake.write_text(
        "#!/bin/bash\n"
        'cmd="$*"\n'
        'case "$cmd" in\n'
        '  *date*) printf "%s\\n" "$STEP_END" ;;\n'
        '  *"docker logs"*)\n'
        '    since=""; until=""\n'
        '    [[ "$cmd" =~ --since[[:space:]]+\\\'([^\\\']+)\\\' ]] && since="${BASH_REMATCH[1]}"\n'
        '    [[ "$cmd" =~ --until[[:space:]]+\\\'([^\\\']+)\\\' ]] && until="${BASH_REMATCH[1]}"\n'
        '    python3 "$STUB/filter_logs.py" "$STUB/raw.log" "$since" "$until"\n'
        "    ;;\n"
        "  *) : ;;\n"
        "esac\n",
        encoding="utf-8",
    )
    fake.chmod(0o755)
    (tmp / "filter_logs.py").write_text(
        "import sys\n"
        "path, since, until = sys.argv[1], sys.argv[2], sys.argv[3]\n"
        "\n"
        "def bound(value):\n"
        "    if not value:\n"
        "        return None\n"
        "    # docker: граница без дроби -- целая секунда; с дробью -- точный момент.\n"
        "    if '.' not in value:\n"
        "        return value.replace('Z', '.000000000Z')\n"
        "    return value\n"
        "\n"
        "lo, hi = bound(since), bound(until)\n"
        "whole_second_until = bool(until) and '.' not in until\n"
        "for line in open(path, encoding='utf-8'):\n"
        "    ts = line.split(' ', 1)[0]\n"
        "    if lo and ts < lo:\n"
        "        continue\n"
        "    if hi and (ts > hi or (whole_second_until and ts.startswith(hi.split('.')[0]))):\n"
        "        continue\n"
        "    sys.stdout.write(line)\n",
        encoding="utf-8",
    )
    stub = tmp.as_posix()
    return "\n".join(
        [
            "set -u",
            f'STUB="{stub}"; export STUB',
            f'STEP_END="{STEP_END}"; export STEP_END',
            f'OUT_DIR="{stub}/out"; mkdir -p "$OUT_DIR"',
            f'SCRIPT_DIR_E2E="{SCRIPTS_DIR.as_posix()}"',
            f'ROBOT_SSH="bash {fake.as_posix()}"',
            'log() { echo ">>> $*"; }',
            'ensure_outdir() { mkdir -p "$OUT_DIR"; }',
            "E2E_TTS_PROVIDER=minimax",
            "E2E_REGISTRATION_ACK_WAIT_SEC=0",
            "",
        ]
    )


def run_check_acceptance(tmp_path: Path, label: str = "n701_silence_check"):
    acc_s = json.dumps(ACC, ensure_ascii=False).replace("'", "'\\''")
    script = "\n".join(
        [
            _stub(tmp_path),
            extract_function("check_acceptance"),
            f"check_acceptance {label} '{acc_s}' '{STEP_START}'",
            'echo "RC=$?"',
            "",
        ]
    )
    proc = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=120)
    out = proc.stdout.decode("utf-8", "replace")
    err = proc.stderr.decode("utf-8", "replace")
    assert "syntax error" not in err, err
    m = re.search(r"^RC=(\d+)$", out, re.M)
    assert m, (out, err)
    return int(m.group(1)), out, err


@needs_bash
def test_foreign_stop_music_before_step_is_not_a_fail(tmp_path):
    """Чужой stop_music до шага и stop_music следующего шага не красят шаг."""
    rc, out, err = run_check_acceptance(tmp_path)
    assert rc == 0, (out, err)
    assert "forbidden tool calls invoked" not in out
    acc = json.loads((tmp_path / "out" / "acceptance.json").read_text(encoding="utf-8"))
    assert acc["pass"] is True, acc
    assert acc["forbidden_calls"] == []


@needs_bash
def test_step_window_is_saved_as_visible_artifact(tmp_path):
    """Окно шага -- сырые строки с docker-префиксами, файл без точки, рядом со steps.jsonl."""
    label = "n701_silence_check"
    rc, out, err = run_check_acceptance(tmp_path, label)
    assert rc == 0, (out, err)
    window = tmp_path / "out" / f"acceptance_window_{label}.txt"
    assert window.is_file(), "окно шага не сохранено видимым файлом: %s" % sorted(
        p.name for p in (tmp_path / "out").iterdir()
    )
    text = window.read_text(encoding="utf-8")
    assert "stop_music" not in text, text
    assert "get_current_time" in text
    assert "play_animation" in text
    assert text.splitlines()[0].startswith(STEP_LINE)


def run_fetch_fallback(tmp_path: Path, docker_rc: str):
    """docker без --until (rc != 0) -- клиентский фильтр по timestamp-префиксу."""
    (tmp_path / "raw.log").write_text("\n".join(LOG_LINES) + "\n", encoding="utf-8")
    fake = tmp_path / "fake_ssh.sh"
    fake.write_text(
        "#!/bin/bash\n"
        'cmd="$*"\n'
        'case "$cmd" in\n'
        '  *"--until "*"--timestamps"*) exit %s ;;\n'
        '  *"docker logs"*) cat "$STUB/raw.log" ;;\n'
        "  *) : ;;\n"
        "esac\n" % docker_rc,
        encoding="utf-8",
    )
    fake.chmod(0o755)
    stub = tmp_path.as_posix()
    script = "\n".join(
        [
            "set -u",
            f'STUB="{stub}"; export STUB',
            f'ROBOT_SSH="bash {fake.as_posix()}"',
            extract_function("fetch_step_window_logs", LIB_LINES, LIB),
            f"fetch_step_window_logs '{STEP_START}' '{STEP_END}'",
            "",
        ]
    )
    proc = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=60)
    return proc.returncode, proc.stdout.decode("utf-8", "replace"), proc.stderr.decode("utf-8", "replace")


@needs_bash
def test_client_filter_drops_lines_after_step_when_docker_has_no_until(tmp_path):
    """Старый docker без --until: ход после шага отсекается фильтром по префиксу."""
    rc, out, err = run_fetch_fallback(tmp_path, "1")
    assert rc == 0, (out, err)
    assert "stop_music" not in out, out
    assert "get_current_time" in out
    assert out.splitlines()[0].startswith(STEP_LINE)
