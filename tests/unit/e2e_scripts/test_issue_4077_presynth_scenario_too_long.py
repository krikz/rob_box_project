# -*- coding: utf-8 -*-
"""#4077 П2 остаток: замер eq в ``presynth_scenario`` → ``E2E_INFRA_FAIL`` до PLAY.

Порог — тот же, что у ``run_step`` и ``phrase_budget.VAD_BUDGET_S``: ``E2E_VAD_MAX_DURATION`` (15.0) минус
``E2E_VAD_OVERHEAD`` (2.5) = 12.5 с. Число не выдумываем: его держит ``test_e2e_scenario_playable.sh``.
Сбой eq WAV или пустой ffprobe — тоже ``E2E_INFRA_FAIL`` (eq / eq_duration), не silent skip до PLAY.

Run:
    /home/ros2/rob_box_project/.venv-core/bin/pytest -v -o addopts= \\
        tests/unit/e2e_scripts/test_issue_4077_presynth_scenario_too_long.py
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / "scripts" / "ci" / "e2e"
E2E_SCRIPT = SCRIPTS_DIR / "e2e_voice_test.sh"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
SCRIPT_LINES = E2E_SCRIPT.read_text(encoding="utf-8").replace("\r\n", "\n").split("\n")

needs_bash = pytest.mark.skipif(
    shutil.which("bash") is None or shutil.which("python3") is None,
    reason="нужны bash и python3",
)


def extract_function(name: str) -> str:
    start = next(i for i, ln in enumerate(SCRIPT_LINES) if ln.startswith(name + "() {"))
    heredoc = None
    for i in range(start + 1, len(SCRIPT_LINES)):
        line = SCRIPT_LINES[i]
        if heredoc is not None:
            if line.strip() == heredoc:
                heredoc = None
            continue
        found = re.search(r"<<-?'([A-Za-z_][A-Za-z0-9_]*)'", line)
        if found:
            heredoc = found.group(1)
        elif line == "}":
            return "\n".join(SCRIPT_LINES[start : i + 1])
    raise AssertionError(name)


FUNCS = [
    "_synth_sha256",
    "synth_dispatch",
    "synth_net_error_class",
    "synth_with_retry",
    "synth_cache_key",
    "synth_cache_dir",
    "synth_wav_ok",
    "synth_cache_prune",
    "synth_command",
    "synth_surface_log",
    "eq_command_wav",
    "check_eq_vad_budget",
    "presynth_scenario",
    "ensure_outdir",
    "emit_step",
    "mark_fail_kind",
]

FAKE_PROVIDER = r"""
resolve_tts_provider() { return 0; }
synth_minimax() {  # $1=text $2=voice $3=out
    printf 'RIFF%060d' 0 > "$3"
    echo "MINIMAX_SYNTH_OK voice=$2"
    echo '{"provider": "minimax", "result": "ok"}' >&2
}
eq_command_wav() {  # без ffmpeg: просто копируем сырой wav → eq
    cp -f "$1" "$2" 2>/dev/null || printf 'RIFF' > "$2"
}
"""


def write_scenario(tmp_path: Path, steps):
    path = tmp_path / "scenario.json"
    path.write_text(
        json.dumps(
            {"steps": [dict(label=lbl, text=t, voice=v) for lbl, t, v in steps]},
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return path


def run_presynth(
    tmp_path: Path,
    body: str,
    ffprobe_dur: str | None = None,
    *,
    eq_fail: bool = False,
    ffprobe_empty: bool = False,
    extra_bash: str = "",
):
    """ffprobe_dur — PATH-обёртка ffprobe с этой длительностью.

    eq_fail — подмена eq_command_wav на отказ; ffprobe_empty — ffprobe без числа.
    """
    out = tmp_path / "out"
    out.mkdir(exist_ok=True)
    cache = tmp_path / "cache"
    bindir = tmp_path / "bin"
    bindir.mkdir(exist_ok=True)
    if ffprobe_empty:
        (bindir / "ffprobe").write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
        (bindir / "ffprobe").chmod(0o755)
    elif ffprobe_dur is not None:
        (bindir / "ffprobe").write_text(
            "#!/bin/bash\necho '%s'\n" % ffprobe_dur,
            encoding="utf-8",
        )
        (bindir / "ffprobe").chmod(0o755)
    overrides = []
    if eq_fail:
        overrides.append('eq_command_wav() { rm -f "$2"; echo "FAKE_EQ_FAIL" >&2; return 1; }')
    script = "\n".join(
        [
            "set -u",
            'export PATH="%s:$PATH"' % bindir.as_posix(),
            'log() { echo ">>> $*"; }',
            'OUT_DIR="%s"' % out.as_posix(),
            'E2E_SYNTH_RETRY_DELAYS="0 0 0"',
            'E2E_SYNTH_CACHE=1; E2E_SYNTH_CACHE_DIR="%s"' % cache.as_posix(),
            "E2E_TTS_PROVIDER=minimax; E2E_TTS_PROVIDER_RESOLVED=minimax",
            "MINIMAX_TTS_MODEL=speech-02-hd; YANDEX_SPEED=1.0",
            "E2E_VAD_MAX_DURATION=15.0; E2E_VAD_OVERHEAD=2.5",
            'E2E_FAIL_KIND=""',
            LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
            "\n".join(extract_function(n) for n in FUNCS),
            FAKE_PROVIDER,
            *overrides,
            extra_bash,
            body,
        ]
    )
    p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=60)
    return (
        p.stdout.decode("utf-8", "replace"),
        p.stderr.decode("utf-8", "replace"),
        p.returncode,
    )


STEPS = [
    ("n101", "Робот, привет", "anton"),
    ("n102", "Робот, как дела", "ermil"),
    ("n103", "Робот, пока", "anton"),
]


@needs_bash
def test_check_eq_vad_budget_uses_shifu_approved_defaults():
    """Порог 12.5 с — не выдуман: дефолты харнесса = phrase_budget.VAD_BUDGET_S."""
    import sys

    sys.path.insert(0, str(REPO_ROOT / "scripts" / "e2e"))
    import phrase_budget as pb  # noqa: E402

    text = "\n".join(SCRIPT_LINES)
    assert 'E2E_VAD_MAX_DURATION="${E2E_VAD_MAX_DURATION:-15.0}"' in text
    assert 'E2E_VAD_OVERHEAD="${E2E_VAD_OVERHEAD:-2.5}"' in text
    assert pb.VAD_BUDGET_S == pytest.approx(12.5)
    body = extract_function("check_eq_vad_budget")
    assert "E2E_VAD_MAX_DURATION" in body and "E2E_VAD_OVERHEAD" in body
    # порог считается как max-overhead, не литерал VAD_BUDGET_S
    assert 'printf "%.2f", m-o' in body
    assert "LC_ALL=C" in body  # иначе ru_RU даёт 12,50 и ломает d>b


@needs_bash
def test_presynth_prints_eq_and_passes_under_budget(tmp_path):
    sc = write_scenario(tmp_path, STEPS)
    out, err, _ = run_presynth(
        tmp_path,
        'presynth_scenario "%s"; echo "RC=$?"; echo "KIND=${E2E_FAIL_KIND:-}"' % sc.as_posix(),
        ffprobe_dur="8.25",
    )
    assert "RC=0" in out, (out, err)
    assert re.search(r"PRESYNTH n101: eq=8\.25s budget=12\.50s", out), out
    assert "PRESYNTH: готово 3 фраз" in out
    assert "E2E_INFRA_FAIL" not in out
    assert re.search(r"^KIND=$", out, re.M), out


@needs_bash
def test_presynth_fails_scenario_too_long_before_later_phrases(tmp_path):
    """eq > 12.5 с → E2E_INFRA_FAIL scenario_too_long на первой фразе, n102/n103 не трогаем."""
    sc = write_scenario(tmp_path, STEPS)
    out, err, _ = run_presynth(
        tmp_path,
        'presynth_scenario "%s"; echo "RC=$?"; echo "KIND=$E2E_FAIL_KIND";'
        'cat "$OUT_DIR/steps.jsonl"' % sc.as_posix(),
        ffprobe_dur="13.78",
    )
    assert "RC=1" in out, (out, err)
    assert "E2E_INFRA_FAIL scenario_too_long" in out
    assert "KIND=infra" in out
    assert "PRESYNTH n101: eq=13.78s budget=12.50s" in out
    assert "n101 FAIL scenario_too_long (13.78s > 12.50s)" in out
    assert "акт не начат — реплика не влезает в окно VAD" in out
    # дальше первой фразы не идём
    assert "PRESYNTH n102:" not in out
    assert "PRESYNTH n103:" not in out
    assert "PRESYNTH: готово" not in out


@needs_bash
def test_presynth_boundary_at_budget_is_ok_tight_warning(tmp_path):
    """ровно 12.50 — не > budget; 12.0 < d <= 12.5 — предупреждение запаса < 1s."""
    sc = write_scenario(tmp_path, [("nX", "Робот, тест", "anton")])
    out, err, _ = run_presynth(
        tmp_path,
        'presynth_scenario "%s"; echo "RC=$?"' % sc.as_posix(),
        ffprobe_dur="12.50",
    )
    assert "RC=0" in out, (out, err)
    assert "scenario_too_long" not in out
    assert "запас < 1s" in out


@needs_bash
def test_presynth_just_over_budget_is_infra_fail(tmp_path):
    sc = write_scenario(tmp_path, [("nX", "Робот, тест", "anton")])
    out, err, _ = run_presynth(
        tmp_path,
        'presynth_scenario "%s"; echo "RC=$?"' % sc.as_posix(),
        ffprobe_dur="12.51",
    )
    assert "RC=1" in out, (out, err)
    assert "E2E_INFRA_FAIL scenario_too_long" in out


@needs_bash
def test_presynth_eq_prep_fail_is_infra_before_play(tmp_path):
    """eq_command_wav упал / eq WAV пуст → E2E_INFRA_FAIL eq, акт не начат, n102 не трогаем."""
    sc = write_scenario(tmp_path, STEPS)
    out, err, _ = run_presynth(
        tmp_path,
        'presynth_scenario "%s"; echo "RC=$?"; echo "KIND=$E2E_FAIL_KIND";'
        'cat "$OUT_DIR/steps.jsonl"' % sc.as_posix(),
        ffprobe_dur="8.0",
        eq_fail=True,
    )
    assert "RC=1" in out, (out, err)
    assert "E2E_INFRA_FAIL eq" in out
    assert "KIND=infra" in out
    assert "n101 FAIL eq" in out
    assert "сбой подготовки WAV до PLAY" in out
    assert "PRESYNTH n102:" not in out
    assert "PRESYNTH: готово" not in out


@needs_bash
def test_presynth_unmeasured_duration_is_infra_before_play(tmp_path):
    """Пустой/битый ffprobe в PRESYNTH → E2E_INFRA_FAIL eq_duration, не silent skip."""
    sc = write_scenario(tmp_path, STEPS)
    out, err, _ = run_presynth(
        tmp_path,
        'presynth_scenario "%s"; echo "RC=$?"; echo "KIND=$E2E_FAIL_KIND";'
        'cat "$OUT_DIR/steps.jsonl"' % sc.as_posix(),
        ffprobe_empty=True,
    )
    assert "RC=1" in out, (out, err)
    assert "E2E_INFRA_FAIL eq_duration" in out
    assert "KIND=infra" in out
    assert "n101 FAIL eq_duration" in out
    assert "длительность eq не измерена" in out
    assert "PRESYNTH n102:" not in out
    assert "PRESYNTH: готово" not in out
    # не путать с scenario_too_long и не advisory-skip
    assert "scenario_too_long" not in out
    assert "проверка окна VAD пропущена" not in out


@needs_bash
def test_final_fail_marker_maps_infra_to_e2e_infra_fail():
    text = "\n".join(SCRIPT_LINES)
    assert 'infra)      fail_marker="E2E_INFRA_FAIL"' in text
    assert "E2E_INFRA_FAIL scenario_too_long" in text
    # presynth зовёт helper, а не дублирует порог
    body = extract_function("presynth_scenario")
    assert "check_eq_vad_budget" in body
    assert "eq_command_wav" in body
    assert "E2E_INFRA_FAIL eq" in body
    assert "E2E_INFRA_FAIL eq_duration" in body
    assert '! -s "$eq_wav"' in body
    chk = extract_function("check_eq_vad_budget")
    assert "E2E_EQ_FAIL_REASON=eq_duration" in chk
    assert '[ "$prefix" = "PRESYNTH" ]' in chk


@needs_bash
def test_run_step_vad_check_delegates_to_same_helper():
    """run_step и presynth — один check_eq_vad_budget, порог не разъедется."""
    # тело run_step большое; ищем по фрагменту вокруг 1b
    text = "\n".join(SCRIPT_LINES)
    idx = text.index("# 1b. Реплика влезает в окно VAD")
    chunk = text[idx : idx + 500]
    assert "check_eq_vad_budget" in chunk
    assert "mark_fail_kind infra" in chunk
    assert "FAIL scenario_too_long" in chunk
