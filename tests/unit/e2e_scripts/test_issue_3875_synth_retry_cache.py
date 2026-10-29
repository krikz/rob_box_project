"""Issue #3875 -- синтез фраз харнесса: ретраи сетевых отказов, пред-синтез акта, кэш между прогонами.

Живой повод: на katana мигал DNS, `synth_minimax` падал NameResolutionError -> MINIMAX_NETWORKERROR ->
`E2E_STEP ... FAIL synth` посреди акта 1 (runs 39862350722, 39863945631). Робот тут ни при чём.

Исполняются НАСТОЯЩИЕ ``synth_command`` / ``synth_with_retry`` / ``presynth_scenario`` /
``run_step`` из ``e2e_voice_test.sh``; провайдер -- фейковая bash-функция ``synth_minimax``,
которая первые N вызовов отвечает сетевой ошибкой в формате настоящего ``synth_minimax``.
Один тест гоняет настоящий python-синтезатор по несуществующему хосту (настоящий NameResolutionError).

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_3875_synth_retry_cache.py -v --no-cov
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

from tests.unit.e2e_scripts.test_issue_3857_run_step import _run as run_step_harness

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
E2E_SCRIPT = SCRIPTS_DIR / "e2e_voice_test.sh"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
SCRIPT_LINES = E2E_SCRIPT.read_text(encoding="utf-8").replace("\r\n", "\n").split("\n")

needs_bash = pytest.mark.skipif(
    shutil.which("bash") is None or shutil.which("python3") is None, reason="нужны bash и python3"
)

NET_ERR = (
    '{"provider": "minimax", "voice": "Russian_ReliableMan", "result": "fail", "error_class": "NetworkError", '
    '"detail": "ConnectionError: HTTPSConnectionPool(host=\'api.minimax.io\', port=443): Max retries exceeded '
    "with url: /v1/t2a_v2 (Caused by NameResolutionError(\\\"<urllib3.connection.HTTPSConnection object>: "
    'Failed to resolve api.minimax.io\\\"))", "latency_ms": 12}'
)
AUTH_ERR = (
    '{"provider": "minimax", "voice": "Russian_ReliableMan", "result": "fail", "error_class": "AuthError", '
    '"detail": "minimax API error 1004: auth failed", "latency_ms": 5}'
)


def extract_function(name):
    """Тело функции харнесса до закрывающей `}` в 1-й колонке (heredoc'и с `}` пропускаются)."""
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


SYNTH_FUNCS = [
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
    "presynth_scenario",
]
HARNESS_FUNCS = ["ensure_outdir", "emit_step", "mark_fail_kind"]


def synth_definitions():
    """Настоящие функции синтеза + фейковый провайдер (счётчик вызовов в $CALLS)."""
    return "\n".join(extract_function(n) for n in SYNTH_FUNCS) + "\n" + FAKE_PROVIDER


FAKE_PROVIDER = r"""
resolve_tts_provider() { return 0; }
synth_minimax() {  # $1=text $2=voice $3=out
    local n=$(( $(cat "$CALLS" 2>/dev/null || echo 0) + 1 ))
    echo "$n" > "$CALLS"
    if [ "$n" -le "${FAIL_FIRST:-0}" ]; then
        echo "$FAIL_JSON" >&2
        echo "${FAIL_EXIT:-MINIMAX_NETWORKERROR}" >&2
        return 1
    fi
    printf 'RIFF%060d' 0 > "$3"
    echo "MINIMAX_SYNTH_OK voice=$2"
    echo '{"provider": "minimax", "result": "ok"}' >&2
}
"""


def run_bash(tmp_path, body, fail_first=0, fail_json=NET_ERR, env_extra=""):
    out = tmp_path / "out"
    out.mkdir(exist_ok=True)
    cache = tmp_path / "cache"
    script = "\n".join(
        [
            "set -u",
            'log() { echo ">>> $*"; }',
            'OUT_DIR="%s"' % out.as_posix(),
            'CALLS="%s/calls"' % tmp_path.as_posix(),
            "FAIL_FIRST=%d" % fail_first,
            "FAIL_JSON='%s'" % fail_json.replace("'", "'\\''"),
            'E2E_SYNTH_RETRY_DELAYS="0 0 0"',
            'E2E_SYNTH_CACHE=1; E2E_SYNTH_CACHE_DIR="%s"' % cache.as_posix(),
            "E2E_TTS_PROVIDER=minimax; E2E_TTS_PROVIDER_RESOLVED=minimax",
            "MINIMAX_TTS_MODEL=speech-02-hd; YANDEX_SPEED=1.0",
            env_extra,
            LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
            "\n".join(extract_function(n) for n in HARNESS_FUNCS),
            synth_definitions(),
            body,
            'echo "CALLS=$(cat "$CALLS" 2>/dev/null || echo 0)"',
        ]
    )
    p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=120)
    return p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace")


def write_scenario(tmp_path, steps):
    path = tmp_path / "scenario.json"
    path.write_text(
        json.dumps({"steps": [dict(label=lbl, text=t, voice=v) for lbl, t, v in steps]}, ensure_ascii=False),
        encoding="utf-8",
    )
    return path


# --------------------------------------------------------------------------------------
# 1. Ретраи сетевых отказов
# --------------------------------------------------------------------------------------
@needs_bash
class TestRetry:
    def test_two_network_errors_then_third_succeeds(self, tmp_path):
        out, err = run_bash(
            tmp_path,
            'synth_command "Робот, привет" anton "$OUT_DIR/cmd_a.wav" > "$OUT_DIR/s.log" 2>&1; echo "RC=$?";'
            'cat "$OUT_DIR/s.log"; test -s "$OUT_DIR/cmd_a.wav" && echo WAV_OK',
            fail_first=2,
        )
        assert "RC=0" in out, (out, err)
        assert "WAV_OK" in out
        assert "CALLS=3" in out
        assert "synth retry 1/3 (NameResolutionError)" in out
        assert "synth retry 2/3 (NameResolutionError)" in out
        assert "synth retry 3/3" not in out

    def test_network_errors_exhaust_after_three_retries(self, tmp_path):
        out, err = run_bash(
            tmp_path,
            'synth_command "Робот, привет" anton "$OUT_DIR/cmd_a.wav" > "$OUT_DIR/s.log" 2>&1; echo "RC=$?";'
            'cat "$OUT_DIR/s.log"; echo "LAST=$(tail -1 "$OUT_DIR/s.log" | cut -c1-30)"',
            fail_first=99,
        )
        assert "RC=1" in out, (out, err)
        assert "CALLS=4" in out  # первая попытка + 3 повтора (2/5/10 с)
        assert "synth retry 3/3 (NameResolutionError)" in out
        assert "synth retry исчерпан" in out
        # tail -1 лога -- последний отказ провайдера (его печатает run_step в строке FAIL)
        assert "LAST=MINIMAX_NETWORKERROR" in out

    def test_non_network_error_is_not_retried(self, tmp_path):
        out, err = run_bash(
            tmp_path,
            'synth_command "Робот, привет" anton "$OUT_DIR/cmd_a.wav" > "$OUT_DIR/s.log" 2>&1; echo "RC=$?";'
            'cat "$OUT_DIR/s.log"',
            fail_first=99,
            fail_json=AUTH_ERR,
            env_extra="FAIL_EXIT=MINIMAX_AUTHERROR",
        )
        assert "RC=1" in out, (out, err)
        assert "CALLS=1" in out  # AuthError повторять незачем: ответ тот же
        assert "synth retry" not in out

    def test_default_backoff_is_2_5_10(self):
        text = "\n".join(SCRIPT_LINES)
        assert 'E2E_SYNTH_RETRY_DELAYS="${E2E_SYNTH_RETRY_DELAYS:-2 5 10}"' in text

    def test_real_python_synth_classifies_dns_failure_as_network_error(self, tmp_path):
        """Настоящий synth_minimax -> настоящий NameResolutionError (хост .invalid) -> ретрай."""
        pytest.importorskip("requests")
        out, err = run_bash(
            tmp_path,
            "unset -f synth_minimax; source /dev/stdin <<'EOF_FN'\n%s\nEOF_FN\n" % extract_function("synth_minimax")
            + 'MINIMAX_API_KEY=x MINIMAX_TTS_BASE_URL=http://tts-host.invalid '
            'synth_command "Робот, привет" anton "$OUT_DIR/cmd_a.wav" > "$OUT_DIR/s.log" 2>&1; echo "RC=$?";'
            'cat "$OUT_DIR/s.log"',
        )
        assert "RC=1" in out, (out, err)
        assert re.search(r"synth retry 1/3 \((NameResolutionError|ConnectionError)\)", out), out
        assert "synth retry 3/3" in out


# --------------------------------------------------------------------------------------
# 2. Кэш между прогонами
# --------------------------------------------------------------------------------------
@needs_bash
class TestCache:
    def test_second_call_is_cache_hit_without_provider(self, tmp_path):
        out, err = run_bash(
            tmp_path,
            'synth_command "Робот, играй" anton "$OUT_DIR/a.wav" > "$OUT_DIR/s1.log" 2>&1; echo "RC1=$?";'
            'synth_command "Робот, играй" anton "$OUT_DIR/b.wav" > "$OUT_DIR/s2.log" 2>&1; echo "RC2=$?";'
            'cat "$OUT_DIR/s2.log"; cmp "$OUT_DIR/a.wav" "$OUT_DIR/b.wav" && echo SAME',
        )
        assert "RC1=0" in out and "RC2=0" in out, (out, err)
        assert "CALLS=1" in out  # вторая озвучка -- из кэша, провайдер не вызывался
        assert "synth cache hit: minimax/English_Jovialman" in out
        assert "SAME" in out

    def test_cache_survives_run_dir_cleanup(self, tmp_path):
        """Кэш лежит вне OUT_DIR: cleanup прогона (rm -rf /tmp/e2e_v2_*) его не трогает."""
        out, err = run_bash(
            tmp_path,
            'synth_command "Робот, играй" anton "$OUT_DIR/a.wav" > /dev/null 2>&1;'
            'rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR";'
            'synth_command "Робот, играй" anton "$OUT_DIR/a.wav" 2>&1 | head -1; test -s "$OUT_DIR/a.wav" && echo WAV_OK',
        )
        assert "CALLS=1" in out, (out, err)
        assert "synth cache hit" in out and "WAV_OK" in out

    @pytest.mark.parametrize(
        "second",
        [
            'synth_command "Робот, играй" ermil "$OUT_DIR/b.wav"',  # другой голос
            'synth_command "Робот, играй!" anton "$OUT_DIR/b.wav"',  # другой текст
            'YANDEX_SPEED=1.3 synth_command "Робот, играй" anton "$OUT_DIR/b.wav"',  # другая скорость
            'MINIMAX_TTS_MODEL=speech-02-turbo synth_command "Робот, играй" anton "$OUT_DIR/b.wav"',  # другая модель
        ],
    )
    def test_key_covers_voice_text_and_params(self, tmp_path, second):
        out, err = run_bash(
            tmp_path,
            'synth_command "Робот, играй" anton "$OUT_DIR/a.wav" > /dev/null 2>&1;' + second + " > /dev/null 2>&1",
        )
        assert "CALLS=2" in out, (out, err)

    def test_failed_synth_is_not_cached(self, tmp_path):
        out, err = run_bash(
            tmp_path,
            'synth_command "Робот, играй" anton "$OUT_DIR/a.wav" > /dev/null 2>&1; echo "RC1=$?";'
            'synth_command "Робот, играй" anton "$OUT_DIR/a.wav" > /dev/null 2>&1; echo "RC2=$?"',
            fail_first=4,
        )
        # 1-й вызов: 4 попытки подряд сетевой отказ -> RC1=1; 2-й: провайдер ожил -> RC2=0
        assert "RC1=1" in out and "RC2=0" in out and "CALLS=5" in out, (out, err)

    def test_cache_off_still_reuses_within_run(self, tmp_path):
        """E2E_SYNTH_CACHE=0: постоянный кэш не пишется, но пред-синтез акта доезжает до шагов."""
        out, err = run_bash(
            tmp_path,
            'synth_command "Робот, играй" anton "$OUT_DIR/a.wav" > /dev/null 2>&1;'
            'synth_command "Робот, играй" anton "$OUT_DIR/b.wav" > /dev/null 2>&1;'
            'ls "$E2E_SYNTH_CACHE_DIR" 2>/dev/null | wc -l',
            env_extra="E2E_SYNTH_CACHE=0",
        )
        assert "CALLS=1" in out, (out, err)
        assert not (tmp_path / "cache").exists() or not list((tmp_path / "cache").iterdir())

    def test_cache_dir_default_is_outside_run_tmp(self):
        text = "\n".join(SCRIPT_LINES)
        assert '${HOME:-/tmp}/.cache/rob_box_e2e/synth' in text
        # cleanup прогона чистит /tmp/e2e_v2_<run_id>; кэш в домашнем каталоге ros2, а не там
        assert 'OUT_DIR="/tmp/e2e_v2_${RUN_ID}"' in text


# --------------------------------------------------------------------------------------
# 3. Пред-синтез акта
# --------------------------------------------------------------------------------------
@needs_bash
class TestPresynth:
    STEPS = [("n101", "Робот, привет", "anton"), ("n102", "Робот, как дела", "ermil"), ("n103", "Робот, пока", "anton")]

    def test_flaky_network_during_presynth_is_absorbed_by_retries(self, tmp_path):
        sc = write_scenario(tmp_path, self.STEPS)
        out, err = run_bash(
            tmp_path,
            'presynth_scenario "%s"; echo "RC=$?"; ls "$OUT_DIR" | grep -c "^cmd_.*wav$"; echo "KIND=${E2E_FAIL_KIND:-}"'
            % sc.as_posix(),
            fail_first=2,
        )
        assert "RC=0" in out, (out, err)
        assert "STEP n101: synth retry 1/3 (NameResolutionError)" in out  # в основном логе, с меткой шага
        assert "PRESYNTH: готово 3 фраз" in out
        assert "E2E_INFRA_FAIL" not in out
        assert re.search(r"^KIND=$", out, re.M), out  # пред-синтез прошёл: причина FAIL не помечена

    def test_network_down_fails_before_any_play_with_infra_fail(self, tmp_path):
        sc = write_scenario(tmp_path, self.STEPS)
        out, err = run_bash(
            tmp_path,
            'E2E_FAIL_KIND=""; presynth_scenario "%s"; echo "RC=$?"; echo "KIND=$E2E_FAIL_KIND";'
            'cat "$OUT_DIR/steps.jsonl"' % sc.as_posix(),
            fail_first=99,
        )
        assert "RC=1" in out, (out, err)
        assert "E2E_INFRA_FAIL synth" in out
        assert "KIND=synth" in out
        assert "n101 FAIL synth" in out  # E2E_STEP/steps.jsonl: отказ на первой же фразе
        assert "CALLS=4" in out  # дальше первой фразы не идём: "сразу", а не 3 x 4 попытки
        assert "n102" not in out and "n103" not in out

    def test_cache_hits_make_presynth_offline(self, tmp_path):
        """Фразы уже в кэше (прошлый прогон) -- сети нет вообще, а пред-синтез проходит."""
        sc = write_scenario(tmp_path, self.STEPS)
        out, err = run_bash(
            tmp_path,
            'presynth_scenario "%s" > /dev/null; FAIL_FIRST=99; echo 0 > "$CALLS"; rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR";'
            'presynth_scenario "%s"; echo "RC=$?"' % (sc.as_posix(), sc.as_posix()),
        )
        assert "RC=0" in out, (out, err)
        assert "PRESYNTH: готово 3 фраз (из кэша 3)" in out
        assert "CALLS=0" in out

    def test_main_flow_runs_presynth_before_recording_isolation_and_first_step(self):
        text = "\n".join(SCRIPT_LINES)
        call = text.index('presynth_scenario "$SCENARIO_FILE" || E2E_PRESYNTH_FAILED=1')
        assert call < text.index("start_recording\n\n# Advisory health probe")
        assert call < text.index('    isolate_act_start "$SCENARIO_FILE"')
        assert call < text.index('    apply_node_params "$SCENARIO_FILE"')
        assert call < text.index("    while IFS=$'\\x1f' eval \"$E2E_SCENARIO_ROW_READ\"; do\n        [ -z \"$idx\" ] && continue\n        case \"$retry_acceptance\"")
        # провал пред-синтеза: акт не играется, роботу не звонят
        assert '[ "$E2E_PRESYNTH_FAILED" != "1" ]' in text
        assert 'if [ "$E2E_PRESYNTH_FAILED" = "1" ]; then\n    # Issue #3875' in text

    def test_infra_fail_marker_is_printed_for_synth_kind(self):
        text = "\n".join(SCRIPT_LINES)
        assert 'synth)      fail_marker="E2E_INFRA_FAIL"' in text


# --------------------------------------------------------------------------------------
# 4. Шаг run_step на настоящем коде: ретраи доезжают до PLAY / отказ -- до PLAY не доходит
# --------------------------------------------------------------------------------------
@needs_bash
class TestRunStep:
    def _body(self, tmp_path, fail_first):
        funcs = "\n".join(extract_function(n) for n in SYNTH_FUNCS) + "\n" + FAKE_PROVIDER
        calls = (tmp_path / "calls").as_posix()
        return "\n".join(
            [
                funcs,
                'CALLS="%s"; FAIL_FIRST=%d' % (calls, fail_first),
                "FAIL_JSON='%s'" % NET_ERR.replace("'", "'\\''"),
                'E2E_SYNTH_RETRY_DELAYS="0 0 0"; E2E_SYNTH_CACHE=0; E2E_FAIL_KIND=""',
                "E2E_TTS_PROVIDER=minimax; E2E_TTS_PROVIDER_RESOLVED=minimax",
                'run_step "Робот привет" anton a cycle; echo "RC=$?"',
                'echo "CALLS=$(cat "%s")"' % calls,
            ]
        )

    def test_step_survives_two_dns_flakes_and_plays(self, tmp_path):
        out, err, _ = run_step_harness(tmp_path, self._body(tmp_path, 2))
        assert "RC=0" in out, (out, err)
        assert "CALLS=3" in out
        assert "PLAY_COUNT=1" in out
        assert "STEP a: synth retry 1/3 (NameResolutionError)" in out
        assert "STEP a: synth retry 2/3 (NameResolutionError)" in out

    def test_step_fails_as_synth_when_network_stays_down_and_never_plays(self, tmp_path):
        out, err, _ = run_step_harness(tmp_path, self._body(tmp_path, 99))
        assert "RC=1" in out, (out, err)
        assert "E2E_STEP a FAIL synth" in out
        assert "PLAY_COUNT=0" in out


# --------------------------------------------------------------------------------------
# 5. Весь харнесс целиком: сеть билд-машины мертва -> E2E_INFRA_FAIL synth, ни одного PLAY
# --------------------------------------------------------------------------------------
@needs_bash
def test_whole_harness_with_dead_network_fails_infra_before_first_play(tmp_path):
    pytest.importorskip("requests")
    bindir = tmp_path / "bin"
    bindir.mkdir()
    play_log = tmp_path / "play.log"
    (bindir / "paplay").write_text('#!/bin/bash\necho PAPLAY "$@" >> "%s"\n' % play_log.as_posix(), encoding="utf-8")
    for name in ("ffmpeg", "ffprobe", "pactl"):
        (bindir / name).write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
    (tmp_path / "robot.sh").write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
    for p in bindir.iterdir():
        p.chmod(0o755)
    scenario = write_scenario(tmp_path, [("n101", "Робот, привет", "anton"), ("n102", "Робот, как дела", "ermil")])
    run_id = "t3875_%d" % abs(hash(str(tmp_path)) % 10**8)
    script = "\n".join(
        [
            'export PATH="%s:$PATH" HOME="%s"' % (bindir.as_posix(), (tmp_path / "home").as_posix()),
            'export ROBOT_SSH_OVERRIDE="bash %s/robot.sh" GITHUB_RUN_ID=%s' % (tmp_path.as_posix(), run_id),
            "export MINIMAX_API_KEY=x MINIMAX_TTS_BASE_URL=http://tts-host.invalid E2E_SYNTH_RETRY_DELAYS='0 0 0'",
            'export SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
            'bash "%s" --scenario "%s" --tts-provider minimax --acceptance-skip' % (E2E_SCRIPT.as_posix(), scenario.as_posix()),
            'echo "HARNESS_RC=$?"',
        ]
    )
    try:
        p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=170)
        out = p.stdout.decode("utf-8", "replace")
        assert "E2E_INFRA_FAIL synth" in out, out[-3000:]  # сразу, из пред-синтеза
        assert "E2E_VERDICT FAIL" in out
        assert "synth retry 3/3" in out
        assert "E2E_STEP n101 FAIL synth" in out
        assert "STEP n102" not in out  # дальше первой фразы не идём
        assert "PLAY attempt" not in out
        assert not play_log.exists(), "paplay не должен вызываться до успешного пред-синтеза"
    finally:
        shutil.rmtree("/tmp/e2e_v2_%s" % run_id, ignore_errors=True)
