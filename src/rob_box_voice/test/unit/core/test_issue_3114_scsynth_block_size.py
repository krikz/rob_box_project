"""Guard для block size scsynth (issue #3114).

``-z`` у scsynth — размер БЛОКА обработки (control rate = SR / block), а не
аппаратного буфера. При ``-H jack`` аппаратный буфер задаёт jackd
(``jack_get_buffer_size``), ``-Z`` JACK-драйвер не читает. Раньше скрипт
жёстко ставил ``-z 1024`` с комментарием «= period_size dmix (JACK требует
совпадения)» — это неверно, и при 16 кГц давало control rate 15.6 Гц.

Сейчас размер задаётся env ``SCSYNTH_BLOCK_SIZE``. Дефолт СОЗНАТЕЛЬНО
оставлен 1024 (поведение до #3114): менять его без замера CPU на Pi
запрещено issue. После замера (scripts/music/scsynth_block_bench.sh)
«переключение одним флагом» = поменять ``EXPECTED_DEFAULT`` ниже и дефолт
в start_supercollider.sh + docker-compose.yaml.
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path


def _repo_root(start: Path) -> Path:
    """Корень репо в dev и в CI (test_ws), как в test_scsynth_creates_client_group."""
    override = os.environ.get("ROB_BOX_REPO_ROOT")
    if override and (Path(override) / "docker").is_dir():
        return Path(override).resolve()
    for parent in [start, *start.parents]:
        if (parent / "src").is_dir() and (parent / "docker").is_dir():
            return parent
    raise RuntimeError(f"repo root not found for {start!s}; set ROB_BOX_REPO_ROOT")


REPO = _repo_root(Path(__file__).resolve())
SCRIPT = REPO / "docker/vision/scripts/supercollider/start_supercollider.sh"
COMPOSE = REPO / "docker/vision/docker-compose.yaml"

ALLOWED = {64, 128, 256, 512, 1024}
# Поменять ТОЛЬКО по результатам замера на Pi (issue #3114 / #3115).
EXPECTED_DEFAULT = 1024
JACKD_PERIOD = 1024


def _text() -> str:
    return SCRIPT.read_text(encoding="utf-8")


def _scsynth_invocation(script: str) -> str:
    out: list[str] = []
    started = False
    for line in script.splitlines():
        s = line.strip()
        if not started:
            if s.startswith("scsynth"):
                started = True
                out.append(line)
            continue
        out.append(line)
        if s.endswith("&"):
            break
    return "\n".join(out)


def _block_section(script: str) -> str:
    start = script.index("# ── Block size")
    end = script.index("echo \"[SuperCollider] JACK running.", start)
    return script[start:end]


def test_scsynth_z_comes_from_env_not_literal() -> None:
    inv = _scsynth_invocation(_text())
    assert re.search(r'-z "\$SCSYNTH_BLOCK_SIZE"', inv), inv
    assert not re.search(r"-z\s+\d", inv), f"литерал -z вернулся:\n{inv}"


def test_scsynth_does_not_pass_Z_under_jack() -> None:
    # -Z JACK-драйвером игнорируется; его появление = та же путаница снова.
    inv = _scsynth_invocation(_text())
    assert "-H jack" in inv
    assert not re.search(r"(^|\s)-Z\s", inv), inv


def test_default_block_size_is_explicit_and_unchanged() -> None:
    text = _text()
    m = re.search(r'^SCSYNTH_BLOCK_SIZE="\$\{SCSYNTH_BLOCK_SIZE:-(\d+)\}"$', text, re.M)
    assert m, "дефолт должен быть явным: SCSYNTH_BLOCK_SIZE=\"${SCSYNTH_BLOCK_SIZE:-N}\""
    assert int(m.group(1)) == EXPECTED_DEFAULT
    m2 = re.search(r"^SCSYNTH_BLOCK_SIZE_DEFAULT=(\d+)$", text, re.M)
    assert m2 and int(m2.group(1)) == EXPECTED_DEFAULT


def test_allowed_set_in_script_matches_and_divides_jack_period() -> None:
    section = _block_section(_text())
    m = re.search(r"^\s*([0-9|]+)\)\s*;;", section, re.M)
    assert m, section
    values = {int(v) for v in m.group(1).split("|")}
    assert values == ALLOWED
    for v in values:
        assert JACKD_PERIOD % v == 0, f"{v} не делит период jackd {JACKD_PERIOD}"
    assert re.search(rf"-p {JACKD_PERIOD}\b", _text()), "период jackd поменялся — перепроверь ALLOWED"


def _run_section(env_value: str | None) -> subprocess.CompletedProcess:
    section = _block_section(_text())
    prog = "set -euo pipefail\n" + section + 'echo "RESULT=$SCSYNTH_BLOCK_SIZE"\n'
    env = {"PATH": "/usr/bin:/bin"}
    if env_value is not None:
        env["SCSYNTH_BLOCK_SIZE"] = env_value
    return subprocess.run(["bash", "-c", prog], capture_output=True, text=True, env=env)


def test_block_section_runtime_behaviour() -> None:
    r = _run_section(None)
    assert r.returncode == 0 and f"RESULT={EXPECTED_DEFAULT}" in r.stdout, r
    for ok in ("64", "256"):
        r = _run_section(ok)
        assert f"RESULT={ok}" in r.stdout, r
    for bad in ("100", "2048", "abc", ""):
        r = _run_section(bad)
        assert r.returncode == 0, r
        if bad == "":
            # пустой env = не задан → дефолт без ошибки (${VAR:-default})
            assert f"RESULT={EXPECTED_DEFAULT}" in r.stdout, r
            continue
        assert f"RESULT={EXPECTED_DEFAULT}" in r.stdout, r
        assert "ERROR" in r.stdout, f"невалидное значение {bad!r} проглочено молча: {r}"


def test_comment_no_longer_claims_z_equals_dmix_period() -> None:
    text = _text()
    assert "JACK требует совпадения" not in text
    assert not re.search(r"#\s*-z\s+1024\s+Размер буфера", text)
    for line in text.splitlines():
        if line.lstrip().startswith("#") and "-z" in line:
            assert "period_size" not in line, line
    # и -D 0 — это «не грузить synthdefs», а не realtime
    assert "Отключить realtime scheduling" not in text


def test_compose_passes_env_with_same_default() -> None:
    text = COMPOSE.read_text(encoding="utf-8")
    # блок сервиса supercollider: от "  supercollider:" до следующего сервиса
    m = re.search(r"^  supercollider:\n(.*?)(?=^  [A-Za-z0-9_-]+:\n)", text, re.M | re.S)
    assert m, "сервис supercollider не найден в docker-compose.yaml"
    line = f"- SCSYNTH_BLOCK_SIZE=${{SCSYNTH_BLOCK_SIZE:-{EXPECTED_DEFAULT}}}"
    assert line in m.group(1), f"нет `{line}` в environment сервиса supercollider"


def test_script_bash_syntax() -> None:
    r = subprocess.run(["bash", "-n", str(SCRIPT)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
