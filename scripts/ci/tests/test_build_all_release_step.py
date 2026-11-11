"""Guard: шаг подписанного релиза в "L: Build All Services" не может сломать сборку и деплой (ADR-0168 §5 п. 2, #4375).

Решение Шифу 09.11: подключить сборку релиза к Build All, «если это нам не сломает работу». Отсюда контракт:

1. Шаги релиза — последние шаги последнего job'а (`summary` ждёт все сборки Pi, после него никто не идёт).
2. У каждого шага `continue-on-error: true` — провал релиза не красит Build All.
3. Нет секрета ключа подписи → шаг-гейт честно пропускает релиз с `::warning::` и выходит 0 (проверено прогоном bash).
4. Релиз только с develop (канал dev) и только когда все три Pi собрались.
5. Шаги не трогают `.image-versions.*`, деплой и робота (ни ssh, ни image-versions, ни git push).
6. Подпись сверяется с открытым ключом из репо `host/chuangong/keys/release.pub` до публикации.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
BUILD_ALL_WF = REPO_ROOT / ".github" / "workflows" / "L-Build-All-Services.yml"
PUBKEY = REPO_ROOT / "host" / "chuangong" / "keys" / "release.pub"
SECRET = "CHUANGONG_SIGNING_KEY"
PI_JOBS = ("build-main-services", "build-sound-services", "build-vision-services")


def _jobs() -> dict:
    return yaml.safe_load(BUILD_ALL_WF.read_text(encoding="utf-8"))["jobs"]


def _release_steps() -> list[dict]:
    return [s for s in _jobs()["summary"]["steps"] if str(s.get("name", "")).startswith("Release:")]


def _gate() -> dict:
    return next(s for s in _release_steps() if s.get("id") == "release-gate")


def test_release_steps_are_last_steps_of_last_job():
    jobs = _jobs()
    summary = jobs["summary"]
    assert set(PI_JOBS) <= set(summary["needs"]), "summary должен ждать все сборки Pi"
    assert summary.get("if") == "always()"
    for name, job in jobs.items():
        needs = job.get("needs") or []
        needs = [needs] if isinstance(needs, str) else needs
        assert "summary" not in needs, f"{name} ждёт summary — релиз перестал быть последним"
    steps = summary["steps"]
    release = _release_steps()
    assert [s["name"] for s in release] == [
        "Release: gate (ADR-0168)",
        "Release: checkout (ADR-0168)",
        "Release: build, sign, publish (ADR-0168)",
    ]
    assert steps[-len(release) :] == release, "шаги релиза должны стоять в самом конце job'а summary"


def test_every_release_step_continues_on_error():
    for step in _release_steps():
        assert step.get("continue-on-error") is True, step["name"]
        assert "!cancelled()" in step.get("if", ""), step["name"]


def test_build_steps_run_only_when_gate_enabled():
    for step in _release_steps():
        if step.get("id") == "release-gate":
            continue
        assert "steps.release-gate.outputs.enabled == 'true'" in step["if"], step["name"]


def test_gate_reads_secret_presence_and_build_results():
    env = _gate()["env"]
    assert env["HAS_SIGNING_KEY"] == "${{ secrets.%s != '' }}" % SECRET
    for job, var in zip(PI_JOBS, ("MAIN_RESULT", "SOUND_RESULT", "VISION_RESULT")):
        assert env[var] == "${{ needs.%s.result }}" % job
    # Сам ключ в гейт не передаётся — только факт наличия.
    assert all("secrets.%s }}" % SECRET not in str(v) for v in env.values())


def test_secret_reaches_only_the_signing_step():
    publish = _release_steps()[-1]
    assert publish["env"] == {SECRET: "${{ secrets.%s }}" % SECRET}
    text = BUILD_ALL_WF.read_text(encoding="utf-8")
    assert text.count("secrets.%s" % SECRET) == 2, "секрет упомянут только в гейте и шаге подписи"


def test_publish_step_does_not_touch_deploy_or_robot():
    run = _release_steps()[-1]["run"]
    for cmd in ("release.py build", "release.py sign", "release.py verify", "release.py publish"):
        assert cmd in run, cmd
    assert run.index("release.py verify") < run.index("release.py publish"), "проверка подписи — до публикации"
    assert "--channel dev" in run and "--docker-tag dev" in run
    assert "--pubkey host/chuangong/keys/release.pub" in run
    for forbidden in ("image-versions", "ssh", "git push", "10.1.1.1", "chuangong.timer", "systemctl"):
        assert forbidden not in run, f"шаг релиза не должен содержать {forbidden!r}"


def test_repo_public_key_is_a_minisign_key():
    sys.path.insert(0, str(REPO_ROOT / "host" / "chuangong"))
    from chuangong import minisign

    text = PUBKEY.read_text(encoding="utf-8")
    key = minisign.load_public_key(text)
    assert key.key_id in text.splitlines()[0]
    assert "secret" not in text.lower()


def _run_gate(tmp_path: Path, **overrides: str) -> tuple[subprocess.CompletedProcess, str]:
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("нет bash")
    out = tmp_path / "github_output"
    out.write_text("", encoding="utf-8")
    env = dict(os.environ)
    env.update(
        {
            "HAS_SIGNING_KEY": "true",
            "GH_REF": "refs/heads/develop",
            "MAIN_RESULT": "success",
            "SOUND_RESULT": "success",
            "VISION_RESULT": "success",
            "GITHUB_OUTPUT": str(out),
        }
    )
    env.update(overrides)
    script = tmp_path / "gate.sh"
    script.write_bytes(_gate()["run"].encode("utf-8"))
    proc = subprocess.run([bash, script.name], cwd=tmp_path, env=env, capture_output=True, text=True, encoding="utf-8")
    return proc, out.read_text(encoding="utf-8")


def _enabled(output: str) -> str:
    return [line for line in output.splitlines() if line.startswith("enabled=")][-1]


def test_gate_without_secret_skips_with_warning(tmp_path):
    proc, output = _run_gate(tmp_path, HAS_SIGNING_KEY="false")
    assert proc.returncode == 0, proc.stderr
    assert "::warning::" in proc.stdout and SECRET in proc.stdout
    assert _enabled(output) == "enabled=false"


def test_gate_skips_when_a_pi_build_failed(tmp_path):
    proc, output = _run_gate(tmp_path, SOUND_RESULT="failure")
    assert proc.returncode == 0
    assert "::warning::" in proc.stdout
    assert _enabled(output) == "enabled=false"


@pytest.mark.parametrize("ref", ["refs/heads/main", "refs/heads/feat/x", "refs/tags/v1.0.0"])
def test_gate_skips_outside_develop(tmp_path, ref):
    proc, output = _run_gate(tmp_path, GH_REF=ref)
    assert proc.returncode == 0
    assert _enabled(output) == "enabled=false"


def test_gate_enables_on_green_develop_with_secret(tmp_path):
    proc, output = _run_gate(tmp_path)
    assert proc.returncode == 0, proc.stderr
    assert _enabled(output) == "enabled=true"
