"""Адреса деплоя настраиваются переменными, дефолт — нынешние (Refs #4375).

Смена топологии не должна требовать правки workflow: SSH-адреса Pi и registry
берутся из ROBBOX_MAIN_HOST / ROBBOX_VISION_HOST / ROBBOX_SOUND_HOST /
ROBBOX_REGISTRY_HOST, а без них остаются 10.1.1.10 / .11 / .12 / 10.1.1.249:5000
байт в байт. Иначе выезд робота за свою сеть ломает деплой до правки четырёх
веток case.
"""

import os
import re
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
WORKFLOW = ROOT / ".github" / "workflows" / "L-Deploy and Verify.yml"

DEFAULTS = {
    "main_pi_ip": "10.1.1.10",
    "vision_pi_ip": "10.1.1.11",
    "sound_pi_ip": "10.1.1.12",
    "SERVICE_IMAGE_PREFIX": "10.1.1.249:5000/krikz/rob_box",
}


def _setup_step():
    data = yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))
    found = [
        s
        for job in data["jobs"].values()
        for s in job.get("steps", [])
        if s.get("id") == "setup"
    ]
    assert len(found) == 1
    return found[0]["run"]


def _setup_until_outputs():
    """Кусок шага setup, который только пишет GITHUB_ENV. Дальше идут
    github.ref_name / sudo apt-get — их этот тест не исполняет. Раньше граница
    была маркером бана #4653; бан снят (#5110), граница — первая запись
    GITHUB_OUTPUT."""
    text = _setup_step()
    marker = 'echo "branch='
    assert marker in text, "граница прогона setup (echo branch) не найдена"
    return text.split(marker)[0]


def _run(environment, registry_source, hosts):
    """Прогон шага setup до записи outputs. Слепой бан #4653 снят (#5110):
    local+production больше не делает exit 1 в Setup."""
    text = _setup_until_outputs()
    text = text.replace("${{ inputs.environment }}", environment)
    text = text.replace("${{ inputs.registry_source }}", registry_source)
    env = dict(os.environ)
    env.update(hosts)
    env["GITHUB_ENV"] = "gh_env"
    proc = subprocess.run(
        ["bash", "-e", "-c", text],
        cwd=Path.cwd(),
        env=env,
        capture_output=True,
        timeout=30,
    )
    assert proc.returncode == 0, proc.stderr.decode("utf-8", "replace")
    out = {}
    for line in Path("gh_env").read_text(encoding="utf-8").splitlines():
        key, _, value = line.partition("=")
        out[key] = value
    return out


@pytest.fixture(autouse=True)
def _clean_env(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    for name in (
        "ROBBOX_MAIN_HOST",
        "ROBBOX_VISION_HOST",
        "ROBBOX_SOUND_HOST",
        "ROBBOX_REGISTRY_HOST",
    ):
        monkeypatch.delenv(name, raising=False)


@pytest.mark.parametrize("environment", ["production", "staging", "test", "local"])
def test_default_hosts_are_the_current_addresses(environment):
    """Без переменных адреса байт в байт прежние во всех четырёх ветках."""
    got = _run(environment, "local", {})
    for key, value in DEFAULTS.items():
        assert got[key] == value, f"{environment}: {key}"


def test_override_replaces_hosts_without_editing_the_workflow():
    """Переменные меняют и SSH-адреса, и registry; тег образа ветка не трогает."""
    got = _run(
        "staging",
        "local",
        {
            "ROBBOX_MAIN_HOST": "10.9.1.10",
            "ROBBOX_VISION_HOST": "10.9.1.11",
            "ROBBOX_SOUND_HOST": "10.9.1.12",
            "ROBBOX_REGISTRY_HOST": "10.9.1.249:5000",
        },
    )
    assert got["main_pi_ip"] == "10.9.1.10"
    assert got["vision_pi_ip"] == "10.9.1.11"
    assert got["sound_pi_ip"] == "10.9.1.12"
    assert got["SERVICE_IMAGE_PREFIX"] == "10.9.1.249:5000/krikz/rob_box"
    assert got["IMAGE_TAG"] == "dev"


def test_github_registry_ignores_the_registry_override():
    """Override registry действует только на local: github остаётся ghcr."""
    got = _run("staging", "github", {"ROBBOX_REGISTRY_HOST": "10.9.1.249:5000"})
    assert got["SERVICE_IMAGE_PREFIX"] == "ghcr.io/krikz/rob_box"


def test_chrony_uses_the_same_main_host_default():
    """setup_chrony.sh берёт сервер из ROBBOX_MAIN_HOST, без неё — 10.1.1.10."""
    text = (ROOT / "scripts" / "maintenance" / "setup_chrony.sh").read_text(
        encoding="utf-8"
    )
    assert 'DEFAULT_SERVER="${ROBBOX_MAIN_HOST:-10.1.1.10}"' in text


def test_workflow_has_no_bare_host_literal_in_the_setup_step():
    """Адрес в шаге встречается только как дефолт переменной, не голым литералом."""
    step = _setup_until_outputs()
    bare = re.findall(r"(?<![:-])\b10\.1\.1\.(?:10|11|12|249)\b", step)
    assert bare == [], bare
