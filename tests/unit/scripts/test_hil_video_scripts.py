"""Скрипты ролика из HIL (scripts/hil/video, docs/runbooks/hil-video.md): синтаксис и ссылка из docs/README.md.

Настоящий прогон нужен стенд и песочница; здесь — только то, что можно проверить без них.
"""

import py_compile
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
VIDEO = ROOT / "scripts" / "hil" / "video"


@pytest.mark.parametrize("name", sorted(p.name for p in VIDEO.glob("*.py")))
def test_python_scripts_compile(name, tmp_path):
    py_compile.compile(str(VIDEO / name), cfile=str(tmp_path / "x.pyc"), doraise=True)


def _bash_works() -> bool:
    # на Windows `bash` бывает заглушкой WSL без дистрибутива: тогда `bash -c true` падает
    if shutil.which("bash") is None:
        return False
    try:
        return subprocess.run(["bash", "-c", "true"], capture_output=True, timeout=20).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


@pytest.mark.skipif(not _bash_works(), reason="нет рабочего bash")
@pytest.mark.parametrize("name", sorted(p.name for p in VIDEO.glob("*.sh")))
def test_shell_scripts_syntax(name):
    subprocess.run(["bash", "-n", name], check=True, cwd=VIDEO)


def test_runbook_linked_from_docs_readme():
    assert (ROOT / "docs" / "runbooks" / "hil-video.md").is_file()
    assert "runbooks/hil-video.md" in (ROOT / "docs" / "README.md").read_text(encoding="utf-8")
