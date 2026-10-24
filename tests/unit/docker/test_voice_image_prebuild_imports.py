"""Регресс #3683: шаг ``RUN python3 /tmp/fix_brass_scd.py`` в образе voice-assistant
идёт ДО colcon build, поэтому его импорты должны резолвиться из исходников.
"""
import importlib.util
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
SCRIPT = REPO / "docker/sound/voice_assistant/fix_brass_scd.py"
DOCKERFILE = REPO / "docker/sound/voice_assistant/Dockerfile"


def _load_script():
    spec = importlib.util.spec_from_file_location("fix_brass_scd", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_script_imports_resolve_from_sources_only():
    roots = _load_script().package_roots(REPO)
    # editable-установки из .pth отсекаем, оставляя stdlib/site-packages.
    code = "import sys; sys.path[:] = %r + [p for p in sys.path if 'rob_box' not in p]; " % roots
    code += "from rob_box_voice.core.renardo_synthdef_patches import apply_renardo_synthdef_patches"
    r = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True, cwd=str(REPO / "docs"))
    assert r.returncode == 0, r.stderr


def test_dialog_sources_copied_before_script_runs():
    lines = DOCKERFILE.read_text(encoding="utf8").splitlines()
    run = next(i for i, l in enumerate(lines) if l.startswith("RUN python3 /tmp/fix_brass_scd.py"))
    before = "\n".join(lines[:run])
    for pkg in ("rob_box_core", "rob_box_dialog", "rob_box_voice"):
        assert "COPY src/%s/%s " % (pkg, pkg) in before, pkg
