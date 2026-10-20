"""Пакет чистый: без ROS, без renardo, без сети (ADR-0149 §2.2)."""

import ast
import pathlib
import subprocess
import sys

import rob_box_music

PKG = pathlib.Path(rob_box_music.__file__).parent
FORBIDDEN = {"rclpy", "renardo", "renardo_lib", "FoxDot", "requests", "httpx", "aiohttp", "urllib3", "socket",
             "websockets", "rob_box_mcp_tools", "rob_box_voice", "rob_box_core", "std_msgs", "ament_index_python"}


def _imports(path):
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            yield from (a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            yield node.module.split(".")[0]


def test_no_forbidden_imports_in_sources():
    bad = {str(p.relative_to(PKG)): sorted(set(_imports(p)) & FORBIDDEN) for p in PKG.rglob("*.py")}
    assert {k: v for k, v in bad.items() if v} == {}


def test_import_pulls_no_forbidden_modules_at_runtime():
    code = (
        "import sys, rob_box_music.knowledge, rob_box_music.model\n"
        f"bad = sorted(m for m in sys.modules if m.split('.')[0] in {sorted(FORBIDDEN)!r})\n"
        "print(bad); sys.exit(1 if bad else 0)\n"
    )
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, cwd=str(PKG.parent))
    assert out.returncode == 0, out.stdout + out.stderr
