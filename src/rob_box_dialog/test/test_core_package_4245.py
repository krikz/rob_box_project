"""Ядро диалога пакетом без ROS (ADR-0160 §8 К1, карточка #4245).

Проверяется то, что РЕАЛЬНО поставлено пакетом ``rob-box-dialog-core`` (``packaging/dialog-core``): список модулей
берётся из RECORD установленного дистрибутива, а не из разметки. Без поставленного пакета (colcon-окружение робота,
G-Run Tests) тесты пропускаются - их гоняет CI-джоб «Dialog core package» в чистом venv на 3.10/3.11/3.12.

* в исходниках пакета нет ROS на верхнем уровне модуля (ленивый импорт внутри функции - можно: так хост робота
  подключает ``rclpy`` только у себя);
* каждый модуль пакета импортируется в чистом venv, и после импорта всех в ``sys.modules`` нет ни одного ROS-модуля;
* ``ROB_BOX_CORE_FROM_PACKAGE=1`` - тесты этого прогона взяли ядро из пакета, а не из ``src/`` чекаута.
"""

import ast
import importlib.metadata as md
import json
import os
import pathlib
import subprocess
import sys

import pytest

DIST = "rob-box-dialog-core"
#: Корни ROS (Humble): клиент, сообщения, сервисы, индекс пакетов.
ROS = {"rclpy", "rcl_interfaces", "std_msgs", "sensor_msgs", "geometry_msgs", "builtin_interfaces",
       "ament_index_python", "rosidl_runtime_py", "rob_box_dialog_msgs", "rob_box_perception_msgs",
       "rob_box_supervisor_msgs", "robot_sensor_hub_msg"}

try:
    _DIST = md.distribution(DIST)
except md.PackageNotFoundError:  # pragma: no cover - окружение без пакета
    _DIST = None

pytestmark = pytest.mark.skipif(_DIST is None, reason=f"{DIST} не поставлен (pip install packaging/dialog-core)")


def _sources():
    """``(имя модуля, путь)`` всех .py, поставленных пакетом."""
    for f in _DIST.files or ():
        if f.suffix == ".py" and "__pycache__" not in f.parts:
            name = ".".join(f.with_suffix("").parts)
            yield name.removesuffix(".__init__"), pathlib.Path(f.locate())


def _top_level_imports(path):
    """Корни импортов верхнего уровня модуля (тело функций и классов-методов не смотрим: там ленивый импорт)."""
    tree = ast.parse(path.read_text(encoding="utf-8-sig"))
    for node in tree.body:
        for sub in ast.walk(node) if isinstance(node, (ast.If, ast.Try)) else (node,):
            if isinstance(sub, ast.Import):
                yield from (a.name.split(".")[0] for a in sub.names)
            elif isinstance(sub, ast.ImportFrom) and sub.level == 0 and sub.module:
                yield sub.module.split(".")[0]


def test_package_ships_core_and_bridge():
    names = {n for n, _ in _sources()}
    for must in ("rob_box_dialog.turn", "rob_box_core.utterance", "rob_box_llm.provider",
                 "rob_box_harness.core.assembly", "rob_box_dialog_host.dialog_v2_ports",
                 "rob_box_dialog_host.command_bridge", "rob_box_dialog_host.voice_memory",
                 "rob_box_dialog_host.speaker_memory", "rob_box_dialog_host.tts_voice_registry",
                 "rob_box_dialog_host.music_player_state"):  # связка хода - пакет rob_box_dialog_host (Ш1г)
        assert must in names, must
    assert not [n for n in names if n.startswith("rob_box_voice")], "rob_box_voice в ядре не ставится (Ш1г-2)"
    assert not [n for n in names if ".test." in f".{n}." or n.endswith("_node")], "тесты и ROS-ноды не в ядре"


def test_no_ros_import_at_module_top_level():
    bad = {n: sorted(set(_top_level_imports(p)) & ROS) for n, p in _sources()}
    assert {n: v for n, v in bad.items() if v} == {}


def test_every_module_imports_without_ros():
    names = sorted(n for n, _ in _sources())
    code = (
        "import importlib, json, sys\n"
        f"names = {names!r}\n"
        "failed = {}\n"
        "for n in names:\n"
        "    try:\n"
        "        importlib.import_module(n)\n"
        "    except Exception as e:\n"
        "        failed[n] = f'{type(e).__name__}: {e}'\n"
        f"ros = sorted(m for m in sys.modules if m.split('.')[0] in {sorted(ROS)!r})\n"
        "print(json.dumps({'failed': failed, 'ros': ros}, ensure_ascii=False))\n"
    )
    # -I: без cwd и PYTHONPATH в sys.path - только то, что поставлено в окружение
    out = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True, encoding="utf-8")
    assert out.returncode == 0, out.stderr
    got = json.loads(out.stdout.strip().splitlines()[-1])
    assert got == {"failed": {}, "ros": []}


@pytest.mark.skipif(os.environ.get("ROB_BOX_CORE_FROM_PACKAGE") != "1", reason="только в прогоне из пакета")
def test_core_comes_from_package_not_checkout():
    import rob_box_core
    import rob_box_dialog
    import rob_box_harness
    import rob_box_llm
    import rob_box_dialog_host.dialog_v2_ports as ports

    where = {m.__name__: pathlib.Path(m.__file__).resolve() for m in
             (rob_box_core, rob_box_dialog, rob_box_harness, rob_box_llm, ports)}
    src = {n: str(p) for n, p in where.items() if "src" in p.parts}
    assert src == {}, src
