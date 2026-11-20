"""Хост проверяется на установленном теге ядра, а не на копии в ``src/`` (#4932, #4245).

Гард #4891 сверяет имена импорта с деревом тега. Этого мало: pytest монорепо кладёт
``src/<пакет>`` в ``sys.path`` раньше site-packages, и зелёный прогон смотрит локальную
копию, а образ — ``rob-box-dialog-core`` из pip. Этот тест запускает чистый интерпретатор
без ``src`` на пути и требует, чтобы импорт шёл из site-packages пина Dockerfile.

Подмена ловится так: если каталог ``src/<пакет>`` оказывается на ``sys.path`` раньше
установленного пакета, импорт резолвится в чекаут. Тест это детектит и падает, а не
зеленеет. Готовый префикс задаётся ``ROB_BOX_DIALOG_CORE_PREFIX`` (изолированный, не
системный Python). Без префикса проба пина пропускается с причиной: джоб без venv
не может её исполнить, а джоб с venv префикс выставляет и skip не получает.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[5]
_PIN_RE = re.compile(r"rob_box_dialog_core@([\w.]+)")
_PROBE = textwrap.dedent(
    """\
    import sys
    from pathlib import Path

    # Подмена приходит аргументом и вставляется в начало sys.path до импорта:
    # так ловится и PYTHONPATH чекаута, и pytest, который кладёт src раньше пакета.
    # Флаг -P на 3.10 нет, -I выкидывает PYTHONPATH — поэтому путь явный.
    pin = sys.argv[1]
    planted = [p for p in sys.argv[2:] if p]
    for entry in reversed(planted):
        sys.path.insert(0, entry)
    import rob_box_dialog

    src = [Path(p).resolve() for p in planted]
    origin = Path(rob_box_dialog.__file__).resolve()
    print(f"IMPORT {origin}")
    print("PATH")
    for entry in sys.path:
        print(entry)
    if any(part == "src" and part_parent.name in {
        "rob_box_core", "rob_box_dialog", "rob_box_dialog_host",
        "rob_box_harness", "rob_box_llm", "rob_box_music",
    } for part_parent in (origin.parent,) for part in origin.parts):
        raise SystemExit(f"подмена: импорт из чекаута {origin}")
    if "site-packages" not in origin.parts:
        raise SystemExit(f"импорт не из site-packages: {origin}")
    ahead = [
        entry for entry in sys.path
        if entry and Path(entry).resolve() in src
    ]
    if ahead:
        raise SystemExit("подмена: src на sys.path раньше пакета: " + ", ".join(ahead))
    # Версия в metadata тега может не совпасть с именем тега (v0.7.9 нёс 0.7.8; v0.7.10 несёт 0.7.10).
    # Сверяем direct_url: pip пишет туда запрошенный тег.
    from importlib.metadata import distribution

    direct = distribution("rob-box-dialog-core").read_text("direct_url.json") or ""
    print(f"DIRECT {direct}")
    if pin not in direct:
        raise SystemExit(f"установлен не тег {pin}: {direct}")
    print(f"PIN {pin}")
    """
)


def _src_dirs() -> list[Path]:
    return [
        _REPO / "src" / name
        for name in (
            "rob_box_core",
            "rob_box_dialog",
            "rob_box_dialog_host",
            "rob_box_harness",
            "rob_box_llm",
            "rob_box_music",
        )
    ]


def _probe(
    python: str, pin: str, planted: list[Path] | None = None
) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    # -I: ни cwd, ни PYTHONPATH окружения. Подмену теста передаём аргументом.
    return subprocess.run(
        [python, "-I", "-c", _PROBE, pin, *(str(path) for path in (planted or []))],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
        check=False,
    )


def test_src_on_pythonpath_is_detected_as_substitution():
    """Подсунутый ``src/`` детектится: проба падает и пишет «подмена»."""
    got = _probe(sys.executable, "local-src", planted=_src_dirs())
    assert got.returncode != 0, got.stdout + got.stderr
    assert "подмена" in (got.stdout + got.stderr)


def test_installed_pin_import_is_not_shadowed_by_src():
    """Чистый префикс с тегом из Dockerfile импортирует ядро из site-packages.

    Префикс готовит вызывающий (``ROB_BOX_DIALOG_CORE_PREFIX``): ``python -m venv``
    и ``pip install rob-box-dialog-core@<тег>``. В джобе ``Unit Tests (ROS2 Humble)``
    такого venv нет — там тест пропускается с причиной, а не красится. Где venv есть
    (джоб ``Dialog core package``), префикс выставлен и проба обязана пройти: skip
    там не срабатывает, проверка не ослаблена.
    """
    prefix = os.environ.get("ROB_BOX_DIALOG_CORE_PREFIX")
    if not prefix:
        pytest.skip(
            "нет ROB_BOX_DIALOG_CORE_PREFIX: в этом джобе нет venv с "
            "rob-box-dialog-core@<тег из Dockerfile>; гард гоняет джоб, который venv ставит"
        )
    python = str(Path(prefix) / "bin" / "python")
    assert Path(python).is_file(), f"в префиксе нет {python}"
    dockerfile = (_REPO / "docker" / "sound" / "voice_assistant" / "Dockerfile").read_text(encoding="utf-8")
    match = _PIN_RE.search(dockerfile)
    assert match, "пин rob_box_dialog_core@ не найден в Dockerfile voice-assistant"
    got = _probe(python, match.group(1))
    assert got.returncode == 0, got.stdout + got.stderr
    assert "site-packages" in got.stdout
    assert "подмена" not in got.stdout
