"""Регресс #5053: образ telegram-bot обязан нести rob_box_supervisor.

``face_roster_view.py`` импортирует ``rob_box_supervisor.face_roster_tool``
на старте ноды (цепочка ``telegram_node`` → ``handlers.commands``). База
``ros2-zenoh`` этого пакета не содержит. Пропавший COPY или пропавший
``--packages-select`` снова дают ``ModuleNotFoundError`` и рестарт-петлю.

Сети нет: читается только Dockerfile. Сборка образа этим тестом не
запускается.
"""

from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
DOCKERFILE = REPO / "docker" / "sound" / "telegram_bot" / "Dockerfile"

_COPY_PACKAGE = (
    "COPY src/rob_box_supervisor/package.xml /ws/src/rob_box_supervisor/",
    "COPY src/rob_box_supervisor/setup.py",
    "COPY src/rob_box_supervisor/setup.cfg",
    "COPY src/rob_box_supervisor/resource",
    "COPY src/rob_box_supervisor/rob_box_supervisor /ws/src/rob_box_supervisor/rob_box_supervisor",
)
_SELECT_RE = re.compile(r"colcon build[^\n]*--packages-select(?P<pkgs>[^\n\\]*)")


def _text() -> str:
    return DOCKERFILE.read_text(encoding="utf-8")


def _colcon_selects() -> str:
    """Пакеты из ``colcon build --packages-select`` (продолжение через ``\\``)."""
    lines = _text().splitlines()
    chunks: list[str] = []
    capturing = False
    for line in lines:
        stripped = line.strip()
        if "colcon build" in stripped and "--packages-select" in stripped:
            capturing = True
        if capturing:
            chunks.append(stripped.rstrip("\\").strip())
            if not stripped.endswith("\\"):
                break
    assert chunks, "colcon build --packages-select пропал из telegram_bot/Dockerfile"
    return " ".join(chunks)


def test_dockerfile_copies_supervisor_sources() -> None:
    """Исходники ament_python-пакета копируются до сборки."""
    text = _text()
    for needle in _COPY_PACKAGE:
        assert needle in text, needle


def test_dockerfile_builds_supervisor_with_telegram() -> None:
    """Сборка выбирает оба пакета. Ядро диалога в select не входит."""
    selected = _colcon_selects()
    assert "rob_box_supervisor" in selected.split()
    assert "rob_box_telegram" in selected.split()
    assert "rob_box_core" not in selected.split()
    assert "rob_box_dialog" not in selected.split()


def test_supervisor_copy_is_after_dialog_core_pip() -> None:
    """pip ядра идёт раньше colcon: сборка хоста не должна затирать site-packages."""
    text = _text()
    pip_at = text.find("rob-box-dialog-core @")
    copy_at = text.find("COPY src/rob_box_supervisor/package.xml")
    build_at = text.find("colcon build")
    assert pip_at != -1, "пин rob-box-dialog-core пропал"
    assert 0 < pip_at < copy_at < build_at


def test_msgpack_apt_present_for_supervisor_codec() -> None:
    """Кодек AV-14 импортирует msgpack; apt-пакет объявлен exec_depend супервизора."""
    assert "python3-msgpack" in _text()
