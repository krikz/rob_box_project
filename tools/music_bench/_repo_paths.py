"""Единая точка sys.path для стенда (issue #2977).

``rob_box_mcp_tools`` не установлен как пакет (pip show — пусто, см. PR
issue #2977): он живёт исходником в ``src/rob_box_mcp_tools/rob_box_mcp_tools``
и в тестах пакета становится импортируемым только потому, что pytest
запускают с ``cwd=src/rob_box_mcp_tools``. Этот стенд — отдельный CLI под
``tools/``, cwd произвольный, поэтому путь добавляется явно, один раз, до
первого импорта ``rob_box_mcp_tools.core.*``.

Импортируется только ``core.*`` (arranger, harmonize, rtttl*) — эти модули
не тянут ``rclpy`` (проверено: ``grep -rl "^import rclpy" core/`` — пусто),
поэтому стенд работает на голом Python/Windows, без ROS2 и без robota.
"""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_MCP_TOOLS_SRC = _REPO_ROOT / "src" / "rob_box_mcp_tools"


def ensure_importable() -> None:
    """Добавить ``src/rob_box_mcp_tools`` в ``sys.path``, если его там нет."""
    candidate = str(_MCP_TOOLS_SRC)
    if _MCP_TOOLS_SRC.exists() and candidate not in sys.path:
        sys.path.insert(0, candidate)


ensure_importable()
