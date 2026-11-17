"""Тесты пакета rob_box_sim без ROS: модули импортируются прямо из src/rob_box_sim."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "src" / "rob_box_sim"))
