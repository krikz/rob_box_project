"""Тестовый профиль робота для тестов ядра (ADR-0167 Ш1б-2б): вейки робота живут в профиле, а не в ядре.

``ROBOT_WAKE`` / ``ROBOT_OPERATOR_WAKE`` - таблица вейков тестового профиля (``fixtures/robot_profile``).
``ROBOT_RISKY`` - рискованные вейки той же таблицы (``{word, risky: true}``, включаются ``risky_enabled``).
``conftest.py`` ставит ``ROBOT_WAKE`` умолчанием правил на время каждого теста - как хост робота, который всегда
передаёт свою таблицу.
"""

from __future__ import annotations

from pathlib import Path
from typing import Tuple

import yaml

from rob_box_dialog.profile import load_profile

PROFILE_DIR = Path(__file__).resolve().parent / "fixtures" / "robot_profile"
WAKE_TABLE = PROFILE_DIR / "wake_words.yaml"

_PROFILE = load_profile(PROFILE_DIR)
ROBOT_WAKE: Tuple[str, ...] = _PROFILE.wake_personality
ROBOT_OPERATOR_WAKE: Tuple[str, ...] = _PROFILE.wake_operator
#: Владельцы тулов профиля робота (ADR-0167 Ш1в): умолчание среза тулов на время теста (conftest).
ROBOT_OWNERS: Tuple[str, ...] = _PROFILE.owners
ROBOT_RISKY: Tuple[str, ...] = tuple(
    item["word"]
    for item in yaml.safe_load(WAKE_TABLE.read_text(encoding="utf-8"))["personality"]
    if isinstance(item, dict) and item.get("risky")
)
