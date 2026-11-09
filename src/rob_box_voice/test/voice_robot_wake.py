"""Вейки робота для тестов rob_box_voice - из профиля робота в чекауте (ADR-0167 Ш1б-2б): в ядре их больше нет.

``ROBOT_WAKE`` / ``ROBOT_OPERATOR_WAKE`` - таблица вейков ``docker/sound/config/assistant.yaml`` (то же, что
``robot_profile.load()`` хостов робота); ``ROBOT_RISKY`` - её рискованные записи (``{word, risky: true}``).
"""

from __future__ import annotations

from pathlib import Path
from typing import Tuple

import yaml

from rob_box_dialog.profile import load_profile

PROFILE = Path(__file__).resolve().parents[3] / "docker" / "sound" / "config" / "assistant.yaml"

_PROFILE = load_profile(PROFILE)
ROBOT_WAKE: Tuple[str, ...] = _PROFILE.wake_personality
ROBOT_OPERATOR_WAKE: Tuple[str, ...] = _PROFILE.wake_operator
ROBOT_RISKY: Tuple[str, ...] = tuple(
    item["word"]
    for item in yaml.safe_load((PROFILE.parent / "wake_words.yaml").read_text(encoding="utf-8"))["personality"]
    if isinstance(item, dict) and item.get("risky")
)
