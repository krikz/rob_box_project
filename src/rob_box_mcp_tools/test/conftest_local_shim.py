"""
conftest.py — local env shim для тестов operator_admin.

Подменяет rob_box_voice на Mock-пакет, потому что в локальной среде
(вне CI-контейнера) пакет собран не полностью (отсутствует rob_box_voice.core),
а tools/music.py тянет его на импорте. Это типично для monorepo, где каждый
ROS-пакет собирается colcon-ом.

В CI (image rob-box-ci:humble) пакеты реальные, моки не нужны и не мешают
(если спецификация rob_box_voice корректна — conftest проверяет).
"""

from __future__ import annotations

import importlib.util
import sys
from unittest.mock import Mock


_REAL_ROB_BOX_VOICE = "rob_box_voice" in sys.modules or (
    importlib.util.find_spec("rob_box_voice") is not None
    and importlib.util.find_spec("rob_box_voice.core") is not None
)


def _install_voice_shim() -> None:
    if _REAL_ROB_BOX_VOICE:
        return

    fake = Mock()
    fake.core = Mock()
    sys.modules["rob_box_voice"] = fake
    sys.modules["rob_box_voice.core"] = fake.core
    sys.modules["rob_box_voice.core.music_stack_validation"] = fake.core.music_stack_validation
    sys.modules.setdefault("rob_box_voice.utils", Mock())
    sys.modules["rob_box_voice.utils.redact"] = Mock(
        redact_upstream_body=lambda text: text,
    )


_install_voice_shim()