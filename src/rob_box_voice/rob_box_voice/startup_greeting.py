#!/usr/bin/env python3
"""Стартовая фраза v1 (issue #1003): данные и выбор перенесены в ``rob_box_dialog.startup`` (#3912).

Одна реализация на v1 и v2: ``dialogue_node`` (inline-приветствие через ``startup_greeting_sec``) и ``dialog_node``
(``core.dialog_v2_parity``) берут фразу оттуда. Модуль уходит вместе с v1 в PR-15.
"""

from __future__ import annotations

import random  # noqa: F401 — тесты v1 патчат ``rob_box_voice.startup_greeting.random.choice``

from rob_box_dialog.startup import GREETINGS, pick_greeting

__all__ = ["GREETINGS", "pick_greeting"]
