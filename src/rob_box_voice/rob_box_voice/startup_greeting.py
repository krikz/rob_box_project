#!/usr/bin/env python3
"""
startup_greeting.py — данные и хелперы приветствия при старте (issue #1003).

Чистый Python без rclpy, чтобы юнит-тесты работали без ROS2:
* GREETINGS — 9 прикольных фраз (требование 6–10);
* THINKING_SOUND / FINISH_SOUNDS — триггеры звуков через sound_node;
* pick_greeting() / pick_finish_sound() — случайный выбор.

Этот модуль использует dialogue_node (inline-приветствие через
startup_greeting_sec) и standalone-нода startup_greeting_node.py.
"""

from __future__ import annotations

import random

# Прикольные варианты. Тон — дружелюбный кот-робот, не сухой.
# Их 9, попадает под требование "6–10 вариантов".
GREETINGS: tuple[str, ...] = (
    "Мяу! Я на связи! Все системы в норме, можно играть!",
    "Привет-привет! Только что проснулся, готов к приключениям!",
    "Я вернулся! Датчики заряжены, мурчание включено.",
    "Здравствуйте! Загрузка завершена — скучать было некогда.",
    "Онлайн! Усы на месте, ушки на макушке.",
    "Готов помогать! Только не обижайте мои сенсоры.",
    "Рад вас слышать! Все модули мурлычут.",
    "Пи-пи-пип! Ой, то есть — доброе утро, хозяин.",
    "Слушаю вас внимательно. Ну, насколько микрофон позволяет.",
)

# Звуки, которые мы умеем триггерить через sound_node.
THINKING_SOUND = "thinking"
FINISH_SOUNDS: tuple[str, ...] = ("cute", "very_cute")


def pick_greeting(override: str | None = None) -> str:
    """Вернуть фразу приветствия.

    Если ``override`` непустой — используем его (явная настройка
    оператора через параметр startup_greeting_text), иначе — случайная
    фраза из GREETINGS.
    """
    if override and override.strip():
        return override.strip()
    return random.choice(GREETINGS)


def pick_finish_sound() -> str:
    """Случайный «радостный» звук из FINISH_SOUNDS."""
    return random.choice(FINISH_SOUNDS)
