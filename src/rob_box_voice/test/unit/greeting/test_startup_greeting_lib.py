"""
test_startup_greeting_lib.py — Unit-тесты startup_greeting.py (issue #1003).

Чистые тесты без ROS2: проверяем GREETINGS (6–10 прикольных фраз),
pick_greeting (случайный выбор + override) и pick_finish_sound.
"""

from __future__ import annotations

from unittest.mock import patch

from rob_box_voice.startup_greeting import (
    FINISH_SOUNDS,
    GREETINGS,
    THINKING_SOUND,
    pick_finish_sound,
    pick_greeting,
)


class TestGreetingsData:
    """Константы GREETINGS."""

    def test_greetings_count_between_6_and_10(self) -> None:
        assert 6 <= len(GREETINGS) <= 10

    def test_greetings_all_non_empty(self) -> None:
        for phrase in GREETINGS:
            assert isinstance(phrase, str)
            assert phrase.strip()

    def test_greetings_contain_myau(self) -> None:
        assert any("Мяу" in p and "на связи" in p for p in GREETINGS)

    def test_finish_sounds(self) -> None:
        assert FINISH_SOUNDS == ("cute", "very_cute")

    def test_thinking_sound(self) -> None:
        assert THINKING_SOUND == "thinking"


class TestPickGreeting:
    def test_pick_greeting_returns_member(self) -> None:
        assert pick_greeting() in GREETINGS

    def test_pick_greeting_override_wins(self) -> None:
        assert pick_greeting("Я на связи, все системы в норме!") == (
            "Я на связи, все системы в норме!"
        )

    def test_pick_greeting_empty_override_ignored(self) -> None:
        with patch("rob_box_voice.startup_greeting.random.choice") as m:
            m.return_value = GREETINGS[0]
            assert pick_greeting("") == GREETINGS[0]
            assert pick_greeting("   ") == GREETINGS[0]

    def test_pick_greeting_override_stripped(self) -> None:
        assert pick_greeting("  Привет!  ") == "Привет!"


class TestPickFinishSound:
    def test_pick_finish_sound_returns_member(self) -> None:
        assert pick_finish_sound() in FINISH_SOUNDS
