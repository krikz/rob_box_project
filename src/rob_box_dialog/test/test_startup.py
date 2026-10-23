"""Стартовая фраза (#1003): перенос v1 ``startup_greeting.py`` в ``rob_box_dialog.startup`` (#3912)."""

from __future__ import annotations

from unittest.mock import patch

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.startup import GREETINGS, pick_greeting


def test_greetings_are_the_catalog_and_six_to_ten() -> None:
    assert GREETINGS == phrases.catalog()["startup.greeting"]
    assert 6 <= len(GREETINGS) <= 10
    assert all(isinstance(p, str) and p.strip() for p in GREETINGS)
    assert any("Мяу" in p and "на связи" in p for p in GREETINGS)


def test_pick_is_a_member_and_override_wins_stripped() -> None:
    assert pick_greeting() in GREETINGS
    assert pick_greeting("  Привет!  ") == "Привет!"


def test_blank_override_falls_back_to_random_choice() -> None:
    with patch("rob_box_dialog.startup.random.choice") as choice:
        choice.return_value = GREETINGS[0]
        assert pick_greeting("") == GREETINGS[0]
        assert pick_greeting("   ") == GREETINGS[0]


def test_timings_are_v1() -> None:
    assert kn.STARTUP_GREETING_DELAY_S == 12.0
    assert kn.STARTUP_GREETING_STEPS_S == (2.0, 1.5)
